#include "local_node.h"
#include "scan/scan_planner.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace local_planning
{
namespace
{
double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

double yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return std::atan2(2*(q.w*q.z + q.x*q.y), n - 2*(q.y*q.y + q.z*q.z));
}

double angle(double a) {return std::atan2(std::sin(a), std::cos(a));}

bool validPose(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  const double n = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.position.z) && std::isfinite(n) && n > 1e-9;
}

bool finiteTwist(const geometry_msgs::msg::Twist & t)
{
  return std::isfinite(t.linear.x) && std::isfinite(t.linear.y) &&
         std::isfinite(t.linear.z) && std::isfinite(t.angular.x) &&
         std::isfinite(t.angular.y) && std::isfinite(t.angular.z);
}

bool horizontal(const geometry_msgs::msg::Quaternion & q, double tolerance)
{
  const double n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  const double roll = std::atan2(2*(q.w*q.x + q.y*q.z), n - 2*(q.x*q.x + q.y*q.y));
  const double pitch = std::asin(std::clamp(2*(q.w*q.y - q.z*q.x)/n, -1.0, 1.0));
  return std::abs(roll) <= tolerance && std::abs(pitch) <= tolerance;
}

std::string relativePath(const std::filesystem::path & config, const std::string & filename)
{
  if (filename.empty()) {return {};}
  const std::filesystem::path path(filename);
  return (path.is_absolute() ? path : config.parent_path()/path).lexically_normal().string();
}
}  // namespace

class LocalNode : public rclcpp::Node {
public:
  LocalNode():Node("local_node") {
    const auto file=declare_parameter<std::string>("config_file",
      ament_index_cpp::get_package_share_directory("local_planning")+"/config/local_cfg.yaml");
    loadConfig(file);
    if (algorithm_=="scan") {planner_=std::make_unique<ScanPlanner>(scan_config_);}
    else {throw std::runtime_error("局部算法尚未接入："+algorithm_);}
    planner_->loadMap(map_file_);
    publisher_=create_publisher<interfaces::msg::LocalTrajectory>("/local_trajectory",10);
    path_sub_=create_subscription<interfaces::msg::GlobalPath>("/global_path",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](interfaces::msg::GlobalPath::ConstSharedPtr msg){onPath(*msg);});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/robot/odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){onOdom(*msg);});
    RCLCPP_INFO(get_logger(),"理想局部规划就绪，按 odom 自动推进局部段");
  }
private:
  void loadConfig(const std::string & filename) {
    const auto cfg=YAML::LoadFile(filename);algorithm_=cfg["algorithm"].as<std::string>();
    auto positive=[](const YAML::Node & n,const char * key) {
      const double value=n[key].as<double>();
      if (!std::isfinite(value)||value<=0) {throw std::runtime_error(std::string("无效参数：")+key);}
      return value;
    };
    range_=positive(cfg,"planning_range");timeout_=positive(cfg,"planning_timeout");
    odom_timeout_=positive(cfg,"odom_timeout");heading_range_=positive(cfg,"segment_heading_change");
    if (heading_range_>std::acos(-1.0)) {throw std::runtime_error("分段转角必须不超过 pi");}
    map_file_=relativePath(filename,cfg["map_file"].as<std::string>());
    const auto path=cfg["ideal_config"]?cfg["ideal_config"].as<std::string>():"";
    const auto ideal=YAML::LoadFile(path.empty()?
      ament_index_cpp::get_package_share_directory("control")+"/config/ideal.yaml":relativePath(filename,path));
    limits_.reference_height=positive(ideal,"reference_height");
    limits_.position_tolerance=positive(ideal,"position_tolerance");
    limits_.orientation_tolerance=positive(ideal,"orientation_tolerance");
    limits_.speed=positive(cfg,"linear_speed");limits_.yaw_rate=positive(cfg,"angular_speed");
    limits_.acceleration=positive(cfg,"max_acceleration");
    scan_config_=YAML::Clone(cfg["scan"]);scan_config_["point_clearance"]=positive(cfg,"point_clearance");
  }
  void stop(const std::string & reason) {
    pending_=executing_=false;path_=interfaces::msg::GlobalPath();
    RCLCPP_ERROR(get_logger(),"%s",reason.c_str());
  }
  void onPath(const interfaces::msg::GlobalPath & path) {
    if (executing_) {RCLCPP_WARN(get_logger(),"执行中拒绝新全局路径");return;}
    pending_=false;path_=interfaces::msg::GlobalPath();
    if (path.points.empty()) {return;}
    if (path.header.frame_id!="world" || path.points.size()<2 || path.modes.size()+1!=path.points.size() ||
      !std::all_of(path.points.begin(),path.points.end(),validPose) ||
      !std::all_of(path.modes.begin(),path.modes.end(),[&](const std::string & m){return planner_->supports(m);})) {
      stop("全局路径格式无效");return;
    }
    path_=path;begin_=0;pending_=true;plan();
  }
  static double stamp(const nav_msgs::msg::Odometry & m) {
    return m.header.stamp.sec+m.header.stamp.nanosec*1e-9;
  }
  bool atGoal(const geometry_msgs::msg::Pose & target) const {
    return distance(odom_.pose.pose.position,target.position)<=limits_.position_tolerance &&
      std::abs(angle(yaw(odom_.pose.pose.orientation)-yaw(target.orientation)))<=limits_.orientation_tolerance;
  }
  bool idle() const {
    const auto & v=odom_.twist.twist;
    return std::hypot(std::hypot(v.linear.x,v.linear.y),v.linear.z)<=1e-3 && std::abs(v.angular.z)<=1e-3;
  }
  void onOdom(const nav_msgs::msg::Odometry & msg) {
    if (msg.header.frame_id!="world" || msg.child_frame_id!="base_link" || !validPose(msg.pose.pose) ||
      !finiteTwist(msg.twist.twist)) {return;}
    if (have_odom_ && stamp(msg)<stamp(odom_)) {stop("仿真时间回退，停止任务");}
    odom_=msg;have_odom_=true;received_=std::chrono::steady_clock::now();
    if (executing_) {
      // Planning can outlive several simulation frames. Begin timing only from
      // post-publication feedback, and re-anchor when actual motion is observed.
      if (!std::isfinite(execution_stamp_)) {execution_stamp_=stamp(odom_);}
      if (!motion_seen_ && !idle()) {motion_seen_=true;execution_stamp_=stamp(odom_);}
      const double elapsed=stamp(odom_)-execution_stamp_;
      if (elapsed>=duration_ && atGoal(target_) && idle()) {
        executing_=false;begin_=end_;
        RCLCPP_INFO(get_logger(),"局部段执行完成，global index=%zu/%zu",begin_,path_.points.size()-1);
        if (begin_+1>=path_.points.size()) {
          RCLCPP_INFO(get_logger(),"全局任务执行完成");path_=interfaces::msg::GlobalPath();return;
        }
        pending_=true;
      } else if (elapsed>duration_+odom_timeout_+1) {stop("局部段未按期到达，停止推进");}
    }
    if (pending_) {plan();}
  }
  interfaces::msg::GlobalPath segment() {
    end_=begin_+1;double length=distance(path_.points[begin_].position,path_.points[end_].position);
    const auto & first=path_.points[begin_].position;
    const auto & second=path_.points[end_].position;
    double initial_heading=std::atan2(second.y-first.y,second.x-first.x);
    bool have_heading=std::hypot(second.x-first.x,second.y-first.y)>1e-8;
    while (end_+1<path_.points.size()) {
      const auto & a=path_.points[end_].position;
      const auto & b=path_.points[end_+1].position;
      if (std::hypot(b.x-a.x,b.y-a.y)>1e-8) {
        const double heading=std::atan2(b.y-a.y,b.x-a.x);
        // Keep sharp global-path bends as real segment endpoints, so smoothing
        // cannot cut an entire stair-entry/platform corner in one local horizon.
        if (have_heading && std::abs(angle(heading-initial_heading))>heading_range_+1e-6) {break;}
        if (!have_heading) {initial_heading=heading;have_heading=true;}
      }
      const double next=distance(path_.points[end_].position,path_.points[end_+1].position);
      if (length+next>range_) {break;}
      length+=next;++end_;
    }
    interfaces::msg::GlobalPath result;result.header=path_.header;
    result.points.assign(path_.points.begin()+begin_,path_.points.begin()+end_+1);
    result.modes.assign(path_.modes.begin()+begin_,path_.modes.begin()+end_);
    return result;
  }
  void validate(const interfaces::msg::LocalTrajectory & tr,const LocalRequest & request) const {
    if (tr.header.frame_id!="world" || tr.points.size()<2 || !planner_->supports(tr.mode)) {
      throw std::runtime_error("轨迹格式无效");
    }
    geometry_msgs::msg::Pose previous;double previous_time=0;
    Eigen::Vector3d previous_velocity=Eigen::Vector3d::Zero();double previous_dt=0;
    for (std::size_t i=0;i<tr.points.size();++i) {
      const auto & p=tr.points[i];
      if (p.transforms.size()!=1 || p.time_from_start.sec<0 || p.time_from_start.nanosec>=1000000000u) {
        throw std::runtime_error("轨迹点结构无效");
      }
      const auto & t=p.transforms[0];geometry_msgs::msg::Pose pose;
      pose.position.x=t.translation.x;pose.position.y=t.translation.y;pose.position.z=t.translation.z;pose.orientation=t.rotation;
      const double time=p.time_from_start.sec+p.time_from_start.nanosec*1e-9;
      if (!validPose(pose)||!horizontal(pose.orientation,1e-6)||(i==0?time!=0:time<=previous_time)) {
        throw std::runtime_error("轨迹位姿或时间无效");
      }
      if (i==0) {
        if (distance(pose.position,request.state.pose.pose.position)>limits_.position_tolerance ||
          std::abs(angle(yaw(pose.orientation)-yaw(request.state.pose.pose.orientation)))>limits_.orientation_tolerance) {
          throw std::runtime_error("轨迹首点不衔接实际里程计");
        }
      } else {
        const double dt=time-previous_time;
        const Eigen::Vector3d velocity((pose.position.x-previous.position.x)/dt,
          (pose.position.y-previous.position.y)/dt,(pose.position.z-previous.position.z)/dt);
        if (velocity.norm()>limits_.speed+1e-8 || std::abs(angle(yaw(pose.orientation)-yaw(previous.orientation)))/dt>limits_.yaw_rate+1e-8 ||
          (velocity-previous_velocity).norm()/(i==1?dt*.5:(dt+previous_dt)*.5)>limits_.acceleration+1e-7) {
          throw std::runtime_error("轨迹通用运动限制超限");
        }
        previous_velocity=velocity;previous_dt=dt;
      }
      previous=pose;previous_time=time;
    }
    if (previous_velocity.norm()/(previous_dt*.5)>limits_.acceleration+1e-7) {
      throw std::runtime_error("轨迹末端加速度超限");
    }
    auto goal=request.segment.points.back();goal.position.z+=limits_.reference_height;
    if (distance(previous.position,goal.position)>limits_.position_tolerance ||
      std::abs(angle(yaw(previous.orientation)-yaw(goal.orientation)))>limits_.orientation_tolerance) {
      throw std::runtime_error("轨迹终点不衔接当前参考段");
    }
  }
  void plan() {
    if (!pending_||!have_odom_||executing_) {return;}
    if (std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count()>odom_timeout_) {return;}
    pending_=false;
    try {
      if (!idle()) {throw std::runtime_error("局部规划需要静止起点");}
      const auto start=std::chrono::steady_clock::now();
      const auto deadline=start+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_));
      LocalRequest request{odom_,segment(),limits_,deadline};
      auto result=planner_->plan(request);
      if (std::chrono::steady_clock::now()>=deadline) {throw std::runtime_error("局部规划超时");}
      if (!result.success) {throw std::runtime_error(result.reason);}
      validate(result.trajectory,request);
      if (!publisher_->get_subscription_count()) {throw std::runtime_error("理想执行器未连接，轨迹未发布");}
      const auto & last=result.trajectory.points.back();const auto & t=last.transforms[0];
      target_.position.x=t.translation.x;target_.position.y=t.translation.y;target_.position.z=t.translation.z;target_.orientation=t.rotation;
      duration_=last.time_from_start.sec+last.time_from_start.nanosec*1e-9;
      execution_stamp_=std::numeric_limits<double>::quiet_NaN();motion_seen_=false;
      result.trajectory.header.stamp=odom_.header.stamp;
      publisher_->publish(result.trajectory);executing_=true;
      RCLCPP_INFO(get_logger(),"局部段发布 index=%zu..%zu points=%zu planning_ms=%.3f duration=%.3f %s",
        begin_,end_,result.trajectory.points.size(),std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count(),duration_,result.reason.c_str());
    } catch (const std::exception & e) {stop(std::string("局部规划失败：")+e.what());}
  }
  std::string algorithm_,map_file_;YAML::Node scan_config_;MotionLimits limits_;
  double range_{},heading_range_{},timeout_{},odom_timeout_{},execution_stamp_{},duration_{};
  bool pending_{false},executing_{false},have_odom_{false},motion_seen_{false};std::size_t begin_{},end_{};
  std::chrono::steady_clock::time_point received_;
  nav_msgs::msg::Odometry odom_;geometry_msgs::msg::Pose target_;interfaces::msg::GlobalPath path_;
  std::unique_ptr<LocalPlanner> planner_;
  rclcpp::Publisher<interfaces::msg::LocalTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<interfaces::msg::GlobalPath>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
}  // namespace local_planning

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);int status=0;
  try {rclcpp::spin(std::make_shared<local_planning::LocalNode>());}
  catch (const std::exception & e) {RCLCPP_ERROR(rclcpp::get_logger("local_node"),"局部节点初始化失败：%s",e.what());status=1;}
  rclcpp::shutdown();return status;
}
