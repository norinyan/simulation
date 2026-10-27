#include "local_node.h"
#include "scan/scan_planner.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

namespace local_planning
{
namespace
{
std::string jsonString(const std::string & value)
{
  static const char * hex = "0123456789abcdef";
  std::string out = "\"";
  for (const unsigned char c : value) {
    if (c == '"' || c == '\\') {out += '\\'; out += c;}
    else if (c < 0x20) {out += "\\u00"; out += hex[c >> 4]; out += hex[c & 15];}
    else {out += c;}
  }
  return out + "\"";
}
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
    auto_execute_=declare_parameter<bool>("auto_execute",true);
    status_publisher_=create_publisher<std_msgs::msg::String>(
      "/local_planning/status",rclcpp::QoS(1).reliable().transient_local());
    timing_publisher_=create_publisher<std_msgs::msg::String>("/local_planning/timing",rclcpp::QoS(1).reliable().transient_local());
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
    start_service_=create_service<std_srvs::srv::Trigger>("~/start_task",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response){startTask(*response);});
    publishStatus("idle","等待全局路径");
    RCLCPP_INFO(get_logger(),"理想局部规划就绪，auto_execute=%s",auto_execute_?"true":"false");
  }
private:
  void publishStatus(const std::string & state,const std::string & reason) {
    std_msgs::msg::String message;
    const auto total=path_.points.empty()?0:path_.points.size()-1;
    message.data="{\"state\":"+jsonString(state)+",\"reason\":"+jsonString(reason)+
      ",\"current\":"+std::to_string(begin_)+",\"total\":"+std::to_string(total)+"}";
    status_publisher_->publish(message);
  }
  void startTask(std_srvs::srv::Trigger::Response & response) {
    if (task_active_ || job_.valid()) {response.message="任务正在执行或规划尚未结束";return;}
    if (path_.points.size()<2) {response.message="没有可执行的全局路径";return;}
    if (!have_odom_ ||
      std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count()>odom_timeout_) {
      response.message="需要新鲜有效的里程计";return;
    }
    if (!idle()) {response.message="机器人尚未静止";return;}
    auto start=path_.points.front();start.position.z+=limits_.reference_height;
    if (!atGoal(start)) {response.message="全局路径起点未衔接实际机器人位姿";return;}
    begin_=end_=0; progress_=0; pending_=task_active_=true; terminal_=false;
    response.success=true;response.message="任务已受理，等待局部规划";
    publishStatus("planning",response.message);
    // The next odom callback starts planning after the service has replied.
  }
  void loadConfig(const std::string & filename) {
    const auto cfg=YAML::LoadFile(filename);algorithm_=cfg["algorithm"].as<std::string>();
    auto positive=[](const YAML::Node & n,const char * key) {
      const double value=n[key].as<double>();
      if (!std::isfinite(value)||value<=0) {throw std::runtime_error(std::string("无效参数：")+key);}
      return value;
    };
    range_=positive(cfg,"planning_range");timeout_=positive(cfg,"planning_timeout");
    odom_timeout_=positive(cfg,"odom_timeout");
    period_=positive(cfg,"replan_period");lead_=positive(cfg,"trajectory_lead_time");
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
    pending_=task_active_=false;
    publishStatus("failed",reason);path_=interfaces::msg::GlobalPath();
    RCLCPP_ERROR(get_logger(),"%s",reason.c_str());
  }
  void onPath(const interfaces::msg::GlobalPath & path) {
    if (task_active_) {RCLCPP_WARN(get_logger(),"任务执行中拒绝新全局路径");return;}
    pending_=false;path_=interfaces::msg::GlobalPath();begin_=end_=0;progress_=0;
    active_=queued_=interfaces::msg::LocalTrajectory();terminal_=false;
    if (path.points.empty()) {publishStatus("idle","全局路径已清空");return;}
    if (path.header.frame_id!="world" || path.points.size()<2 || path.modes.size()+1!=path.points.size() ||
      !std::all_of(path.points.begin(),path.points.end(),validPose) ||
      !std::all_of(path.modes.begin(),path.modes.end(),[&](const std::string & m){return planner_->supports(m);})) {
      stop("全局路径格式无效");return;
    }
    path_=path;begin_=0;
    publishStatus("ready","全局路径已缓存，等待启动任务");
    if (auto_execute_) {pending_=task_active_=true;plan();}
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
  using Trajectory=interfaces::msg::LocalTrajectory;
  struct State {Eigen::Vector3d p,v,a;double heading{},rate{};};
  static double startTime(const Trajectory & tr) {
    return tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9;
  }
  static double pointTime(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint & p) {
    return p.time_from_start.sec+p.time_from_start.nanosec*1e-9;
  }
  static State sample(const Trajectory & tr,double now) {
    const double time=std::max(0.0,now-startTime(tr));
    const auto & last=tr.points.back();
    auto xyz=[](const auto & p){return Eigen::Vector3d(p.x,p.y,p.z);};
    if (time>=pointTime(last)) {return {xyz(last.transforms[0].translation),Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),yaw(last.transforms[0].rotation),0};}
    const auto end=std::upper_bound(tr.points.begin(),tr.points.end(),time,
      [](double t,const auto & p){return t<pointTime(p);});
    const auto & b=*end;const auto & a=*(end-1);
    const double h=pointTime(b)-pointTime(a),u=(time-pointTime(a))/h,u2=u*u,u3=u2*u;
    const auto pa=xyz(a.transforms[0].translation),pb=xyz(b.transforms[0].translation);
    const auto va=xyz(a.velocities[0].linear),vb=xyz(b.velocities[0].linear);
    const double w0=a.velocities[0].angular.z,w1=b.velocities[0].angular.z;
    const double delta=angle(yaw(b.transforms[0].rotation)-yaw(a.transforms[0].rotation));
    return {(2*u3-3*u2+1)*pa+(u3-2*u2+u)*h*va+(-2*u3+3*u2)*pb+(u3-u2)*h*vb,
      (6*u2-6*u)/h*pa+(3*u2-4*u+1)*va+(-6*u2+6*u)/h*pb+(3*u2-2*u)*vb,
      (12*u-6)/(h*h)*pa+(6*u-4)/h*va+(-12*u+6)/(h*h)*pb+(6*u-2)/h*vb,
      yaw(a.transforms[0].rotation)+(-2*u3+3*u2)*delta+(u3-2*u2+u)*h*w0+(u3-u2)*h*w1,
      (-6*u2+6*u)/h*delta+(3*u2-4*u+1)*w0+(3*u2-2*u)*w1};
  }
  double project(const geometry_msgs::msg::Point & body,double minimum) const {
    const Eigen::Vector3d p(body.x,body.y,body.z-limits_.reference_height);
    double cumulative=0,best=std::numeric_limits<double>::infinity(),result=minimum;
    for (std::size_t i=1;i<path_.points.size();++i) {
      const auto & a=path_.points[i-1].position;const auto & b=path_.points[i].position;
      const Eigen::Vector3d origin(a.x,a.y,a.z),edge(b.x-a.x,b.y-a.y,b.z-a.z);
      const double length=edge.norm();
      if (cumulative>minimum+range_) {break;}
      if (length>1e-9 && cumulative+length>=minimum) {
        const double t=std::clamp((p-origin).dot(edge)/(length*length),
          std::clamp((minimum-cumulative)/length,0.0,1.0),1.0);
        const double error=(origin+t*edge-p).squaredNorm();
        if (error<best) {best=error;result=cumulative+t*length;}
      }
      cumulative+=length;
    }
    return result;
  }
  interfaces::msg::GlobalPath segment(const nav_msgs::msg::Odometry & predicted) {
    const double from=project(predicted.pose.pose.position,progress_);
    double cumulative=0;
    interfaces::msg::GlobalPath result;result.header=path_.header;
    auto first=predicted.pose.pose;first.position.z-=limits_.reference_height;
    result.points.push_back(first);end_=path_.points.size()-1;
    for (std::size_t i=1;i<path_.points.size();++i) {
      const auto & a=path_.points[i-1];const auto & b=path_.points[i];
      const double length=distance(a.position,b.position),next=cumulative+length;
      if (next>from+1e-5) {
        auto point=b;
        if (next>from+range_) {
          const double t=std::clamp((from+range_-cumulative)/length,0.0,1.0);
          point.position.x=a.position.x+t*(b.position.x-a.position.x);
          point.position.y=a.position.y+t*(b.position.y-a.position.y);
          point.position.z=a.position.z+t*(b.position.z-a.position.z);
        }
        result.points.push_back(point);result.modes.push_back(path_.modes[i-1]);end_=i;
        if (next>=from+range_) {break;}
      }
      cumulative=next;
    }
    if (result.points.size()<2) {throw std::runtime_error("当前参考路径过短");}
    job_terminal_=end_+1==path_.points.size() &&
      distance(result.points.back().position,path_.points.back().position)<1e-7;
    return result;
  }
  void onOdom(const nav_msgs::msg::Odometry & msg) {
    if (msg.header.frame_id!="world" || msg.child_frame_id!="base_link" || !validPose(msg.pose.pose) ||
      !finiteTwist(msg.twist.twist)) {return;}
    if (have_odom_ && stamp(msg)<stamp(odom_)) {stop("仿真时间回退，停止任务");active_.points.clear();queued_.points.clear();}
    odom_=msg;have_odom_=true;received_=std::chrono::steady_clock::now();
    const double now=stamp(msg);
    if (!queued_.points.empty() && now>=startTime(queued_)) {
      active_=std::move(queued_);queued_=Trajectory();terminal_=queued_terminal_;
    }
    if (job_.valid() && job_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      auto result=job_.get();
      std_msgs::msg::String timing;
      timing.data="{\"elapsed_ms\":"+std::to_string(planning_ms_)+",\"success\":"+(result.success?"true":"false")+"}";
      timing_publisher_->publish(timing);
      if (task_active_) {
        try {
          if (!result.success) {throw std::runtime_error(result.reason);}
          if (stamp(request_.state)<=now) {throw std::runtime_error("规划结果已错过预约切换时刻");}
          validate(result.trajectory,request_);
          if (!publisher_->get_subscription_count()) {throw std::runtime_error("理想执行器未连接");}
          result.trajectory.header.stamp=request_.state.header.stamp;
          queued_=std::move(result.trajectory);queued_terminal_=job_terminal_;
          publisher_->publish(queued_);last_plan_=now;
          publishStatus("executing","滚动轨迹已发布，预约连续接替");
          RCLCPP_INFO(get_logger(),"滚动发布 index=%zu..%zu switch=%.3f planning_ms=%.3f terminal=%d",
            begin_,end_,startTime(queued_),planning_ms_,queued_terminal_);
        } catch (const std::exception & e) {stop(std::string("局部规划失败，保留旧轨迹停车末端：")+e.what());}
      }
    }
    if (!task_active_) {return;}
    progress_=project(odom_.pose.pose.position,progress_);
    double accumulated=0;begin_=0;
    for (std::size_t i=1;i<path_.points.size();++i) {
      accumulated+=distance(path_.points[i-1].position,path_.points[i].position);
      if (accumulated>progress_+1e-6) {break;}begin_=i;
    }
    if (terminal_ && !active_.points.empty() && now>=startTime(active_)+pointTime(active_.points.back())) {
      auto goal=path_.points.back();goal.position.z+=limits_.reference_height;
      if (atGoal(goal) && idle()) {
        begin_=path_.points.size()-1;task_active_=false;
        publishStatus("completed","全局终点已到达且机器人静止");return;
      }
      if (now>startTime(active_)+pointTime(active_.points.back())+odom_timeout_) {
        stop("终点未按期到达");return;
      }
    }
    if (!job_.valid() && queued_.points.empty() && !terminal_ && (pending_ || now-last_plan_>=period_)) {plan();}
  }
  void validate(const interfaces::msg::LocalTrajectory & tr,const LocalRequest & request) const {
    if (tr.header.frame_id!="world" || tr.points.size()<2 || !planner_->supports(tr.mode)) {
      throw std::runtime_error("轨迹格式无效");
    }
    geometry_msgs::msg::Pose previous;double previous_time=0;
    Eigen::Vector3d previous_velocity=Eigen::Vector3d::Zero();
    for (std::size_t i=0;i<tr.points.size();++i) {
      const auto & p=tr.points[i];
      if (p.transforms.size()!=1 || p.velocities.size()!=1 || p.accelerations.size()!=1 || p.time_from_start.sec<0 || p.time_from_start.nanosec>=1000000000u) {
        throw std::runtime_error("轨迹点结构无效");
      }
      const auto & t=p.transforms[0];geometry_msgs::msg::Pose pose;
      pose.position.x=t.translation.x;pose.position.y=t.translation.y;pose.position.z=t.translation.z;pose.orientation=t.rotation;
      const double time=p.time_from_start.sec+p.time_from_start.nanosec*1e-9;
      if (!validPose(pose)||!horizontal(pose.orientation,1e-6)||(i==0?time!=0:time<=previous_time)) {
        throw std::runtime_error("轨迹位姿或时间无效");
      }
      const auto & v=p.velocities[0];const auto & acc=p.accelerations[0];
      const Eigen::Vector3d velocity(v.linear.x,v.linear.y,v.linear.z),acceleration(acc.linear.x,acc.linear.y,acc.linear.z);
      if (!finiteTwist(v)||!finiteTwist(acc)||velocity.norm()>limits_.speed+1e-6 ||
        acceleration.norm()>limits_.acceleration+1e-6 || std::abs(v.angular.z)>limits_.yaw_rate+1e-6) {
        throw std::runtime_error("轨迹运动导数无效或超限");
      }
      if (i==0) {
        const auto & q=request.state.pose.pose;const auto & body=request.state.twist.twist;
        const double theta=yaw(q.orientation),c=std::cos(theta),s=std::sin(theta);
        const Eigen::Vector3d expected(c*body.linear.x-s*body.linear.y,s*body.linear.x+c*body.linear.y,body.linear.z);
        if (distance(pose.position,q.position)>limits_.position_tolerance ||
          std::abs(angle(yaw(pose.orientation)-theta))>limits_.orientation_tolerance ||
          (velocity-expected).norm()>1e-5 || std::abs(v.angular.z-body.angular.z)>1e-5 ||
          (acceleration-Eigen::Vector3d(request.acceleration.x,request.acceleration.y,request.acceleration.z)).norm()>1e-5) {
          throw std::runtime_error("轨迹首点未衔接预约运动状态");
        }
      } else if ((velocity-previous_velocity).norm()/(time-previous_time)>limits_.acceleration+1e-5) {
        throw std::runtime_error("轨迹加速度超限");
      }
      previous_velocity=velocity;previous=pose;previous_time=time;
    }
    if (previous_velocity.norm()>1e-6 || std::abs(tr.points.back().velocities[0].angular.z)>1e-6) {
      throw std::runtime_error("轨迹缺少停车末端");
    }
    auto goal=request.segment.points.back();goal.position.z+=limits_.reference_height;
    if (distance(previous.position,goal.position)>limits_.position_tolerance ||
      std::abs(angle(yaw(previous.orientation)-yaw(goal.orientation)))>limits_.orientation_tolerance) {
      throw std::runtime_error("轨迹终点不衔接当前参考段");
    }
  }
  void plan() {
    if (!have_odom_ || !task_active_ || job_.valid()) {return;}
    pending_=false;
    try {
      request_=LocalRequest();request_.state=odom_;request_.limits=limits_;request_.warm_start=active_;
      const double switch_time=stamp(odom_)+lead_;
      if (!active_.points.empty()) {
        const auto state=sample(active_,switch_time);
        auto & pose=request_.state.pose.pose;pose.position.x=state.p.x();pose.position.y=state.p.y();pose.position.z=state.p.z();
        pose.orientation.x=pose.orientation.y=0;pose.orientation.z=std::sin(state.heading/2);pose.orientation.w=std::cos(state.heading/2);
        const double c=std::cos(state.heading),s=std::sin(state.heading);
        auto & body=request_.state.twist.twist;body=geometry_msgs::msg::Twist();
        body.linear.x=c*state.v.x()+s*state.v.y();body.linear.y=-s*state.v.x()+c*state.v.y();
        body.linear.z=state.v.z();body.angular.z=state.rate;
        request_.acceleration.x=state.a.x();request_.acceleration.y=state.a.y();request_.acceleration.z=state.a.z();
      }
      request_.state.header.stamp=rclcpp::Time(static_cast<int64_t>(std::llround(switch_time*1e9)));
      request_.segment=segment(request_.state);
      request_.deadline=std::chrono::steady_clock::now()+
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_));
      publishStatus("planning","运动中生成下一条局部轨迹");
      const auto request=request_;
      job_=std::async(std::launch::async,[this,request] {
        const auto start=std::chrono::steady_clock::now();
        auto result=planner_->plan(request);
        if (std::chrono::steady_clock::now()>=request.deadline) {
          result.success=false;result.reason="局部规划超时";
        }
        planning_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        return result;
      });
    } catch (const std::exception & e) {stop(std::string("滚动规划请求无效：")+e.what());}
  }
  std::string algorithm_,map_file_;YAML::Node scan_config_;MotionLimits limits_;
  double range_{},timeout_{},odom_timeout_{},period_{},lead_{},progress_{},last_plan_{},planning_ms_{};
  bool auto_execute_{true},task_active_{false};
  bool pending_{false},have_odom_{false},terminal_{false},queued_terminal_{false},job_terminal_{false};std::size_t begin_{},end_{};
  std::chrono::steady_clock::time_point received_;
  nav_msgs::msg::Odometry odom_;interfaces::msg::GlobalPath path_;
  std::unique_ptr<LocalPlanner> planner_;
  Trajectory active_,queued_;LocalRequest request_;
  std::future<LocalResult> job_;
  rclcpp::Publisher<interfaces::msg::LocalTrajectory>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_,timing_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
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
