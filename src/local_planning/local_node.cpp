#include "local_node.h"
#include "scan/scan_planner.h"

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

class LocalNode : public rclcpp::Node
{
public:
  LocalNode() : Node("local_node")
  {
    const auto default_config = ament_index_cpp::get_package_share_directory("local_planning") +
      "/config/local_cfg.yaml";
    loadConfig(declare_parameter<std::string>("config_file", default_config));
    initPlanner();
    if (planner_) {loadMap();}
    publisher_ = create_publisher<interfaces::msg::LocalTrajectory>(
      "/local_trajectory", rclcpp::QoS(1).reliable().durability_volatile());
    path_sub_ = create_subscription<interfaces::msg::GlobalPath>(
      "/global_path", rclcpp::QoS(1).reliable().transient_local(),
      [this](interfaces::msg::GlobalPath::ConstSharedPtr msg) {onGlobalPath(*msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/robot/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onOdom(*msg);});
    RCLCPP_INFO(get_logger(), "局部节点就绪；algorithm=%s；每条路径仅处理一次", algorithm_.c_str());
  }

private:
  void loadConfig(const std::string & filename)
  {
    const auto cfg = YAML::LoadFile(filename);
    algorithm_ = cfg["algorithm"].as<std::string>();
    scan_config_ = cfg["scan"];
    if (algorithm_ != "teb" && algorithm_ != "ego" && algorithm_ != "scan" && algorithm_ != "mppi") {
      throw std::runtime_error("未知局部算法：" + algorithm_);
    }
    range_ = cfg["planning_range"].as<double>();
    timeout_ = cfg["planning_timeout"].as<double>();
    odom_timeout_ = cfg["odom_timeout"].as<double>();
    if (!std::isfinite(range_) || range_ <= 0 || !std::isfinite(timeout_) || timeout_ <= 0 ||
      !std::isfinite(odom_timeout_) || odom_timeout_ <= 0) {
      throw std::runtime_error("planning_range、planning_timeout 和 odom_timeout 必须为正有限数");
    }
    map_file_ = relativePath(filename, cfg["map_file"].as<std::string>());
    auto control_file = cfg["control_config"] ? cfg["control_config"].as<std::string>() : "";
    control_file = control_file.empty() ?
      ament_index_cpp::get_package_share_directory("control") + "/config/control.yaml" :
      relativePath(filename, control_file);
    const auto control = YAML::LoadFile(control_file);
    if (algorithm_ == "scan") {
      scan_config_["control_config"] = control_file;
      const auto robot_file = scan_config_["robot_file"] ? scan_config_["robot_file"].as<std::string>() : "";
      scan_config_["robot_file"] = robot_file.empty() ?
        ament_index_cpp::get_package_share_directory("control") + "/urdf/robot.urdf" :
        relativePath(filename, robot_file);
    }
    auto positive = [&control](const char * section, const char * key) {
        const double value = control[section][key].as<double>();
        if (!std::isfinite(value) || value <= 0) {
          throw std::runtime_error(std::string("无效控制约束：") + section + "." + key);
        }
        return value;
      };
    limits_.wheel_speed = positive("wheel", "linear_speed");
    limits_.wheel_yaw_rate = positive("wheel", "angular_speed");
    limits_.wheel_height = positive("wheel", "body_height");
    limits_.foot_speed = positive("foot", "linear_speed");
    limits_.foot_yaw_rate = positive("foot", "angular_speed");
    limits_.foot_height = positive("foot", "body_height");
    limits_.swing_duration = positive("foot", "swing_duration");
    limits_.step_budget = positive("foot", "step_budget");
    limits_.stair_run = positive("stair", "run");
    limits_.stair_rise = positive("stair", "rise");
    limits_.position_tolerance = positive("trajectory", "position_tolerance");
    limits_.orientation_tolerance = positive("trajectory", "orientation_tolerance");
    limits_.geometry_tolerance = positive("trajectory", "geometry_tolerance");
  }

  void initPlanner()
  {
    if (algorithm_ == "scan") {
      planner_ = std::make_unique<ScanPlanner>(scan_config_);
      return;
    }
    // 后续迁入算法时仅在这里创建对应适配器，不提供占位路线。
    RCLCPP_ERROR(get_logger(), "局部算法 %s 尚未接入，不会生成或发布轨迹", algorithm_.c_str());
  }

  void loadMap()
  {
    if (map_file_.empty() || !std::ifstream(map_file_).good()) {
      throw std::runtime_error("局部地图文件不可读：" + map_file_);
    }
    planner_->loadMap(map_file_);
  }

  void onGlobalPath(const interfaces::msg::GlobalPath & path)
  {
    pending_ = false;
    path_ = interfaces::msg::GlobalPath();
    if (path.points.empty()) {return;}
    if (path.header.frame_id != "world" || path.points.size() < 2 ||
      path.modes.size() != path.points.size() - 1 ||
      !std::all_of(path.points.begin(), path.points.end(), validPose) ||
      !std::all_of(path.modes.begin(), path.modes.end(), [](const std::string & mode) {
        return mode == "wheel" || mode == "foot";
      }))
    {
      RCLCPP_ERROR(get_logger(), "全局路径无效：需要 world 有效位姿及逐段 wheel/foot 模式");
      return;
    }
    path_ = path;
    pending_ = true;
    plan();
  }

  void onOdom(const nav_msgs::msg::Odometry & odom)
  {
    have_odom_ = odom.header.frame_id == "world" && odom.child_frame_id == "base_link" &&
      validPose(odom.pose.pose) &&
      finiteTwist(odom.twist.twist);
    if (!have_odom_) {
      RCLCPP_WARN(get_logger(), "忽略无效里程计：需要 world/base_link 位姿和有限速度");
      return;
    }
    odom_ = odom;
    odom_received_ = std::chrono::steady_clock::now();
    if (pending_) {plan();}
  }

  interfaces::msg::GlobalPath selectSegment() const
  {
    // 用两种站高比较地面参考，避免把本体 Z 当作支撑面高度。
    std::size_t begin = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path_.points.size(); ++i) {
      auto wheel = path_.points[i].position;
      auto foot = wheel;
      wheel.z += limits_.wheel_height;
      foot.z += limits_.foot_height;
      const double d = std::min(distance(wheel, odom_.pose.pose.position),
        distance(foot, odom_.pose.pose.position));
      if (d < best) {best = d; begin = std::min(i, path_.points.size() - 2);}
    }
    const auto & mode = path_.modes[begin];
    std::size_t end = begin + 1;
    double length = distance(path_.points[begin].position, path_.points[end].position);
    // 足式同模式段整体保留，避免将单阶动作截断。
    while (end < path_.modes.size() && path_.modes[end] == mode) {
      const double next = distance(path_.points[end].position, path_.points[end + 1].position);
      if (mode == "wheel" && length + next > range_) {break;}
      length += next;
      ++end;
    }
    interfaces::msg::GlobalPath segment;
    segment.header = path_.header;
    segment.points.assign(path_.points.begin() + begin, path_.points.begin() + end + 1);
    segment.modes.assign(end - begin, mode);
    return segment;
  }

  void plan()
  {
    if (!pending_ || !have_odom_) {return;}
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - odom_received_).count() > odom_timeout_) {
      RCLCPP_WARN(get_logger(), "里程计已过期，等待新状态后处理当前路径");
      return;
    }
    pending_ = false;  // 成功或失败均只处理一次，不随里程计反复重试。
    if (!planner_) {
      RCLCPP_ERROR(get_logger(), "局部规划失败：算法 %s 尚未接入", algorithm_.c_str());
      return;
    }
    try {
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout_));
      LocalRequest request{odom_, selectSegment(), limits_, deadline};
      if (!planner_->supports(request.segment.modes.front())) {
        throw std::runtime_error("算法不支持当前运动模式");
      }
      auto result = planner_->plan(request);
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("局部规划超时，结果未发布");
      }
      if (!result.success) {throw std::runtime_error(result.reason.empty() ? "局部算法失败" : result.reason);}
      validateTrajectory(result.trajectory, request);
      publishTrajectory(std::move(result.trajectory));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "局部规划失败：%s", e.what());
    }
  }

  void validateTrajectory(
    const interfaces::msg::LocalTrajectory & trajectory, const LocalRequest & request) const
  {
    const auto & mode = request.segment.modes.front();
    if (trajectory.header.frame_id != "world" || trajectory.mode != mode || trajectory.points.size() < 2) {
      throw std::runtime_error("轨迹坐标系、模式或点数无效");
    }
    geometry_msgs::msg::Pose previous;
    double previous_time = -1;
    for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto & point = trajectory.points[i];
      if (point.transforms.size() != 1 || point.velocities.size() > 1 || point.accelerations.size() > 1 ||
        (!point.velocities.empty() && !finiteTwist(point.velocities.front())) ||
        (!point.accelerations.empty() && !finiteTwist(point.accelerations.front())) ||
        point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u)
      {
        throw std::runtime_error("轨迹点结构或时间无效");
      }
      geometry_msgs::msg::Pose pose;
      const auto & transform = point.transforms.front();
      pose.position.x = transform.translation.x;
      pose.position.y = transform.translation.y;
      pose.position.z = transform.translation.z;
      pose.orientation = transform.rotation;
      const double time = point.time_from_start.sec + point.time_from_start.nanosec*1e-9;
      if (!validPose(pose) || !horizontal(pose.orientation, limits_.orientation_tolerance) ||
        (i == 0 ? time != 0 : time <= previous_time))
      {
        throw std::runtime_error("轨迹位姿无效或时间不递增");
      }
      if (i == 0) {
        auto expected = request.state.pose.pose;
        expected.position.z = request.segment.points.front().position.z +
          (mode == "wheel" ? limits_.wheel_height : limits_.foot_height);
        if (distance(pose.position, expected.position) > limits_.position_tolerance ||
          std::abs(angle(yaw(pose.orientation) - yaw(expected.orientation))) > limits_.orientation_tolerance)
        {
          throw std::runtime_error("轨迹首点未衔接实际位置和目标模式站高");
        }
      } else {
        const double dt = time - previous_time;
        const double theta = angle(yaw(pose.orientation) - yaw(previous.orientation));
        const double travel = distance(pose.position, previous.position);
        const bool planar = std::abs(pose.position.z - previous.position.z) <= limits_.geometry_tolerance;
        if ((mode == "wheel" && !planar) ||
          (planar && travel/dt > (mode == "wheel" ? limits_.wheel_speed : limits_.foot_speed) + 1e-9) ||
          std::abs(theta)/dt > (mode == "wheel" ? limits_.wheel_yaw_rate : limits_.foot_yaw_rate) + 1e-9)
        {
          throw std::runtime_error("轨迹超出基本平面速度或角速度约束");
        }
        // 完整台阶、足步预算和关节能力由对应算法接入时校验；这里不复制控制器运动学。

      }
      previous = pose;
      previous_time = time;
    }
  }

  void publishTrajectory(interfaces::msg::LocalTrajectory trajectory)
  {
    trajectory.header.stamp = now();
    publisher_->publish(trajectory);
    RCLCPP_INFO(get_logger(), "已发布单段 %s 轨迹，共 %zu 点；不代表执行完成",
      trajectory.mode.c_str(), trajectory.points.size());
  }

  std::string algorithm_, map_file_;
  YAML::Node scan_config_;
  double range_{}, timeout_{}, odom_timeout_{};
  std::chrono::steady_clock::time_point odom_received_;
  MotionLimits limits_;
  std::unique_ptr<LocalPlanner> planner_;
  bool have_odom_{false}, pending_{false};
  nav_msgs::msg::Odometry odom_;
  interfaces::msg::GlobalPath path_;
  rclcpp::Publisher<interfaces::msg::LocalTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<interfaces::msg::GlobalPath>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
}  // namespace local_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<local_planning::LocalNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("local_node"), "局部节点初始化失败：%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
