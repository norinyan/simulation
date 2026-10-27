#include "global_node.h"
#include "pct/pct_planner.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <interfaces/msg/global_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

namespace global_planning
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
bool validPose(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.position.z) && std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w) &&
         std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 &&
         std::abs(q.z*q.z + q.w*q.w - 1.0) < 1e-3;
}
double yaw(const geometry_msgs::msg::Pose & p)
{
  return 2 * std::atan2(p.orientation.z, p.orientation.w);
}
bool samePose(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return std::hypot(std::hypot(a.position.x-b.position.x, a.position.y-b.position.y),
    a.position.z-b.position.z) <= 1e-3 &&
    std::abs(std::remainder(yaw(a)-yaw(b), 2*std::acos(-1.0))) <= 1e-3;
}
}

class GlobalNode : public rclcpp::Node
{
public:
  GlobalNode() : Node("global_node")
  {
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/global_planning/status", rclcpp::QoS(1).reliable().transient_local());
    timing_publisher_ = create_publisher<std_msgs::msg::String>(
      "/global_planning/timing", rclcpp::QoS(1).reliable().transient_local());
    publisher_ = create_publisher<interfaces::msg::GlobalPath>(
      "/global_path", rclcpp::QoS(1).reliable().transient_local());
    try {
      loadConfig();
      initPlanner();
      if (planner_) {loadMap();}
    } catch (const std::exception & e) {
      error_ = e.what();
    }
    if (!error_.empty()) {publishFailure(error_);}
    else {publishStatus("idle", "等待全局规划请求");}

    input_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = input_group_;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/robot/odom",
      rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        onOdom(*msg);
      }, options);
    goal_sub_ = create_subscription<std_msgs::msg::String>("/global_planning/goal", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {onGoal(msg->data);}, options);
    replan_service_ = create_service<std_srvs::srv::Trigger>("~/replan",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {onReplan(*response);},
      rmw_qos_profile_services_default, input_group_);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {processRequest();});
    if (startup_ && error_.empty()) {onGoal(goal_name_);}
  }

private:
  struct Pending {std::string goal; bool actual; geometry_msgs::msg::Pose start;};

  void publishStatus(const std::string & state, const std::string & reason)
  {
    std_msgs::msg::String message;
    message.data = "{\"state\":" + jsonString(state) + ",\"reason\":" + jsonString(reason) + "}";
    status_publisher_->publish(message);
  }

  void loadConfig()
  {
    const auto default_file = ament_index_cpp::get_package_share_directory("global_planning") +
      "/config/global_cfg.yaml";
    const auto file = declare_parameter<std::string>("config_file", default_file);
    config_file_ = file;
    const auto config = YAML::LoadFile(file);
    const auto settings = config["planning"];
    algorithm_ = settings["algorithm"].as<std::string>();
    start_name_ = settings["start"].as<std::string>();
    goal_name_ = settings["goal"].as<std::string>();
    source_ = settings["start_source"].as<std::string>();
    timeout_ = settings["planning_timeout"].as<double>();
    odom_timeout_ = settings["odom_timeout"].as<double>();
    startup_ = settings["plan_on_startup"].as<bool>();
    map_file_ = settings["map_file"].as<std::string>();
    if (!map_file_.empty() && std::filesystem::path(map_file_).is_relative()) {
      map_file_ = (std::filesystem::path(file).parent_path() / map_file_).string();
    }
    if (source_ != "preset" && source_ != "odom") {
      throw std::runtime_error("start_source must be preset or odom");
    }
    const auto limits = config["constraints"];
    constraints_ = {limits["length"].as<double>(), limits["width"].as<double>(),
      limits["safety_margin"].as<double>(), limits["max_step"].as<double>(),
      limits["clearance"].as<double>()};
    for (double v : {timeout_, odom_timeout_}) {
      if (!std::isfinite(v) || v <= 0) {throw std::runtime_error("Invalid positive constraint");}
    }
    for (double v : {constraints_.length, constraints_.width, constraints_.max_step,
      constraints_.clearance, constraints_.safety_margin}) {
      if (!std::isfinite(v) || v < 0) {throw std::runtime_error("Invalid traversal constraint");}
    }
    const auto points = config["points"];
    if (!points.IsMap() || points.size() == 0) {throw std::runtime_error("Missing points");}
    for (const auto & entry : points) {
      const auto pose = entry.second["pose"];
      if (!pose.IsSequence() || pose.size() != 4) {
        throw std::runtime_error("Point pose must be [x, y, z, yaw]");
      }
      geometry_msgs::msg::Pose p;
      p.position.x = pose[0].as<double>(); p.position.y = pose[1].as<double>();
      p.position.z = pose[2].as<double>();
      const double heading = pose[3].as<double>();
      p.orientation.z = std::sin(heading/2); p.orientation.w = std::cos(heading/2);
      if (!validPose(p)) {throw std::runtime_error("Invalid point pose");}
      points_.emplace(entry.first.as<std::string>(), p);
    }
    if (!points_.count(goal_name_) || (source_ == "preset" && !points_.count(start_name_))) {
      throw std::runtime_error("Unknown configured start or goal point");
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu points: start=%s goal=%s algorithm=%s",
      points_.size(), start_name_.c_str(), goal_name_.c_str(), algorithm_.c_str());
  }

  void initPlanner()
  {
    if (algorithm_ != "pct" && algorithm_ != "mesh" && algorithm_ != "gb") {
      throw std::runtime_error("Unknown global algorithm: " + algorithm_);
    }
    if (algorithm_ == "pct") {planner_ = makePctPlanner(config_file_); return;}
    // 其余算法尚未接入，不使用替代算法。
    throw std::runtime_error("Global algorithm not integrated: " + algorithm_);
  }

  void loadMap()
  {
    if (constraints_.length <= 0 || constraints_.width <= 0 ||
      constraints_.max_step <= 0 || constraints_.clearance <= 0) {
      throw std::runtime_error("Configure verified robot traversal constraints before planning");
    }
    if (map_file_.empty() || !std::ifstream(map_file_).good()) {
      throw std::runtime_error("Map file is not readable: " + map_file_);
    }
    planner_->loadMap(map_file_);
  }

  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg.header.frame_id != "world" || msg.child_frame_id != "base_link" ||
      !validPose(msg.pose.pose)) {odom_.reset(); return;}
    odom_ = msg.pose.pose;
    odom_time_ = std::chrono::steady_clock::now();
  }

  bool validOdom() const  // 调用方持有 mutex_。
  {
    return odom_.has_value() &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-odom_time_).count() <=
      odom_timeout_;
  }

  bool submit(const std::string & goal, bool actual_start, std::string & reason)
  {
    // 调用方持有 mutex_，拒绝与发布失败路径期间不允许另一个请求插入。
    if (busy_) {reason = "Planner is busy"; return false;}
    if (!error_.empty()) {reason = error_; return false;}
    if (!points_.count(goal)) {reason = "Unknown goal point: " + goal; return false;}
    if (actual_start && !validOdom()) {reason = "Valid robot odometry required"; return false;}
    busy_ = true;
    current_goal_ = goal;
    pending_ = Pending{goal, actual_start, actual_start ? *odom_ : points_.at(start_name_)};
    reason = "Planning request accepted";
    publishStatus("planning", reason);
    return true;
  }

  void onGoal(const std::string & goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (busy_) {RCLCPP_WARN(get_logger(), "Planner is busy; goal rejected"); return;}
    std::string reason;
    if (!submit(goal, source_ == "odom", reason)) {publishFailure(reason);}
  }

  void onReplan(std_srvs::srv::Trigger::Response & response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_goal_.empty()) {response.message = "No current goal"; return;}
    response.success = submit(current_goal_, true, response.message);
  }

  PlanRequest resolveEndpoints(const Pending & pending)
  {
    PlanRequest request;
    request.start = pending.actual ? planner_->groundPose(pending.start) : pending.start;
    request.goal = points_.at(pending.goal);
    request.constraints = constraints_;
    request.deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_));
    if (!validPose(request.start)) {throw std::runtime_error("Invalid ground start pose");}
    return request;
  }

  interfaces::msg::GlobalPath buildGlobalPath(const PlanResult & result)
  {
    interfaces::msg::GlobalPath message;
    message.header.frame_id = "world";
    message.header.stamp = now();
    message.points = result.points;
    for (const auto terrain : result.terrain) {
      switch (terrain) {
        case Terrain::Flat: message.modes.push_back("wheel"); break;
        case Terrain::Stair:
        case Terrain::StairLanding: message.modes.push_back("foot"); break;
        default: throw std::runtime_error("Unknown terrain tag");
      }
    }
    return message;
  }

  void validateResult(const interfaces::msg::GlobalPath & path, const PlanRequest & request)
  {
    if (path.points.size() < 2 || path.modes.size()+1 != path.points.size() ||
      !std::all_of(path.points.begin(), path.points.end(), validPose) ||
      !samePose(path.points.front(), request.start) || !samePose(path.points.back(), request.goal)) {
      throw std::runtime_error("Invalid global path or endpoints");
    }
  }

  void plan(const Pending & pending)
  {
    const auto request = resolveEndpoints(pending);
    const auto started = std::chrono::steady_clock::now();
    auto timing = [&](bool success) {
      std_msgs::msg::String message;
      message.data = "{\"elapsed_ms\":" + std::to_string(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now()-started).count()) + ",\"success\":" + (success ? "true" : "false") + "}";
      timing_publisher_->publish(message);
    };
    PlanResult result;
    try {result = planner_->plan(request);}
    catch (...) {timing(false); throw;}
    timing(result.success && std::chrono::steady_clock::now() < request.deadline);
    if (std::chrono::steady_clock::now() >= request.deadline) {
      throw std::runtime_error("Global planning timeout");
    }
    if (!result.success) {throw std::runtime_error("Global planning failed: " + result.reason);}
    const auto message = buildGlobalPath(result);
    validateResult(message, request);
    publisher_->publish(message);
    publishStatus("ready", "全局规划成功，路径已发布");
    RCLCPP_INFO(get_logger(), "Published global path: %zu points", message.points.size());
  }

  void publishFailure(const std::string & reason)
  {
    interfaces::msg::GlobalPath empty;
    empty.header.frame_id = "world"; empty.header.stamp = now();
    publisher_->publish(empty);
    publishStatus("failed", reason);
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
  }

  void processRequest()
  {
    std::optional<Pending> request;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      request.swap(pending_);
    }
    if (!request) {return;}
    try {plan(*request);} catch (const std::exception & e) {publishFailure(e.what());}
    busy_ = false;
  }

  std::map<std::string, geometry_msgs::msg::Pose> points_;
  std::string algorithm_, start_name_, goal_name_, source_, map_file_, current_goal_, error_, config_file_;
  TraversalConstraints constraints_;
  double timeout_{60}, odom_timeout_{1};
  bool startup_{false};
  std::atomic_bool busy_{false};
  std::mutex mutex_;
  std::optional<geometry_msgs::msg::Pose> odom_;
  std::chrono::steady_clock::time_point odom_time_;
  std::optional<Pending> pending_;
  std::unique_ptr<GlobalPlanner> planner_;
  rclcpp::CallbackGroup::SharedPtr input_group_;
  rclcpp::Publisher<interfaces::msg::GlobalPath>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_, timing_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace global_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<global_planning::GlobalNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
