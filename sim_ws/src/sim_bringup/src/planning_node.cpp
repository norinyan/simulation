#include "astar.h"
#include "teb_optimizer.h"
#include "footstep_planner.h"

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using planning::BoxObstacle;
using planning::PlanRequest;
using planning::Pose;
constexpr double kTolerance = 1e-6;

void require(bool condition, const std::string& message)
{
  if (!condition) throw std::runtime_error(message);
}

double scalar(const YAML::Node& node)
{
  const double value = node.as<double>();
  require(std::isfinite(value), "Non-finite scene value");
  return value;
}

std::vector<double> values(const YAML::Node& node, std::size_t count)
{
  require(node.IsSequence() && node.size() == count, "Incorrect scene vector size");
  std::vector<double> result;
  for (const auto& item : node) result.push_back(scalar(item));
  return result;
}

void equal(double a, double b, const std::string& description)
{
  require(std::abs(a-b) < kTolerance, description);
}

Pose pose(const YAML::Node& node)
{
  const auto p = values(node, 4);
  return {p[0], p[1], p[2], p[3]};
}

planning::MapBounds bounds(const YAML::Node& node)
{
  const auto b = values(node, 4);
  require(b[0] < b[1] && b[2] < b[3], "Invalid scene bounds");
  return {b[0], b[1], b[2], b[3]};
}

struct Scene
{
  std::string world_name;
  PlanRequest lower;
  PlanRequest upper;
  double stair_x{0};
  double rise{0};
  double tread{0};
  double min_y{0};
  double max_y{0};
  int count{0};
};

Scene loadScene(const std::string& filename, double margin)
{
  const auto config = YAML::LoadFile(filename);
  require(config["schema_version"].as<int>() == 1, "Unsupported scene schema");
  const auto scene = config["scene"], robot = config["robot"];
  require(scene["units"]["length"].as<std::string>() == "m" &&
          scene["units"]["angle"].as<std::string>() == "rad", "Expected metres/radians");
  Scene result;
  result.world_name = scene["name"].as<std::string>();
  require(!result.world_name.empty(), "Empty Gazebo world name");
  auto& lo = result.lower;
  auto& up = result.upper;
  lo.frame_id = up.frame_id = scene["frame"].as<std::string>();
  require(!lo.frame_id.empty(), "Empty scene frame");
  lo.bounds = bounds(scene["lower_bounds"]);
  up.bounds = bounds(scene["upper_bounds"]);
  lo.start = pose(scene["waypoints"]["A"]);
  lo.goal = pose(scene["waypoints"]["E"]);
  up.start = pose(scene["waypoints"]["X"]);
  up.goal = pose(scene["waypoints"]["B"]);
  lo.footprint = up.footprint = {scalar(robot["wheel_mode_length"]), scalar(robot["wheel_mode_width"])};
  lo.safety_margin = up.safety_margin = margin;
  require(margin >= 0 && std::isfinite(margin), "Invalid safety margin");
  require(lo.footprint.length > 0 && lo.footprint.width > 0, "Invalid robot footprint");
  for (const auto& collection : {scene["obstacles"], scene["walls"]}) {
    require(collection.IsSequence(), "Obstacle/wall collection must be a sequence");
    for (const auto& entry : collection) {
      require(entry["shape"].as<std::string>() == "box", "Only box scene obstacles are supported");
      const auto c = values(entry["center"], 3), s = values(entry["size"], 3);
      require(s[0] > 0 && s[1] > 0 && s[2] > 0, "Invalid obstacle size");
      lo.obstacles.push_back({c[0], c[1], c[2], s[0], s[1], s[2]});
    }
  }
  up.obstacles = lo.obstacles;
  const double lower_z = scalar(scene["ground_heights"]["lower"]);
  const double upper_z = scalar(scene["ground_heights"]["upper"]);
  equal(lo.start.z, lower_z, "A is not on lower surface");
  equal(lo.goal.z, lower_z, "E is not on lower surface");
  equal(up.start.z, upper_z, "X is not on upper surface");
  equal(up.goal.z, upper_z, "B is not on upper surface");
  const auto stair = scene["stair"];
  result.stair_x = scalar(stair["start_x"]);
  result.rise = scalar(stair["rise"]);
  result.tread = scalar(stair["tread"]);
  result.count = stair["count"].as<int>();
  const auto passage = values(stair["passage_y"], 2);
  result.min_y = passage[0];
  result.max_y = passage[1];
  require(result.rise > 0 && result.tread > 0 && result.count > 0 && result.count <= 10000,
          "Invalid staircase dimensions");
  equal(result.max_y-result.min_y, scalar(stair["width"]), "Inconsistent stair passage width");
  equal((result.max_y+result.min_y)/2, scalar(stair["center_y"]), "Inconsistent stair centre");
  equal(lo.bounds.max_x, result.stair_x, "Lower platform does not meet stairs");
  equal(up.bounds.min_x, result.stair_x+result.count*result.tread, "Upper platform does not meet stairs");
  equal(upper_z, lower_z+result.count*result.rise, "Stair/platform height mismatch");
  for (const auto& b : {lo.bounds, up.bounds}) {
    equal(b.min_y, result.min_y, "Platform/stair lower Y mismatch");
    equal(b.max_y, result.max_y, "Platform/stair upper Y mismatch");
  }
  const auto steps = stair["steps"];
  require(steps.IsSequence() && steps.size() == static_cast<std::size_t>(result.count),
          "Stair count/geometry mismatch");
  for (int i = 0; i < result.count; ++i) {
    const auto step = steps[i];
    require(step["shape"].as<std::string>() == "box", "Unsupported step shape");
    const auto c = values(step["center"], 3), s = values(step["size"], 3);
    equal(c[0]-s[0]/2, result.stair_x+i*result.tread, "Step leading edge mismatch");
    equal(s[0], result.tread, "Step tread mismatch");
    equal(c[1]-s[1]/2, result.min_y, "Step lower Y mismatch");
    equal(c[1]+s[1]/2, result.max_y, "Step upper Y mismatch");
    equal(c[2]+s[2]/2, scalar(step["top_z"]), "Step top mismatch");
    equal(scalar(step["top_z"]), lower_z+(i+1)*result.rise, "Step rise mismatch");
  }
  return result;
}

std::vector<Pose> stairReference(const Scene& scene)
{
  const auto& entry = scene.lower.goal;
  const auto& exit = scene.upper.start;
  equal(entry.y, exit.y, "E/X must share an aligned stair lane");
  equal(entry.yaw, 0, "Stair reference expects +X entry yaw");
  equal(exit.yaw, 0, "Stair reference expects +X exit yaw");
  const double half_width = scene.lower.footprint.width/2+scene.lower.safety_margin;
  require(entry.y-half_width >= scene.min_y && entry.y+half_width <= scene.max_y,
          "Robot footprint does not fit stair passage");
  require(entry.x <= scene.stair_x && exit.x >= scene.stair_x+scene.count*scene.tread,
          "Stair waypoints do not bracket the steps");
  // Check the complete aligned corridor against scene obstacles/walls. Terrain
  // boxes are intentionally separate: this is a geometric reference, not gait feasibility.
  const double half_length = scene.lower.footprint.length/2+scene.lower.safety_margin;
  for (const auto& box : scene.lower.obstacles) {
    if (box.center_z+box.size_z/2 <= entry.z+kTolerance) continue;
    const bool overlap = box.center_x+box.size_x/2 >= entry.x-half_length &&
      box.center_x-box.size_x/2 <= exit.x+half_length &&
      box.center_y+box.size_y/2 >= entry.y-half_width &&
      box.center_y-box.size_y/2 <= entry.y+half_width;
    require(!overlap, "Stair reference corridor blocked by a scene obstacle/wall");
  }
  std::vector<Pose> path{entry, {scene.stair_x, entry.y, entry.z, 0}};
  for (int i = 0; i < scene.count; ++i) {
    const double x = scene.stair_x+i*scene.tread;
    if (i > 0) path.push_back({x, entry.y, entry.z+i*scene.rise, 0});
    path.push_back({x, entry.y, entry.z+(i+1)*scene.rise, 0});
  }
  path.push_back(exit);
  return path;
}

planning::FootLocalPlanRequest footRequest(const std::string& filename, const Scene& scene,
                                         const std::vector<Pose>& reference)
{
  planning::FootLocalPlanRequest request;
  request.geometry = scene.lower;
  request.geometry.start = scene.lower.goal;
  request.geometry.goal = scene.upper.start;
  request.geometry.bounds.max_x = scene.upper.bounds.max_x;
  request.geometry.safety_margin = 0;  // Foot support margin is configured separately.
  request.reference_path = reference;
  const auto config = YAML::LoadFile(filename);
  const auto terrain = config["scene"], robot = config["robot"];
  auto box = [](const YAML::Node& node) {
    require(node["shape"].as<std::string>() == "box", "Unsupported foot terrain shape");
    const auto c = values(node["center"], 3), s = values(node["size"], 3);
    return BoxObstacle{c[0], c[1], c[2], s[0], s[1], s[2]};
  };
  for (const auto& surface : terrain["surfaces"]) request.terrain.push_back(box(surface.second));
  for (const auto& step : terrain["stair"]["steps"]) request.terrain.push_back(box(step));
  auto xyz = [](const YAML::Node& node) {
    const auto v = values(node, 3);
    return std::array<double, 3>{v[0], v[1], v[2]};
  };
  auto& r = request.robot;
  r.body_size = xyz(robot["body_size"]);
  r.shoulder_offset = xyz(robot["shoulder_offset"]);
  r.foot_size = xyz(robot["foot_size"]);
  r.folded_pitch = {scalar(robot["folded_shoulder_pitch"]), scalar(robot["folded_knee_pitch"]),
                    scalar(robot["folded_ankle_pitch"])};
  r.upper_leg_length = scalar(robot["upper_leg_length"]);
  r.lower_leg_length = scalar(robot["lower_leg_length"]);
  r.leg_width = scalar(robot["leg_width"]);
  r.foot_center_offset_x = scalar(robot["foot_center_offset_x"]);
  const auto joint_limits = values(robot["leg_pitch_limits"], 2), lift_limits = values(robot["lift_limits"], 2);
  r.joint_min = joint_limits[0]; r.joint_max = joint_limits[1];
  r.lift_min = lift_limits[0]; r.lift_max = lift_limits[1];
  r.wheel_radius = scalar(robot["wheel_radius"]);
  r.wheel_width = scalar(robot["wheel_width"]);
  r.wheel_track = scalar(robot["wheel_track"]);
  r.carrier_drop = scalar(robot["carrier_drop"]);
  return request;
}

double length(const std::vector<Pose>& path, bool spatial)
{
  double result = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double xy = std::hypot(path[i].x-path[i-1].x, path[i].y-path[i-1].y);
    result += spatial ? std::hypot(xy, path[i].z-path[i-1].z) : xy;
  }
  return result;
}

std::string csvString(const std::string& value)
{
  std::string result = "\"";
  for (char c : value) {
    if (c == '"') result += '"';
    result += c;
  }
  return result+'"';
}

std::string defaultOutputRoot()
{
  if (const char* state = std::getenv("XDG_STATE_HOME")) {
    if (std::filesystem::path(state).is_absolute()) return (std::filesystem::path(state)/"simulation/global_planning").string();
  }
  if (const char* home = std::getenv("HOME")) {
    return (std::filesystem::path(home)/".local/state/simulation/global_planning").string();
  }
  return (std::filesystem::temp_directory_path()/"simulation/global_planning").string();
}

class PlanningNode final : public rclcpp::Node
{
public:
  PlanningNode() : Node("planning_node")
  {
    scene_file_ = declare_parameter<std::string>("scene_file", "");
    resolution_ = declare_parameter<double>("resolution", 0.05);
    margin_ = declare_parameter<double>("safety_margin", 0.05);
    max_cells_ = declare_parameter<int64_t>("max_grid_cells", 2000000);
    local_enabled_ = declare_parameter<bool>("local_enabled", false);
    teb_options_.max_v = declare_parameter<double>("teb.max_v", .30);
    teb_options_.max_w = declare_parameter<double>("teb.max_w", .80);
    teb_options_.max_a = declare_parameter<double>("teb.max_a", .30);
    teb_options_.max_alpha = declare_parameter<double>("teb.max_alpha", 1.0);
    teb_options_.time_horizon = declare_parameter<double>("teb.time_horizon", 2.2);
    teb_options_.spacing = declare_parameter<double>("teb.spacing", .15);
    teb_options_.iterations = declare_parameter<int>("teb.iterations", 30);
    teb_options_.solver_time_limit = declare_parameter<double>("teb.solver_time_limit", .10);
    teb_options_.max_lateral_velocity = declare_parameter<double>("teb.max_lateral_velocity", .01);
    teb_options_.collision_translation_step = declare_parameter<double>("teb.collision_translation_step", .01);
    teb_options_.collision_rotation_step = declare_parameter<double>("teb.collision_rotation_step", .02);
    local_margin_ = declare_parameter<double>("teb.safety_margin", .05);
    foot_enabled_ = declare_parameter<bool>("foot_enabled", false);
    foot_options_.resolution = declare_parameter<double>("foot.resolution", .05);
    foot_options_.max_step = declare_parameter<double>("foot.max_step", .50);
    foot_options_.edge_margin = declare_parameter<double>("foot.edge_margin", .025);
    foot_options_.swing_clearance = declare_parameter<double>("foot.swing_clearance", .08);
    foot_options_.step_duration = declare_parameter<double>("foot.step_duration", 9.0);
    foot_options_.sample_dt = declare_parameter<double>("foot.sample_dt", .02);
    foot_options_.max_foot_v = declare_parameter<double>("foot.max_foot_v", .8);
    foot_options_.max_foot_a = declare_parameter<double>("foot.max_foot_a", 3.0);
    foot_options_.max_body_v = declare_parameter<double>("foot.max_body_v", .3);
    foot_options_.max_body_a = declare_parameter<double>("foot.max_body_a", 1.0);
    foot_options_.max_joint_v = declare_parameter<double>("foot.max_joint_v", 1.0);
    foot_options_.max_joint_a = declare_parameter<double>("foot.max_joint_a", 20.0);
    foot_options_.body_height = declare_parameter<double>("foot.body_height", .43);
    foot_options_.wheel_lift = declare_parameter<double>("foot.wheel_lift", .13);
    foot_options_.max_expansions = declare_parameter<int>("foot.max_expansions", 10000);
    const auto root = declare_parameter<std::string>("output_dir", defaultOutputRoot());
    require(!root.empty(), "Empty planning output_dir");
    const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
    output_dir_ = std::filesystem::path(root)/("run_"+std::to_string(stamp));
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/planning/global_path", qos);
    status_publisher_ = create_publisher<std_msgs::msg::String>("/planning/status", qos);
    if (foot_enabled_) {
      footstep_publisher_ = create_publisher<nav_msgs::msg::Path>("/planning/footstep_path", qos);
      foot_trajectory_publisher_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/planning/foot_trajectory", qos);
      foot_status_publisher_ = create_publisher<std_msgs::msg::String>("/planning/foot_status", qos);
    }
    if (local_enabled_) {
      const auto local_qos = rclcpp::QoS(2).reliable().transient_local();
      local_path_publisher_ = create_publisher<nav_msgs::msg::Path>("/planning/local_path", local_qos);
      local_status_publisher_ = create_publisher<std_msgs::msg::String>("/planning/local_status", local_qos);
    }
    interactive_ = declare_parameter<bool>("interactive", false);
    setupTask();
    require(!(interactive_ && autonomous_), "interactive and autonomous are mutually exclusive");
    if (interactive_) {
      plan_service_ = create_service<std_srvs::srv::Trigger>("/planning/plan", [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (task_ready_ || terminal_) {response->success=false;response->message="Reset the robot before planning another task";return;}
        try {run();response->success=plan_ready_;response->message=plan_ready_?"PLAN_READY: A -> E -> X -> B":"Planning failed; inspect planning status";}
        catch(const std::exception& e){response->success=false;response->message=e.what();}
      });
      execute_service_ = create_service<std_srvs::srv::Trigger>("/planning/execute", [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (!plan_ready_ || task_ready_ || terminal_) {response->success=false;response->message="Plan first; executed/stopped tasks require resetting the robot";return;}
        if (!have_odom_ || !have_status_ || !have_feet_ || !at(task_scene_.lower.start,wheelHeight(),.01) || !stopped() || executor_.find("READY_WHEEL")!=0) {
          response->success=false;response->message="Robot must be ready and stopped at preset A";return;
        }
        try {
          beginTask();
          // Manual requests can arrive at any wall-timer phase. Start the first
          // control update now, then schedule full periods from this request.
          task_timer_->reset();taskTick();
          response->success=true;response->message="Task accepted";
        } catch(const std::exception& e){
          if(task_ready_ && !terminal_)finishTask("FAILED",e.what());
          response->success=false;response->message=e.what();
        }
      });
      stop_service_ = create_service<std_srvs::srv::Trigger>("/planning/stop", [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
        if(task_ready_ && !terminal_)finishTask("STOPPED","Stopped by operator; reset robot before another task");
        else {velocity(0,0);modeCommand("STOP");}
        plan_ready_=false;terminal_=true;response->success=true;response->message="Stop sent; reset robot before another task";
      });
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (get_parameter("use_sim_time").as_bool() && now().nanoseconds() == 0) return;
      timer_->cancel();
      if (!interactive_) run();
    });
  }

  ~PlanningNode() override
  {
    if (visual_timer_) visual_timer_->cancel();
    clearLocalVisuals();
    clearVisualGroup(foot_visuals_);
    clearPathVisuals();
    visual_publisher_.reset();
    delete_publisher_.reset();
    if (visual_node_) visual_node_->Fini();
    visual_node_.reset();
    if (gazebo_started_) gazebo::client::shutdown();
  }

private:
  using Sections = std::array<std::vector<Pose>, 3>;

  void clearVisualGroup(std::vector<gazebo::msgs::Visual>& visuals)
  {
    if (delete_publisher_ && delete_publisher_->HasConnections()) {
      for (const auto& visual : visuals) {
        gazebo::msgs::Request removal;
        removal.set_id(0);
        removal.set_request("entity_delete");
        removal.set_data(visual.name());
        delete_publisher_->Publish(removal);
      }
    }
    visuals.clear();
  }

  void clearLocalVisuals()
  {
    for (auto& group : local_visuals_) clearVisualGroup(group);
    last_local_draw_ = {};
  }

  void clearPathVisuals()
  {
    clearVisualGroup(path_visuals_);
    if (delete_publisher_ && delete_publisher_->HasConnections()) {
      // Drain deletions on shutdown, including local paths and foot markers.
      const auto deadline = std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
      while (delete_publisher_->GetOutgoingCount() > 0 && std::chrono::steady_clock::now() < deadline) {
        delete_publisher_->SendMessage();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

  void publishVisualGroup(const std::vector<gazebo::msgs::Visual>& visuals)
  {
    if (!visual_publisher_ || !visual_publisher_->HasConnections()) return;
    for (const auto& visual : visuals) visual_publisher_->Publish(visual);
  }

  void setLocalVisuals(const nav_msgs::msg::Path& path, std::size_t group, bool force = false)
  {
    if (!visual_publisher_) return;  // Standalone ROS planning needs no Gazebo master.
    const auto wall = std::chrono::steady_clock::now();
    if (!force && last_local_draw_.time_since_epoch().count() != 0 &&
        wall-last_local_draw_ < std::chrono::milliseconds(400)) return;
    last_local_draw_ = wall;
    auto& visuals = local_visuals_.at(group);
    clearVisualGroup(visuals);
    ++local_draw_generation_;
    std::vector<gazebo::msgs::Visual> next;
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto& a = path.poses[i-1].pose.position;
      const auto& b = path.poses[i].pose.position;
      const double dx = b.x-a.x, dy = b.y-a.y;
      const double distance = std::hypot(dx, dy);
      if (distance < kTolerance) continue;
      gazebo::msgs::Visual visual;
      // Distinct generations prevent delayed deletions from removing a newer band.
      visual.set_name("local_teb_"+output_dir_.filename().string()+"_"+
                      std::to_string(group)+"_"+std::to_string(local_draw_generation_)+"_"+std::to_string(next.size()));
      visual.set_parent_name(visual_world_);
      visual.set_is_static(true);
      visual.set_cast_shadows(false);
      gazebo::msgs::Set(visual.mutable_pose(), ignition::math::Pose3d(
        (a.x+b.x)/2, (a.y+b.y)/2, (a.z+b.z)/2+.065,
        0, 0, std::atan2(dy, dx)));
      visual.mutable_geometry()->set_type(gazebo::msgs::Geometry::BOX);
      gazebo::msgs::Set(visual.mutable_geometry()->mutable_box()->mutable_size(),
                        ignition::math::Vector3d(distance, .035, .015));
      const ignition::math::Color color(.1, 1, .25, 1);
      gazebo::msgs::Set(visual.mutable_material()->mutable_ambient(), color);
      gazebo::msgs::Set(visual.mutable_material()->mutable_diffuse(), color);
      next.push_back(std::move(visual));
    }
    visuals = std::move(next);
    publishVisualGroup(visuals);
  }

  void setFootVisuals(const planning::FootLocalPlanResult& result)
  {
    if (!visual_publisher_) return;
    clearVisualGroup(foot_visuals_);
    if (!result.success) return;
    const auto size = values(YAML::LoadFile(scene_file_)["robot"]["foot_size"], 3);
    for (std::size_t i = 0; i < result.footsteps.size(); ++i) {
      const auto& step = result.footsteps[i];
      gazebo::msgs::Visual visual;
      visual.set_name("planned_footstep_"+output_dir_.filename().string()+"_"+std::to_string(i));
      visual.set_parent_name(visual_world_);
      visual.set_is_static(true);
      visual.set_cast_shadows(false);
      // Planner poses are collision centres; draw the sole just above its support surface.
      gazebo::msgs::Set(visual.mutable_pose(), ignition::math::Pose3d(
        step.to.x, step.to.y, step.to.z-size[2]/2+.012, 0, 0, step.to.yaw));
      visual.mutable_geometry()->set_type(gazebo::msgs::Geometry::BOX);
      gazebo::msgs::Set(visual.mutable_geometry()->mutable_box()->mutable_size(),
                        ignition::math::Vector3d(size[0], size[1], .008));
      const auto color = step.side == 0 ? ignition::math::Color(.05, .8, 1, 1)
                                        : ignition::math::Color(1, .45, .05, 1);
      gazebo::msgs::Set(visual.mutable_material()->mutable_ambient(), color);
      gazebo::msgs::Set(visual.mutable_material()->mutable_diffuse(), color);
      // Visual transport creates no physical model or collision geometry.
      foot_visuals_.push_back(std::move(visual));
    }
    publishVisualGroup(foot_visuals_);
  }

  void publishPathVisuals()
  {
    if (!visual_publisher_ || !visual_publisher_->HasConnections()) return;
    for (const auto& group : local_visuals_) publishVisualGroup(group);
    publishVisualGroup(foot_visuals_);
    if (path_visuals_.empty()) return;
    // Stable names update existing visuals. Replaying the whole path also lets
    // a late/reconnected GUI receive every segment, not just the last message.
    for (const auto& visual : path_visuals_) visual_publisher_->Publish(visual);
    if (!visuals_announced_) {
      RCLCPP_INFO(get_logger(), "Gazebo path visible: %zu segments (blue=wheel, orange=stair reference)",
                  path_visuals_.size());
      visuals_announced_ = true;
    }
  }

  void setPathVisuals(const Sections& sections, const std::string& world)
  {
    visual_world_ = world;
    if (!visual_publisher_) {
      require(gazebo::client::setup(), "Could not start Gazebo visualization transport");
      gazebo_started_ = true;
      visual_node_.reset(new gazebo::transport::Node());
      visual_node_->Init(world);
      visual_publisher_ = visual_node_->Advertise<gazebo::msgs::Visual>("~/visual", 1000);
      delete_publisher_ = visual_node_->Advertise<gazebo::msgs::Request>("~/request", 1000);
      visual_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publishPathVisuals(); });
    }
    clearPathVisuals();
    visuals_announced_ = false;
    for (std::size_t s = 0; s < sections.size(); ++s) {
      for (std::size_t i = 1; i < sections[s].size(); ++i) {
        const auto& a = sections[s][i-1];
        const auto& b = sections[s][i];
        const double dx = b.x-a.x, dy = b.y-a.y, dz = b.z-a.z;
        const double horizontal = std::hypot(dx, dy), spatial = std::hypot(horizontal, dz);
        if (spatial < kTolerance) continue;  // Keep vertical stair segments.
        gazebo::msgs::Visual visual;
        visual.set_name("planning_path_"+output_dir_.filename().string()+"_"+
                        std::to_string(s)+"_"+std::to_string(i));
        visual.set_parent_name(world);
        visual.set_is_static(true);
        visual.set_cast_shadows(false);
        gazebo::msgs::Set(visual.mutable_pose(), ignition::math::Pose3d(
          (a.x+b.x)/2, (a.y+b.y)/2, (a.z+b.z)/2+0.04,
          0, -std::atan2(dz, horizontal), std::atan2(dy, dx)));
        visual.mutable_geometry()->set_type(gazebo::msgs::Geometry::BOX);
        gazebo::msgs::Set(visual.mutable_geometry()->mutable_box()->mutable_size(),
                          ignition::math::Vector3d(spatial, 0.045, 0.015));
        const auto color = s == 1 ? ignition::math::Color(1, .45, .05, 1)
                                  : ignition::math::Color(.05, .35, 1, 1);
        gazebo::msgs::Set(visual.mutable_material()->mutable_ambient(), color);
        gazebo::msgs::Set(visual.mutable_material()->mutable_diffuse(), color);
        // Visual messages only: no model/collision is inserted into the world.
        path_visuals_.push_back(std::move(visual));
      }
    }
    publishPathVisuals();
  }

  void save(const Sections& sections, bool success, const std::string& reason, double ms)
  {
    std::filesystem::create_directories(output_dir_);
    std::ofstream path;
    path.exceptions(std::ios::failbit | std::ios::badbit);
    path.open(output_dir_/"path.csv");
    path << std::setprecision(17) << "section,mode,index,x_m,y_m,ground_z_m,yaw_rad\n";
    std::size_t count = 0;
    double horizontal = 0, spatial = 0;
    for (std::size_t s = 0; s < sections.size(); ++s) {
      horizontal += length(sections[s], false);
      spatial += length(sections[s], true);
      for (std::size_t i = 0; i < sections[s].size(); ++i) {
        const auto& p = sections[s][i];
        path << s << ',' << (s == 1 ? "stair_reference" : "wheel") << ',' << i << ','
             << p.x << ',' << p.y << ',' << p.z << ',' << p.yaw << '\n';
        ++count;
      }
    }
    path.close();
    std::ofstream result;
    result.exceptions(std::ios::failbit | std::ios::badbit);
    result.open(output_dir_/"result.csv");
    result << std::setprecision(17)
           << "success,reason,point_count,horizontal_length_m,spatial_length_m,planning_ms,"
              "resolution_m,safety_margin_m,lower_length_m,upper_length_m,stair_reference_only\n"
           << success << ',' << csvString(reason) << ',' << count << ',' << horizontal << ','
           << spatial << ',' << ms << ',' << resolution_ << ',' << margin_ << ','
           << length(sections[0], false) << ',' << length(sections[2], false) << ",1\n";
    result.close();
  }

  void run()
  {
    plan_ready_=false;local_ok_=false;foot_command_.points.clear();
    const auto begin = std::chrono::steady_clock::now();
    Sections sections;
    bool success = false;
    std::string reason;
    std::string world;
    Scene scene;
    try {
      require(max_cells_ > 0, "Invalid max_grid_cells");
      scene = loadScene(scene_file_, margin_);
      world = scene.world_name;
      frame_ = scene.lower.frame_id;
      RCLCPP_INFO(get_logger(), "Scene %s: %zu obstacle/wall boxes; A=(%.3f,%.3f), E=(%.3f,%.3f), X=(%.3f,%.3f), B=(%.3f,%.3f)",
                  scene_file_.c_str(), scene.lower.obstacles.size(), scene.lower.start.x, scene.lower.start.y,
                  scene.lower.goal.x, scene.lower.goal.y, scene.upper.start.x, scene.upper.start.y,
                  scene.upper.goal.x, scene.upper.goal.y);
      planning::AStar astar({resolution_, static_cast<std::size_t>(max_cells_)});
      planning::GlobalPlanner& planner = astar;
      const auto lower = planner.plan(scene.lower);
      require(lower.success, "A->E: "+lower.failure_reason);
      const auto upper = planner.plan(scene.upper);
      require(upper.success, "X->B: "+upper.failure_reason);
      sections = {lower.path, stairReference(scene), upper.path};
      success = true;
      reason = "OK";
    } catch (const std::exception& error) {
      reason = error.what();
      sections = {};
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-begin).count();
    try {
      save(sections, success, reason, ms);
    } catch (const std::exception& error) {
      success = false;
      sections = {};
      reason += std::string("; CSV_ERROR: ")+error.what();
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_;
    path.header.stamp = now();
    for (const auto& section : sections) for (const auto& p : section) {
      if (!path.poses.empty()) {
        const auto& last = path.poses.back().pose.position;
        if (std::abs(last.x-p.x) < kTolerance && std::abs(last.y-p.y) < kTolerance &&
            std::abs(last.z-p.z) < kTolerance) continue;
      }
      geometry_msgs::msg::PoseStamped point;
      point.header = path.header;
      point.pose.position.x = p.x;
      point.pose.position.y = p.y;
      point.pose.position.z = p.z;
      point.pose.orientation.z = std::sin(p.yaw/2);
      point.pose.orientation.w = std::cos(p.yaw/2);
      path.poses.push_back(point);
    }
    path_publisher_->publish(path);
    std_msgs::msg::String status;
    status.data = std::string(success ? "SUCCESS: " : "FAILED: ")+reason+
      "; stair=geometric_reference_only; output="+output_dir_.string();
    status_publisher_->publish(status);
    if (success) {
      RCLCPP_INFO(get_logger(), "%s; points=%zu; horizontal_length=%.3f m; planning=%.3f ms",
                  status.data.c_str(), path.poses.size(), length(sections[0], false)+
                  length(sections[1], false)+length(sections[2], false), ms);
    } else {
      RCLCPP_ERROR(get_logger(), "%s", status.data.c_str());
    }
    // Unified sim launch uses simulation time. Standalone planning invocations
    // keep their existing ROS/CSV behavior without needing a Gazebo master.
    try {
      if (success && get_parameter("use_sim_time").as_bool()) setPathVisuals(sections, world);
      else clearPathVisuals();
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(), "Path visualization unavailable: %s", error.what());
    }
    if (local_enabled_) runLocal(scene, sections, success, reason);
    if (foot_enabled_) runFoot(scene, sections, success, reason);
    task_scene_ = scene; task_sections_ = sections;
    plan_ready_ = success && local_enabled_ && foot_enabled_ && local_ok_ && !foot_command_.points.empty();
    if (autonomous_) beginTask();
  }

  void beginTask()
  {
    std::filesystem::create_directories(output_dir_);
    actual_.exceptions(std::ios::failbit | std::ios::badbit);
    events_.exceptions(std::ios::failbit | std::ios::badbit);
    bands_.exceptions(std::ios::failbit | std::ios::badbit);
    actual_.open(output_dir_/"actual_trajectory.csv"); events_.open(output_dir_/"task_events.csv");
    bands_.open(output_dir_/"teb_execution.csv");
    actual_ << "sim_time_s,state,x_m,y_m,body_z_m,yaw_rad,v_mps,w_rps,left_x_m,left_y_m,left_z_m,right_x_m,right_y_m,right_z_m,executor_status\n";
    events_ << "sim_time_s,state,reason,x_m,y_m,body_z_m,yaw_rad,v_mps,w_rps\n";
    bands_ << "plan_id,segment,odom_stamp_s,initial_x_m,initial_y_m,initial_yaw_rad,initial_v_mps,initial_w_rps,index,time_s,x_m,y_m,yaw_rad,v_mps,w_rps,planning_ms,seed_spacing_m\n";
    task_begin_=phase_begin_=now().seconds(); wall_begin_=std::chrono::steady_clock::now();
    task_ready_=true;
    if (!plan_ready_) finishTask("FAILED", "GLOBAL_OR_FOOT_PLAN_UNAVAILABLE_OR_PLANNERS_DISABLED");
    else enterTask("WAIT_EXECUTOR");
  }

  void saveLocal(const std::array<planning::LocalPlanResult, 2>& results)
  {
    std::filesystem::create_directories(output_dir_);
    std::ofstream trajectory, summary;
    trajectory.exceptions(std::ios::failbit | std::ios::badbit);
    summary.exceptions(std::ios::failbit | std::ios::badbit);
    trajectory.open(output_dir_/"local_trajectory.csv");
    summary.open(output_dir_/"local_result.csv");
    trajectory << std::setprecision(17)
      << "segment,index,time_from_start_s,x_m,y_m,ground_z_m,yaw_rad,linear_velocity_mps,"
         "angular_velocity_rps,linear_acceleration_mps2,angular_acceleration_rps2\n";
    summary << std::setprecision(17)
      << "segment,success,reason,static_prediction,initial_v_mps,initial_w_rps,requested_horizon_s,"
         "duration_s,point_count,path_length_m,planning_ms,optimization_cost,candidates_evaluated,"
         "max_v_mps,max_w_rps,max_a_mps2,max_alpha_rps2,max_lateral_velocity_mps\n";
    for (std::size_t s = 0; s < results.size(); ++s) {
      const auto& result = results[s];
      const char* segment = s == 0 ? "A-E" : "X-B";
      for (std::size_t i = 0; i < result.trajectory.size(); ++i) {
        const auto& point = result.trajectory[i]; const auto& p = point.pose;
        trajectory << segment << ',' << i << ',' << point.time_from_start << ',' << p.x << ',' << p.y
                   << ',' << p.z << ',' << p.yaw << ',' << point.linear_velocity << ','
                   << point.angular_velocity << ',' << point.linear_acceleration << ','
                   << point.angular_acceleration << '\n';
      }
      summary << segment << ',' << result.success << ',' << csvString(result.failure_reason)
              << ",1,0,0," << teb_options_.time_horizon << ',' << result.duration << ','
              << result.trajectory.size() << ',' << result.path_length << ',' << result.planning_ms << ','
              << result.optimization_cost << ',' << result.candidates_evaluated << ','
              << result.max_linear_velocity << ',' << result.max_angular_velocity << ','
              << result.max_linear_acceleration << ',' << result.max_angular_acceleration << ','
              << result.max_lateral_velocity << '\n';
    }
    trajectory.close(); summary.close();
  }

  void runLocal(const Scene& scene, const Sections& sections, bool global_ok, const std::string& global_reason)
  {
    std::array<planning::LocalPlanResult, 2> results;
    for (std::size_t s = 0; s < results.size(); ++s) {
      try {
        require(global_ok, "GLOBAL_REFERENCE_UNAVAILABLE: "+global_reason);
        planning::LocalPlanRequest request;
        request.geometry = s == 0 ? scene.lower : scene.upper;
        request.geometry.safety_margin = local_margin_;
        // Keep the existing mode sections; never pass sections[1] (stairs).
        request.reference_path = sections[s == 0 ? 0 : 2];
        request.initial_linear_velocity = 0;
        request.initial_angular_velocity = 0;
        planning::TebOptimizer teb(teb_options_);
        planning::WheelLocalPlanner& optimizer = teb;
        results[s] = optimizer.plan(request);
      } catch (const std::exception& error) {
        results[s].failure_reason = error.what();
      }
    }
    try {
      saveLocal(results);
    } catch (const std::exception& error) {
      for (auto& result : results) {
        result = {}; result.failure_reason = std::string("LOCAL_CSV_ERROR: ")+error.what();
      }
    }
    local_ok_=results[0].success && results[1].success;
    for (std::size_t s = 0; s < results.size(); ++s) {
      const auto& result = results[s];
      const char* segment = s == 0 ? "A-E" : "X-B";
      nav_msgs::msg::Path path;
      path.header.frame_id = frame_;
      // Status carries this exact stamp as the segment identifier. Keep both
      // samples with depth=2; never concatenate two disconnected local bands.
      path.header.stamp = now();
      if (s > 0 && rclcpp::Time(path.header.stamp) <= last_local_stamp_) {
        path.header.stamp = last_local_stamp_+rclcpp::Duration(0,1);
      }
      last_local_stamp_ = rclcpp::Time(path.header.stamp);
      for (const auto& sample : result.trajectory) {
        geometry_msgs::msg::PoseStamped point;
        point.header = path.header;
        point.header.stamp = last_local_stamp_+rclcpp::Duration::from_seconds(sample.time_from_start);
        point.pose.position.x = sample.pose.x; point.pose.position.y = sample.pose.y;
        point.pose.position.z = sample.pose.z;
        point.pose.orientation.z = std::sin(sample.pose.yaw/2);
        point.pose.orientation.w = std::cos(sample.pose.yaw/2);
        path.poses.push_back(point);
      }
      local_path_publisher_->publish(path);
      if (!autonomous_) setLocalVisuals(path, s, true);
      std_msgs::msg::String status;
      status.data = std::string(result.success ? "SUCCESS" : "FAILED")+"; segment="+segment+
        "; path_stamp="+std::to_string(path.header.stamp.sec)+":"+std::to_string(path.header.stamp.nanosec)+
        "; state=static_initial_zero_velocity; execution=none; requested_horizon_s="+
        std::to_string(teb_options_.time_horizon)+"; duration_s="+std::to_string(result.duration)+
        "; reason="+(result.success ? "OK" : result.failure_reason)+"; output="+output_dir_.string();
      local_status_publisher_->publish(status);
      if (result.success) RCLCPP_INFO(get_logger(), "TEB %s; points=%zu; predicted_length=%.3f m", status.data.c_str(), path.poses.size(), result.path_length);
      else RCLCPP_ERROR(get_logger(), "TEB %s", status.data.c_str());
    }
  }

  void saveFoot(const planning::FootLocalPlanResult& result)
  {
    std::filesystem::create_directories(output_dir_);
    std::ofstream steps, trajectory;
    steps.exceptions(std::ios::failbit | std::ios::badbit);
    trajectory.exceptions(std::ios::failbit | std::ios::badbit);
    steps.open(output_dir_/"footstep_path.csv");
    trajectory.open(output_dir_/"foot_trajectory.csv");
    steps << std::setprecision(17)
      << "step,side,from_x_m,from_y_m,from_z_m,foot_center_x_m,foot_center_y_m,foot_center_z_m,edge_margin_m,touchdown_time_s\n";
    for (std::size_t i = 0; i < result.footsteps.size(); ++i) {
      const auto& s = result.footsteps[i];
      steps << i+1 << ',' << (s.side == 0 ? "left" : "right") << ',' << s.from.x << ',' << s.from.y
            << ',' << s.from.z << ',' << s.to.x << ',' << s.to.y << ',' << s.to.z << ','
            << s.edge_margin << ',' << s.touchdown_time << '\n';
    }
    trajectory << std::setprecision(17)
      << "time_from_start_s,body_x_m,body_y_m,body_z_m,left_x_m,left_y_m,left_z_m,right_x_m,right_y_m,right_z_m";
    for (const char* derivative : {"velocity", "acceleration"}) {
      for (const char* axis : {"body_x", "body_y", "body_z", "left_x", "left_z", "right_x", "right_z"}) {
        trajectory << ',' << axis << '_' << derivative << (std::string(derivative) == "velocity" ? "_mps" : "_mps2");
      }
    }
    trajectory << '\n';
    for (const auto& s : result.trajectory) {
      trajectory << s.time_from_start << ',' << s.body.x << ',' << s.body.y << ',' << s.body.z;
      for (const auto& foot : s.feet) trajectory << ',' << foot.x << ',' << foot.y << ',' << foot.z;
      for (double v : s.velocity) trajectory << ',' << v;
      for (double a : s.acceleration) trajectory << ',' << a;
      trajectory << '\n';
    }
    steps.close(); trajectory.close();
  }

  void runFoot(const Scene& scene, const Sections& sections, bool global_ok, const std::string& global_reason)
  {
    planning::FootLocalPlanResult result;
    try {
      require(global_ok, "GLOBAL_REFERENCE_UNAVAILABLE: "+global_reason);
      const auto request = footRequest(scene_file_, scene, sections[1]);
      planning::FootstepPlanner implementation(foot_options_);
      planning::FootLocalPlanner& planner = implementation;
      result = planner.plan(request);
    } catch (const std::exception& error) {
      result.failure_reason = error.what();
    }
    try { saveFoot(result); }
    catch (const std::exception& error) {
      result = {}; result.failure_reason = std::string("FOOT_CSV_ERROR: ")+error.what();
    }
    nav_msgs::msg::Path footsteps;
    footsteps.header.frame_id = frame_;
    footsteps.header.stamp = now();
    const rclcpp::Time stamp(footsteps.header.stamp);
    for (const auto& step : result.footsteps) {
      geometry_msgs::msg::PoseStamped p;
      p.header = footsteps.header;
      p.header.stamp = stamp+rclcpp::Duration::from_seconds(step.touchdown_time);
      p.pose.position.x = step.to.x; p.pose.position.y = step.to.y; p.pose.position.z = step.to.z;
      p.pose.orientation.w = 1;
      footsteps.poses.push_back(p);
    }
    trajectory_msgs::msg::MultiDOFJointTrajectory trajectory;
    trajectory.header = footsteps.header;
    trajectory.joint_names = {"body", "left_foot_center", "right_foot_center"};
    for (const auto& sample : result.trajectory) {
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
      point.time_from_start = rclcpp::Duration::from_seconds(sample.time_from_start);
      const std::array<Pose, 3> poses{sample.body, sample.feet[0], sample.feet[1]};
      for (std::size_t i = 0; i < poses.size(); ++i) {
        geometry_msgs::msg::Transform transform;
        transform.translation.x = poses[i].x; transform.translation.y = poses[i].y; transform.translation.z = poses[i].z;
        transform.rotation.w = 1;
        point.transforms.push_back(transform);
        geometry_msgs::msg::Twist v, a;
        const auto axis = i == 0 ? 0 : 3+2*(i-1);
        v.linear.x = sample.velocity[axis]; a.linear.x = sample.acceleration[axis];
        v.linear.z = sample.velocity[i == 0 ? 2 : axis+1];
        a.linear.z = sample.acceleration[i == 0 ? 2 : axis+1];
        if (i == 0) { v.linear.y = sample.velocity[1]; a.linear.y = sample.acceleration[1]; }
        point.velocities.push_back(v); point.accelerations.push_back(a);
      }
      trajectory.points.push_back(point);
    }
    footstep_publisher_->publish(footsteps);
    foot_trajectory_publisher_->publish(trajectory);
    if (result.success) foot_command_ = trajectory;
    try { setFootVisuals(result); }
    catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(), "Foot visualization unavailable: %s", error.what());
    }
    std_msgs::msg::String status;
    const double duration = result.trajectory.empty() ? 0 : result.trajectory.back().time_from_start;
    status.data = std::string(result.success ? "SUCCESS" : "FAILED")+
      "; segment=E-X; state=static_leg_posture; execution=none; model=geometry_kinematics_only; first_side=left"+
      "; steps="+std::to_string(result.footsteps.size())+"; samples="+std::to_string(result.trajectory.size())+
      "; duration_s="+std::to_string(duration)+"; expanded="+std::to_string(result.expanded)+
      "; planning_ms="+std::to_string(result.planning_ms)+"; reason="+(result.success ? "OK" : result.failure_reason)+
      "; output="+output_dir_.string();
    foot_status_publisher_->publish(status);
    if (result.success) RCLCPP_INFO(get_logger(), "Foot %s", status.data.c_str());
    else RCLCPP_ERROR(get_logger(), "Foot %s", status.data.c_str());
  }

  // Automatic task scheduling stays in this node; planners retain their existing APIs.
  void setupTask()
  {
    autonomous_ = declare_parameter<bool>("autonomous", false);
    feedback_timeout_ = declare_parameter<double>("task.feedback_timeout", 1.0);
    command_timeout_ = declare_parameter<double>("task.command_timeout", .8);
    stage_timeout_ = declare_parameter<double>("task.stage_timeout", 240.0);
    task_timeout_ = declare_parameter<double>("task.timeout", 600.0);
    position_tolerance_ = declare_parameter<double>("task.position_tolerance", .002);
    yaw_tolerance_ = declare_parameter<double>("task.yaw_tolerance", .0005);
    approach_gain_ = declare_parameter<double>("task.approach_gain", .5);
    declare_parameter<double>("task.morph_duration", 4.0);
    if (!autonomous_ && !interactive_) return;
    require(feedback_timeout_>0 && command_timeout_>0 && stage_timeout_>0 && task_timeout_>0 &&
            position_tolerance_>0 && yaw_tolerance_>0 && approach_gain_>0, "Invalid task execution parameters");
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/robot/cmd_vel", 1);
    mode_pub_ = create_publisher<std_msgs::msg::String>("/robot/mode_command", 10);
    execution_pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/robot/foot_trajectory", rclcpp::QoS(1).reliable());
    task_pub_ = create_publisher<std_msgs::msg::String>("/planning/task_status", rclcpp::QoS(1).transient_local());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/robot/odom",1,[this](nav_msgs::msg::Odometry::SharedPtr msg) {
      if(msg->header.frame_id!=frame_ || msg->child_frame_id!="base_link")return;
      odom_=*msg; odom_stamp_=rclcpp::Time(msg->header.stamp).seconds();
      const auto& p=msg->pose.pose;
      actual_pose_={p.position.x,p.position.y,p.position.z,2*std::atan2(p.orientation.z,p.orientation.w)};
      wall_odom_=std::chrono::steady_clock::now();have_odom_=true;
    });
    executor_sub_ = create_subscription<std_msgs::msg::String>("/robot/execution_status",10,[this](std_msgs::msg::String::SharedPtr msg) {
      executor_=msg->data; wall_status_=std::chrono::steady_clock::now(); have_status_=true;
    });
    feet_sub_ = create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>("/robot/foot_feedback",10,[this](trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg) {
      if(msg->header.frame_id!=frame_||msg->points.size()!=1||msg->points[0].transforms.size()!=3)return;
      feet_=*msg; feet_stamp_=rclcpp::Time(msg->header.stamp).seconds();wall_feet_=std::chrono::steady_clock::now();have_feet_=true;
      if(task_ready_ && !terminal_) {
        const auto& t=msg->points[0].transforms;
        actual_ << std::setprecision(17) << feet_stamp_ << ',' << task_state_ << ','
          << t[0].translation.x << ',' << t[0].translation.y << ',' << t[0].translation.z << ','
          << 2*std::atan2(t[0].rotation.z,t[0].rotation.w) << ',' << odom_.twist.twist.linear.x << ',' << odom_.twist.twist.angular.z;
        for(int i=1;i<3;i++)actual_ << ',' << t[i].translation.x << ',' << t[i].translation.y << ',' << t[i].translation.z;
        actual_ << ',' << csvString(executor_) << '\n';
      }
    });
    task_timer_ = create_wall_timer(std::chrono::milliseconds(100),[this](){
      if(!task_ready_||terminal_)return;
      try {taskTick();} catch(const std::exception& e){finishTask("FAILED",e.what());}
    });
  }
  static double angle(double x) { return std::atan2(std::sin(x),std::cos(x)); }
  bool stopped() const {return std::abs(odom_.twist.twist.linear.x)<.001 && std::abs(odom_.twist.twist.angular.z)<.001;}
  void modeCommand(const std::string& command) {std_msgs::msg::String msg;msg.data=command;mode_pub_->publish(msg);}
  void velocity(double v,double w) {
    geometry_msgs::msg::TwistStamped msg;msg.header.stamp=now();msg.header.frame_id="base_link";
    msg.twist.linear.x=v;msg.twist.angular.z=w;velocity_pub_->publish(msg);
  }
  void enterTask(const std::string& state,const std::string& reason="") {
    if (state != "WHEEL_A_E" && state != "WHEEL_X_B") clearLocalVisuals();
    task_state_=state;phase_begin_=now().seconds();
    if (state == "WHEEL_A_E" || state == "WHEEL_X_B") last_control_time_ = -1;
    events_ << std::setprecision(17) << phase_begin_ << ',' << state << ',' << csvString(reason) << ','
      << actual_pose_.x << ',' << actual_pose_.y << ',' << actual_pose_.z << ',' << actual_pose_.yaw << ','
      << odom_.twist.twist.linear.x << ',' << odom_.twist.twist.angular.z << '\n';events_.flush();
    std_msgs::msg::String msg;msg.data=state+"; reason="+reason+"; output="+output_dir_.string();task_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),"Task %s",msg.data.c_str());
  }
  void finishTask(const std::string& state,const std::string& reason) {
    velocity(0,0);modeCommand("STOP");enterTask(state,reason);terminal_=true;
    std::ofstream result(output_dir_/"task_result.csv");
    const auto& goal=task_scene_.upper.goal;
    result << std::setprecision(17) << "state,reason,simulation_duration_s,wall_duration_s,position_error_m,height_error_m,yaw_error_rad,final_v_mps,final_w_rps,teb_plans,final_x_m,final_y_m,final_body_z_m,executor_status\n"
      << state << ',' << csvString(reason) << ',' << now().seconds()-task_begin_ << ','
      << std::chrono::duration<double>(std::chrono::steady_clock::now()-wall_begin_).count() << ','
      << std::hypot(actual_pose_.x-goal.x,actual_pose_.y-goal.y) << ','
      << actual_pose_.z-goal.z-wheelHeight() << ',' << std::abs(angle(actual_pose_.yaw-goal.yaw)) << ','
      << odom_.twist.twist.linear.x << ',' << odom_.twist.twist.angular.z << ',' << plan_id_ << ','
      << actual_pose_.x << ',' << actual_pose_.y << ',' << actual_pose_.z << ',' << csvString(executor_) << '\n';
    actual_.flush();events_.flush();bands_.flush();
  }
  double wheelHeight() const {
    const auto robot=YAML::LoadFile(scene_file_)["robot"];
    return scalar(robot["wheel_radius"])+scalar(robot["carrier_drop"]);
  }
  bool at(const Pose& goal,double height,double tolerance) const {
    return std::hypot(actual_pose_.x-goal.x,actual_pose_.y-goal.y)<=tolerance &&
      std::abs(actual_pose_.z-goal.z-height)<.006 && std::abs(angle(actual_pose_.yaw-goal.yaw))<yaw_tolerance_;
  }
  void drive(bool upper) {
    const double control_time = now().seconds();
    const double control_dt = last_control_time_ < 0 ? .1 : std::clamp(control_time-last_control_time_, .02, .2);
    last_control_time_ = control_time;
    auto geometry=upper?task_scene_.upper:task_scene_.lower;
    const auto& goal=geometry.goal;
    const double distance=std::hypot(actual_pose_.x-goal.x,actual_pose_.y-goal.y);
    if(distance<=position_tolerance_) {
      const double error=angle(goal.yaw-actual_pose_.yaw);
      velocity(0,std::abs(error)<yaw_tolerance_?0:std::clamp(2*error,-teb_options_.max_w,teb_options_.max_w));
      if(stopped()&&at(goal,wheelHeight(),position_tolerance_)) {
        if(upper)finishTask("DONE","Reached B and stopped in wheel mode");
        else {enterTask("DEPLOY");modeCommand("LEG");}
      }
      return;
    }
    planning::LocalPlanRequest request;request.geometry=geometry;
    request.geometry.start={actual_pose_.x,actual_pose_.y,actual_pose_.z-wheelHeight(),actual_pose_.yaw};
    // Surface Z remains the configured plane; reject unexpected body height.
    require(std::abs(request.geometry.start.z-geometry.start.z)<.006,"WHEEL_GROUND_HEIGHT_MISMATCH");
    request.geometry.start.z=geometry.start.z;request.geometry.safety_margin=local_margin_;
    request.reference_path=task_sections_[upper?2:0];
    request.initial_linear_velocity=std::clamp(odom_.twist.twist.linear.x,0.,teb_options_.max_v);
    request.initial_angular_velocity=std::clamp(odom_.twist.twist.angular.z,-teb_options_.max_w,teb_options_.max_w);
    auto options=teb_options_;
    // Give TEB the same terminal speed envelope that the executor will track.
    // Keep its admissible initial speed equal to the measured odometry speed.
    options.max_v=std::min(teb_options_.max_v,std::max(request.initial_linear_velocity+1e-6,approach_gain_*distance));
    planning::LocalPlanResult result;
    // A coarse band can collapse an interior knot at a reference corner.
    // Refine the existing optimizer's discretization; accept only its fully
    // validated output, with the same footprint, clearance and motion bounds.
    const double base_spacing=options.spacing;
    for(double spacing:{base_spacing,base_spacing*2/3,base_spacing/3}) {
      options.spacing=spacing;
      options.solver_time_limit=std::min(teb_options_.solver_time_limit,command_timeout_/18);
      planning::TebOptimizer teb(options);result=teb.plan(request);
      if(result.success)break;
    }
    require(result.success && result.trajectory.size()>1,"TEB_EXECUTION_PLAN: "+result.failure_reason);
    // Interval velocities belong to interval midpoints, not the next 100 ms tick.
    // Follow the band's validated initial acceleration over one control period.
    // Jumping straight to its first mean velocity distorts timing and can make
    // the next docking band infeasible even though the current band is valid.
    const auto& first=result.trajectory[1];
    const double advance=std::min(control_dt, first.time_from_start/2);
    const double planned_v=std::max(0.,request.initial_linear_velocity+first.linear_acceleration*advance);
    const double planned_w=request.initial_angular_velocity+first.angular_acceleration*advance;
    const double target=std::min({planned_v,approach_gain_*distance,
      std::sqrt(2*teb_options_.max_a*std::max(0.,distance-position_tolerance_/2))});
    const double scale=planned_v>1e-8?std::clamp(target/planned_v,0.,1.):1.;
    velocity(std::clamp(target,0.,teb_options_.max_v),std::clamp(planned_w*scale,-teb_options_.max_w,teb_options_.max_w));
    nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=now();
    for(std::size_t i=0;i<result.trajectory.size();i++) {
      const auto& p=result.trajectory[i];geometry_msgs::msg::PoseStamped point;point.header=path.header;
      point.pose.position.x=p.pose.x;point.pose.position.y=p.pose.y;point.pose.position.z=p.pose.z;
      point.pose.orientation.z=std::sin(p.pose.yaw/2);point.pose.orientation.w=std::cos(p.pose.yaw/2);path.poses.push_back(point);
      bands_ << std::setprecision(17) << plan_id_ << ',' << (upper?"X-B":"A-E") << ',' << odom_stamp_ << ','
        << actual_pose_.x << ',' << actual_pose_.y << ',' << actual_pose_.yaw << ','
        << request.initial_linear_velocity << ',' << request.initial_angular_velocity << ',' << i << ','
        << p.time_from_start << ',' << p.pose.x << ',' << p.pose.y << ',' << p.pose.yaw << ','
        << p.linear_velocity << ',' << p.angular_velocity << ',' << result.planning_ms << ',' << options.spacing << '\n';
    }
    ++plan_id_;local_path_publisher_->publish(path);
    setLocalVisuals(path, 0);
    std_msgs::msg::String status;status.data="RUNNING; segment="+std::string(upper?"X-B":"A-E")+"; execution=closed_loop; plan_id="+std::to_string(plan_id_);local_status_publisher_->publish(status);
  }
  void taskTick() {
    const double t=now().seconds();const auto wall=std::chrono::steady_clock::now();
    auto age=[&](const auto& since){return std::chrono::duration<double>(wall-since).count();};
    if(task_state_=="WAIT_EXECUTOR") {
      if(!have_odom_||!have_status_||!have_feet_||velocity_pub_->get_subscription_count()==0||execution_pub_->get_subscription_count()==0||mode_pub_->get_subscription_count()==0) {
        require(t-phase_begin_<30 && age(wall_begin_)<60,"EXECUTOR_STARTUP_TIMEOUT");return;
      }
      require(at(task_scene_.lower.start,wheelHeight(),.01)&&stopped()&&executor_.find("READY_WHEEL")==0,"INITIAL_POSE_OR_MODE_MISMATCH");
      enterTask("WHEEL_A_E");
    }
    require(t-task_begin_<task_timeout_ && t-phase_begin_<stage_timeout_ && age(wall_begin_)<2*task_timeout_,"TASK_OR_STAGE_TIMEOUT");
    require(t-odom_stamp_>=-.1 && t-odom_stamp_<feedback_timeout_ && t-feet_stamp_>=-.1 && t-feet_stamp_<feedback_timeout_ &&
      age(wall_odom_)<feedback_timeout_&&age(wall_feet_)<feedback_timeout_&&age(wall_status_)<feedback_timeout_,"EXECUTION_FEEDBACK_TIMEOUT");
    require(executor_.find("FAILED")!=0,"EXECUTOR: "+executor_);
    modeCommand("HEARTBEAT");
    if(task_state_=="WHEEL_A_E"||task_state_=="WHEEL_X_B")drive(task_state_=="WHEEL_X_B");
    else if(task_state_=="DEPLOY"&&executor_.find("LEG_READY")==0) {
      require(at(task_scene_.lower.goal,foot_options_.body_height,.004),"LEG_ENTRY_POSE_MISMATCH");
      foot_command_.header.stamp=now();execution_pub_->publish(foot_command_);enterTask("FOOT_E_X");
    } else if(task_state_=="FOOT_E_X"&&executor_.find("FOOT_DONE")==0) {
      require(at(task_scene_.upper.start,foot_options_.body_height,.004),"FOOT_EXIT_POSE_MISMATCH");
      const auto& end=foot_command_.points.back();const auto& measured=feet_.points[0];
      // Independent DDS topics may deliver completion status before the final feet sample.
      if(rclcpp::Duration(measured.time_from_start).seconds()<rclcpp::Duration(end.time_from_start).seconds()-.001)return;
      for(int i=0;i<3;i++) {
        const auto& a=end.transforms[i].translation;const auto& b=measured.transforms[i].translation;
        require(std::hypot(a.x-b.x,a.y-b.y)<.003&&std::abs(a.z-b.z)<.003,"FOOT_ENDPOINT_FEEDBACK_MISMATCH");
      }
      require(rclcpp::Duration(measured.time_from_start).seconds()>=rclcpp::Duration(end.time_from_start).seconds()-.001,"FOOT_COMPLETION_FEEDBACK_MISSING");
      enterTask("FOLD");modeCommand("WHEEL");
    } else if(task_state_=="FOLD"&&executor_.find("WHEEL_READY")==0) {
      require(at(task_scene_.upper.start,wheelHeight(),.004)&&stopped(),"WHEEL_EXIT_POSE_MISMATCH");enterTask("WHEEL_X_B");
    }
  }
  bool interactive_{false},plan_ready_{false},local_ok_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_,execute_service_,stop_service_;
  bool autonomous_{false},task_ready_{false},terminal_{false},have_odom_{false},have_status_{false},have_feet_{false};
  std::string task_state_,executor_;
  Scene task_scene_;Sections task_sections_;
  Pose actual_pose_;
  double feedback_timeout_,command_timeout_,stage_timeout_,task_timeout_,position_tolerance_,yaw_tolerance_,approach_gain_;
  double task_begin_{0},phase_begin_{0},odom_stamp_{-100},feet_stamp_{-100},last_control_time_{-1};
  int plan_id_{0};
  std::chrono::steady_clock::time_point wall_begin_,wall_odom_,wall_feet_,wall_status_;
  std::ofstream actual_,events_,bands_;
  nav_msgs::msg::Odometry odom_;
  trajectory_msgs::msg::MultiDOFJointTrajectory foot_command_,feet_;
  rclcpp::TimerBase::SharedPtr task_timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_,task_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr execution_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr executor_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr feet_sub_;

  std::string scene_file_, frame_;
  double resolution_, margin_;
  int64_t max_cells_;
  std::filesystem::path output_dir_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  bool gazebo_started_{false};
  bool visuals_announced_{false};
  gazebo::transport::NodePtr visual_node_;
  gazebo::transport::PublisherPtr visual_publisher_;
  gazebo::transport::PublisherPtr delete_publisher_;
  std::vector<gazebo::msgs::Visual> path_visuals_;
  std::array<std::vector<gazebo::msgs::Visual>, 2> local_visuals_;
  std::vector<gazebo::msgs::Visual> foot_visuals_;
  std::string visual_world_;
  std::size_t local_draw_generation_{0};
  std::chrono::steady_clock::time_point last_local_draw_;
  rclcpp::TimerBase::SharedPtr visual_timer_;
  bool local_enabled_{false};
  double local_margin_{0.05};
  planning::TebOptions teb_options_;
  rclcpp::Time last_local_stamp_{0,0,RCL_ROS_TIME};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr local_status_publisher_;
  bool foot_enabled_{false};
  planning::FootstepOptions foot_options_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr footstep_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr foot_trajectory_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr foot_status_publisher_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PlanningNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("planning_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
