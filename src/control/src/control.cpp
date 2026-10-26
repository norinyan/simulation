#include "control.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace control
{
// 插件生命周期与接口接入：加载模型、参数和轨迹订阅。
WheelLegControl::~WheelLegControl()
{
  update_.reset();
  std::lock_guard<std::mutex> lock(gate_->mutex);
  gate_->active = false;
}

void WheelLegControl::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  node_ = gazebo_ros::Node::Get(sdf);
  // 关节硬限制以模型中的 URDF 为唯一来源。
  for (size_t i = 0; i < names_.size(); ++i) {
    joints_[i] = model_->GetJoint(names_[i]);
    if (!joints_[i]) {
      RCLCPP_ERROR(node_->get_logger(), "缺少关节：%s", names_[i].c_str());
      return;
    }
    q_[i] = joints_[i]->Position(0);
    lower_[i] = joints_[i]->LowerLimit(0);
    upper_[i] = joints_[i]->UpperLimit(0);
    speed_limits_[i] = joints_[i]->GetVelocityLimit(0);
    if (!std::isfinite(speed_limits_[i]) || speed_limits_[i] <= 0.0) {
      RCLCPP_ERROR(node_->get_logger(), "关节速度上限无效：%s", names_[i].c_str());
      return;
    }
  }
  try {
    LoadConfig(sdf);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node_->get_logger(), "控制配置加载失败：%s", error.what());
    return;
  }
  for (const auto & link : model_->GetLinks()) {
    link->SetGravityMode(false);
    link->SetKinematic(true);
  }
  pose_ = model_->WorldPose();
  leg_ = sdf->Get<bool>("initial_leg_mode", false).first;
  ground_ = pose_.Pos().Z() - (leg_ ? cfg_.foot_height : cfg_.wheel_height);
  feet_ = GroundFeet(pose_);
  if (leg_) {
    q_[0] = cfg_.wheel_lift;
    if (!SolveLegs()) {
      RCLCPP_ERROR(node_->get_logger(), "初始足式站姿不可达");
      return;
    }
  }
  reported_pose_ = pose_;
  reported_q_ = q_;
  last_time_ = last_publish_ = model_->GetWorld()->SimTime().Double();
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/robot/odom", 10);
  joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  trajectory_sub_ = node_->create_subscription<interfaces::msg::LocalTrajectory>(
    "/local_trajectory", 10,
    [this, gate = gate_](interfaces::msg::LocalTrajectory::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(gate->mutex);
      if (gate->active) {ReceiveTrajectory(*msg);}
    });
  update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    [this, gate = gate_](const gazebo::common::UpdateInfo & info) {
      std::lock_guard<std::mutex> lock(gate->mutex);
      if (gate->active) {Update(info);}
    });
  RCLCPP_INFO(node_->get_logger(), "理想运动学控制已加载");
}

// 配置加载：仅维护行为参数，缺失或无效配置直接报错。
void WheelLegControl::LoadConfig(sdf::ElementPtr sdf)
{
  const char * environment = std::getenv("WHEEL_LEG_CONTROL_CONFIG");
  const std::string path = sdf->HasElement("config_file") ? sdf->Get<std::string>("config_file") :
    (environment && *environment ? environment :
    ament_index_cpp::get_package_share_directory("control") + "/config/control.yaml");
  const auto yaml = YAML::LoadFile(path);
  auto positive = [&yaml](const char * group, const char * key) {
      const double value = (group ? yaml[group][key] : yaml[key]).as<double>();
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error(std::string(group ? group : "") + "." + key + " 必须为正有限数");
      }
      return value;
    };
  cfg_.wheel_linear_speed = positive("wheel", "linear_speed");
  cfg_.wheel_angular_speed = positive("wheel", "angular_speed");
  cfg_.wheel_height = positive("wheel", "body_height");
  cfg_.foot_linear_speed = positive("foot", "linear_speed");
  cfg_.foot_angular_speed = positive("foot", "angular_speed");
  cfg_.foot_height = positive("foot", "body_height");
  cfg_.wheel_lift = positive("foot", "wheel_lift");
  cfg_.swing_duration = positive("foot", "swing_duration");
  cfg_.step_budget = positive("foot", "step_budget");
  cfg_.swing_lift = positive("foot", "swing_lift");
  cfg_.stair_speed = positive("stair", "speed");
  cfg_.stair_run = positive("stair", "run");
  cfg_.stair_rise = positive("stair", "rise");
  cfg_.stair_lift = positive("stair", "swing_lift");
  cfg_.stair_first_up = positive("stair", "first_body_up");
  cfg_.stair_first_down = positive("stair", "first_body_down");
  cfg_.mode_duration = positive(nullptr, "mode_duration");
  cfg_.feedback_period = positive(nullptr, "feedback_period");
  cfg_.position_tolerance = positive("trajectory", "position_tolerance");
  cfg_.orientation_tolerance = positive("trajectory", "orientation_tolerance");
  cfg_.geometry_tolerance = positive("trajectory", "geometry_tolerance");
  if (cfg_.wheel_lift < lower_[0] || cfg_.wheel_lift > upper_[0] ||
    cfg_.stair_first_up > cfg_.stair_run || cfg_.stair_first_down > cfg_.stair_run ||
    cfg_.foot_height <= cfg_.wheel_height)
  {
    throw std::runtime_error("站高、轮架位置或台阶分段位移不合法");
  }
  RCLCPP_INFO(node_->get_logger(), "控制参数已加载：%s", path.c_str());
}

// 轮式控制：本体按前后速度和偏航速度运动，轮角同步积分。
// MoveBody 同时供足步控制复用，只负责本体平面运动。
void WheelLegControl::MoveBody(double dt, double forward, double yaw_rate)
{
  const double yaw = pose_.Rot().Yaw();
  const double middle = yaw + yaw_rate * dt * 0.5;
  pose_.Pos().X() += forward * std::cos(middle) * dt;
  pose_.Pos().Y() += forward * std::sin(middle) * dt;
  pose_.Rot() = ignition::math::Quaterniond(0.0, 0.0, yaw + yaw_rate * dt);
}

void WheelLegControl::Wheel(double dt)
{
  double v = std::clamp(velocity_.linear.x, -cfg_.wheel_linear_speed, cfg_.wheel_linear_speed);
  double w = std::clamp(velocity_.angular.z, -cfg_.wheel_angular_speed, cfg_.wheel_angular_speed);
  const double scale = std::max(1.0, std::max(
    std::abs((v - 0.21 * w) / 0.14) / speed_limits_[1],
    std::abs((v + 0.21 * w) / 0.14) / speed_limits_[2]));
  v /= scale;
  w /= scale;
  MoveBody(dt, v, w);
  q_[0] = 0.0;
  q_[1] += (v - 0.21 * w) / 0.14 * dt;
  q_[2] += (v + 0.21 * w) / 0.14 * dt;
  std::fill(q_.begin() + 3, q_.end(), 0.0);
}

// 足步控制：包含足端插值与正逆运动学、平地行走和上下台阶。
// 足端工具：平滑插值、分阶段抬落足和落足位置计算。
double WheelLegControl::Smooth(double t) {return t * t * (3.0 - 2.0 * t);}

ignition::math::Vector3d WheelLegControl::SwingFoot(
  const ignition::math::Vector3d & from, const ignition::math::Vector3d & to,
  double t, double lift)
{
  const double top = std::max(from.Z(), to.Z()) + lift;
  if (t < 0.25) {
    auto result = from;
    result.Z() += (top - from.Z()) * Smooth(t * 4.0);
    return result;
  }
  if (t < 0.75) {
    auto result = from + (to - from) * Smooth((t - 0.25) * 2.0);
    result.Z() = top;
    return result;
  }
  auto result = to;
  result.Z() = top + (to.Z() - top) * Smooth((t - 0.75) * 4.0);
  return result;
}

std::array<ignition::math::Vector3d, 2> WheelLegControl::GroundFeet(
  const ignition::math::Pose3d & body) const
{
  std::array<ignition::math::Vector3d, 2> result;
  for (size_t i = 0; i < 2; ++i) {
    result[i] = body.Pos() + body.Rot().RotateVector(
      ignition::math::Vector3d(0.14, i == 0 ? 0.34 : -0.34, 0.0));
    result[i].Z() = ground_ + 0.0175;
  }
  return result;
}

// 双腿逆运动学：只在两腿解均有效且满足 URDF 限位时更新关节目标。
bool WheelLegControl::SolveLegs()
{
  auto next = q_;
  for (size_t i = 0; i < 2; ++i) {
    auto p = pose_.Rot().RotateVectorReverse(feet_[i] - pose_.Pos()) -
      ignition::math::Vector3d(0.04, i == 0 ? 0.34 : -0.34, 0.06);
    double yaw = std::hypot(p.X(), p.Y()) < 1e-8 ? 0.0 : std::atan2(p.Y(), p.X());
    if (yaw > M_PI_2) {yaw -= M_PI;}
    if (yaw < -M_PI_2) {yaw += M_PI;}
    const double x = p.X() * std::cos(yaw) + p.Y() * std::sin(yaw);
    const double c = (x * x + p.Z() * p.Z() - 2.0 * 0.28 * 0.28) / (2.0 * 0.28 * 0.28);
    if (c < -1.0 || c > 1.0) {return false;}
    const double knee = -std::acos(c);
    const double shoulder = std::atan2(-x, -p.Z()) -
      std::atan2(0.28 * std::sin(knee), 0.28 + 0.28 * std::cos(knee));
    const size_t j = 3 + i * 4;
    next[j] = yaw;
    next[j + 1] = shoulder - 1.15;
    next[j + 2] = knee + 2.3;
    next[j + 3] = -next[j + 1] - next[j + 2];
    for (size_t k = j; k < j + 4; ++k) {
      if (!std::isfinite(next[k]) || next[k] < lower_[k] || next[k] > upper_[k]) {
        return false;
      }
    }
  }
  q_ = next;
  return true;
}

// 台阶动作初始化：仅由已校验的足式轨迹调用，沿用原有落足目标。
void WheelLegControl::StartStair(bool up, double start, double duration)
{
  velocity_ = geometry_msgs::msg::Twist();
  stair_active_ = true;
  stair_up_ = up;
  stair_start_ = start;
  stair_duration_ = duration;
  stair_pose_ = pose_;
  feet_start_ = feet_;
  auto end = pose_;
  end.Pos() += pose_.Rot().RotateVector(ignition::math::Vector3d(cfg_.stair_run, 0, 0));
  feet_goal_ = GroundFeet(end);
  for (auto & foot : feet_goal_) {foot.Z() += up ? cfg_.stair_rise : -cfg_.stair_rise;}
}

// 足步控制：台阶段执行上下阶，否则按轨迹速度交替摆足。
void WheelLegControl::Leg(double now, double dt)
{
  const auto old_pose = pose_;
  const auto old_feet = feet_;
  bool finished = false;
  if (stair_active_) {
    const double t = std::clamp((now - stair_start_) / stair_duration_, 0.0, 1.0);
    finished = t >= 1.0;
    // 上下台阶子动作，仍属于足步控制；保留原有两阶段移身与跟步顺序。
    // 先落领先足，再移身跟步；下阶时先降身。
    const bool up = stair_up_;
    const bool first = t < 0.5;
    const double u = first ? t * 2.0 : (t - 0.5) * 2.0;
    const double s = Smooth(u);
    const double first_run = up ? cfg_.stair_first_up : cfg_.stair_first_down;
    const double x = first ? first_run * s :
      first_run + (cfg_.stair_run - first_run) * s;
    const double z = up ? (first ? 0.0 : cfg_.stair_rise * s) :
      (first ? -cfg_.stair_rise * s : -cfg_.stair_rise);
    pose_ = stair_pose_;
    pose_.Pos() += pose_.Rot().RotateVector(ignition::math::Vector3d(x, 0, z));
    feet_ = feet_start_;
    if (!first) {feet_[0] = feet_goal_[0];}
    const size_t side = first ? 0 : 1;
    feet_[side] = SwingFoot(feet_start_[side], feet_goal_[side], u, cfg_.stair_lift);
  } else {
    // 平地连续步态：使用轨迹平面段速度。
    double v = std::clamp(velocity_.linear.x, -cfg_.foot_linear_speed, cfg_.foot_linear_speed);
    double w = std::clamp(velocity_.angular.z, -cfg_.foot_angular_speed, cfg_.foot_angular_speed);
    // 限制每步位移，避开髋偏航的侧向奇异位形。
    const double scale = std::max(1.0,
      (std::abs(v) + 0.34 * std::abs(w)) * cfg_.swing_duration / cfg_.step_budget);
    v /= scale;
    w /= scale;
    MoveBody(dt, v, w);
    if (!swing_ && (std::abs(v) > 1e-6 || std::abs(w) > 1e-6)) {
      swing_ = true;
      swing_start_ = now;
      swing_from_ = feet_[swing_side_];
      auto future = pose_;
      MoveBody(cfg_.swing_duration, v, w);
      swing_to_ = GroundFeet(pose_)[swing_side_];
      pose_ = future;
    }
    if (swing_) {
      const double t = std::clamp((now - swing_start_) / cfg_.swing_duration, 0.0, 1.0);
      feet_[swing_side_] = swing_from_ + (swing_to_ - swing_from_) * Smooth(t);
      feet_[swing_side_].Z() += cfg_.swing_lift * std::sin(M_PI * t);
      if (t >= 1.0) {swing_ = false; swing_side_ = 1 - swing_side_;}
    }
  }
  q_[0] = cfg_.wheel_lift;
  if (!SolveLegs()) {
    pose_ = old_pose;
    feet_ = old_feet;
    FreezeMotion();
    RCLCPP_WARN(node_->get_logger(), "足端目标不可达，运动已停止");
  } else if (finished) {
    ground_ += stair_up_ ? cfg_.stair_rise : -cfg_.stair_rise;
    stair_active_ = false;
  }
}

// 足端正运动学：根据关节位置恢复足端位置，供轮足变换记录起始状态。
std::array<ignition::math::Vector3d, 2> WheelLegControl::FootPositions(
  const std::array<double, 11> & joints) const
{
  std::array<ignition::math::Vector3d, 2> result;
  for (size_t i = 0; i < 2; ++i) {
    const size_t j = 3 + i * 4;
    const double a = 1.15 + joints[j + 1];
    const double b = a - 2.3 + joints[j + 2];
    const double x = -0.28 * (std::sin(a) + std::sin(b));
    const double z = -0.28 * (std::cos(a) + std::cos(b));
    result[i] = pose_.Pos() + pose_.Rot().RotateVector(ignition::math::Vector3d(
        0.04 + x * std::cos(joints[j]),
        (i == 0 ? 0.34 : -0.34) + x * std::sin(joints[j]), 0.06 + z));
  }
  return result;
}

// 轮足变换：静止后记录姿态，复用原有轮转足、足转轮的两阶段动作。
bool WheelLegControl::SetMode(bool leg)
{
  if (switching_ || stair_active_ || swing_) {return false;}
  velocity_ = geometry_msgs::msg::Twist();
  frozen_ = false;
  switching_ = true;
  target_leg_ = leg;
  mode_start_ = last_time_;
  mode_pose_ = pose_;
  mode_lift_ = q_[0];
  mode_feet_ = FootPositions(q_);
  return true;
}

void WheelLegControl::SwitchMode(double now)
{
  const auto old_pose = pose_;
  const auto old_feet = feet_;
  const auto old_q = q_;
  const double t = std::clamp((now - mode_start_) / cfg_.mode_duration, 0.0, 1.0);
  const bool first = t < 0.5;
  const double s = Smooth(first ? t * 2.0 : (t - 0.5) * 2.0);
  bool valid = false;
  pose_ = mode_pose_;
  if (target_leg_) {
    // 先落足，再升身收轮。
    const auto planted = GroundFeet(mode_pose_);
    for (size_t i = 0; i < 2; ++i) {
      feet_[i] = first ? mode_feet_[i] + (planted[i] - mode_feet_[i]) * s : planted[i];
    }
    if (!first) {pose_.Pos().Z() += (ground_ + cfg_.foot_height - mode_pose_.Pos().Z()) * s;}
    q_[0] = first ? mode_lift_ : mode_lift_ + (cfg_.wheel_lift - mode_lift_) * s;
    valid = SolveLegs();
  } else {
    // 先降身落轮，再收腿。
    pose_.Pos().Z() += (ground_ + cfg_.wheel_height - mode_pose_.Pos().Z()) * (first ? s : 1.0);
    q_[0] = first ? mode_lift_ * (1.0 - s) : 0.0;
    feet_ = mode_feet_;
    valid = SolveLegs();
    if (!first && valid) {
      // 轮子落地后按关节收腿，避开折叠末端的逆解奇异点。
      for (size_t i = 3; i < q_.size(); ++i) {q_[i] *= 1.0 - s;}
      feet_ = FootPositions(q_);
    }
  }
  if (!valid) {
    pose_ = old_pose;
    feet_ = old_feet;
    q_ = old_q;
    FreezeMotion();
    RCLCPP_WARN(node_->get_logger(), "切换目标不可达，已保持当前姿态");
  } else if (t >= 1.0) {
    leg_ = target_leg_;
    switching_ = false;
  }
}

// 轨迹执行：校验消息、准备目标模式，再按相对时间复用轮式或足步控制。
void WheelLegControl::ClearTrajectory()
{
  trajectory_.clear();
  trajectory_index_ = 0;
  trajectory_started_ = segment_started_ = false;
  velocity_ = geometry_msgs::msg::Twist();
}

// 异常冻结与正常轨迹结束分开：冻结会立即取消摆足，需重新生成机器人恢复。
void WheelLegControl::FreezeMotion()
{
  ClearTrajectory();
  stair_active_ = false;
  swing_ = switching_ = false;
  frozen_ = true;
}

void WheelLegControl::ReceiveTrajectory(const interfaces::msg::LocalTrajectory & msg)
{
  auto reject = [this](const char * reason) {
      RCLCPP_WARN(node_->get_logger(), "轨迹拒绝：%s", reason);
    };
  if (msg.mode != "wheel" && msg.mode != "foot") {reject("mode 必须为 wheel 或 foot"); return;}
  const bool foot = msg.mode == "foot";
  if (msg.header.frame_id != "world") {reject("仅支持 world 坐标系"); return;}
  if (msg.points.size() < 2) {reject("至少需要两个轨迹点"); return;}
  if (frozen_ || stair_active_ || (switching_ && target_leg_ != foot)) {
    reject("当前动作未完成或控制已冻结（冻结后需重新生成机器人）");
    return;
  }
  // header.stamp 不作启动门槛；运动时间只使用各点的 time_from_start。
  auto finite_twist = [](const geometry_msgs::msg::Twist & v) {
      return std::isfinite(v.linear.x) && std::isfinite(v.linear.y) &&
             std::isfinite(v.linear.z) && std::isfinite(v.angular.x) &&
             std::isfinite(v.angular.y) && std::isfinite(v.angular.z);
    };
  // 只缓存待提交的执行段；任一点校验失败都不影响正在执行的轨迹。
  std::vector<Segment> next;
  ignition::math::Pose3d previous;
  double previous_time = 0.0;
  auto expected = pose_;
  // 首点描述目标模式准备完成后的 world 绝对位姿，不包含变形过程。
  if (switching_ || foot != leg_) {
    expected.Pos().Z() = ground_ + (foot ? cfg_.foot_height : cfg_.wheel_height);
  }
  for (size_t i = 0; i < msg.points.size(); ++i) {
    const auto & point = msg.points[i];
    if (point.transforms.size() != 1 || point.velocities.size() > 1 ||
      point.accelerations.size() > 1 ||
      (!point.velocities.empty() && !finite_twist(point.velocities[0])) ||
      (!point.accelerations.empty() && !finite_twist(point.accelerations[0])) ||
      point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u)
    {
      reject("轨迹点必须包含一个本体变换，时间和速度必须有效"); return;
    }
    const auto & transform = point.transforms[0];
    const auto & p = transform.translation;
    const auto & q = transform.rotation;
    const double norm = std::hypot(std::hypot(q.w, q.x), std::hypot(q.y, q.z));
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(norm) || norm < 1e-9)
    {
      reject("位姿含非有限数或无效四元数"); return;
    }
    ignition::math::Quaterniond rotation(q.w / norm, q.x / norm, q.y / norm, q.z / norm);
    if (std::abs(rotation.Roll()) > cfg_.orientation_tolerance ||
      std::abs(rotation.Pitch()) > cfg_.orientation_tolerance)
    {
      reject("当前运动学模型仅支持水平本体与偏航"); return;
    }
    const ignition::math::Pose3d pose(p.x, p.y, p.z, 0.0, 0.0, rotation.Yaw());
    const double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
    if ((i == 0 && time != 0.0) || (i > 0 && time <= previous_time)) {
      reject("首点时间必须为 0，后续时间必须严格递增"); return;
    }
    if (i == 0) {
      if (pose.Pos().Distance(expected.Pos()) > cfg_.position_tolerance ||
        std::abs(ignition::math::Angle(pose.Rot().Yaw() - expected.Rot().Yaw()).
        Normalized().Radian()) > cfg_.orientation_tolerance)
      {
        reject("首点必须衔接当前本体位置和目标模式站高"); return;
      }
      previous = pose;
      previous_time = time;
      continue;
    }
    Segment segment;
    segment.from = previous;
    segment.to = pose;
    segment.begin = previous_time;
    segment.end = time;
    const double duration = segment.end - segment.begin;
    const auto delta = segment.from.Rot().RotateVectorReverse(
      segment.to.Pos() - segment.from.Pos());
    const double theta = ignition::math::Angle(
      segment.to.Rot().Yaw() - segment.from.Rot().Yaw()).Normalized().Radian();
    if (std::abs(delta.Z()) > cfg_.geometry_tolerance) {
      // 当前只执行与场景尺寸相符的完整单阶，不把任意变高曲线解释为楼梯。
      if (!foot || std::abs(delta.X() - cfg_.stair_run) > cfg_.geometry_tolerance ||
        std::abs(delta.Y()) > cfg_.geometry_tolerance || std::abs(theta) > 1e-6 ||
        std::abs(std::abs(delta.Z()) - cfg_.stair_rise) > cfg_.geometry_tolerance ||
        duration + 1e-9 < cfg_.stair_run / cfg_.stair_speed)
      {
        reject("变高段必须为足式单阶前进，尺寸和时长须符合配置"); return;
      }
      segment.stair = true;
    } else {
      // 相邻位姿定义直线、原地转向或常曲率圆弧；保持旧 MoveBody 的非完整约束。
      const double half = theta * 0.5;
      const double lateral = -delta.X() * std::sin(half) + delta.Y() * std::cos(half);
      if (std::abs(lateral) > cfg_.geometry_tolerance) {
        reject("平面轨迹含无法执行的横向位移"); return;
      }
      const double sinc = std::abs(half) < 1e-8 ? 1.0 : std::sin(half) / half;
      segment.forward = (delta.X() * std::cos(half) + delta.Y() * std::sin(half)) /
        (sinc * duration);
      segment.yaw_rate = theta / duration;
      if (!std::isfinite(segment.forward) || !std::isfinite(segment.yaw_rate)) {
        reject("轨迹位移与时间计算得到非有限速度"); return;
      }
      const double v = std::abs(segment.forward), w = std::abs(segment.yaw_rate);
      if (v > (foot ? cfg_.foot_linear_speed : cfg_.wheel_linear_speed) + 1e-9 ||
        w > (foot ? cfg_.foot_angular_speed : cfg_.wheel_angular_speed) + 1e-9 ||
        (foot && (v + 0.34 * w) * cfg_.swing_duration > cfg_.step_budget + 1e-9) ||
        (!foot && (std::abs((segment.forward - 0.21 * segment.yaw_rate) / 0.14) >
        speed_limits_[1] + 1e-9 ||
        std::abs((segment.forward + 0.21 * segment.yaw_rate) / 0.14) > speed_limits_[2] + 1e-9)))
      {
        reject("轨迹时间要求超出轮式或足式速度限制"); return;
      }
    }
    next.push_back(segment);
    previous = pose;
    previous_time = time;
  }
  // 同模式平面轨迹可更新；切换和单阶动作不在中途重启。
  trajectory_ = std::move(next);
  trajectory_leg_ = foot;
  trajectory_index_ = 0;
  trajectory_started_ = segment_started_ = false;
  velocity_ = geometry_msgs::msg::Twist();
  RCLCPP_INFO(node_->get_logger(), "接受 %s 轨迹，%zu 段", msg.mode.c_str(), trajectory_.size());
}

void WheelLegControl::ExecuteTrajectory(double now, double dt)
{
  auto fail = [this](const char * reason) {
      RCLCPP_WARN(node_->get_logger(), "轨迹停止：%s", reason);
      ClearTrajectory();
    };
  if (!trajectory_started_) {
    velocity_ = geometry_msgs::msg::Twist();
    if (trajectory_leg_ != leg_) {
      if (swing_) {Leg(now, dt); return;}
      if (!SetMode(trajectory_leg_)) {fail("请等待当前动作完成");}
      return;
    }
    if (pose_.Pos().Distance(trajectory_.front().from.Pos()) > cfg_.position_tolerance) {
      fail("准备完成后起点无法衔接"); return;
    }
    // 准备/切换完成后开始相对计时，避免准备耗时占用轨迹运动时间。
    trajectory_start_ = last_time_;
    trajectory_started_ = true;
  }
  while (trajectory_index_ < trajectory_.size()) {
    const auto & segment = trajectory_[trajectory_index_];
    const double begin = trajectory_start_ + segment.begin;
    const double end = trajectory_start_ + segment.end;
    // 在浮点误差内取精确末帧，保证阶步动作完成并累计 ground_。
    const double sample = now + 1e-9 >= end ? end : now;
    const double motion_dt = sample - std::max(last_time_, begin);
    if (motion_dt <= 0.0 && now < end) {return;}
    if (!segment_started_) {
      if (segment.stair) {
        if (swing_) {fail("台阶前须增加保持段，等待摆足落地"); return;}
        StartStair(segment.to.Pos().Z() > segment.from.Pos().Z(), begin, segment.end - segment.begin);
      }
      segment_started_ = true;
    }
    if (segment.stair) {
      Leg(sample, std::max(0.0, motion_dt));
    } else if (motion_dt > 0.0) {
      velocity_.linear.x = segment.forward;
      velocity_.angular.z = segment.yaw_rate;
      if (leg_) {Leg(sample, motion_dt);} else {Wheel(motion_dt);}
    }
    // 足步失败会冻结并清除轨迹，必须先退出，再访问当前段。
    if (frozen_) {fail("逆解或关节检查失败，请重新生成机器人"); return;}
    if (now + 1e-9 < end) {return;}
    if (pose_.Pos().Distance(segment.to.Pos()) > cfg_.position_tolerance ||
      std::abs(ignition::math::Angle(pose_.Rot().Yaw() - segment.to.Rot().Yaw()).
      Normalized().Radian()) > cfg_.orientation_tolerance)
    {
      fail("段终点偏差超限"); return;
    }
    ++trajectory_index_;
    segment_started_ = false;
    velocity_ = geometry_msgs::msg::Twist();
  }
  ClearTrajectory();
  RCLCPP_INFO(node_->get_logger(), "轨迹执行完成");
}

// 状态反馈：使用模型实际位姿和关节位置发布 odom、joint_states。
void WheelLegControl::Publish(double now, double dt, const ignition::math::Pose3d & previous)
{
  const builtin_interfaces::msg::Time stamp = rclcpp::Time(static_cast<int64_t>(now * 1e9));
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link";
  const auto actual = model_->GetLink("base_link")->WorldPose();
  odom.pose.pose.position.x = actual.Pos().X();
  odom.pose.pose.position.y = actual.Pos().Y();
  odom.pose.pose.position.z = actual.Pos().Z();
  odom.pose.pose.orientation.x = actual.Rot().X();
  odom.pose.pose.orientation.y = actual.Rot().Y();
  odom.pose.pose.orientation.z = actual.Rot().Z();
  odom.pose.pose.orientation.w = actual.Rot().W();
  const auto velocity = actual.Rot().RotateVectorReverse((actual.Pos() - previous.Pos()) / dt);
  odom.twist.twist.linear.x = velocity.X();
  odom.twist.twist.linear.y = velocity.Y();
  odom.twist.twist.linear.z = velocity.Z();
  odom.twist.twist.angular.z = ignition::math::Angle(
    actual.Rot().Yaw() - previous.Rot().Yaw()).Normalized().Radian() / dt;
  odom_pub_->publish(odom);

  sensor_msgs::msg::JointState state;
  state.header.stamp = stamp;
  state.name.assign(names_.begin(), names_.end());
  for (size_t i = 0; i < joints_.size(); ++i) {
    const double position = joints_[i]->Position(0);
    state.position.push_back(position);
    const double change = position - reported_q_[i];
    state.velocity.push_back((i == 1 || i == 2 ?
      ignition::math::Angle(change).Normalized().Radian() : change) / dt);
    reported_q_[i] = position;
  }
  joint_pub_->publish(state);
}

// 仿真更新：统一调度轮足变换和轨迹运动，并在写入模型前检查关节速度。
void WheelLegControl::Update(const gazebo::common::UpdateInfo & info)
{
  const double now = info.simTime.Double();
  const double dt = now - last_time_;
  if (dt <= 0.0) {
    if (dt < 0.0) {
      last_time_ = last_publish_ = now;
      reported_pose_ = pose_;
      FreezeMotion();
    }
    return;
  }
  const auto previous_pose = pose_;
  const auto previous_q = q_;
  const auto previous_feet = feet_;
  const double previous_ground = ground_;
  const bool previous_leg = leg_;
  const bool was_switching = switching_;
  if (switching_) {
    SwitchMode(now);
  } else if (!frozen_) {
    if (!trajectory_.empty()) {ExecuteTrajectory(now, dt);}
    else if (leg_) {Leg(now, dt);} else {Wheel(dt);}
  }
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (std::abs(q_[i] - previous_q[i]) > speed_limits_[i] * dt + 1e-6) {
      pose_ = previous_pose;
      q_ = previous_q;
      feet_ = previous_feet;
      ground_ = previous_ground;
      leg_ = previous_leg;
      FreezeMotion();
      RCLCPP_WARN(node_->get_logger(), "关节速度超限，已保持姿态：%s", names_[i].c_str());
      break;
    }
  }
  if (was_switching && !switching_ && !frozen_) {
    RCLCPP_INFO(node_->get_logger(), "%s切换完成", leg_ ? "足式" : "轮式");
  }
  model_->SetWorldPose(pose_);
  for (size_t i = 0; i < joints_.size(); ++i) {
    joints_[i]->SetPosition(0, q_[i]);
  }
  if (now - last_publish_ >= cfg_.feedback_period) {
    Publish(now, now - last_publish_, reported_pose_);
    reported_pose_ = model_->GetLink("base_link")->WorldPose();
    last_publish_ = now;
  }
  last_time_ = now;
}

GZ_REGISTER_MODEL_PLUGIN(WheelLegControl)
}  // namespace control
