#ifndef CONTROL__CONTROL_H_
#define CONTROL__CONTROL_H_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <interfaces/msg/local_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace control
{
// 理想运动学 Gazebo 插件：接收本体轨迹，统一推进运动并发布状态。
class WheelLegControl : public gazebo::ModelPlugin
{
public:
  ~WheelLegControl() override;
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  // 原有并发保护：串行访问 ROS 回调和仿真更新共享的数据，不创建线程。
  struct CallbackGuard
  {
    std::mutex mutex;
    bool active{true};
  };

  // 行为参数从 YAML 加载；关节位置、速度限制直接使用 URDF。
  struct Config
  {
    // 轮式控制：本体速度上限与站高。
    double wheel_linear_speed{}, wheel_angular_speed{}, wheel_height{};
    // 足步控制：平地步态和上下台阶。
    double foot_linear_speed{}, foot_angular_speed{}, foot_height{}, wheel_lift{};
    double swing_duration{}, step_budget{}, swing_lift{};
    // 轮足变换与状态反馈周期。
    double mode_duration{}, feedback_period{};
    // 轨迹起终点衔接与当前运动能力的匹配容差。
    double position_tolerance{}, orientation_tolerance{}, geometry_tolerance{};
  } cfg_;

  // 足式按 XYZ + yaw 插值；轮式保留恒定 v/w 圆弧执行。
  struct Segment
  {
    ignition::math::Pose3d from, to;  // world 下 base_link 的起终点。
    double begin{}, end{}, forward{}, yaw_rate{};  // 相对轨迹时间、前后速度和偏航速度。
  };

  void LoadConfig(sdf::ElementPtr sdf);

  // 轮式控制：平面本体运动与双轮转动。
  void MoveBody(double dt, double forward, double yaw_rate);
  void Wheel(double dt);

  // 足步控制：足端插值与正逆解、平地步态和上下台阶。
  static double Smooth(double t);
  static ignition::math::Vector3d SwingFoot(
    const ignition::math::Vector3d & from, const ignition::math::Vector3d & to,
    double t, double lift);
  std::array<ignition::math::Vector3d, 2> GroundFeet(
    const ignition::math::Pose3d & body) const;
  bool SolveLegs();
  bool LegAngles(const ignition::math::Pose3d & body,
    const ignition::math::Vector3d & foot, size_t side, std::array<double, 11> & next) const;
  // 射线查询只在 Gazebo 更新线程使用，排除机器人自身碰撞体。
  bool Height(double x, double y, double ceiling, double & z) const;
  bool Surface(ignition::math::Vector3d & foot, double yaw, double ceiling) const;
  bool Landing(const ignition::math::Pose3d & body, size_t side,
    ignition::math::Vector3d & foot) const;
  ignition::math::Pose3d SampleTrajectory(double time) const;
  void Leg(double now, double dt);
  std::array<ignition::math::Vector3d, 2> FootPositions(
    const std::array<double, 11> & joints) const;

  // 轮足变换：记录起始姿态，按既有的两阶段动作切换形态。
  bool SetMode(bool leg);
  void SwitchMode(double now);

  // 轨迹执行：目标模式准备完成后，以 time_from_start 相对计时。
  void ReceiveTrajectory(const interfaces::msg::LocalTrajectory & msg);
  void ExecuteTrajectory(double now, double dt);
  void ClearTrajectory();
  // 异常时冻结；仅清理执行状态，姿态回滚由调用处负责。
  void FreezeMotion();

  // 状态反馈与仿真更新：检查关节速度，写入模型并发布 odom/joint_states。
  void Publish(double now, double dt, const ignition::math::Pose3d & previous);
  void Update(const gazebo::common::UpdateInfo & info);

  // 当前轨迹及执行进度；header.stamp 不参与开始时间调度。
  std::vector<Segment> trajectory_;
  size_t trajectory_index_{0};
  double trajectory_start_{0.0};
  bool trajectory_leg_{false}, trajectory_started_{false}, segment_started_{false};
  rclcpp::Subscription<interfaces::msg::LocalTrajectory>::SharedPtr trajectory_sub_;

  // 模型关节：轮架、双轮、左腿四关节、右腿四关节，顺序与 URDF 对应。
  const std::array<std::string, 11> names_{
    "wheel_lift_joint", "left_wheel_joint", "right_wheel_joint",
    "left_hip_yaw_joint", "left_shoulder_joint", "left_knee_joint", "left_ankle_joint",
    "right_hip_yaw_joint", "right_shoulder_joint", "right_knee_joint", "right_ankle_joint"};
  std::array<gazebo::physics::JointPtr, 11> joints_;
  std::array<double, 11> q_{}, reported_q_{}, lower_{}, upper_{}, speed_limits_{};

  // 仿真模型与反馈采样；ground_ 仅缓存轮足切换处的地形高度。
  gazebo::physics::ModelPtr model_;
  gazebo::physics::RayShapePtr terrain_ray_;
  // 同一更新帧内复用场景包围盒，最终高度仍由真实碰撞几何射线求交得到。
  mutable double terrain_cache_time_{-1.0};
  mutable std::vector<std::pair<gazebo::physics::CollisionPtr,
    ignition::math::AxisAlignedBox>> terrain_collisions_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_;
  ignition::math::Pose3d pose_, reported_pose_;
  double last_time_{0.0}, last_publish_{0.0};
  double ground_{0.0};
  bool leg_{false};  // 当前实际模式：false 为轮式，true 为足式。

  // 轮足变换状态：目标模式及切换前的本体、轮架和足端快照。
  bool switching_{false}, target_leg_{false};
  double mode_start_{0.0}, mode_lift_{0.0};
  ignition::math::Pose3d mode_pose_;
  std::array<ignition::math::Vector3d, 2> mode_feet_{};

  // 支撑足固定在 world 中，摆足目标由未来本体轨迹和碰撞表面确定。
  std::array<ignition::math::Vector3d, 2> feet_{};
  bool swing_{false};
  size_t swing_side_{0};
  double swing_start_{0.0}, swing_duration_{0.0}, swing_clearance_{0.0};
  double next_foot_query_{0.0};
  ignition::math::Vector3d swing_from_, swing_to_;

  // 异常冻结及轨迹执行输入；冻结后通过重新生成机器人恢复。
  bool frozen_{false};
  geometry_msgs::msg::Twist velocity_;
  std::shared_ptr<CallbackGuard> gate_{std::make_shared<CallbackGuard>()};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
};
}  // namespace control

#endif  // CONTROL__CONTROL_H_
