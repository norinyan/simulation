#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <array>
#include <chrono>

namespace local_planning
{
class ScanMap
{
public:
  struct RobotGeometry {
    double foot_length{}, hip_radius{}, wheel_radius{}, wheel_half_track{}, wheel_speed_limit{};
  };
  struct FootPose {Eigen::Vector3d body; double yaw{}, time{};};
  void load(const std::string & filename);
  void loadRobot(const std::string & filename, double wheel_lift);
  const RobotGeometry & robotGeometry() const {return robot_geometry_;}
  // Nominal static articulated pose, not a claim about the controller's retained
  // support feet or swing trajectories. Non-standing queries only cover rigid
  // base/carrier/wheels, since spline control points are not standing poses.
  bool robotCollision(const Eigen::Vector3d & body, double yaw, bool foot,
    double margin, double tolerance, bool standing = true,
    const std::array<Eigen::Vector3d,2> * actual_feet = nullptr,
    const std::array<std::array<double,2>,2> * contact_heights = nullptr) const;
  // Replays the existing controller's foot state, prediction, and swing rules.
  // Throws on collision, unavailable landing, IK/velocity limit, or deadline.
  void validateFootTrajectory(const std::vector<FootPose> & poses,
    double swing_duration, double swing_lift, double feedback_period,
    double margin, double tolerance, std::chrono::steady_clock::time_point deadline) const;
  // Same downward query interval as the controller; returns the highest real
  // collision-box surface, including pitched boxes, rather than an AABB top.
  bool height(double x, double y, double ceiling, double depth, double & z) const;
  bool support(const Eigen::Vector3d & body, double expected_ground,
    double tolerance) const;
  bool surface(double x, double y, double yaw, double ceiling,
    double tolerance, double & z) const;
  bool footLanding(const Eigen::Vector3d & body, double yaw, int side,
    double tolerance, Eigen::Vector3d & foot) const;
  std::size_t size() const {return boxes_.size();}

private:
  struct Box
  {
    Eigen::Vector3d center, half, minimum, maximum;
    Eigen::Matrix3d rotation;
  };
  struct Joint {
    std::string name, parent, child, type;
    Eigen::Isometry3d origin;
    Eigen::Vector3d axis;
    double lower{}, upper{}, speed{};
  };
  struct Shape {
    std::string link;
    Eigen::Isometry3d origin;
    Eigen::Vector3d half;
  };
  std::vector<Box> boxes_;
  std::vector<Joint> joints_;
  std::vector<Shape> shapes_;
  RobotGeometry robot_geometry_;
  double wheel_lift_{};
  const Joint & jointByName(const std::string & name) const;
  bool legAngles(const Eigen::Vector3d & body, double yaw, const Eigen::Vector3d & foot,
    int side, std::array<double,4> & q) const;
};
}  // namespace local_planning
