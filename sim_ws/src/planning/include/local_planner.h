#ifndef PLANNING_LOCAL_PLANNER_H
#define PLANNING_LOCAL_PLANNER_H

#include "global_planner.h"

#include <array>

namespace planning
{

// Wheel-mode input/output types; retain their existing names for callers.
struct LocalPlanRequest
{
  // Reuse frame, bounds, boxes, footprint, margin and start/segment goal.
  PlanRequest geometry;
  std::vector<Pose> reference_path;  // One wheel-mode section only.
  double initial_linear_velocity{0.0};
  double initial_angular_velocity{0.0};
};

struct TrajectoryPoint
{
  Pose pose;
  double time_from_start{0.0};
  // At index 0 these are the requested initial velocities. At later indices,
  // velocities are means over the preceding interval; acceleration uses the
  // time between interval midpoints (half the first dt at the initial state).
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double linear_acceleration{0.0};
  double angular_acceleration{0.0};
};

struct LocalPlanResult
{
  bool success{false};
  std::vector<TrajectoryPoint> trajectory;
  std::string failure_reason;
  double planning_ms{0.0};
  double optimization_cost{0.0};  // Selected Ceres band cost, before retiming/windowing.
  double duration{0.0};
  double path_length{0.0};
  double max_linear_velocity{0.0};
  double max_angular_velocity{0.0};
  double max_linear_acceleration{0.0};
  double max_angular_acceleration{0.0};
  double max_lateral_velocity{0.0};
  int candidates_evaluated{0};
};

class LocalPlanner
{
public:
  virtual ~LocalPlanner() = default;
};

class WheelLocalPlanner : public LocalPlanner
{
public:
  virtual LocalPlanResult plan(const LocalPlanRequest& request) = 0;
};

struct FootRobotGeometry
{
  std::array<double, 3> body_size{};
  std::array<double, 3> shoulder_offset{};  // Left shoulder; right mirrors Y.
  std::array<double, 3> foot_size{};
  std::array<double, 3> folded_pitch{};  // Shoulder, knee, ankle installation angles.
  double upper_leg_length{0.0};
  double lower_leg_length{0.0};
  double leg_width{0.0};
  double foot_center_offset_x{0.0};
  double joint_min{0.0};
  double joint_max{0.0};
  double wheel_radius{0.0};
  double wheel_width{0.0};
  double wheel_track{0.0};
  double carrier_drop{0.0};
  double lift_min{0.0};
  double lift_max{0.0};
};

struct FootLocalPlanRequest
{
  // Start/goal are ground projections at the foot-mode entry/exit.
  PlanRequest geometry;
  std::vector<Pose> reference_path;
  std::vector<BoxObstacle> terrain;  // Actual solid slabs/steps, not obstacles.
  FootRobotGeometry robot;
};

struct Footstep
{
  int side{0};  // 0=left, 1=right; poses refer to foot collision centres.
  Pose from;
  Pose to;
  double edge_margin{0.0};
  double touchdown_time{0.0};
};

struct FootTrajectoryPoint
{
  double time_from_start{0.0};
  Pose body;  // Absolute base_link pose, unlike the request's ground poses.
  std::array<Pose, 2> feet{};  // Absolute collision-centre poses; level soles.
  // Analytic derivatives: body X/Y/Z, left X/Z, right X/Z. Planar +X motion.
  std::array<double, 7> velocity{};
  std::array<double, 7> acceleration{};
};

struct FootLocalPlanResult
{
  bool success{false};
  std::vector<Footstep> footsteps;
  std::vector<FootTrajectoryPoint> trajectory;
  std::string failure_reason;
  int expanded{0};
  double planning_ms{0.0};
};

class FootLocalPlanner : public LocalPlanner
{
public:
  virtual FootLocalPlanResult plan(const FootLocalPlanRequest& request) = 0;
};

}  // namespace planning

#endif  // PLANNING_LOCAL_PLANNER_H
