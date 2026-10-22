#ifndef PLANNING_LOCAL_PLANNER_H
#define PLANNING_LOCAL_PLANNER_H

#include "global_planner.h"

#include <array>
#include <map>
#include <optional>

namespace planning
{

struct WheelMotionLimits
{
  double max_v{.30}, max_w{.80}, max_a{.30}, max_alpha{1.0};
  double time_horizon{2.2}, max_lateral_velocity{.01};
  double collision_translation_step{.01}, collision_rotation_step{.02};
};

struct FootMotionLimits
{
  double max_step{.50}, edge_margin{.025}, swing_clearance{.08};
  double step_duration{9.0}, sample_dt{.02};
  double max_foot_v{.8}, max_foot_a{3.0}, max_body_v{.3}, max_body_a{1.0};
  double max_joint_v{1.0}, max_joint_a{20.0};
  double body_height{.43}, wheel_lift{.13};
};

// Wheel-mode input/output types; retain their existing names for callers.
struct LocalPlanRequest
{
  // Reuse frame, bounds, boxes, footprint, margin and start/segment goal.
  PlanRequest geometry;
  std::vector<Pose> reference_path;  // One wheel-mode section only.
  double initial_linear_velocity{0.0};
  double initial_angular_velocity{0.0};
  std::optional<WheelMotionLimits> limits;
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

// Clip at the nearest segment projection, retaining upcoming in-place turns.
std::vector<Pose> remainingWheelReference(const std::vector<Pose>& path,const Pose& actual);

// Execution and prediction share wheel tracking and bounded-acceleration math.
struct WheelTrackingCommand {double linear{0},angular{0};};
inline constexpr double kWheelBrakingPadding = .003;
// Follow one checked global-path edge, without a local planner or detour.
WheelTrackingCommand trackGlobalWheelGoal(const Pose& actual,const Pose& goal,
    const WheelMotionLimits& limits,double approach_gain,double position_tolerance);
WheelTrackingCommand trackWheel(const Pose& actual,const Pose& goal,
    const std::vector<TrajectoryPoint>& band,const WheelMotionLimits& limits,
    double control_dt,double approach_gain,double position_tolerance,bool path_tracking,
    double trajectory_elapsed=0);
Pose integrateWheel(Pose pose,double& v,double& w,WheelTrackingCommand command,
    const WheelMotionLimits& limits,double dt);
bool wheelBrakingClear(const Pose& actual,double v,double w,WheelTrackingCommand command,
    const WheelMotionLimits& limits,const PlanRequest& geometry);

struct LocalPlanResult
{
  bool success{false};
  std::vector<TrajectoryPoint> trajectory;
  std::string failure_reason;
  double planning_ms{0.0};
  double optimization_cost{0.0};  // Selected solver cost, before retiming/windowing.
  double duration{0.0};
  double path_length{0.0};
  double max_linear_velocity{0.0};
  double max_angular_velocity{0.0};
  double max_linear_acceleration{0.0};
  double max_angular_acceleration{0.0};
  double max_lateral_velocity{0.0};
  int candidates_evaluated{0};
  // Optional logging only. Controllers must not depend on solver diagnostics.
  std::map<std::string, double> diagnostics;
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

LocalPlanResult runPlanner(WheelLocalPlanner& planner, const LocalPlanRequest& request);

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
  double hip_yaw_min{-1.57}, hip_yaw_max{1.57};
  double wheel_radius{0.0};
  double wheel_width{0.0};
  double wheel_track{0.0};
  double carrier_drop{0.0};
  double lift_min{0.0};
  double lift_max{0.0};
};

struct FootTrajectoryPoint;
struct FootLocalPlanRequest
{
  // Start/goal are ground projections at the foot-mode entry/exit.
  PlanRequest geometry;
  std::vector<Pose> reference_path;
  std::vector<BoxObstacle> terrain;  // Actual solid slabs/steps, not obstacles.
  FootRobotGeometry robot;
  std::optional<FootMotionLimits> limits;
  // Stopped mode-boundary stance; absent only for nominal planning previews.
  std::optional<Pose> current_body;
  std::array<Pose,2> current_feet{};
  std::array<bool,2> supported{true,true};
  bool spatial{false};  // World-frame straight flights and foot-mode platform turns.
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
  std::array<Pose,3> spatial_velocity{}, spatial_acceleration{}; // XYZ and yaw derivatives.
};

struct FootLocalPlanResult
{
  bool success{false};
  std::vector<Footstep> footsteps;
  std::vector<FootTrajectoryPoint> trajectory;
  std::string failure_reason;
  int expanded{0};
  double planning_ms{0.0};
  double validation_ms{0.0};
};

// Shared quintic Hermite polynomial: planner validation and executor use identical math.
double footInterpolate(double p,double end,double v,double ev,double a,double ea,double dt,double u);
FootTrajectoryPoint interpolateFootTrajectory(const FootTrajectoryPoint& a,const FootTrajectoryPoint& b,double u);
void canonicalizeFootCoordinates(std::vector<FootTrajectoryPoint>& samples);
void setSpatialFootDerivatives(std::vector<FootTrajectoryPoint>& samples);

struct FootJointPose {
  std::array<double,4> joints{}; // Hip yaw, shoulder pitch, knee pitch, ankle pitch.
  Pose hip, knee, ankle;
  double upper_pitch{0}, lower_pitch{0};
};
// Shared kinematics for planning validation and the ideal tracking executor.
FootJointPose footInverseKinematics(const Pose& body,const Pose& foot,int side,const FootRobotGeometry& robot);





class FootLocalPlanner : public LocalPlanner
{
public:
  virtual FootLocalPlanResult plan(const FootLocalPlanRequest& request) = 0;
};

FootLocalPlanResult runPlanner(FootLocalPlanner& planner, const FootLocalPlanRequest& request);

std::unique_ptr<WheelLocalPlanner> makeWheelPlanner(const std::string&,const PlannerParameters&);
std::unique_ptr<FootLocalPlanner> makeFootPlanner(const std::string&,const PlannerParameters&);

}  // namespace planning

#endif  // PLANNING_LOCAL_PLANNER_H
