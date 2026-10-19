#ifndef PLANNING_TEB_OPTIMIZER_H
#define PLANNING_TEB_OPTIMIZER_H

#include "local_planner.h"

namespace planning
{

struct TebOptions
{
  double max_v{0.30};
  double max_w{0.80};
  double max_a{0.30};
  double max_alpha{1.0};
  double time_horizon{2.2};  // Seconds, not reference-path arc length.
  double spacing{0.15};
  int iterations{30};
  double solver_time_limit{0.10};
  double max_lateral_velocity{0.01};
  double collision_translation_step{0.01};
  double collision_rotation_step{0.02};
};

// Ceres timed elastic band migrated from wheel_leg_demo. Planning only:
// no ROS/Gazebo, feedback, commands, or execution. The output is a validated
// prediction up to time_horizon, not the result of traversing a whole section.
class TebOptimizer final : public WheelLocalPlanner
{
public:
  explicit TebOptimizer(TebOptions options = {});
  LocalPlanResult plan(const LocalPlanRequest& request) override;

private:
  TebOptions options_;
};

}  // namespace planning

#endif  // PLANNING_TEB_OPTIMIZER_H
