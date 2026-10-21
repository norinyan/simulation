#ifndef PLANNING_TEB_OPTIMIZER_H
#define PLANNING_TEB_OPTIMIZER_H

#include "local_planner.h"

namespace planning
{

struct TebOptions : WheelMotionLimits
{
  double spacing{0.15};
  int iterations{30};
  double solver_time_limit{0.10};
};

std::unique_ptr<WheelLocalPlanner> makeWheelPlanner(const std::string& name, TebOptions options = {});

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
