#ifndef PLANNING_FOOTSTEP_PLANNER_H
#define PLANNING_FOOTSTEP_PLANNER_H

#include "local_planner.h"

namespace planning
{

struct FootstepOptions
{
  double resolution{0.05};
  double max_step{0.50};
  double edge_margin{0.025};
  double swing_clearance{0.08};
  double step_duration{9.0};
  double sample_dt{0.02};
  double max_foot_v{0.8};
  double max_foot_a{3.0};
  double max_body_v{0.3};
  double max_body_a{1.0};
  double max_joint_v{1.0};
  double max_joint_a{20.0};
  double body_height{0.43};
  double wheel_lift{0.13};
  int max_expansions{10000};
};

// Alternating planar footstep A* and C2, lift/translate/lower quintic swings,
// migrated from wheel_leg_demo/foot_planner.hpp. Geometry/kinematics only;
// no execution, feedback, stability or dynamic-balance controller.
class FootstepPlanner final : public FootLocalPlanner
{
public:
  explicit FootstepPlanner(FootstepOptions options = {});
  FootLocalPlanResult plan(const FootLocalPlanRequest& request) override;

private:
  FootstepOptions options_;
};

}  // namespace planning

#endif  // PLANNING_FOOTSTEP_PLANNER_H
