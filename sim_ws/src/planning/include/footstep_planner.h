#ifndef PLANNING_FOOTSTEP_PLANNER_H
#define PLANNING_FOOTSTEP_PLANNER_H

#include "local_planner.h"

namespace planning
{

struct FootstepOptions : FootMotionLimits
{
  double resolution{0.05};
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
