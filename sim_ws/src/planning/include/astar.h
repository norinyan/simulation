#ifndef PLANNING_ASTAR_H
#define PLANNING_ASTAR_H

#include "global_planner.h"

#include <cstddef>

namespace planning
{

struct AStarOptions
{
  double resolution{0.05};
  std::size_t max_grid_cells{2000000};
};

// Plan on one horizontal surface per request. Boxes extending above that surface
// are conservatively projected to XY; this interface has no robot height.
// Legacy planar maps use a circumscribed circle with rectangular docking.
// Maps with finite supports or rotated obstacles use a position/yaw lattice
// and swept rectangular bodies, including in-place turns and floor openings.
class AStar final : public GlobalPlanner
{
public:
  explicit AStar(AStarOptions options = {});
  PlanResult plan(const PlanRequest& request) override;

private:
  AStarOptions options_;
};

std::unique_ptr<GlobalPlanner> makeGlobalPlanner(const std::string& name, AStarOptions options = {});

}  // namespace planning

#endif  // PLANNING_ASTAR_H
