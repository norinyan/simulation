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
// Search/turns use a circumscribed circle. Short, heading-aligned endpoint
// connectors use the swept rectangle to permit docking near platform edges.
class AStar final : public GlobalPlanner
{
public:
  explicit AStar(AStarOptions options = {});
  PlanResult plan(const PlanRequest& request) override;

private:
  AStarOptions options_;
};

}  // namespace planning

#endif  // PLANNING_ASTAR_H
