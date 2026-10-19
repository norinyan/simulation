#ifndef PLANNING_GLOBAL_PLANNER_H
#define PLANNING_GLOBAL_PLANNER_H

#include <string>
#include <vector>

namespace planning
{

// All geometry uses metres/radians in the request's coordinate frame.
// Poses describe ground projections, rather than robot base_link heights.
struct Pose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct MapBounds
{
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
};

// Axis-aligned solid box, expressed by its centre and full XYZ dimensions.
// Boundary walls can be supplied in the same obstacle collection.
struct BoxObstacle
{
  double center_x{0.0};
  double center_y{0.0};
  double center_z{0.0};
  double size_x{0.0};
  double size_y{0.0};
  double size_z{0.0};
};

// Full occupied rectangle, centred on the robot reference point.
// Length follows the robot's local X axis, width its local Y axis.
// Includes folded limbs/feet when used for wheel-mode planning.
struct RobotFootprint
{
  double length{0.0};
  double width{0.0};
};

struct PlanRequest
{
  std::string frame_id;
  MapBounds bounds;  // XY planning domain, before footprint/margin erosion.
  std::vector<BoxObstacle> obstacles;
  Pose start;
  Pose goal;
  RobotFootprint footprint;
  double safety_margin{0.0};  // Additional clearance, outside the footprint.
};

struct PlanResult
{
  bool success{false};
  // Ordered ground poses in the request frame; includes both endpoints on success.
  std::vector<Pose> path;
  std::string failure_reason;  // Empty on success; explains invalid input/no path, etc.
};

class GlobalPlanner
{
public:
  virtual ~GlobalPlanner() = default;
  virtual PlanResult plan(const PlanRequest& request) = 0;
};

}  // namespace planning

#endif  // PLANNING_GLOBAL_PLANNER_H
