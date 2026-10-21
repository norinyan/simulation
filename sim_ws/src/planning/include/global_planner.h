#ifndef PLANNING_GLOBAL_PLANNER_H
#define PLANNING_GLOBAL_PLANNER_H

#include <string>
#include <vector>
#include <atomic>
#include <chrono>
#include <memory>

namespace planning
{

// A cooperative budget shared by a task's algorithms. The caller also rejects
// late results; cancelling never grants permission to execute an old result.
struct PlanControl
{
  std::chrono::steady_clock::time_point deadline{std::chrono::steady_clock::time_point::max()};
  std::shared_ptr<std::atomic_bool> cancelled;

  std::string failure() const {
    if (cancelled && cancelled->load()) return "PLANNING_CANCELLED";
    if (std::chrono::steady_clock::now() >= deadline) return "PLANNING_TIMEOUT";
    return {};
  }
  double remaining() const {
    return std::chrono::duration<double>(deadline-std::chrono::steady_clock::now()).count();
  }
};

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

// Horizontal oriented solid box, expressed by centre, full XYZ dimensions and yaw.
// Boundary walls can be supplied in the same obstacle collection.
struct BoxObstacle
{
  double center_x{0.0};
  double center_y{0.0};
  double center_z{0.0};
  double size_x{0.0};
  double size_y{0.0};
  double size_z{0.0};
  double yaw{0.0};
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
  PlanControl control;
  // Horizontal support rectangles at start.z. Empty is allowed only for legacy
  // adapters with require_support=false; navigation never infers support from bounds.
  std::vector<BoxObstacle> supports;
  bool require_support{false};
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

// Common caller boundary: catches exceptions, rejects late/malformed paths,
// and validates swept geometry independently of the selected algorithm.
PlanResult runPlanner(GlobalPlanner& planner, const PlanRequest& request);
bool isPoseCollisionFree(const Pose& pose, const PlanRequest& request, double padding = 0);
bool isMotionCollisionFree(const Pose& start, const Pose& end, const PlanRequest& request,
                           double translation_step = .01, double rotation_step = .02);

}  // namespace planning

#endif  // PLANNING_GLOBAL_PLANNER_H
