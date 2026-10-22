#ifndef PLANNING_GLOBAL_PLANNER_H
#define PLANNING_GLOBAL_PLANNER_H

#include <string>
#include <vector>
#include <atomic>
#include <chrono>
#include <memory>
#include <map>

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

// Solid box by centre, full XYZ dimensions and orientation. Wheel maps supply
// horizontal boxes/projections; spatial foot checks also use roll and pitch.
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
  double roll{0.0}, pitch{0.0};  // Used by spatial foot collision checks.
};

// Full occupied rectangle, centred on the robot reference point.
// Length follows the robot's local X axis, width its local Y axis.
// Includes folded limbs/feet when used for wheel-mode planning.
struct RobotFootprint
{
  double length{0.0};
  double width{0.0};
};

enum class TravelMode { Wheel, Foot };

struct SupportRegion {
  std::string id;
  double height{0};
  MapBounds bounds;
  std::vector<BoxObstacle> supports, obstacles;
};

// Directed connections: reference follows actual tread tops and the platform.
struct StairConnection {
  std::string id, from, to;
  std::vector<Pose> reference;
  std::vector<BoxObstacle> treads, platforms;
  bool enabled{true};
};

struct SceneMap {
  std::string frame_id;
  std::vector<BoxObstacle> solids;  // Static World collisions only.
  std::vector<SupportRegion> regions;
  std::vector<StairConnection> connections;
};

struct TraversalConstraints {
  double wheel_height{0.41};
  double body_ground_clearance{0.34}, body_height{0.18};
  double max_step_height{0.17}, min_tread{0.18};
};

struct PathSegment {
  std::size_t begin{0}, end{0};  // Inclusive, adjacent segments share a pose.
  TravelMode mode{TravelMode::Wheel};
  std::string region_id, connection_id;
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
  std::shared_ptr<const SceneMap> map;
  TraversalConstraints traversal;
};

struct PlanResult
{
  bool success{false};
  // Ordered ground poses in the request frame; includes both endpoints on success.
  std::vector<Pose> path;
  std::string failure_reason;  // Empty on success; explains invalid input/no path, etc.
  std::vector<PathSegment> segments;
  double spatial_length{0};
  std::size_t mode_switches{0};
  double algorithm_ms{0}, validation_ms{0};
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
PlanRequest regionRequest(const PlanRequest& request, const SupportRegion& region);
std::string validateGlobalPath(const PlanRequest& request, PlanResult& result);
std::string validateConnection(const PlanRequest& request, const StairConnection& connection);
bool isPoseCollisionFree(const Pose& pose, const PlanRequest& request, double padding = 0);
bool isMotionCollisionFree(const Pose& start, const Pose& end, const PlanRequest& request,
                           double translation_step = .01, double rotation_step = .02);

std::vector<std::string> plannerNames(const std::string& kind);

using PlannerParameters = std::map<std::string,double>;
PlannerParameters plannerDefaults(const std::string& kind,const std::string& name);
std::string plannerParameterPrefix(const std::string& kind,const std::string& name);
std::unique_ptr<GlobalPlanner> makeGlobalPlanner(const std::string&,const PlannerParameters&);

}  // namespace planning

#endif  // PLANNING_GLOBAL_PLANNER_H
