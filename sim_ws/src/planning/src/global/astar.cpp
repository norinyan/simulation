#include "astar.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace planning
{
namespace
{
constexpr double kEpsilon = 1e-9;

bool finite(double value) { return std::isfinite(value); }
double distance(const Pose& a, const Pose& b) { return std::hypot(a.x-b.x, a.y-b.y); }

bool poseFinite(const Pose& p)
{
  return finite(p.x) && finite(p.y) && finite(p.z) && finite(p.yaw);
}

bool intersectsBox(const Pose& a, const Pose& b, const BoxObstacle& box)
{
  double first = 0.0, last = 1.0;
  const std::array<double, 2> start{a.x, a.y}, end{b.x, b.y};
  const std::array<double, 2> center{box.center_x, box.center_y};
  const std::array<double, 2> size{box.size_x, box.size_y};
  for (std::size_t axis = 0; axis < 2; ++axis) {
    const double lo = center[axis]-size[axis]/2, hi = center[axis]+size[axis]/2;
    const double delta = end[axis]-start[axis];
    if (std::abs(delta) < kEpsilon) {
      if (start[axis] < lo || start[axis] > hi) return false;
    } else {
      const double t0 = (lo-start[axis])/delta, t1 = (hi-start[axis])/delta;
      first = std::max(first, std::min(t0, t1));
      last = std::min(last, std::max(t0, t1));
      if (first > last) return false;
    }
  }
  return true;
}

double pointSegmentSquared(double x, double y, const Pose& a, const Pose& b)
{
  const double dx = b.x-a.x, dy = b.y-a.y, length2 = dx*dx+dy*dy;
  const double t = length2 > 0 ? std::clamp(((x-a.x)*dx+(y-a.y)*dy)/length2, 0.0, 1.0) : 0;
  const double ex = x-a.x-t*dx, ey = y-a.y-t*dy;
  return ex*ex+ey*ey;
}

double segmentBoxSquared(const Pose& a, const Pose& b, const BoxObstacle& box)
{
  if (intersectsBox(a, b, box)) return 0.0;
  auto pointBoxSquared = [&](const Pose& p) {
    const double dx = std::max(std::abs(p.x-box.center_x)-box.size_x/2, 0.0);
    const double dy = std::max(std::abs(p.y-box.center_y)-box.size_y/2, 0.0);
    return dx*dx+dy*dy;
  };
  double result = std::min(pointBoxSquared(a), pointBoxSquared(b));
  for (double sx : {-1.0, 1.0}) for (double sy : {-1.0, 1.0}) {
    result = std::min(result, pointSegmentSquared(box.center_x+sx*box.size_x/2,
                                                box.center_y+sy*box.size_y/2, a, b));
  }
  return result;
}

class CollisionChecker
{
public:
  explicit CollisionChecker(const PlanRequest& request)
  : request_(request), radius_(std::hypot(request.footprint.length, request.footprint.width)/2
                               + request.safety_margin) {}

  double radius() const { return radius_; }

  bool circleFree(const Pose& point) const { return straightFree(point, point); }

  bool straightFree(const Pose& a, const Pose& b) const
  {
    const auto& bounds = request_.bounds;
    for (const auto& p : {a, b}) {
      if (p.x-radius_ < bounds.min_x-kEpsilon || p.x+radius_ > bounds.max_x+kEpsilon ||
          p.y-radius_ < bounds.min_y-kEpsilon || p.y+radius_ > bounds.max_y+kEpsilon) return false;
    }
    for (const auto& box : request_.obstacles) {
      if (relevant(box) && segmentBoxSquared(a, b, box) <= radius_*radius_) return false;
    }
    return true;
  }

  // Exact swept OBB for a translation parallel to a fixed robot heading.
  bool dockingFree(const Pose& a, const Pose& b, double yaw) const
  {
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double dx = b.x-a.x, dy = b.y-a.y;
    if (std::abs(-s*dx+c*dy) > 1e-7 || c*dx+s*dy < -kEpsilon) return false;
    const double hx = request_.footprint.length/2+request_.safety_margin+distance(a, b)/2;
    const double hy = request_.footprint.width/2+request_.safety_margin;
    const double x = (a.x+b.x)/2, y = (a.y+b.y)/2;
    const double ex = std::abs(c)*hx+std::abs(s)*hy;
    const double ey = std::abs(s)*hx+std::abs(c)*hy;
    const auto& bounds = request_.bounds;
    if (x-ex < bounds.min_x-kEpsilon || x+ex > bounds.max_x+kEpsilon ||
        y-ey < bounds.min_y-kEpsilon || y+ey > bounds.max_y+kEpsilon) return false;
    for (const auto& box : request_.obstacles) {
      if (!relevant(box)) continue;
      const double ox = box.size_x/2, oy = box.size_y/2;
      const double tx = box.center_x-x, ty = box.center_y-y;
      // Separating axes: world X/Y and robot X/Y. Touching counts as collision.
      if (std::abs(tx) > ex+ox || std::abs(ty) > ey+oy ||
          std::abs(c*tx+s*ty) > hx+std::abs(c)*ox+std::abs(s)*oy ||
          std::abs(-s*tx+c*ty) > hy+std::abs(s)*ox+std::abs(c)*oy) continue;
      return false;
    }
    return true;
  }

private:
  bool relevant(const BoxObstacle& box) const
  {
    return box.center_z+box.size_z/2 > request_.start.z+1e-6;
  }
  const PlanRequest& request_;
  double radius_;
};

bool validate(const PlanRequest& r)
{
  const auto& b = r.bounds;
  if (!poseFinite(r.start) || !poseFinite(r.goal) ||
      !finite(b.min_x) || !finite(b.max_x) || !finite(b.min_y) || !finite(b.max_y) ||
      b.min_x >= b.max_x || b.min_y >= b.max_y ||
      !finite(r.footprint.length) || !finite(r.footprint.width) ||
      r.footprint.length <= 0 || r.footprint.width <= 0 ||
      !finite(r.safety_margin) || r.safety_margin < 0) return false;
  for (const auto& o : r.obstacles) {
    if (!finite(o.center_x) || !finite(o.center_y) || !finite(o.center_z) ||
        !finite(o.size_x) || !finite(o.size_y) || !finite(o.size_z) ||
        o.size_x <= 0 || o.size_y <= 0 || o.size_z <= 0) return false;
  }
  return finite(std::hypot(r.footprint.length, r.footprint.width)/2+r.safety_margin);
}

bool endpointAnchor(const Pose& endpoint, bool is_start, double resolution,
                    const CollisionChecker& checker, Pose& anchor)
{
  if (!checker.dockingFree(endpoint, endpoint, endpoint.yaw)) return false;
  anchor = endpoint;
  if (checker.circleFree(anchor)) return true;
  // Search only the short approach needed to transition to a turning envelope.
  // Bound the loop independently of an excessively small grid resolution.
  const double step = std::max(resolution/2, checker.radius()/128);
  for (double d = step; d <= 2*checker.radius()+step; d += step) {
    const double direction = is_start ? 1.0 : -1.0;
    anchor = endpoint;
    anchor.x += direction*d*std::cos(endpoint.yaw);
    anchor.y += direction*d*std::sin(endpoint.yaw);
    if (checker.circleFree(anchor) &&
        (is_start ? checker.dockingFree(endpoint, anchor, endpoint.yaw)
                  : checker.dockingFree(anchor, endpoint, endpoint.yaw))) return true;
  }
  return false;
}

void appendDistinct(std::vector<Pose>& path, const Pose& point)
{
  if (path.empty() || distance(path.back(), point) > kEpsilon) path.push_back(point);
}
}  // namespace

AStar::AStar(AStarOptions options) : options_(options)
{
  if (!finite(options_.resolution) || options_.resolution <= 0 || options_.max_grid_cells == 0) {
    throw std::invalid_argument("Invalid AStar resolution/grid limit");
  }
}

PlanResult AStar::plan(const PlanRequest& request)
{
  auto failure = [](const std::string& reason) { return PlanResult{false, {}, reason}; };
  if (!validate(request)) return failure("INVALID_REQUEST");
  if (std::abs(request.start.z-request.goal.z) > 1e-6) return failure("NON_PLANAR_REQUEST");
  const CollisionChecker checker(request);
  Pose start, goal;
  if (!endpointAnchor(request.start, true, options_.resolution, checker, start)) {
    return failure("START_COLLISION_OR_NO_ALIGNED_APPROACH");
  }
  if (!endpointAnchor(request.goal, false, options_.resolution, checker, goal)) {
    return failure("GOAL_COLLISION_OR_NO_ALIGNED_APPROACH");
  }
  const auto& bounds = request.bounds;
  const double nx_value = std::floor((bounds.max_x-bounds.min_x)/options_.resolution)+1;
  const double ny_value = std::floor((bounds.max_y-bounds.min_y)/options_.resolution)+1;
  if (!finite(nx_value) || !finite(ny_value) || nx_value < 1 || ny_value < 1 ||
      nx_value > static_cast<double>(options_.max_grid_cells) ||
      ny_value > static_cast<double>(options_.max_grid_cells)/nx_value) {
    return failure("GRID_LIMIT_EXCEEDED");
  }
  const auto nx = static_cast<std::size_t>(nx_value), ny = static_cast<std::size_t>(ny_value);
  const auto count = nx*ny;
  const auto none = std::numeric_limits<std::size_t>::max();
  auto point = [&](std::size_t id) {
    return Pose{bounds.min_x+static_cast<double>(id%nx)*options_.resolution,
                bounds.min_y+static_cast<double>(id/nx)*options_.resolution, start.z, 0};
  };
  auto connector = [&](const Pose& p) {
    const auto ix = std::llround((p.x-bounds.min_x)/options_.resolution);
    const auto iy = std::llround((p.y-bounds.min_y)/options_.resolution);
    std::size_t best = none;
    double best_distance = std::numeric_limits<double>::infinity();
    for (long long dy = -1; dy <= 1; ++dy) for (long long dx = -1; dx <= 1; ++dx) {
      const auto x = ix+dx, y = iy+dy;
      if (x < 0 || y < 0 || x >= static_cast<long long>(nx) || y >= static_cast<long long>(ny)) continue;
      const auto id = static_cast<std::size_t>(y)*nx+static_cast<std::size_t>(x);
      const double d = distance(p, point(id));
      if (d < best_distance && checker.straightFree(p, point(id))) { best = id; best_distance = d; }
    }
    return best;
  };
  const auto source = connector(start), target = connector(goal);
  if (source == none || target == none) return failure("ENDPOINT_GRID_CONNECTOR_BLOCKED");
  std::vector<double> costs(count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parent(count, none);
  std::vector<bool> closed(count, false);
  using Item = std::pair<double, std::size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  costs[source] = 0;
  open.emplace(distance(point(source), point(target)), source);
  while (!open.empty()) {
    const auto id = open.top().second;
    open.pop();
    if (closed[id]) continue;
    closed[id] = true;
    if (id == target) break;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;
      const auto x = static_cast<long long>(id%nx)+dx, y = static_cast<long long>(id/nx)+dy;
      if (x < 0 || y < 0 || x >= static_cast<long long>(nx) || y >= static_cast<long long>(ny)) continue;
      const auto next = static_cast<std::size_t>(y)*nx+static_cast<std::size_t>(x);
      if (closed[next] || !checker.straightFree(point(id), point(next))) continue;
      const double cost = costs[id]+options_.resolution*std::hypot(dx, dy);
      if (cost < costs[next]) {
        costs[next] = cost;
        parent[next] = id;
        open.emplace(cost+distance(point(next), point(target)), next);
      }
    }
  }
  if (!closed[target]) return failure("NO_PATH");
  std::vector<Pose> grid;
  for (auto id = target; id != none; id = parent[id]) grid.push_back(point(id));
  std::reverse(grid.begin(), grid.end());
  std::vector<Pose> raw{start};
  for (const auto& p : grid) appendDistinct(raw, p);
  appendDistinct(raw, goal);
  std::vector<Pose> simplified{raw.front()};
  for (std::size_t i = 0; i+1 < raw.size();) {
    std::size_t j = raw.size()-1;
    while (j > i+1 && !checker.straightFree(raw[i], raw[j])) --j;
    simplified.push_back(raw[j]);
    i = j;
  }
  std::vector<Pose> path{request.start};
  for (const auto& p : simplified) appendDistinct(path, p);
  appendDistinct(path, request.goal);
  for (std::size_t i = 0; i+1 < path.size(); ++i) {
    path[i].yaw = std::atan2(path[i+1].y-path[i].y, path[i+1].x-path[i].x);
  }
  path.front().yaw = request.start.yaw;
  if (path.size() == 1 && std::abs(request.start.yaw-request.goal.yaw) > kEpsilon) {
    path.push_back(request.goal);
  }
  path.back() = request.goal;
  return PlanResult{true, std::move(path), {}};
}

}  // namespace planning
