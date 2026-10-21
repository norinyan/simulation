#include "footstep_planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <utility>

namespace planning
{
namespace
{
constexpr double kEpsilon = 1e-6;
using Sample = FootTrajectoryPoint;

struct Context
{
  const FootLocalPlanRequest& request;
  const FootstepOptions& options;
  double max_rise;
};

bool finitePose(const Pose& p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(p.yaw);
}

// The Demo's planar two-link IK, generalized to the actual segment lengths
// and joint installation angles. Feet remain horizontal at their ankle joints.
std::array<double, 3> footJoints(const Sample& s, int side, const Context& c)
{
  const auto& r = c.request.robot;
  const double dx = s.feet[side].x-r.foot_center_offset_x-s.body.x-r.shoulder_offset[0];
  const double down = s.body.z+r.shoulder_offset[2]-s.feet[side].z;
  const double d = std::hypot(dx, down), a = r.upper_leg_length, b = r.lower_leg_length;
  if (d >= a+b-1e-5 || d <= std::abs(a-b)+1e-5) return {NAN, NAN, NAN};
  const double alpha = std::acos(std::clamp((d*d+a*a-b*b)/(2*d*a), -1.0, 1.0));
  const double beta = std::acos(std::clamp((d*d+b*b-a*a)/(2*d*b), -1.0, 1.0));
  const double theta = std::atan2(-dx, down), upper = theta+alpha, lower = theta-beta;
  return {upper-r.folded_pitch[0], lower-upper-r.folded_pitch[1], -lower-r.folded_pitch[2]};
}

double footY(int side, const Context& c)
{
  return c.request.geometry.start.y+(side == 0 ? 1 : -1)*c.request.robot.shoulder_offset[1];
}

double footGround(double x, int side, const Context& c)
{
  double height = -std::numeric_limits<double>::infinity();
  for (const auto& b : c.request.terrain) {
    if (std::abs(x-b.center_x) <= b.size_x/2+kEpsilon &&
        std::abs(footY(side, c)-b.center_y) <= b.size_y/2+kEpsilon) {
      height = std::max(height, b.center_z+b.size_z/2);
    }
  }
  return height;
}

// Merge coplanar tread/platform tops, then clip away higher terrain. This
// preserves the Demo's edge rule without inventing an edge at the level join
// between the last tread and the upper platform, or using a synthetic stair.
double footEdge(double x, int side, const Context& c)
{
  const double h = footGround(x, side, c), y = footY(side, c);
  const double hy = c.request.robot.foot_size[1]/2+c.options.edge_margin;
  const auto& bounds = c.request.geometry.bounds;
  if (!std::isfinite(h) || y-hy < bounds.min_y || y+hy > bounds.max_y) return -1;
  std::vector<std::pair<double, double>> intervals;
  for (const auto& b : c.request.terrain) {
    if (std::abs(b.center_z+b.size_z/2-h) < kEpsilon &&
        y-hy >= b.center_y-b.size_y/2-kEpsilon && y+hy <= b.center_y+b.size_y/2+kEpsilon) {
      intervals.emplace_back(std::max(bounds.min_x, b.center_x-b.size_x/2),
                             std::min(bounds.max_x, b.center_x+b.size_x/2));
    }
  }
  std::sort(intervals.begin(), intervals.end());
  double lo = 0, hi = 0;
  bool supported = false;
  for (std::size_t i = 0; i < intervals.size();) {
    lo = intervals[i].first; hi = intervals[i++].second;
    while (i < intervals.size() && intervals[i].first <= hi+kEpsilon) hi = std::max(hi, intervals[i++].second);
    if (x >= lo && x <= hi) { supported = true; break; }
  }
  if (!supported) return -1;
  for (const auto& b : c.request.terrain) {
    if (b.center_z+b.size_z/2 <= h+kEpsilon || std::abs(y-b.center_y) >= hy+b.size_y/2) continue;
    const double left = b.center_x-b.size_x/2, right = b.center_x+b.size_x/2;
    if (left > x) hi = std::min(hi, left);
    else if (right < x) lo = std::max(lo, right);
    else return -1;
  }
  return std::min(x-lo, hi-x)-c.request.robot.foot_size[0]/2;
}

bool footLanding(double x, int side, const Context& c)
{
  return footEdge(x, side, c) >= c.options.edge_margin-kEpsilon;
}

struct Part
{
  BoxObstacle box;
  double pitch{0.0};
};

// SAT in XZ plus the independent Y interval, as in the Demo self check.
bool boxHit(const Part& a, const Part& b)
{
  if (std::abs(a.box.center_y-b.box.center_y) >= (a.box.size_y+b.box.size_y)/2-kEpsilon) return false;
  for (double angle : {a.pitch, a.pitch+std::acos(-1.0)/2, b.pitch, b.pitch+std::acos(-1.0)/2}) {
    const double ux = std::cos(angle), uz = -std::sin(angle);
    auto extent = [&](const Part& p) {
      return std::abs(ux*std::cos(p.pitch)-uz*std::sin(p.pitch))*p.box.size_x/2+
             std::abs(ux*std::sin(p.pitch)+uz*std::cos(p.pitch))*p.box.size_z/2;
    };
    if (std::abs((a.box.center_x-b.box.center_x)*ux+(a.box.center_z-b.box.center_z)*uz) >=
        extent(a)+extent(b)-kEpsilon) return false;
  }
  return true;
}

bool footPoseValid(const Sample& s, const Context& c, std::string* why = nullptr)
{
  auto fail = [&](const char* reason) { if (why) *why = reason; return false; };
  if (!finitePose(s.body) || !finitePose(s.feet[0]) || !finitePose(s.feet[1])) return fail("NONFINITE_POSE");
  const auto& r = c.request.robot;
  const double sx = s.body.x+r.shoulder_offset[0], sz = s.body.z+r.shoulder_offset[2];
  std::vector<Part> parts{
    {{s.body.x, s.body.y, s.body.z, r.body_size[0], r.body_size[1], r.body_size[2]}},
    {{sx, s.body.y, sz, r.leg_width, 2*r.shoulder_offset[1], r.leg_width}},
    // Conservative whole wheel/carrier envelope; includes the raised carriage
    // and connector within the wheel radius, without copying old link sizes.
    {{s.body.x, s.body.y, s.body.z-r.carrier_drop+c.options.wheel_lift,
      2*r.wheel_radius, r.wheel_track+r.wheel_width, 2*r.wheel_radius}}
  };
  for (int side = 0; side < 2; ++side) {
    const auto q = footJoints(s, side, c);
    for (double joint : q) {
      if (!std::isfinite(joint)) return fail("IK_UNREACHABLE");
      if (joint < r.joint_min-kEpsilon || joint > r.joint_max+kEpsilon) return fail("JOINT_LIMIT");
    }
    const double a = q[0]+r.folded_pitch[0], b = a+q[1]+r.folded_pitch[1];
    const double ex = sx-r.upper_leg_length*std::sin(a), ez = sz-r.upper_leg_length*std::cos(a);
    parts.push_back({{sx-r.upper_leg_length/2*std::sin(a), footY(side, c),
                      sz-r.upper_leg_length/2*std::cos(a), r.leg_width, r.leg_width, r.upper_leg_length}, a});
    parts.push_back({{ex-r.lower_leg_length/2*std::sin(b), footY(side, c),
                      ez-r.lower_leg_length/2*std::cos(b), r.leg_width, r.leg_width, r.lower_leg_length}, b});
    parts.push_back({{s.feet[side].x, s.feet[side].y, s.feet[side].z,
                      r.foot_size[0], r.foot_size[1], r.foot_size[2]}});
  }
  const auto& bounds = c.request.geometry.bounds;
  for (const auto& p : parts) {
    const double hx = (std::abs(std::cos(p.pitch))*p.box.size_x+std::abs(std::sin(p.pitch))*p.box.size_z)/2;
    if (p.box.center_x-hx < bounds.min_x || p.box.center_x+hx > bounds.max_x ||
        p.box.center_y-p.box.size_y/2 < bounds.min_y || p.box.center_y+p.box.size_y/2 > bounds.max_y) return fail("BOUNDARY");
    for (const auto& boxes : {&c.request.terrain, &c.request.geometry.obstacles}) {
      for (const auto& b : *boxes) if (boxHit(p, {b})) return fail("TERRAIN_OR_OBSTACLE_COLLISION");
    }
  }
  for (std::size_t i = 0; i < parts.size(); ++i) for (std::size_t j = i+1; j < parts.size(); ++j) {
    // Rigid mounts and adjacent joints intentionally intersect at their joints.
    if (i < 3 && j < 3) continue;
    if (i == 1 && (j == 3 || j == 6)) continue;
    if ((i == 3 && j == 4) || (i == 4 && j == 5) ||
        (i == 6 && j == 7) || (i == 7 && j == 8)) continue;
    if (boxHit(parts[i], parts[j])) return fail("SELF_COLLISION");
  }
  return true;
}

double footQ(double u) { return u*u*u*(10+u*(-15+6*u)); }
double footQd(double u) { return 30*u*u*(1-u)*(1-u); }
double footQdd(double u) { return 60*u*(1-u)*(1-2*u); }

std::vector<Sample> footTransition(const Sample& start, int side, double target, const Context& c)
{
  const auto& o = c.options;
  const auto& r = c.request.robot;
  Sample end = start;
  end.feet[side].x = target;
  end.feet[side].z = footGround(target, side, c)+r.foot_size[2]/2;
  end.body.x = (end.feet[0].x+end.feet[1].x)/2-r.shoulder_offset[0]-r.foot_center_offset_x;
  end.body.z = std::min(end.feet[0].z, end.feet[1].z)-r.foot_size[2]/2+o.body_height;
  const double peak = std::max(start.feet[side].z, end.feet[side].z)+o.swing_clearance;
  const int n = std::max(4, 4*static_cast<int>(std::ceil(o.step_duration/o.sample_dt/4)));
  std::vector<Sample> samples;
  samples.reserve(n+1);
  for (int i = 0; i <= n; ++i) {
    const double u = static_cast<double>(i)/n;
    Sample s = start;
    s.velocity = {}; s.acceleration = {};
    s.time_from_start = start.time_from_start+u*o.step_duration;
    auto mix = [](double a, double b, double phase) { return a+(b-a)*footQ(phase); };
    s.body.x = mix(start.body.x, end.body.x, u);
    s.body.z = mix(start.body.z, end.body.z, u);
    if (u < .25) s.feet[side].z = mix(start.feet[side].z, peak, u*4);
    else if (u < .75) {
      s.feet[side].x = mix(start.feet[side].x, target, (u-.25)*2); s.feet[side].z = peak;
    } else {
      s.feet[side].x = target; s.feet[side].z = mix(peak, end.feet[side].z, (u-.75)*4);
    }
    auto derivatives = [&](int axis, double a, double b, double phase, double scale) {
      s.velocity[axis] = (b-a)*footQd(phase)*scale/o.step_duration;
      s.acceleration[axis] = (b-a)*footQdd(phase)*scale*scale/(o.step_duration*o.step_duration);
    };
    derivatives(0, start.body.x, end.body.x, u, 1);
    derivatives(2, start.body.z, end.body.z, u, 1);
    if (u < .25) derivatives(4+side*2, start.feet[side].z, peak, u*4, 4);
    else if (u < .75) derivatives(3+side*2, start.feet[side].x, target, (u-.25)*2, 2);
    else derivatives(4+side*2, peak, end.feet[side].z, (u-.75)*4, 4);
    samples.push_back(s);
  }
  return samples;
}

bool footMotionValid(const std::vector<Sample>& samples, const Context& c, std::string* why = nullptr)
{
  std::array<double, 13> old{}, velocity{};
  double old_time = 0;
  const auto& o = c.options;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if(const auto reason=c.request.geometry.control.failure();!reason.empty()) {
      if(why) *why=reason;
      return false;
    }
    const auto& s = samples[i];
    if (!footPoseValid(s, c, why)) return false;
    const auto l = footJoints(s, 0, c), r = footJoints(s, 1, c);
    const std::array<double, 13> p{s.body.x, s.body.y, s.body.z, s.feet[0].x, s.feet[0].z,
      s.feet[1].x, s.feet[1].z, l[0], l[1], l[2], r[0], r[1], r[2]};
    for (std::size_t k = 0; k < 7; ++k) {
      if (std::abs(s.velocity[k]) > (k < 3 ? o.max_body_v : o.max_foot_v)+kEpsilon ||
          std::abs(s.acceleration[k]) > (k < 3 ? o.max_body_a : o.max_foot_a)+kEpsilon) {
        if (why) *why = "MOTION_LIMIT";
        return false;
      }
    }
    if (i) {
      const double dt = s.time_from_start-old_time;
      if (!(dt > 0)) { if (why) *why = "INVALID_TIME"; return false; }
      for (std::size_t k = 0; k < p.size(); ++k) {
        const double v = (p[k]-old[k])/dt, a = (v-velocity[k])/dt;
        const double vmax = k < 3 ? o.max_body_v : k < 7 ? o.max_foot_v : o.max_joint_v;
        const double amax = k < 3 ? o.max_body_a : k < 7 ? o.max_foot_a : o.max_joint_a;
        if (std::abs(v) > vmax+kEpsilon || (i > 1 && std::abs(a) > amax+kEpsilon)) {
          if (why) *why = "MOTION_LIMIT";
          return false;
        }
        velocity[k] = v;
      }
    }
    old = p; old_time = s.time_from_start;
  }
  return true;
}

void validateRequest(const FootLocalPlanRequest& r, const FootstepOptions& o)
{
  const auto& g = r.geometry; const auto& robot = r.robot; const auto& b = g.bounds;
  if (g.frame_id.empty() || !finitePose(g.start) || !finitePose(g.goal) ||
      !std::isfinite(b.min_x) || !std::isfinite(b.max_x) || !std::isfinite(b.min_y) || !std::isfinite(b.max_y) ||
      b.min_x >= b.max_x || b.min_y >= b.max_y || r.terrain.empty() || r.reference_path.size() < 2) {
    throw std::invalid_argument("INVALID_FOOT_REQUEST");
  }
  if (std::abs(g.start.y-g.goal.y) > kEpsilon || std::abs(g.start.yaw) > kEpsilon ||
      std::abs(g.goal.yaw) > kEpsilon || g.goal.x <= g.start.x) throw std::invalid_argument("PLANAR_FORWARD_STAIRS_REQUIRED");
  for (const auto& p : r.reference_path) {
    if (!finitePose(p) || std::abs(p.y-g.start.y) > kEpsilon || std::abs(p.yaw) > kEpsilon) throw std::invalid_argument("INVALID_STAIR_REFERENCE");
  }
  for (const auto& pair : {std::pair<Pose, Pose>{g.start, r.reference_path.front()}, {g.goal, r.reference_path.back()}}) {
    if (std::abs(pair.first.x-pair.second.x) > kEpsilon || std::abs(pair.first.z-pair.second.z) > kEpsilon) throw std::invalid_argument("STAIR_REFERENCE_ENDPOINT_MISMATCH");
  }
  for (const auto* boxes : {&r.terrain, &g.obstacles}) for (const auto& box : *boxes) {
    for (double v : {box.center_x, box.center_y, box.center_z}) if (!std::isfinite(v)) throw std::invalid_argument("INVALID_BOX");
    for (double v : {box.size_x, box.size_y, box.size_z}) if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("INVALID_BOX");
  }
  for (const auto& size : {robot.body_size, robot.foot_size}) for (double v : size) {
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("INVALID_ROBOT_SIZE");
  }
  for (double v : robot.shoulder_offset) if (!std::isfinite(v)) throw std::invalid_argument("INVALID_SHOULDER");
  for (double v : robot.folded_pitch) if (!std::isfinite(v)) throw std::invalid_argument("INVALID_JOINT_GEOMETRY");
  for (double v : {robot.upper_leg_length, robot.lower_leg_length, robot.leg_width,
                   robot.wheel_radius, robot.wheel_width, robot.wheel_track, robot.carrier_drop}) {
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("INVALID_ROBOT_GEOMETRY");
  }
  if (!std::isfinite(robot.foot_center_offset_x) || !std::isfinite(robot.joint_min) || !std::isfinite(robot.joint_max) ||
      robot.joint_min >= robot.joint_max || robot.shoulder_offset[1] <= 0 ||
      !std::isfinite(robot.lift_min) || !std::isfinite(robot.lift_max) ||
      o.wheel_lift < robot.lift_min || o.wheel_lift > robot.lift_max ||
      o.step_duration/o.sample_dt > 20000 || (g.goal.x-g.start.x)/o.resolution > 10000) {
    throw std::invalid_argument("INVALID_ROBOT_OR_SAMPLING_LIMIT");
  }
}

FootLocalPlanResult planFeet(const FootLocalPlanRequest& request, const FootstepOptions& options)
{
  if(const auto reason=request.geometry.control.failure();!reason.empty()) throw std::runtime_error(reason);
  validateRequest(request, options);
  FootLocalPlanResult out;
  std::vector<double> heights;
  for (const auto& b : request.terrain) heights.push_back(b.center_z+b.size_z/2);
  std::sort(heights.begin(), heights.end());
  double max_rise = 0;
  for (std::size_t i = 1; i < heights.size(); ++i) max_rise = std::max(max_rise, heights[i]-heights[i-1]);
  const Context c{request, options, max_rise};
  const auto& r = request.robot;
  Sample start;
  start.body = request.geometry.start; start.body.z += options.body_height;
  const double offset = r.shoulder_offset[0]+r.foot_center_offset_x;
  for (int side = 0; side < 2; ++side) {
    const double x = start.body.x+offset;
    start.feet[side] = {x, footY(side, c), footGround(x, side, c)+r.foot_size[2]/2, 0};
    if (!footLanding(x, side, c) || std::abs(footGround(x, side, c)-request.geometry.start.z) > kEpsilon) throw std::invalid_argument("INVALID_START_FOOTHOLD");
  }
  std::string reason;
  if (!footPoseValid(start, c, &reason)) throw std::invalid_argument("INVALID_START: "+reason);
  const double goal = request.geometry.goal.x+offset;
  for (int side = 0; side < 2; ++side) {
    if (!footLanding(goal, side, c) || std::abs(footGround(goal, side, c)-request.geometry.goal.z) > kEpsilon) throw std::invalid_argument("INVALID_GOAL_FOOTHOLD");
  }
  std::vector<double> candidates{start.feet[0].x};
  for (double x = start.feet[0].x+options.resolution; x < goal-kEpsilon; x += options.resolution) {
    if (footLanding(x, 0, c) && footLanding(x, 1, c)) candidates.push_back(x);
  }
  candidates.push_back(goal);
  const int count = static_cast<int>(candidates.size()), last = count-1;
  struct State {
    int left, right, next;
    bool operator<(const State& b) const { return std::array<int, 3>{left, right, next} < std::array<int, 3>{b.left, b.right, b.next}; }
  };
  struct Item {
    double f, g; State state;
    bool operator<(const Item& b) const { return f > b.f; }
  };
  std::map<State, double> costs;
  std::map<State, State> parents;
  std::priority_queue<Item> open;
  auto heuristic = [&](State s) { return (2*goal-candidates[s.left]-candidates[s.right])/options.max_step; };
  const State initial{0, 0, 0};
  costs[initial] = 0; open.push({heuristic(initial), 0, initial});
  State finish{}; bool found = false;
  std::string rejected;
  while (!open.empty() && out.expanded < options.max_expansions) {
    if(const auto reason=request.geometry.control.failure();!reason.empty()) throw std::runtime_error(reason);
    const auto item = open.top(); open.pop(); const auto s = item.state;
    if (item.g > costs[s]+1e-9) continue;
    ++out.expanded;
    if (s.left == last && s.right == last) { finish = s; found = true; break; }
    Sample pose = start;
    for (int side = 0; side < 2; ++side) {
      pose.feet[side].x = candidates[side == 0 ? s.left : s.right];
      pose.feet[side].z = footGround(pose.feet[side].x, side, c)+r.foot_size[2]/2;
    }
    pose.body.x = (pose.feet[0].x+pose.feet[1].x)/2-offset;
    pose.body.z = std::min(pose.feet[0].z, pose.feet[1].z)-r.foot_size[2]/2+options.body_height;
    const int old = s.next == 0 ? s.left : s.right, other = s.next == 0 ? s.right : s.left;
    for (int target = old+1; target < count; ++target) {
      const double dx = candidates[target]-candidates[old];
      if (dx > options.max_step+kEpsilon) break;
      if (candidates[target] < candidates[other]-kEpsilon || std::abs(candidates[target]-candidates[other]) > options.max_step) continue;
      const double rise = footGround(candidates[target], s.next, c)-footGround(candidates[old], s.next, c);
      if (rise > c.max_rise+kEpsilon || rise < -kEpsilon) continue;
      if (!footMotionValid(footTransition(pose, s.next, candidates[target], c), c, &rejected)) continue;
      State next = s;
      if (s.next == 0) next.left = target; else next.right = target;
      next.next = 1-s.next;
      const double g = item.g+1+.1*dx;
      if (!costs.count(next) || g < costs[next]) {
        costs[next] = g; parents[next] = s; open.push({g+heuristic(next), g, next});
      }
    }
  }
  if (!found) {
    out.failure_reason = out.expanded >= options.max_expansions ? "FOOT_SEARCH_LIMIT" : "NO_FEASIBLE_FOOTSTEPS";
    if (!rejected.empty()) out.failure_reason += ": "+rejected;
    return out;
  }
  std::vector<State> route{finish};
  while (parents.count(route.back())) route.push_back(parents[route.back()]);
  std::reverse(route.begin(), route.end());
  out.trajectory.push_back(start);
  for (std::size_t i = 1; i < route.size(); ++i) {
    const int side = route[i-1].next;
    const double to = candidates[side == 0 ? route[i].left : route[i].right];
    const auto segment = footTransition(out.trajectory.back(), side, to, c);
    out.footsteps.push_back({side, out.trajectory.back().feet[side], segment.back().feet[side],
                            footEdge(to, side, c), segment.back().time_from_start});
    out.trajectory.insert(out.trajectory.end(), segment.begin()+1, segment.end());
  }
  out.success = footMotionValid(out.trajectory, c, &out.failure_reason);
  if (!out.success) { out.footsteps.clear(); out.trajectory.clear(); }
  return out;
}
}  // namespace

FootstepPlanner::FootstepPlanner(FootstepOptions options) : options_(options)
{
  for (double v : {options.resolution, options.max_step, options.swing_clearance, options.step_duration,
                   options.sample_dt, options.max_foot_v, options.max_foot_a, options.max_body_v,
                   options.max_body_a, options.max_joint_v, options.max_joint_a, options.body_height}) {
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Invalid footstep options");
  }
  if (!std::isfinite(options.edge_margin) || options.edge_margin < 0 ||
      !std::isfinite(options.wheel_lift) || options.wheel_lift < 0 || options.sample_dt > .05 ||
      options.max_expansions <= 0) throw std::invalid_argument("Invalid footstep margins/sampling/search limit");
}

FootLocalPlanResult FootstepPlanner::plan(const FootLocalPlanRequest& request)
{
  const auto begin = std::chrono::steady_clock::now();
  FootLocalPlanResult out;
  try {
    auto options=options_;
    if(request.limits) static_cast<FootMotionLimits&>(options)=*request.limits;
    // Validate shared limits with the same constructor checks as defaults.
    FootstepPlanner checked(options);
    out = planFeet(request, options);
  }
  catch (const std::exception& e) { out.failure_reason = e.what(); }
  out.planning_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-begin).count();
  return out;
}

std::unique_ptr<FootLocalPlanner> makeFootPlanner(const std::string& name, FootstepOptions options)
{
  if(name=="footstep") return std::make_unique<FootstepPlanner>(options);
  throw std::invalid_argument("Unknown foot local planner: "+name);
}

FootLocalPlanResult runPlanner(FootLocalPlanner& planner, const FootLocalPlanRequest& request)
{
  FootLocalPlanResult result;
  try {
    const auto& control=request.geometry.control;
    if(const auto reason=control.failure();!reason.empty()) throw std::runtime_error(reason);
    if(!request.limits) throw std::invalid_argument("FOOT_MOTION_LIMITS_REQUIRED");
    FootstepOptions options;
    static_cast<FootMotionLimits&>(options)=*request.limits;
    FootstepPlanner checked(options);
    validateRequest(request,options);
    result=planner.plan(request);
    if(const auto reason=control.failure();!reason.empty()) throw std::runtime_error(reason);
    if(!result.success) throw std::runtime_error(result.failure_reason.empty()?"PLANNER_FAILED":result.failure_reason);
    if(result.trajectory.size()<2 || result.trajectory.size()>20000)
      throw std::runtime_error("INVALID_FOOT_SAMPLE_COUNT");
    const auto& first=result.trajectory.front(); const auto& last=result.trajectory.back();
    auto matches=[&](const Pose& body,const Pose& ground) {
      return finitePose(body) && std::hypot(body.x-ground.x,body.y-ground.y)<1e-6 &&
        std::abs(body.z-ground.z-options.body_height)<1e-6 && std::abs(body.yaw-ground.yaw)<1e-6;
    };
    if(first.time_from_start!=0 || !matches(first.body,request.geometry.start) || !matches(last.body,request.geometry.goal))
      throw std::runtime_error("INVALID_FOOT_ENDPOINTS");
    const Context context{request,options,0};
    for(std::size_t i=0;i<result.trajectory.size();++i) {
      const auto& sample=result.trajectory[i];
      if(std::abs(sample.body.y-request.geometry.start.y)>1e-6 || std::abs(sample.body.yaw)>1e-6)
        throw std::runtime_error("INVALID_FOOT_BODY_PLANE");
      if(!std::isfinite(sample.time_from_start) ||
         (i && sample.time_from_start-result.trajectory[i-1].time_from_start>.05+1e-9))
        throw std::runtime_error("INVALID_FOOT_TIME");
      for(const auto& values:{sample.velocity,sample.acceleration}) for(double v:values)
        if(!std::isfinite(v)) throw std::runtime_error("NONFINITE_FOOT_DERIVATIVE");
      int supported=0;
      for(int side=0;side<2;++side) {
        const auto& foot=sample.feet[side];
        if(std::abs(foot.y-footY(side,context))>1e-6 || std::abs(foot.yaw)>1e-6)
          throw std::runtime_error("INVALID_FOOT_PLANE");
        if(footLanding(foot.x,side,context) &&
           std::abs(foot.z-request.robot.foot_size[2]/2-footGround(foot.x,side,context))<1e-6) ++supported;
      }
      if(supported==0 || ((i==0 || i+1==result.trajectory.size()) && supported!=2))
        throw std::runtime_error("INVALID_FOOT_SUPPORT");
    }
    std::string reason;
    if(!footMotionValid(result.trajectory,context,&reason)) throw std::runtime_error("INVALID_FOOT_MOTION: "+reason);
    if(const auto reason=control.failure();!reason.empty()) throw std::runtime_error(reason);
  } catch(const std::exception& e) {result={};result.failure_reason=e.what();}
  return result;
}

}  // namespace planning
