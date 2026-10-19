#include "astar.h"
#include "teb_optimizer.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

static_assert(std::has_virtual_destructor_v<planning::LocalPlanner>);
static_assert(std::is_base_of_v<planning::LocalPlanner, planning::WheelLocalPlanner>);
static_assert(std::is_base_of_v<planning::LocalPlanner, planning::FootLocalPlanner>);
static_assert(std::is_abstract_v<planning::WheelLocalPlanner>);
static_assert(std::is_abstract_v<planning::FootLocalPlanner>);

namespace
{
void expect(bool ok, const char* text) { if (!ok) throw std::runtime_error(text); }
double wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }

void verify(const planning::LocalPlanRequest& request, const planning::TebOptions& c,
            const planning::LocalPlanResult& result)
{
  expect(result.success,result.failure_reason.c_str());
  expect(result.trajectory.size() >= 2,"Empty/point-only trajectory");
  const auto& first = result.trajectory.front();
  expect(first.time_from_start == 0 && first.linear_velocity == request.initial_linear_velocity &&
         first.angular_velocity == request.initial_angular_velocity,"Wrong initial state");
  expect(std::hypot(first.pose.x-request.geometry.start.x,first.pose.y-request.geometry.start.y)<1e-8,
         "Initial position changed");
  expect(result.duration <= c.time_horizon+1e-6 && result.duration > 0,"Wrong temporal horizon");
  for (std::size_t i = 1; i < result.trajectory.size(); ++i) {
    const auto& a = result.trajectory[i-1]; const auto& b = result.trajectory[i];
    const double dt = b.time_from_start-a.time_from_start;
    expect(dt > 0,"Non-monotonic trajectory times");
    const double mid = a.pose.yaw+wrap(b.pose.yaw-a.pose.yaw)/2;
    const double v = ((b.pose.x-a.pose.x)*std::cos(mid)+(b.pose.y-a.pose.y)*std::sin(mid))/dt;
    const double w = wrap(b.pose.yaw-a.pose.yaw)/dt;
    const double interval = i == 1 ? dt/2 : (b.time_from_start-result.trajectory[i-2].time_from_start)/2;
    expect(std::abs(v-b.linear_velocity)<1e-8 && std::abs(w-b.angular_velocity)<1e-8,"Reported speed differs from trajectory");
    expect(v >= -1e-6 && v <= c.max_v+1e-6 && std::abs(w) <= c.max_w+1e-6,"Speed bound violated");
    expect(std::abs((v-a.linear_velocity)/interval) <= c.max_a+1e-6,"Acceleration bound violated");
    expect(std::abs((w-a.angular_velocity)/interval) <= c.max_alpha+1e-6,"Angular acceleration bound violated");
    expect(std::abs(-(b.pose.x-a.pose.x)*std::sin(mid)+(b.pose.y-a.pose.y)*std::cos(mid))/dt <=
           c.max_lateral_velocity+1e-6,"Nonholonomic lateral limit violated");
  }
}

planning::LocalPlanRequest makeRequest()
{
  planning::LocalPlanRequest r;
  r.geometry.frame_id = "test";
  r.geometry.bounds = {-5,5,-4,4};
  r.geometry.start = {-3,0,0,0};
  r.geometry.goal = {3,0,0,0};
  r.geometry.footprint = {.54,.78};
  r.geometry.safety_margin = .05;
  r.reference_path = {r.geometry.start,r.geometry.goal};
  return r;
}
}  // namespace

int main()
{
  try {
    planning::TebOptions options;
    planning::TebOptimizer teb(options);
    planning::WheelLocalPlanner& optimizer = teb;
    auto r = makeRequest();
    verify(r,options,optimizer.plan(r));
    r.geometry.bounds = {-6.85,2,-3.825,3.825};
    r.geometry.start = {-4.5,0,0,0}; r.geometry.goal = {1.6,0,0,0};
    r.geometry.obstacles = {{-2,0,.4,.8,.8,.8}};
    planning::AStar global;
    auto reference = global.plan(r.geometry);
    expect(reference.success,"A* fixture failed");
    r.reference_path = reference.path;
    verify(r,options,optimizer.plan(r));
    r.geometry.bounds = {2.7,11.35,-3.825,3.825};
    r.geometry.start = {3.3,0,.34,0}; r.geometry.goal = {9,0,.34,0};
    r.geometry.obstacles = {{5.8,0,.74,.8,.8,.8}};
    r.reference_path = global.plan(r.geometry).path;
    verify(r,options,optimizer.plan(r));
    // High-curvature wrapping must not become an artificial +/-2pi turn.
    r = makeRequest(); r.geometry.start.yaw = r.geometry.goal.yaw = std::acos(-1.0);
    r.geometry.start.x = 3; r.geometry.goal.x = -3;
    r.reference_path = {r.geometry.start,r.geometry.goal};
    verify(r,options,optimizer.plan(r));
    r = makeRequest();
    r.geometry.obstacles = {{-3,0,.5,1,1,1}};
    expect(!optimizer.plan(r).success,"Accepted occupied initial footprint");
    r = makeRequest(); r.reference_path.back().z = .17;
    expect(!optimizer.plan(r).success,"Accepted staircase in wheel reference");
    r = makeRequest(); r.initial_linear_velocity = options.max_v+1;
    expect(!optimizer.plan(r).success,"Accepted excessive initial velocity");
    r = makeRequest(); r.reference_path[0].x = std::numeric_limits<double>::quiet_NaN();
    expect(!optimizer.plan(r).success,"Accepted NaN reference");
    r = makeRequest(); r.geometry.safety_margin = -1;
    expect(!optimizer.plan(r).success,"Accepted negative margin");
    r = makeRequest(); r.geometry.bounds.min_y = -.1; r.geometry.bounds.max_y = .1;
    expect(!optimizer.plan(r).success,"Accepted insufficient robot clearance");
    bool invalid = false;
    options.time_horizon = 0;
    try { planning::TebOptimizer bad(options); } catch (const std::invalid_argument&) { invalid = true; }
    expect(invalid,"Accepted zero time horizon");
    std::cout << "PASS: Ceres bands, real wheel sections, temporal/motion bounds, wrapping and invalid inputs\n";
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n'; return 1;
  }
  return 0;
}
