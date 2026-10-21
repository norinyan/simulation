#include "astar.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace
{
void expect(bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

planning::PlanRequest request()
{
  planning::PlanRequest r;
  r.frame_id = "test";
  r.bounds = {-5, 5, -4, 4};
  r.start = {-3, 0, 0, 0};
  r.goal = {3, 0, 0, 0};
  r.footprint = {.8, .5};
  r.safety_margin = .05;
  return r;
}

struct StubGlobal final : planning::GlobalPlanner
{
  bool called{false},slow{false},cancel{false};
  planning::PlanResult plan(const planning::PlanRequest& r) override {
    called=true;
    if(slow) std::this_thread::sleep_for(std::chrono::milliseconds(15));
    if(cancel && r.control.cancelled) r.control.cancelled->store(true);
    return {true,{r.start,r.goal},{}};
  }
};

// Independent sampled swept-rectangle checks, including terminal docking.
void verify(const planning::PlanRequest& r, const planning::PlanResult& result)
{
  expect(result.success && !result.path.empty(), result.failure_reason.c_str());
  expect(std::abs(result.path.front().x-r.start.x) < 1e-8 &&
         std::abs(result.path.front().y-r.start.y) < 1e-8 &&
         std::abs(result.path.front().yaw-r.start.yaw) < 1e-8, "Start pose changed");
  expect(std::abs(result.path.back().x-r.goal.x) < 1e-8 &&
         std::abs(result.path.back().y-r.goal.y) < 1e-8 &&
         std::abs(result.path.back().yaw-r.goal.yaw) < 1e-8, "Goal pose changed");
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    const auto& a = result.path[i-1];
    const auto& b = result.path[i];
    const double d = std::hypot(b.x-a.x, b.y-a.y);
    const double yaw = d > 1e-9 ? std::atan2(b.y-a.y, b.x-a.x) : a.yaw;
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double hx = r.footprint.length/2, hy = r.footprint.width/2;
    const double ex = std::abs(c)*hx+std::abs(s)*hy, ey = std::abs(s)*hx+std::abs(c)*hy;
    const int samples = std::max(1, static_cast<int>(std::ceil(d/.002)));
    for (int n = 0; n <= samples; ++n) {
      const double t = static_cast<double>(n)/samples;
      const double x = a.x+t*(b.x-a.x), y = a.y+t*(b.y-a.y);
      expect(x-ex >= r.bounds.min_x-1e-8 && x+ex <= r.bounds.max_x+1e-8 &&
             y-ey >= r.bounds.min_y-1e-8 && y+ey <= r.bounds.max_y+1e-8, "Footprint outside map");
      for (const auto& o : r.obstacles) {
        if (o.center_z+o.size_z/2 <= r.start.z+1e-6) continue;
        const double tx = o.center_x-x, ty = o.center_y-y;
        const double ox = o.size_x/2, oy = o.size_y/2;
        const bool separated = std::abs(tx) > ex+ox || std::abs(ty) > ey+oy ||
          std::abs(c*tx+s*ty) > hx+std::abs(c)*ox+std::abs(s)*oy ||
          std::abs(-s*tx+c*ty) > hy+std::abs(s)*ox+std::abs(c)*oy;
        expect(separated, "Returned path intersects a box");
      }
    }
  }
}
}  // namespace

int main()
{
  try {
    planning::AStar astar;
    planning::GlobalPlanner& planner = astar;
    auto r = request();
    verify(r, planner.plan(r));
    r.obstacles = {{0, 0, .5, 1, 2, 1}};
    auto result = planner.plan(r);
    verify(r, result);
    expect(std::any_of(result.path.begin(), result.path.end(), [](const auto& p) {
      return std::abs(p.y) > 1;
    }), "Did not bypass obstacle");

    // Current scene's E cannot hold a circumscribed turning circle; its aligned
    // rectangle fits. This guards against rejecting the existing interface.
    r = request();
    r.bounds = {-6.85, 2, -3.825, 3.825};
    r.start = {-4.5, 0, 0, 0};
    r.goal = {1.6, 0, 0, 0};
    r.footprint = {.54, .78};
    r.obstacles = {{-2, 0, .4, .8, .8, .8}};
    verify(r, planner.plan(r));
    verify(r, planning::runPlanner(planner,r));

    r = request();
    r.bounds = {-5, 5, -5, 5};
    r.goal = {0, 4.36, 0, std::acos(-1.0)/2};
    r.footprint = {1.2, .4};
    r.safety_margin = .02;
    verify(r, planner.plan(r));

    r = request();
    r.obstacles = {{0, 0, .5, .00001, 8, 1}};
    expect(!planner.plan(r).success, "Tunneled through sub-grid barrier");
    r.obstacles = {{-3, 0, .5, 1, 1, 1}};
    expect(!planner.plan(r).success, "Accepted blocked start");
    r.obstacles = {{3, 0, .5, 1, 1, 1}};
    expect(!planner.plan(r).success, "Accepted blocked goal");
    r = request();
    r.goal.z = .34;
    expect(planner.plan(r).failure_reason == "NON_PLANAR_REQUEST", "Accepted non-planar request");
    r = request();
    r.goal.x = 50;
    expect(!planner.plan(r).success, "Accepted goal outside map");
    r = request();
    r.safety_margin = -1;
    expect(!planner.plan(r).success, "Accepted negative margin");
    r = request();
    r.start.x = std::numeric_limits<double>::quiet_NaN();
    expect(!planner.plan(r).success, "Accepted NaN");
    r = request();
    r.obstacles = {{0, 0, .5, -1, 1, 1}};
    expect(!planner.plan(r).success, "Accepted invalid obstacle dimensions");
    r = request();
    r.goal = r.start;
    verify(r, planner.plan(r));
    r = request();
    r.start.z = r.goal.z = 1;
    r.obstacles = {{0, 0, .4, 1, 8, .8}};
    verify(r, planner.plan(r));
    planning::AStar limited({.05, 10});
    expect(limited.plan(request()).failure_reason == "GRID_LIMIT_EXCEEDED", "Grid limit ignored");
    bool invalid_options = false;
    try { planning::AStar invalid({0, 100}); } catch (const std::invalid_argument&) { invalid_options = true; }
    expect(invalid_options, "Accepted zero resolution");
    auto selected=planning::makeGlobalPlanner("astar");
    verify(request(),planning::runPlanner(*selected,request()));
    bool unknown=false;
    try { planning::makeGlobalPlanner("missing"); } catch(const std::invalid_argument&) {unknown=true;}
    expect(unknown,"Unknown algorithm silently fell back");
    StubGlobal stub;
    r=request();
    verify(r,planning::runPlanner(stub,r));
    expect(stub.called,"Global interface did not dispatch");
    r.obstacles={{0,0,.5,1,2,1}};
    expect(planning::runPlanner(stub,r).failure_reason=="INVALID_GLOBAL_GEOMETRY","Trusted unsafe algorithm output");
    r=request();stub.slow=true;
    r.control.deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(2);
    expect(planning::runPlanner(stub,r).failure_reason=="PLANNING_TIMEOUT","Accepted late algorithm result");
    r=request();stub.slow=false;stub.cancel=true;
    r.control.cancelled=std::make_shared<std::atomic_bool>(false);
    expect(planning::runPlanner(stub,r).failure_reason=="PLANNING_CANCELLED","Accepted cancelled result");
    r=request();r.bounds={-3,3,-3,3};r.start={0,-2,0,std::acos(-1.)/2};r.goal={0,2,0,std::acos(-1.)/2};
    r.footprint={.54,.78};r.require_support=true;r.supports={{0,0,-.1,6,6,.2}};
    r.obstacles={{-1.75,0,.5,2.5,.15,1},{1.75,0,.5,2.5,.15,1}};
    expect(planning::runPlanner(planner,r).success,"Rejected physically passable rectangular doorway");
    r.obstacles.clear();r.start={0,0,0,0};r.goal={1,0,0,0};
    r.supports={{-1.505,0,-.1,2.99,6,.2},{1.505,0,-.1,2.99,6,.2}};
    expect(!planning::isPoseCollisionFree(r.start,r),"Missed thin unsupported gap inside footprint");
    r.supports={{-1.5,0,-.1,3,6,.2},{1.5,0,-.1,3,6,.2}};
    expect(planning::isPoseCollisionFree(r.start,r),"Rejected adjacent coplanar support union");
    r.obstacles={{0,0,.5,.2,3,1,std::acos(-1.)/4}};
    expect(!planning::isPoseCollisionFree(r.start,r),"Ignored rotated obstacle");
    std::cout << "PASS: bypass, oriented docking, collision, thin barrier, invalid input, height and grid-limit cases\n";
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
