#include "astar.h"
#include "teb_optimizer.h"
#include "scan_planner.h"
#include "footstep_planner.h"
#include <cmath>
#include <algorithm>
#include <functional>
#include <limits>
#include <stdexcept>
namespace planning { namespace {
std::unique_ptr<GlobalPlanner> buildGlobal(const PlannerParameters& parameters){
  AStarOptions options;
  for(const auto& p:parameters) {
    if(!std::isfinite(p.second))throw std::invalid_argument("NONFINITE_ALGORITHM_PARAMETER: "+p.first);
    if(p.first=="resolution")options.resolution=static_cast<double>(p.second);
    else if(p.first=="max_grid_cells") {if(p.second<1 || p.second>double(std::numeric_limits<std::size_t>::max()/2) || std::floor(p.second)!=p.second)throw std::invalid_argument("INVALID_MAX_GRID_CELLS");options.max_grid_cells=static_cast<std::size_t>(p.second);}
    else throw std::invalid_argument("UNKNOWN_ALGORITHM_PARAMETER: "+p.first);
  }
  return std::make_unique<AStar>(options);
}
std::unique_ptr<WheelLocalPlanner> buildWheel(const PlannerParameters& parameters){
  TebOptions options;
  for(const auto& p:parameters) {
    if(!std::isfinite(p.second))throw std::invalid_argument("NONFINITE_ALGORITHM_PARAMETER: "+p.first);
    if(p.first=="spacing")options.spacing=static_cast<double>(p.second);
    else if(p.first=="iterations") {if(p.second<1 || p.second>double(std::numeric_limits<int>::max()) || std::floor(p.second)!=p.second)throw std::invalid_argument("INVALID_ALGORITHM_COUNT");options.iterations=static_cast<int>(p.second);}
    else if(p.first=="solver_time_limit")options.solver_time_limit=static_cast<double>(p.second);
    else throw std::invalid_argument("UNKNOWN_ALGORITHM_PARAMETER: "+p.first);
  }
  return std::make_unique<TebOptimizer>(options);
}
std::unique_ptr<WheelLocalPlanner> buildScan(const PlannerParameters& parameters){
  ScanOptions options;
  for(const auto& p:parameters) {
    if(!std::isfinite(p.second))throw std::invalid_argument("NONFINITE_ALGORITHM_PARAMETER: "+p.first);
    if(p.first=="resolution")options.resolution=p.second;
    else if(p.first=="spacing")options.spacing=p.second;
    else if(p.first=="sample_dt")options.sample_dt=p.second;
    else if(p.first=="smooth_weight")options.smooth_weight=p.second;
    else if(p.first=="collision_weight")options.collision_weight=p.second;
    else if(p.first=="feasibility_weight")options.feasibility_weight=p.second;
    else if(p.first=="clearance")options.clearance=p.second;
    else if(p.first=="iterations") {
      if(p.second<1||p.second>1000||std::floor(p.second)!=p.second)throw std::invalid_argument("INVALID_SCAN_ITERATIONS");
      options.iterations=static_cast<int>(p.second);
    } else throw std::invalid_argument("UNKNOWN_ALGORITHM_PARAMETER: "+p.first);
  }
  return std::make_unique<ScanLocalPlanner>(options);
}
std::unique_ptr<FootLocalPlanner> buildFoot(const PlannerParameters& parameters){
  FootstepOptions options;
  for(const auto& p:parameters) {
    if(!std::isfinite(p.second))throw std::invalid_argument("NONFINITE_ALGORITHM_PARAMETER: "+p.first);
    if(p.first=="resolution")options.resolution=static_cast<double>(p.second);
    else if(p.first=="max_expansions") {if(p.second<1 || p.second>double(std::numeric_limits<int>::max()) || std::floor(p.second)!=p.second)throw std::invalid_argument("INVALID_ALGORITHM_COUNT");options.max_expansions=static_cast<int>(p.second);}
    else throw std::invalid_argument("UNKNOWN_ALGORITHM_PARAMETER: "+p.first);
  }
  return std::make_unique<FootstepPlanner>(options);
}
double wrap(double x){return std::atan2(std::sin(x),std::cos(x));}
double distance(const Pose& a,const Pose& b){return std::hypot(a.x-b.x,a.y-b.y);}
bool finitePose(const Pose& p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::isfinite(p.yaw);}
template<class T> struct Entry{std::function<std::unique_ptr<T>(const PlannerParameters&)> create;std::string prefix;PlannerParameters defaults;};
const std::map<std::string,Entry<GlobalPlanner>> globalRegistry{{"astar",{buildGlobal,"",{{"resolution",.05},{"max_grid_cells",2000000}}}}};
const std::map<std::string,Entry<WheelLocalPlanner>> wheelRegistry{
  {"teb",{buildWheel,"teb.",{{"spacing",.15},{"iterations",30},{"solver_time_limit",.10}}}},
  {"scan",{buildScan,"scan.",{{"resolution",.05},{"spacing",.15},{"sample_dt",.05},
    {"smooth_weight",1.0},{"collision_weight",1.0},{"feasibility_weight",.1},{"clearance",.10},{"iterations",100}}}}};
const std::map<std::string,Entry<FootLocalPlanner>> footRegistry{{"footstep",{buildFoot,"foot.",{{"resolution",.05},{"max_expansions",10000}}}}};
}
std::unique_ptr<GlobalPlanner> makeGlobalPlanner(const std::string& name,const PlannerParameters& parameters){const auto it=globalRegistry.find(name);if(it==globalRegistry.end())throw std::invalid_argument("Unknown global planner: "+name);return it->second.create(parameters);}
std::unique_ptr<WheelLocalPlanner> makeWheelPlanner(const std::string& name,const PlannerParameters& parameters){const auto it=wheelRegistry.find(name);if(it==wheelRegistry.end())throw std::invalid_argument("Unknown wheel planner: "+name);return it->second.create(parameters);}
std::unique_ptr<FootLocalPlanner> makeFootPlanner(const std::string& name,const PlannerParameters& parameters){const auto it=footRegistry.find(name);if(it==footRegistry.end())throw std::invalid_argument("Unknown foot planner: "+name);return it->second.create(parameters);}
PlannerParameters plannerDefaults(const std::string& kind,const std::string& name){auto names=plannerNames(kind);if(std::find(names.begin(),names.end(),name)==names.end())throw std::invalid_argument("Unknown "+kind+" planner: "+name);if(kind=="global")return globalRegistry.at(name).defaults;if(kind=="wheel")return wheelRegistry.at(name).defaults;if(kind=="foot")return footRegistry.at(name).defaults;throw std::invalid_argument("Unknown planner kind");}
std::string plannerParameterPrefix(const std::string& kind,const std::string& name){if(kind=="global")return globalRegistry.at(name).prefix;if(kind=="wheel")return wheelRegistry.at(name).prefix;if(kind=="foot")return footRegistry.at(name).prefix;throw std::invalid_argument("Unknown planner kind");}
std::vector<std::string> plannerNames(const std::string& kind){std::vector<std::string> names;if(kind=="global")for(const auto& p:globalRegistry)names.push_back(p.first);else if(kind=="wheel")for(const auto& p:wheelRegistry)names.push_back(p.first);else if(kind=="foot")for(const auto& p:footRegistry)names.push_back(p.first);return names;}
LocalPlanResult runPlanner(WheelLocalPlanner& planner, const LocalPlanRequest& request)
{
  LocalPlanResult result;
  try {
    const auto& control=request.geometry.control;
    if(const auto reason=control.failure();!reason.empty()) throw std::runtime_error(reason);
    if(!request.limits) throw std::invalid_argument("WHEEL_MOTION_LIMITS_REQUIRED");
    const auto& c=*request.limits;
    for(double value:{c.max_v,c.max_w,c.max_a,c.max_alpha,c.time_horizon,c.max_lateral_velocity,
                      c.collision_translation_step,c.collision_rotation_step})
      if(!std::isfinite(value)||value<=0) throw std::invalid_argument("INVALID_WHEEL_LIMITS");
    if(!std::isfinite(request.initial_linear_velocity) || !std::isfinite(request.initial_angular_velocity) ||
       request.initial_linear_velocity<0 || request.initial_linear_velocity>c.max_v+1e-6 ||
       std::abs(request.initial_angular_velocity)>c.max_w+1e-6)
      throw std::invalid_argument("INVALID_INITIAL_VELOCITY");
    result=planner.plan(request);
    if(const auto reason=control.failure();!reason.empty()) throw std::runtime_error(reason);
    if(!result.success) throw std::runtime_error(result.failure_reason.empty()?"PLANNER_FAILED":result.failure_reason);
    const auto& points=result.trajectory;
    if(points.size()<2 || points.size()>20000) throw std::runtime_error("INVALID_LOCAL_SAMPLE_COUNT");
    const auto& first=points.front();
    if(!finitePose(first.pose) || distance(first.pose,request.geometry.start)>1e-6 ||
       std::abs(first.pose.z-request.geometry.start.z)>1e-6 ||
       std::abs(wrap(first.pose.yaw-request.geometry.start.yaw))>1e-6 ||
       !std::isfinite(first.linear_acceleration) || !std::isfinite(first.angular_acceleration) ||
       first.time_from_start!=0 || first.linear_velocity!=request.initial_linear_velocity ||
       first.angular_velocity!=request.initial_angular_velocity)
      throw std::runtime_error("INVALID_LOCAL_INITIAL_STATE");
    double path_length=0,max_v=0,max_w=0,max_a=0,max_alpha=0,max_lateral=0;
    for(std::size_t i=1;i<points.size();++i) {
      const auto& a=points[i-1]; const auto& p=points[i];
      const double dt=p.time_from_start-a.time_from_start;
      if(!finitePose(p.pose) || !std::isfinite(dt) || dt<=0 ||
         !std::isfinite(p.linear_velocity) || !std::isfinite(p.angular_velocity) ||
         !std::isfinite(p.linear_acceleration) || !std::isfinite(p.angular_acceleration) ||
         std::abs(p.pose.z-request.geometry.start.z)>1e-6)
        throw std::runtime_error("INVALID_LOCAL_SAMPLE");
      const double mid=a.pose.yaw+wrap(p.pose.yaw-a.pose.yaw)/2;
      const double dx=p.pose.x-a.pose.x,dy=p.pose.y-a.pose.y;
      const double v=(dx*std::cos(mid)+dy*std::sin(mid))/dt;
      const double w=wrap(p.pose.yaw-a.pose.yaw)/dt;
      const double interval=i==1?dt/2:(p.time_from_start-points[i-2].time_from_start)/2;
      const double acc=(v-a.linear_velocity)/interval,alpha=(w-a.angular_velocity)/interval;
      const double lateral=std::abs(-dx*std::sin(mid)+dy*std::cos(mid))/dt;
      if(std::abs(v-p.linear_velocity)>1e-6 || std::abs(w-p.angular_velocity)>1e-6 ||
         std::abs(acc-p.linear_acceleration)>1e-6 || std::abs(alpha-p.angular_acceleration)>1e-6 ||
         v < -1e-6 || v>c.max_v+1e-6 || std::abs(w)>c.max_w+1e-6 ||
         std::abs(acc)>c.max_a+1e-6 || std::abs(alpha)>c.max_alpha+1e-6 ||
         lateral>c.max_lateral_velocity+1e-6 ||
         !isMotionCollisionFree(a.pose,p.pose,request.geometry,c.collision_translation_step,c.collision_rotation_step))
        throw std::runtime_error("INVALID_LOCAL_MOTION");
      path_length+=distance(a.pose,p.pose);
      max_v=std::max(max_v,std::abs(v));max_w=std::max(max_w,std::abs(w));
      max_a=std::max(max_a,std::abs(acc));max_alpha=std::max(max_alpha,std::abs(alpha));
      max_lateral=std::max(max_lateral,lateral);
    }
    if(points.back().time_from_start>c.time_horizon+1e-6) throw std::runtime_error("LOCAL_HORIZON_EXCEEDED");
    if(const auto reason=control.failure();!reason.empty()) throw std::runtime_error(reason);
    result.duration=points.back().time_from_start; result.path_length=path_length;
    result.max_linear_velocity=max_v;result.max_angular_velocity=max_w;
    result.max_linear_acceleration=max_a;result.max_angular_acceleration=max_alpha;
    result.max_lateral_velocity=max_lateral;
  } catch(const std::exception& e) {result={};result.failure_reason=e.what();}
  return result;
}



}
