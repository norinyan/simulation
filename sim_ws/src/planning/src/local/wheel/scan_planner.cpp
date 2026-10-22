#include "scan_planner.h"
#include "scan/bspline_opt/bspline_optimizer.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace planning {
namespace {
constexpr double kEpsilon=1e-6;
double angle(double a) { return std::atan2(std::sin(a),std::cos(a)); }
double distance(const Pose& a,const Pose& b) { return std::hypot(a.x-b.x,a.y-b.y); }
void checkBudget(const PlanControl& c) {
  if (const auto reason=c.failure();!reason.empty()) throw std::runtime_error(reason);
}
Eigen::Vector3d vector(const Pose& p) { return {p.x,p.y,p.z}; }

struct Band {
  std::vector<Pose> poses;
  std::vector<double> dt;
  void append(Pose p,double interval) { poses.push_back(p);dt.push_back(interval); }
};

// Use the shared trajectory contract: velocities are interval means and the
// acceleration denominator is the distance between interval midpoints.
std::vector<TrajectoryPoint> timed(const Band& band,const LocalPlanRequest& r) {
  std::vector<TrajectoryPoint> out{{r.geometry.start,0,r.initial_linear_velocity,r.initial_angular_velocity,0,0}};
  for (std::size_t i=0;i<band.dt.size();++i) {
    const auto& a=band.poses[i];const auto& b=band.poses[i+1];
    const double mid=a.yaw+angle(b.yaw-a.yaw)/2,dt=band.dt[i];
    const double v=((b.x-a.x)*std::cos(mid)+(b.y-a.y)*std::sin(mid))/dt;
    const double w=angle(b.yaw-a.yaw)/dt;
    const double interval=i?(band.dt[i-1]+dt)/2:dt/2;
    out.push_back({b,out.back().time_from_start+dt,v,w,
                   (v-out.back().linear_velocity)/interval,(w-out.back().angular_velocity)/interval});
  }
  return out;
}

double requiredTimeScale(const Band& band,const LocalPlanRequest& r) {
  const auto& limits=*r.limits;
  checkBudget(r.geometry.control);
  const auto points=timed(band,r);double scale=1.;
  for (std::size_t i=1;i<points.size();++i) {
    const auto& p=points[i];const auto& a=points[i-1].pose;
    if (!std::isfinite(p.linear_velocity)||!std::isfinite(p.angular_velocity)||p.linear_velocity < -kEpsilon)
      throw std::runtime_error("SCAN_NONFORWARD_OR_NONFINITE_TRAJECTORY");
    const double mid=a.yaw+angle(p.pose.yaw-a.yaw)/2;
    const double lateral=std::abs(-(p.pose.x-a.x)*std::sin(mid)+(p.pose.y-a.y)*std::cos(mid))/band.dt[i-1];
    scale=std::max({scale,p.linear_velocity/limits.max_v,std::abs(p.angular_velocity)/limits.max_w,
      lateral/limits.max_lateral_velocity,std::sqrt(std::abs(p.linear_acceleration)/limits.max_a),
      std::sqrt(std::abs(p.angular_acceleration)/limits.max_alpha)});
  }
  if (!std::isfinite(scale)) throw std::runtime_error("SCAN_NONFINITE_RETIMING");
  return scale;
}

// A position spline cannot represent a stationary wheel heading change. Keep
// that boundary maneuver explicit; its sweep is checked by the common caller.
void turn(Band& band,double yaw,double sample_dt,const WheelMotionLimits& limits,double initial_w=0.,double time_scale=1.) {
  const auto start=band.poses.back();const double delta=angle(yaw-start.yaw);
  if (std::abs(delta)<1e-8&&std::abs(initial_w)<1e-8) return;
  const double duration=time_scale*std::max({sample_dt,2.*std::abs(delta)/limits.max_w,
      std::sqrt(6.*std::abs(delta)/limits.max_alpha),2.*std::abs(initial_w)/limits.max_alpha});
  const int count=std::max(2,int(std::ceil(duration/sample_dt)));
  for (int i=1;i<=count;++i) {
    const double u=double(i)/count;
    auto p=start;
    p.yaw=angle(footInterpolate(start.yaw,start.yaw+delta,initial_w,0,0,0,duration,u));
    band.append(p,duration/count);
  }
}

std::vector<Eigen::Vector3d> localReference(const LocalPlanRequest& r,double spacing) {
  std::vector<Eigen::Vector3d> points{vector(r.geometry.start)};
  const double horizon_length=std::max(.8,r.limits->max_v*r.limits->time_horizon*1.5);
  double length=0;
  for (const auto& p:r.reference_path) {
    const auto end=vector(p),begin=points.back();const double d=(end-begin).norm();
    if (d<1e-8) continue;
    const double use=std::min(d,horizon_length-length);
    const int count=std::max(1,int(std::ceil(use/spacing)));
    for (int i=1;i<=count;++i) points.push_back(begin+(end-begin)*(use/d)*double(i)/count);
    length+=use;
    if (length>=horizon_length-1e-8) break;
  }
  // Cubic fitting needs at least four points and at least one free control point.
  while (points.size()>1&&points.size()<7) {
    std::vector<Eigen::Vector3d> dense{points.front()};
    for (std::size_t i=1;i<points.size();++i) {dense.push_back((points[i-1]+points[i])/2);dense.push_back(points[i]);}
    points=std::move(dense);
  }
  return points;
}

void validate(const LocalPlanRequest& r) {
  if (!r.limits||r.reference_path.size()<2) throw std::invalid_argument("INVALID_SCAN_REQUEST");
  const auto& g=r.geometry;
  auto finite=[](const Pose& p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::isfinite(p.yaw);};
  if (!finite(g.start)||!finite(g.goal)||g.footprint.length<=0||g.footprint.width<=0||
      !std::isfinite(g.safety_margin)||g.safety_margin<0||
      g.bounds.min_x>=g.bounds.max_x||g.bounds.min_y>=g.bounds.max_y)
    throw std::invalid_argument("INVALID_SCAN_GEOMETRY");
  for (const auto& p:r.reference_path)
    if (!finite(p)||std::abs(p.z-g.start.z)>kEpsilon) throw std::invalid_argument("NON_PLANAR_WHEEL_REFERENCE");
  if (std::abs(g.start.z-g.goal.z)>kEpsilon||distance(r.reference_path.front(),g.start)>kEpsilon||
      distance(r.reference_path.back(),g.goal)>kEpsilon||std::abs(angle(r.reference_path.back().yaw-g.goal.yaw))>kEpsilon)
    throw std::invalid_argument("SCAN_REFERENCE_ENDPOINT_MISMATCH");
  for (const auto& o:g.obstacles)
    if (!std::isfinite(o.center_x)||!std::isfinite(o.center_y)||!std::isfinite(o.yaw)||
        !std::isfinite(o.size_x)||!std::isfinite(o.size_y)||o.size_x<=0||o.size_y<=0)
      throw std::invalid_argument("INVALID_SCAN_OBSTACLE");
  if (!isPoseCollisionFree(g.start,g)) throw std::invalid_argument("START_FOOTPRINT_COLLISION");
}
}

ScanLocalPlanner::ScanLocalPlanner(ScanOptions options):options_(options) {
  for (double value:{options.resolution,options.spacing,options.sample_dt,options.smooth_weight,
       options.collision_weight,options.feasibility_weight,options.clearance})
    if (!std::isfinite(value)||value<=0) throw std::invalid_argument("INVALID_SCAN_OPTIONS");
  if (options.iterations<1||options.iterations>1000||options.sample_dt>.1||options.resolution<.01||options.spacing<.03)
    throw std::invalid_argument("INVALID_SCAN_OPTIONS");
}

LocalPlanResult ScanLocalPlanner::plan(const LocalPlanRequest& request) {
  const auto begin=std::chrono::steady_clock::now();LocalPlanResult result;
  try {
    checkBudget(request.geometry.control);validate(request);
    const auto& limits=*request.limits;const auto& g=request.geometry;
    Band band;
    auto points=localReference(request,options_.spacing);
    double time_scale=1.;
    if (points.size()>1) {
      const double heading=std::atan2(points[1].y()-points[0].y(),points[1].x()-points[0].x());
      const bool stationary=request.initial_linear_velocity<1e-8;
      const double v=stationary?0:request.initial_linear_velocity;
      const double yaw=stationary?heading:g.start.yaw,w=stationary?0:request.initial_angular_velocity;
      const Eigen::Vector3d velocity(v*std::cos(yaw),v*std::sin(yaw),0);
      double reference_length=0.;
      for(std::size_t i=1;i<points.size();++i)reference_length+=(points[i]-points[i-1]).norm();
      const double knot_distance=reference_length/(points.size()-1);
      double ts=std::max(.15,knot_distance/limits.max_v);
      // Acceleration is not an observed boundary in LocalPlanRequest. Choose a
      // bounded tangential control toward the seed speed instead of restarting
      // with zero tangential acceleration on every rolling request. Leave room
      // for the cubic's acceleration change between discrete interval midpoints.
      // Short references are densified for fitting, so their actual knot distance
      // must determine both timing and acceleration, including after retiming.
      auto boundaryAcceleration=[&](double interval) {
        const double seed_speed=std::min(limits.max_v,knot_distance/interval);
        const double tangential=.5*std::clamp((seed_speed-v)/interval,-limits.max_a,limits.max_a);
        return Eigen::Vector3d(tangential*std::cos(yaw)-v*w*std::sin(yaw),
                               tangential*std::sin(yaw)+v*w*std::cos(yaw),0);
      };
      auto acceleration=boundaryAcceleration(ts);
      Eigen::MatrixXd controls;
      scan_planner::UniformBspline::parameterizeToBspline(ts,points,
        {velocity,Eigen::Vector3d::Zero(),acceleration,Eigen::Vector3d::Zero()},controls);
      auto map=std::make_shared<GridMap>(g,options_.resolution);
      scan_planner::BsplineOptimizer optimizer;
      optimizer.setParam(options_.smooth_weight,options_.collision_weight,options_.feasibility_weight,
                         options_.clearance,limits.max_v,limits.max_a,options_.iterations);
      optimizer.setStationaryBoundaries(stationary);
      optimizer.setEnvironment(map);optimizer.a_star_=std::make_shared<::AStar>();
      const int pool=std::clamp(int(std::ceil((points.back()-points.front()).norm()/options_.resolution))+40,60,300);
      optimizer.a_star_->initGridMap(map,Eigen::Vector3i(pool,pool,3));
      bool feasible=false;
      for (int timing=0;timing<8;++timing) {
        checkBudget(g.control);
        // Changing knot time changes physical derivatives. Re-pin the measured
        // initial velocity and chosen acceleration at the new time, then let
        // SCAN optimize the remaining control points. Scaling sample times alone
        // would silently slow the initial velocity on every replanning cycle.
        acceleration=boundaryAcceleration(ts);
        controls.col(0)=points.front()-ts*velocity+ts*ts*acceleration/3;
        controls.col(1)=points.front()-ts*ts*acceleration/6;
        controls.col(2)=points.front()+ts*velocity+ts*ts*acceleration/3;
        // At zero speed, velocity/acceleration alone do not constrain the
        // tangent's angular rate. Keep the first span straight after alignment
        // so translation starts with the requested zero angular velocity.
        if(stationary)controls.col(3)=controls.col(2)+knot_distance*Eigen::Vector3d(std::cos(yaw),std::sin(yaw),0);
        for (int i=controls.cols()-3;i<controls.cols();++i) controls.col(i)=points.back();
        // A stopped endpoint has no velocity vector to fix its body heading.
        // Preserve the reference arrival tangent before the explicit final turn;
        // a free penultimate span can otherwise hit a narrow mode-switch edge.
        controls.col(controls.cols()-4)=points.back()-knot_distance*(points.back()-points[points.size()-2]).normalized();
        optimizer.initControlPoints(controls,true);checkBudget(g.control);
        ++result.candidates_evaluated;
        const bool success=optimizer.BsplineOptimizeTrajRebound(controls,ts);
        checkBudget(g.control);
        if (!success) throw std::runtime_error("SCAN_NO_FEASIBLE_SPLINE");
        result.optimization_cost=optimizer.cost();
        band=Band{};band.poses.push_back(g.start);
        if (stationary) turn(band,heading,options_.sample_dt,limits,request.initial_angular_velocity,time_scale);
        scan_planner::UniformBspline spline(controls,3,ts);auto derivative=spline.getDerivative();
        const double duration=spline.getTimeSum();
        const int count=std::max(2,int(std::ceil(duration/options_.sample_dt)));
        if (count+band.poses.size()>20000) throw std::runtime_error("SCAN_SAMPLE_LIMIT");
        for (int i=1;i<=count;++i) {
          if (i%32==0) checkBudget(g.control);
          const double t=duration*double(i)/count;
          const auto p=spline.evaluateDeBoorT(t),d=derivative.evaluateDeBoorT(t);
          const double tangent=d.head<2>().norm()>1e-8?std::atan2(d.y(),d.x()):band.poses.back().yaw;
          band.append({p.x(),p.y(),g.start.z,tangent},duration/count);
        }
        if ((points.back()-vector(g.goal)).norm()<kEpsilon) {
          band.append(band.poses.back(),options_.sample_dt);
          turn(band,g.goal.yaw,options_.sample_dt,limits,0.,time_scale);
        }
        const double scale=requiredTimeScale(band,request);
        if (scale<=1.+kEpsilon) {feasible=true;break;}
        ts*=scale*(1.+1e-6);time_scale*=scale*(1.+1e-6);
      }
      if (!feasible) throw std::runtime_error("SCAN_WHEEL_RETIMING_FAILED");
      result.diagnostics["spline_control_points"]=controls.cols();
      result.diagnostics["time_scale"]=time_scale;
    } else {
      bool feasible=false;
      for (int timing=0;timing<8;++timing) {
        band=Band{};band.poses.push_back(g.start);
        turn(band,g.goal.yaw,options_.sample_dt,limits,request.initial_angular_velocity,time_scale);
        if (band.dt.empty()) band.append(band.poses.back(),options_.sample_dt);
        const double scale=requiredTimeScale(band,request);
        if (scale<=1.+kEpsilon) {feasible=true;break;}
        time_scale*=scale*(1.+1e-6);
      }
      if (!feasible) throw std::runtime_error("SCAN_WHEEL_RETIMING_FAILED");
    }
    // Return only the rolling window; never compress time to fit its deadline.
    auto trajectory=timed(band,request);
    for (const auto& p:trajectory) {
      if (p.time_from_start>limits.time_horizon+1e-6) break;
      result.trajectory.push_back(p);
    }
    if (result.trajectory.size()<2) throw std::runtime_error("SCAN_WINDOW_TOO_SHORT");
    result.duration=result.trajectory.back().time_from_start;
    checkBudget(g.control);result.success=true;
  } catch (const std::exception& e) {
    result.success=false;result.trajectory.clear();result.failure_reason=e.what();
  }
  result.planning_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  return result;
}
}
