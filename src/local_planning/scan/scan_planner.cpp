#include "scan_planner.h"

#include <bspline_opt/bspline_optimizer.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace local_planning
{
namespace
{
double angle(double a) {return std::atan2(std::sin(a), std::cos(a));}
Eigen::Vector3d position(const geometry_msgs::msg::Pose & p)
{return {p.position.x, p.position.y, p.position.z};}
double yaw(const geometry_msgs::msg::Quaternion & q)
{return std::atan2(2*(q.w*q.z+q.x*q.y), q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z);}
void checkBudget(const LocalRequest & r)
{
  if (std::chrono::steady_clock::now() >= r.deadline) {throw std::runtime_error("SCAN_TIMEOUT");}
}

// SCAN queries this backend during optimization and projected A*. Actual World
// geometry is retained; missing support is occupied, including outside the map.
class Environment final : public GridMap
{
public:
  Environment(const ScanMap & map, const LocalRequest & request,
    double margin, double resolution)
  : map_(map), request_(request), margin_(margin), resolution_(resolution),
    foot_(request.segment.modes.front() == "foot") {}

  double getResolution() const override {return resolution_;}
  bool expired() const override {return std::chrono::steady_clock::now() >= request_.deadline;}
  int getInflateOccupancy(const Eigen::Vector3d & p, double heading) const override
  {
    if (expired()) {throw std::runtime_error("SCAN_TIMEOUT");}
    double ground;
    // Spline control points are not sampled body poses. Requiring nominal
    // standing height or complete foot landings here creates false obstacles
    // when the fixed-Z control polygon overshoots a step. Keep spatial collision
    // and known support here; validate the actual executed curve separately.
    return map_.robotCollision(p, heading, foot_, margin_, request_.limits.geometry_tolerance, false) ||
      !map_.height(p.x(), p.y(), p.z()-0.10, 0.9, ground);
  }
  bool invalidPose(const Eigen::Vector3d & p, double heading) const
  {
    const auto & limits = request_.limits;
    const double height = foot_ ? limits.foot_height : limits.wheel_height;
    const double tolerance = foot_ ? limits.stair_rise+limits.geometry_tolerance : limits.geometry_tolerance;
    // Feet retain their previous contact through a swing. Rebuilding a static
    // stance at each body sample is not the execution model; the complete timed
    // foot trajectory is checked after timing, using the controller's gait.
    if (map_.robotCollision(p, heading, foot_, margin_, limits.geometry_tolerance, !foot_) ||
      !map_.support(p, p.z()-height, tolerance)) {return true;}
    if (!foot_) {
      // Check actual wheel support centres; contact geometry comes from URDF.
      for (double y : {-map_.robotGeometry().wheel_half_track, map_.robotGeometry().wheel_half_track}) {
          const Eigen::Vector3d contact = p + Eigen::Vector3d(
            -y*std::sin(heading), y*std::cos(heading), 0);
          if (!map_.support(contact, p.z()-height, limits.geometry_tolerance)) {return 1;}
      }
    }
    return 0;
  }
private:
  const ScanMap & map_;
  const LocalRequest & request_;
  double margin_;
  double resolution_;
  bool foot_;
};

struct Pose
{
  Eigen::Vector3d p;
  double heading;
};

std::vector<Eigen::Vector3d> bodyReference(const LocalRequest & request, double spacing)
{
  const bool foot = request.segment.modes.front() == "foot";
  const double height = foot ? request.limits.foot_height : request.limits.wheel_height;
  std::vector<Eigen::Vector3d> route;
  auto start = position(request.state.pose.pose);
  start.z() = request.segment.points.front().position.z+height;
  route.push_back(start);
  for (const auto & pose : request.segment.points) {
    Eigen::Vector3d p = position(pose)+Eigen::Vector3d(0, 0, height);
    if (!foot && std::abs(p.z()-start.z()) > request.limits.geometry_tolerance) {
      throw std::runtime_error("SCAN_NON_PLANAR_WHEEL_REFERENCE");
    }
    if ((p-route.back()).norm() > 1e-7) {route.push_back(p);}
  }
  if (route.size() < 2) {throw std::runtime_error("SCAN_REFERENCE_TOO_SHORT");}
  std::vector<double> lengths{0};
  for (std::size_t i=1; i<route.size(); ++i) {lengths.push_back(lengths.back()+(route[i]-route[i-1]).norm());}
  const int spans = std::max(8, static_cast<int>(std::ceil(lengths.back()/spacing)));
  if (spans > 5000) {throw std::runtime_error("SCAN_REFERENCE_TOO_LONG");}
  std::vector<Eigen::Vector3d> points;
  std::size_t edge=1;
  for (int i=0; i<=spans; ++i) {
    const double s=lengths.back()*i/spans;
    while (edge+1<route.size() && lengths[edge]<s) {++edge;}
    const double t=(s-lengths[edge-1])/(lengths[edge]-lengths[edge-1]);
    points.push_back(route[edge-1]+t*(route[edge]-route[edge-1]));
  }
  return points;
}

void appendTurn(std::vector<Pose> & poses, double target)
{
  const double delta=angle(target-poses.back().heading);
  const int count=static_cast<int>(std::ceil(std::abs(delta)/0.02));
  const Pose first=poses.back();
  for (int i=1; i<=count; ++i) {poses.push_back({first.p, first.heading+delta*i/count});}
}

// Match the controller's wheel arc and foot XYZ interpolation when checking
// between published poses; output sampling alone does not prove swept clearance.
void validateSweep(const std::vector<Pose> & poses, const Environment & env,
  const LocalRequest & request, double resolution)
{
  const bool foot=request.segment.modes.front()=="foot";
  for (std::size_t i=0; i<poses.size(); ++i) {
    checkBudget(request);
    if (env.invalidPose(poses[i].p, poses[i].heading)) {
      throw std::runtime_error("SCAN_COLLISION_OR_NO_SUPPORT at sample "+std::to_string(i)+
        " xyz="+std::to_string(poses[i].p.x())+","+std::to_string(poses[i].p.y())+","+
        std::to_string(poses[i].p.z())+" yaw="+std::to_string(poses[i].heading));
    }
    if (i==0) {continue;}
    const auto & a=poses[i-1]; const auto & b=poses[i];
    const double theta=angle(b.heading-a.heading);
    const Eigen::Vector3d delta=b.p-a.p;
    const double forward=delta.x()*std::cos(a.heading+theta/2)+delta.y()*std::sin(a.heading+theta/2);
    const double lateral=-delta.x()*std::sin(a.heading+theta/2)+delta.y()*std::cos(a.heading+theta/2);
    if (!foot && std::abs(lateral)>request.limits.geometry_tolerance) {
      throw std::runtime_error("SCAN_WHEEL_LATERAL_DISPLACEMENT");
    }
    const int samples=std::max(1, static_cast<int>(std::ceil(
      (delta.norm()+0.5*std::abs(theta))/(resolution*0.5))));
    for (int j=1; j<samples; ++j) {
      const double t=static_cast<double>(j)/samples;
      Eigen::Vector3d p=a.p+t*delta;
      if (!foot && std::abs(theta)>1e-7) {
        const double radius=forward/(2*std::sin(theta/2));
        p=a.p+Eigen::Vector3d(radius*(std::sin(a.heading+t*theta)-std::sin(a.heading)),
          radius*(-std::cos(a.heading+t*theta)+std::cos(a.heading)), 0);
      }
      if (env.invalidPose(p, a.heading+t*theta)) {
        throw std::runtime_error("SCAN_SWEPT_COLLISION_OR_NO_SUPPORT");
      }
    }
  }
}
}  // namespace

ScanPlanner::ScanPlanner(const YAML::Node & cfg)
{
  auto positive=[&cfg](const char * key) {
      const double value=cfg[key].as<double>();
      if (!std::isfinite(value) || value<=0) {throw std::runtime_error(std::string("SCAN_INVALID_")+key);}
      return value;
    };
  resolution_=positive("resolution"); spacing_=positive("control_spacing");
  sample_dt_=positive("sample_dt"); acceleration_=positive("max_acceleration");
  clearance_=positive("clearance"); smooth_weight_=positive("smooth_weight");
  collision_weight_=positive("collision_weight"); feasibility_weight_=positive("feasibility_weight");
  iterations_=cfg["iterations"].as<int>();
  if (iterations_<1 || iterations_>1000 || resolution_<0.01 || spacing_<resolution_ || sample_dt_>0.1) {
    throw std::runtime_error("SCAN_INVALID_DISCRETIZATION_OR_ITERATIONS");
  }
  safety_margin_=positive("safety_margin");
  const auto control=YAML::LoadFile(cfg["control_config"].as<std::string>());
  map_.loadRobot(cfg["robot_file"].as<std::string>(), control["foot"]["wheel_lift"].as<double>());
  stair_speed_=control["stair"]["speed"].as<double>();
  swing_lift_=control["foot"]["swing_lift"].as<double>();
  feedback_period_=control["feedback_period"].as<double>();
  if (!std::isfinite(stair_speed_) || stair_speed_<=0) {throw std::runtime_error("SCAN_INVALID_STAIR_SPEED");}
  if (!std::isfinite(swing_lift_) || swing_lift_<=0 || !std::isfinite(feedback_period_) || feedback_period_<=0) {
    throw std::runtime_error("SCAN_INVALID_GAIT_PARAMETERS");
  }
}

void ScanPlanner::loadMap(const std::string & filename) {map_.load(filename);}
bool ScanPlanner::supports(const std::string & mode) const {return mode=="wheel" || mode=="foot";}

LocalResult ScanPlanner::plan(const LocalRequest & request)
{
  LocalResult result;
  try {
    checkBudget(request);
    if (request.segment.points.size()<2 || request.segment.modes.size()+1!=request.segment.points.size() ||
      !supports(request.segment.modes.front()) || !std::all_of(request.segment.modes.begin(),
      request.segment.modes.end(), [&](const std::string & m) {return m==request.segment.modes.front();})) {
      throw std::runtime_error("SCAN_INVALID_SEGMENT");
    }
    // D4 admits an idle controller only. A rolling restart needs an execution
    // replacement contract and is intentionally outside this stage.
    const auto & v=request.state.twist.twist;
    if (std::hypot(std::hypot(v.linear.x,v.linear.y),v.linear.z)>1e-3 ||
      std::hypot(std::hypot(v.angular.x,v.angular.y),v.angular.z)>1e-3) {
      throw std::runtime_error("SCAN_REQUIRES_IDLE_START");
    }
    const bool foot=request.segment.modes.front()=="foot";
    const auto & limits=request.limits;
    // At least three control spans across a foot and a tread, so stair details
    // are not erased by the upstairs reference's original sampling density.
    const double spacing=foot?std::min({spacing_,map_.robotGeometry().foot_length/3,limits.stair_run/3}):spacing_;
    auto points=bodyReference(request, spacing);
    checkBudget(request);
    auto env=std::make_shared<Environment>(map_,request,safety_margin_,resolution_);
    double speed=foot?limits.foot_speed:limits.wheel_speed;
    bool stairs=false;
    for (const auto & p:points) {stairs=stairs || std::abs(p.z()-points.front().z())>limits.geometry_tolerance;}
    if (foot && stairs) {
      // Controller extends a swing by 4.5s per metre of height difference;
      // reserve two alternating swings per tread rather than use flat speed.
      speed=std::min({speed,stair_speed_,limits.stair_run/(2*(limits.swing_duration+4.5*limits.stair_rise))});
    }
    const double heading0=yaw(request.state.pose.pose.orientation);
    const double heading1=yaw(request.segment.points.back().orientation);
    if (env->invalidPose(points.front(),heading0) || env->invalidPose(points.back(),heading1)) {
      throw std::runtime_error("SCAN_ENDPOINT_COLLISION_OR_NO_SUPPORT");
    }
    const auto first_delta=points[1]-points[0], last_delta=points.back()-points[points.size()-2];
    const double initial_tangent=std::atan2(first_delta.y(),first_delta.x());
    const double terminal_tangent=std::atan2(last_delta.y(),last_delta.x());
    double length=0;
    for (std::size_t i=1;i<points.size();++i) {length+=(points[i]-points[i-1]).norm();}
    const double ts=std::max(sample_dt_, length/(points.size()-1)/speed);
    Eigen::MatrixXd controls;
    scan_planner::UniformBspline::parameterizeToBspline(ts,points,
      {Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()},controls);
    for (int i=0;i<3;++i) {controls.col(i)=points.front();controls.col(controls.cols()-1-i)=points.back();}
    const double interval=length/(points.size()-1);
    controls.col(3)=points.front()+interval*first_delta.normalized();
    controls.col(controls.cols()-4)=points.back()-interval*last_delta.normalized();
    scan_planner::BsplineOptimizer optimizer;
    scan_planner::BsplineOptimizer::Options options;
    options.lambda_smooth=smooth_weight_; options.lambda_collision=collision_weight_;
    options.lambda_feasibility=feasibility_weight_; options.clearance=clearance_;
    options.max_velocity=speed; options.max_acceleration=acceleration_;
    options.max_iterations=iterations_; options.fixed_boundary_points=4;
    optimizer.setParam(options); optimizer.setEnvironment(env);
    optimizer.a_star_=std::make_shared<AStar>();
    Eigen::Vector3d minimum=points.front(),maximum=points.front();
    for (const auto & p:points) {minimum=minimum.cwiseMin(p);maximum=maximum.cwiseMax(p);}
    const Eigen::Vector3i pool=((maximum-minimum).array()/resolution_+80).ceil().cast<int>();
    optimizer.a_star_->initGridMap(env,pool);
    optimizer.a_star_->setReference(points);
    optimizer.initControlPoints(controls,true);
    if (!optimizer.BsplineOptimizeTrajRebound(controls,ts)) {throw std::runtime_error("SCAN_OPTIMIZATION_FAILED");}
    checkBudget(request);
    scan_planner::UniformBspline spline(controls,3,ts);
    auto derivative=spline.getDerivative();
    const double duration=spline.getTimeSum();
    const int samples=std::max(2,static_cast<int>(std::ceil(duration/sample_dt_)));
    if (samples>100000) {throw std::runtime_error("SCAN_SAMPLE_LIMIT");}
    std::vector<Pose> poses{{points.front(),heading0}};
    appendTurn(poses,initial_tangent);
    const std::size_t spline_start=poses.size()-1;
    for (int i=1;i<=samples;++i) {
      checkBudget(request);
      const double t=duration*i/samples;
      const Eigen::Vector3d p=spline.evaluateDeBoorT(t), d=derivative.evaluateDeBoorT(t);
      const double heading=i==samples?terminal_tangent:
        (d.head<2>().norm()>1e-8?std::atan2(d.y(),d.x()):poses.back().heading);
      poses.push_back({p,heading});
    }
    const std::size_t spline_end=poses.size()-1;
    appendTurn(poses,heading1);
    if ((poses[spline_end].p-points.back()).norm()>limits.position_tolerance) {
      throw std::runtime_error("SCAN_ENDPOINT_MISMATCH");
    }
    validateSweep(poses,*env,request,resolution_);
    std::vector<double> times(poses.size(),0);
    double scale=1;
    for (std::size_t i=1;i<poses.size();++i) {
      const double dt=i>spline_start && i<=spline_end?duration/samples:sample_dt_;
      const Eigen::Vector3d delta=poses[i].p-poses[i-1].p;
      const double travel=delta.norm();
      const double theta=std::abs(angle(poses[i].heading-poses[i-1].heading));
      const double half=angle(poses[i].heading-poses[i-1].heading)/2;
      const double sinc=std::abs(half)<1e-8?1:std::sin(half)/half;
      const double arc=std::abs((delta.x()*std::cos(poses[i-1].heading+half)+
        delta.y()*std::sin(poses[i-1].heading+half))/sinc);
      double required=std::max((foot?travel:arc)/speed,theta/(foot?limits.foot_yaw_rate:limits.wheel_yaw_rate));
      if (foot) {required=std::max(required,(travel+map_.robotGeometry().hip_radius*theta)*limits.swing_duration/limits.step_budget);}
      else {
        const auto & g=map_.robotGeometry();
        required=std::max(required,(arc+g.wheel_half_track*theta)/(g.wheel_radius*g.wheel_speed_limit));
      }
      scale=std::max(scale,required/dt);
      times[i]=times[i-1]+dt;
    }
    if (foot) {
      std::vector<ScanMap::FootPose> timed;
      timed.reserve(poses.size());
      for (std::size_t i=0;i<poses.size();++i) {
        timed.push_back({poses[i].p,poses[i].heading,times[i]*scale*(1+1e-6)});
      }
      map_.validateFootTrajectory(timed,limits.swing_duration,swing_lift_,feedback_period_,
        safety_margin_,limits.geometry_tolerance,request.deadline);
    }
    auto & trajectory=result.trajectory;
    trajectory.header.frame_id="world";trajectory.mode=request.segment.modes.front();
    for (std::size_t i=0;i<poses.size();++i) {
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
      geometry_msgs::msg::Transform transform;
      const auto & pose=poses[i];
      transform.translation.x=pose.p.x();transform.translation.y=pose.p.y();transform.translation.z=pose.p.z();
      transform.rotation.z=std::sin(pose.heading/2);transform.rotation.w=std::cos(pose.heading/2);
      point.transforms.push_back(transform);
      const auto ns=static_cast<int64_t>(std::llround(times[i]*scale*(1+1e-6)*1e9));
      point.time_from_start.sec=static_cast<int32_t>(ns/1000000000);
      point.time_from_start.nanosec=static_cast<uint32_t>(ns%1000000000);
      trajectory.points.push_back(std::move(point));
    }
    checkBudget(request); result.success=true;
  } catch (const std::exception & e) {result.reason=e.what();}
  return result;
}
}  // namespace local_planning
