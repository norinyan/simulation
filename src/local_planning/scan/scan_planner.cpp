#include "scan_planner.h"
#include <bspline_opt/bspline_optimizer.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <stdexcept>

namespace local_planning {
namespace {
double angle(double a) {return std::atan2(std::sin(a),std::cos(a));}
Eigen::Vector3d position(const geometry_msgs::msg::Pose & p)
{return {p.position.x,p.position.y,p.position.z};}
double yaw(const geometry_msgs::msg::Quaternion & q)
{return std::atan2(2*(q.w*q.z+q.x*q.y),q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z);}
void budget(const LocalRequest & r) {
  if (std::chrono::steady_clock::now()>=r.deadline) {throw std::runtime_error("SCAN_TIMEOUT");}
}
class Environment final : public GridMap {
public:
  Environment(const ScanMap & map,const LocalRequest & r,double margin,double resolution)
  : map_(map),request_(r),margin_(margin),resolution_(resolution) {}
  double getResolution() const override {return resolution_;}
  bool expired() const override {return std::chrono::steady_clock::now()>=request_.deadline;}
  int getInflateOccupancy(const Eigen::Vector3d & p,double) const override {
    budget(request_);
    double ground;
    return map_.clearance(p)<margin_ || !map_.height(p.x(),p.y(),p.z()-margin_,
      request_.limits.reference_height*2,ground);
  }
private:
  const ScanMap & map_;
  const LocalRequest & request_;
  double margin_,resolution_;
};
struct Pose {Eigen::Vector3d p;double heading;};
void appendTurn(std::vector<Pose> & poses,double target) {
  const auto first=poses.back();const double delta=angle(target-first.heading);
  const int count=static_cast<int>(std::ceil(std::abs(delta)/.02));
  for (int i=1;i<=count;++i) {poses.push_back({first.p,first.heading+delta*i/count});}
}
std::vector<Eigen::Vector3d> reference(const LocalRequest & request,double spacing) {
  std::vector<Eigen::Vector3d> route{position(request.state.pose.pose)};
  for (const auto & p:request.segment.points) {
    const Eigen::Vector3d body=position(p)+Eigen::Vector3d(0,0,request.limits.reference_height);
    if ((body-route.back()).norm()>1e-7) {route.push_back(body);}
  }
  if (route.size()<2) {throw std::runtime_error("SCAN_REFERENCE_TOO_SHORT");}
  std::vector<double> lengths{0};
  for (std::size_t i=1;i<route.size();++i) {lengths.push_back(lengths.back()+(route[i]-route[i-1]).norm());}
  const int spans=std::max(8,static_cast<int>(std::ceil(lengths.back()/spacing)));
  if (spans>5000) {throw std::runtime_error("SCAN_REFERENCE_TOO_LONG");}
  std::vector<Eigen::Vector3d> result;std::size_t edge=1;
  for (int i=0;i<=spans;++i) {
    const double s=lengths.back()*i/spans;
    while (edge+1<route.size() && lengths[edge]<s) {++edge;}
    const double t=(s-lengths[edge-1])/(lengths[edge]-lengths[edge-1]);
    result.push_back(route[edge-1]+t*(route[edge]-route[edge-1]));
  }
  return result;
}
}  // namespace

ScanPlanner::ScanPlanner(const YAML::Node & cfg) {
  auto positive=[&](const char * key) {
    const double value=cfg[key].as<double>();
    if (!std::isfinite(value)||value<=0) {throw std::runtime_error(std::string("SCAN_INVALID_")+key);}
    return value;
  };
  margin_=positive("point_clearance");resolution_=positive("resolution");
  spacing_=positive("control_spacing");sample_dt_=positive("sample_dt");
  clearance_=positive("clearance");smooth_weight_=positive("smooth_weight");
  collision_weight_=positive("collision_weight");feasibility_weight_=positive("feasibility_weight");
  iterations_=cfg["iterations"].as<int>();
  if (iterations_<1 || iterations_>1000 || resolution_<.01 || spacing_<resolution_ || sample_dt_>.1) {
    throw std::runtime_error("SCAN_INVALID_DISCRETIZATION_OR_ITERATIONS");
  }
}
void ScanPlanner::loadMap(const std::string & filename) {map_.load(filename);}
bool ScanPlanner::supports(const std::string & mode) const {return mode=="wheel"||mode=="foot";}

LocalResult ScanPlanner::plan(const LocalRequest & request) {
  LocalResult result;
  try {
    budget(request);
    if (request.segment.points.size()<2 || request.segment.modes.size()+1!=request.segment.points.size() ||
      !std::all_of(request.segment.modes.begin(),request.segment.modes.end(),
        [&](const std::string & m){return supports(m);})) {throw std::runtime_error("SCAN_INVALID_SEGMENT");}
    const auto & limits=request.limits;
    const auto & v=request.state.twist.twist;
    if (std::hypot(std::hypot(v.linear.x,v.linear.y),v.linear.z)>1e-3 || std::abs(v.angular.z)>1e-3) {
      throw std::runtime_error("SCAN_REQUIRES_IDLE_START");
    }
    auto points=reference(request,spacing_);
    auto env=std::make_shared<Environment>(map_,request,margin_,resolution_);
    if (env->getInflateOccupancy(points.front(),0)||env->getInflateOccupancy(points.back(),0)) {
      throw std::runtime_error("SCAN_ENDPOINT_COLLISION_OR_NO_TERRAIN");
    }
    double length=0;
    for (std::size_t i=1;i<points.size();++i) {length+=(points[i]-points[i-1]).norm();}
    const double interval=length/(points.size()-1);
    double ts=std::max(sample_dt_,interval/limits.speed);
    Eigen::MatrixXd controls;
    scan_planner::UniformBspline::parameterizeToBspline(ts,points,
      {Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()},controls);
    for (int i=0;i<3;++i) {controls.col(i)=points.front();controls.col(controls.cols()-1-i)=points.back();}
    controls.col(3)=points.front()+interval*(points[1]-points[0]).normalized();
    controls.col(controls.cols()-4)=points.back()-interval*(points.back()-points[points.size()-2]).normalized();
    // Uniform cubic B-spline derivative polygons bound 3D speed/acceleration.
    // Respect fixed terrain Z before optimization, rather than leave an
    // unchangeable Z feasibility penalty at an XY-only optimizer's time scale.
    for (int i=1;i<controls.cols();++i) {
      ts=std::max(ts,(controls.col(i)-controls.col(i-1)).norm()/limits.speed);
    }
    for (int i=2;i<controls.cols();++i) {
      ts=std::max(ts,std::sqrt((controls.col(i)-2*controls.col(i-1)+controls.col(i-2)).norm()/limits.acceleration));
    }
    scan_planner::BsplineOptimizer optimizer;
    scan_planner::BsplineOptimizer::Options options;
    options.lambda_smooth=smooth_weight_;options.lambda_collision=collision_weight_;
    options.lambda_feasibility=feasibility_weight_;options.clearance=clearance_;
    options.max_velocity=limits.speed;options.max_acceleration=limits.acceleration;
    options.max_iterations=iterations_;options.fixed_boundary_points=4;
    optimizer.setParam(options);optimizer.setEnvironment(env);
    optimizer.a_star_=std::make_shared<AStar>();
    Eigen::Vector3d minimum=points.front(),maximum=points.front();
    for (const auto & p:points) {minimum=minimum.cwiseMin(p);maximum=maximum.cwiseMax(p);}
    optimizer.a_star_->initGridMap(env,((maximum-minimum).array()/resolution_+80).ceil().cast<int>());
    optimizer.a_star_->setReference(points);optimizer.initControlPoints(controls,true);
    if (!optimizer.BsplineOptimizeTrajRebound(controls,ts)) {throw std::runtime_error("SCAN_OPTIMIZATION_FAILED");}
    budget(request);
    scan_planner::UniformBspline spline(controls,3,ts);auto derivative=spline.getDerivative();
    const double duration=spline.getTimeSum();
    const int samples=std::max(2,static_cast<int>(std::ceil(duration/sample_dt_)));
    if (samples>100000) {throw std::runtime_error("SCAN_SAMPLE_LIMIT");}
    const auto first=points[1]-points[0],last=points.back()-points[points.size()-2];
    std::vector<Pose> poses{{points.front(),yaw(request.state.pose.pose.orientation)}};
    appendTurn(poses,std::atan2(first.y(),first.x()));
    const std::size_t spline_start=poses.size()-1;
    for (int i=1;i<=samples;++i) {
      budget(request);const double t=duration*i/samples;
      const auto p=spline.evaluateDeBoorT(t),d=derivative.evaluateDeBoorT(t);
      const double heading=i==samples?std::atan2(last.y(),last.x()):
        (d.head<2>().norm()>1e-8?std::atan2(d.y(),d.x()):poses.back().heading);
      poses.push_back({p,heading});
    }
    const std::size_t spline_end=poses.size()-1;
    appendTurn(poses,yaw(request.segment.points.back().orientation));
    if ((poses[spline_end].p-points.back()).norm()>limits.position_tolerance) {
      throw std::runtime_error("SCAN_ENDPOINT_MISMATCH");
    }
    std::vector<double> times(poses.size(),0);double scale=1,min_clearance=std::numeric_limits<double>::infinity();
    Eigen::Vector3d previous=Eigen::Vector3d::Zero();double previous_dt=0;
    double output_length=0;
    for (std::size_t i=1;i<poses.size();++i) {
      budget(request);
      const double dt=i>spline_start&&i<=spline_end?duration/samples:sample_dt_;
      const auto delta=poses[i].p-poses[i-1].p;const Eigen::Vector3d velocity=delta/dt;
      const double theta=std::abs(angle(poses[i].heading-poses[i-1].heading));
      scale=std::max({scale,velocity.norm()/limits.speed,theta/dt/limits.yaw_rate});
      const double accel=(velocity-previous).norm()/(i==1?dt*.5:(dt+previous_dt)*.5);
      scale=std::max(scale,std::sqrt(accel/limits.acceleration));
      previous=velocity;previous_dt=dt;times[i]=times[i-1]+dt;output_length+=delta.norm();
      const double gap=map_.segmentClearance(poses[i-1].p,poses[i].p);
      min_clearance=std::min(min_clearance,gap);
      if (gap<margin_-1e-9) {throw std::runtime_error("SCAN_EXECUTED_SEGMENT_COLLISION at sample "+std::to_string(i));}
      if (env->getInflateOccupancy(poses[i].p,poses[i].heading)) {
        throw std::runtime_error("SCAN_NO_TERRAIN at sample "+std::to_string(i));
      }
    }
    scale=std::max(scale,std::sqrt(previous.norm()/(previous_dt*.5)/limits.acceleration));
    auto & trajectory=result.trajectory;
    trajectory.header.frame_id="world";trajectory.mode=request.segment.modes.front();
    for (std::size_t i=0;i<poses.size();++i) {
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;geometry_msgs::msg::Transform transform;
      transform.translation.x=poses[i].p.x();transform.translation.y=poses[i].p.y();transform.translation.z=poses[i].p.z();
      transform.rotation.z=std::sin(poses[i].heading/2);transform.rotation.w=std::cos(poses[i].heading/2);
      point.transforms.push_back(transform);
      const auto ns=static_cast<int64_t>(std::llround(times[i]*scale*(1+1e-6)*1e9));
      point.time_from_start.sec=static_cast<int32_t>(ns/1000000000);point.time_from_start.nanosec=static_cast<uint32_t>(ns%1000000000);
      trajectory.points.push_back(std::move(point));
    }
    budget(request);result.success=true;
    result.reason="length="+std::to_string(output_length)+" min_clearance="+std::to_string(min_clearance);
  } catch (const std::exception & e) {result.reason=e.what();}
  return result;
}
}  // namespace local_planning
