#include "scan_planner.h"
#include <bspline_opt/bspline_optimizer.h>
#include <Eigen/QR>
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
  double signedDistance(const Eigen::Vector3d & p,Eigen::Vector3d & gradient) const override {
    budget(request_);return map_.signedDistance(p,gradient);
  }
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
double stamp(const builtin_interfaces::msg::Time & t) {return t.sec+t.nanosec*1e-9;}
double pointTime(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint & p)
{return p.time_from_start.sec+p.time_from_start.nanosec*1e-9;}
Eigen::Vector3d vector(const geometry_msgs::msg::Vector3 & p) {return {p.x,p.y,p.z};}
Eigen::Vector3d oldPosition(const interfaces::msg::LocalTrajectory & tr,double elapsed) {
  if (elapsed>=pointTime(tr.points.back())) {return vector(tr.points.back().transforms[0].translation);}
  const auto end=std::upper_bound(tr.points.begin(),tr.points.end(),elapsed,
    [](double t,const auto & p){return t<pointTime(p);});
  const auto & b=*end;const auto & a=*(end-1);
  const double h=pointTime(b)-pointTime(a),u=(elapsed-pointTime(a))/h;
  return (2*u*u*u-3*u*u+1)*vector(a.transforms[0].translation)+
    (u*u*u-2*u*u+u)*h*vector(a.velocities[0].linear)+
    (-2*u*u*u+3*u*u)*vector(b.transforms[0].translation)+(u*u*u-u*u)*h*vector(b.velocities[0].linear);
}
std::vector<Eigen::Vector3d> reference(const LocalRequest & request,double spacing) {
  std::vector<Eigen::Vector3d> guide{position(request.state.pose.pose)};
  for (const auto & p:request.segment.points) {
    const Eigen::Vector3d body=position(p)+Eigen::Vector3d(0,0,request.limits.reference_height);
    if ((body-guide.back()).norm()>1e-7) {guide.push_back(body);}
  }
  if (guide.size()<2) {throw std::runtime_error("SCAN_REFERENCE_TOO_SHORT");}
  std::vector<double> distance{0};
  for (std::size_t i=1;i<guide.size();++i) {distance.push_back(distance.back()+(guide[i]-guide[i-1]).norm());}
  // As in upstream reboundReplan, reuse the already accepted remaining curve
  // before extending along the new global reference. Never warm-start from a
  // candidate that has not been committed by the execution node.
  std::vector<Eigen::Vector3d> route{guide.front()};
  double used=0;
  const auto & previous=request.warm_start;
  const double elapsed=stamp(request.state.header.stamp)-stamp(previous.header.stamp);
  if (previous.header.frame_id=="world" && previous.points.size()>1 && elapsed>=0 &&
    elapsed<pointTime(previous.points.back())) {
    auto progress=[&](const Eigen::Vector3d & p) {
      double best=std::numeric_limits<double>::infinity(),result=used;
      for (std::size_t i=1;i<guide.size();++i) {
        const Eigen::Vector3d edge=guide[i]-guide[i-1];
        const double ratio=std::clamp((p-guide[i-1]).dot(edge)/edge.squaredNorm(),0.0,1.0);
        const double at=distance[i-1]+ratio*edge.norm();
        if (at+1e-6<used) {continue;}
        const double error=(guide[i-1]+ratio*edge-p).squaredNorm();
        if (error<best) {best=error;result=at;}
      }
      return result;
    };
    for (double t=elapsed+std::min(.05,pointTime(previous.points.back())-elapsed);
      t<=pointTime(previous.points.back())+1e-8;t+=.05) {
      budget(request);const auto p=oldPosition(previous,std::min(t,pointTime(previous.points.back())));
      const double next=progress(p);
      if (next>=distance.back()-spacing) {break;}
      if ((p-route.back()).norm()>1e-7) {route.push_back(p);used=next;}
    }
  }
  for (std::size_t i=1;i<guide.size();++i) {
    if (distance[i]>used+1e-6 && (guide[i]-route.back()).norm()>1e-7) {route.push_back(guide[i]);}
  }
  std::vector<double> lengths{0};
  for (std::size_t i=1;i<route.size();++i) {lengths.push_back(lengths.back()+(route[i]-route[i-1]).norm());}
  const int spans=std::max(8,static_cast<int>(std::ceil(lengths.back()/spacing)));
  if (spans>5000) {throw std::runtime_error("SCAN_REFERENCE_TOO_LONG");}
  std::vector<Eigen::Vector3d> result;std::size_t edge=1;
  for (int i=0;i<=spans;++i) {
    const double at=lengths.back()*i/spans;
    while (edge+1<route.size() && lengths[edge]<at) {++edge;}
    const double t=(at-lengths[edge-1])/(lengths[edge]-lengths[edge-1]);
    result.push_back(route[edge-1]+t*(route[edge]-route[edge-1]));
  }
  return result;
}
Eigen::MatrixXd parameterize(double ts,const std::vector<Eigen::Vector3d> & points,
  const Eigen::Vector3d & velocity,const Eigen::Vector3d & acceleration) {
  // Solve the positional fit with exact p/v/a boundary controls already fixed.
  // Fitting all controls and overwriting the first three afterwards makes the
  // interior inconsistent with those motion conditions during rolling updates.
  const int count=static_cast<int>(points.size()),n=count+2;
  Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(3,n);
  controls.col(0)=points.front()-velocity*ts+acceleration*ts*ts/3;
  controls.col(1)=points.front()-acceleration*ts*ts/6;
  controls.col(2)=points.front()+velocity*ts+acceleration*ts*ts/3;
  for (int i=n-3;i<n;++i) {controls.col(i)=points.back();}
  Eigen::MatrixXd matrix=Eigen::MatrixXd::Zero(count,n-6),rhs(count,3);
  const double weights[3]={1./6,4./6,1./6};
  for (int i=0;i<count;++i) {
    rhs.row(i)=points[i].transpose();
    for (int j=0;j<3;++j) {
      const int col=i+j;
      if (col>=3 && col<n-3) {matrix(i,col-3)=weights[j];}
      else {rhs.row(i)-=weights[j]*controls.col(col).transpose();}
    }
  }
  controls.middleCols(3,n-6)=matrix.colPivHouseholderQr().solve(rhs).transpose();
  return controls;
}
// Maxima of the actual cubic spline, rather than its conservative derivative
// control polygon. A feasible nonzero boundary need not make every *initial*
// optimizable XY control difference feasible.
double motionFactor(scan_planner::UniformBspline & spline,double interval,const MotionLimits & limits,bool z_only) {
  auto velocity=spline.getDerivative(),acceleration=velocity.getDerivative();
  double factor=1;const double duration=spline.getTimeSum();
  for (double t=0;t<duration-1e-9;t+=interval) {
    const double h=std::min(interval,duration-t);
    Eigen::Vector3d v=velocity.evaluateDeBoorT(t),a=acceleration.evaluateDeBoorT(t);
    const Eigen::Vector3d finish=acceleration.evaluateDeBoorT(t+h);
    Eigen::Vector3d jerk=(finish-a)/h;
    if (z_only) {v.head<2>().setZero();a.head<2>().setZero();jerk.head<2>().setZero();}
    auto inspect=[&](double u) {
      factor=std::max(factor,(v+a*u+.5*jerk*u*u).norm()/limits.speed);
    };
    factor=std::max({factor,std::sqrt(a.norm()/limits.acceleration),
      std::sqrt((a+jerk*h).norm()/limits.acceleration)});
    // The derivative of squared speed is cubic. Its monotonic intervals are
    // separated by the quadratic derivative; bisection finds all real roots.
    const double c0=v.dot(a),c1=a.squaredNorm()+v.dot(jerk),c2=1.5*a.dot(jerk),c3=.5*jerk.squaredNorm();
    auto poly=[&](double u){return ((c3*u+c2)*u+c1)*u+c0;};
    std::vector<double> splits{0,h};
    const double disc=4*c2*c2-12*c3*c1;
    if (c3>1e-15 && disc>=0) {
      for (double r:{(-2*c2-std::sqrt(disc))/(6*c3),(-2*c2+std::sqrt(disc))/(6*c3)}) {
        if (r>0 && r<h) {splits.push_back(r);}
      }
    } else if (std::abs(c2)>1e-15) {
      const double r=-c1/(2*c2);if (r>0 && r<h) {splits.push_back(r);}
    }
    std::sort(splits.begin(),splits.end());
    for (std::size_t i=0;i<splits.size();++i) {
      inspect(splits[i]);if (!i || poly(splits[i-1])*poly(splits[i])>=0) {continue;}
      double lo=splits[i-1],hi=splits[i];
      for (int step=0;step<45;++step) {
        const double mid=(lo+hi)/2;if (poly(lo)*poly(mid)<=0) {hi=mid;} else {lo=mid;}
      }
      inspect((lo+hi)/2);
    }
  }
  return factor;
}
void reparameterize(std::vector<Eigen::Vector3d> & points,const Eigen::MatrixXd & controls,double ts) {
  scan_planner::UniformBspline spline(controls,3,ts);const double duration=spline.getTimeSum();
  for (std::size_t i=0;i<points.size();++i) {points[i]=spline.evaluateDeBoorT(duration*i/(points.size()-1));}
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
    const auto & motion=request.state.twist.twist;
    const double initial_yaw=yaw(request.state.pose.pose.orientation);
    const double c=std::cos(initial_yaw),s=std::sin(initial_yaw);
    const Eigen::Vector3d start_velocity(c*motion.linear.x-s*motion.linear.y,
      s*motion.linear.x+c*motion.linear.y,motion.linear.z);
    const Eigen::Vector3d start_acceleration(request.acceleration.x,
      request.acceleration.y,request.acceleration.z);
    const Eigen::Vector3d end_velocity=Eigen::Vector3d::Zero();
    if (!start_velocity.allFinite() || !start_acceleration.allFinite() ||
      start_velocity.norm()>limits.speed+1e-6 ||
      start_acceleration.norm()>limits.acceleration+1e-6 || !std::isfinite(motion.angular.z) ||
      std::abs(motion.angular.z)>limits.yaw_rate+1e-6) {
      throw std::runtime_error("SCAN_INVALID_MOTION_BOUNDARY");
    }
    auto points=reference(request,spacing_);
    auto env=std::make_shared<Environment>(map_,request,margin_,resolution_);
    if (env->getInflateOccupancy(points.front(),0)||env->getInflateOccupancy(points.back(),0)) {
      throw std::runtime_error("SCAN_ENDPOINT_COLLISION_OR_NO_TERRAIN");
    }
    double length=0;
    for (std::size_t i=1;i<points.size();++i) {length+=(points[i]-points[i-1]).norm();}
    const double interval=length/(points.size()-1);
    const double heading_delta=angle(yaw(request.segment.points.back().orientation)-initial_yaw);
    double ts=std::max({sample_dt_,interval/(limits.speed*.5),
      1.5*std::abs(heading_delta)/(limits.yaw_rate*(points.size()-1))});
    Eigen::MatrixXd controls;
    std::string failure="SCAN_MOTION_LIMITS";
    double duration=0;
    bool refining=false;
    // Rebuild the boundary for every new interval. Scaling an already generated
    // trajectory would change its nonzero start velocity and break handover.
    for (int attempt=0;attempt<8;++attempt) {
      budget(request);
      controls=parameterize(ts,points,start_velocity,start_acceleration);
      // Only Z is immutable in the SCAN XY optimizer. Permit its XY initial
      // values to be optimized instead of rejecting them before optimization.
      scan_planner::UniformBspline initial(controls,3,ts);
      const double initial_factor=motionFactor(initial,ts,limits,true);
      if (initial_factor>1+1e-6) {
        reparameterize(points,controls,ts);ts*=initial_factor*1.02;continue;
      }
      scan_planner::BsplineOptimizer optimizer;
      scan_planner::BsplineOptimizer::Options options;
      options.lambda_smooth=smooth_weight_;options.lambda_collision=collision_weight_;
      options.lambda_feasibility=feasibility_weight_;options.clearance=clearance_;
      options.max_velocity=limits.speed;options.max_acceleration=limits.acceleration;
      options.max_iterations=iterations_;options.fixed_boundary_points=3;
      optimizer.setParam(options);optimizer.setEnvironment(env);
      bool optimized;
      if (refining) {
        // Upstream uses a geometry-preserving refinement after allocating more
        // time, rather than searching for a new route with rebound again.
        for (int i=0;i<=controls.cols()-3;++i) {
          optimizer.ref_pts_.push_back(initial.evaluateDeBoorT(i*ts));
        }
        optimized=optimizer.BsplineOptimizeTrajRefine(controls,ts,controls);
      } else {
        optimizer.a_star_=std::make_shared<AStar>();
        Eigen::Vector3d minimum=points.front(),maximum=points.front();
        for (const auto & p:points) {minimum=minimum.cwiseMin(p);maximum=maximum.cwiseMax(p);}
        // With frozen Z, moving freely along a stair changes the body's
        // clearance above its support surface. Retain that height/XY relation.
        if (maximum.z()-minimum.z()>1e-6) {optimizer.ref_pts_=points;}
        optimizer.a_star_->initGridMap(env,((maximum-minimum).array()/resolution_+80).ceil().cast<int>());
        optimizer.a_star_->setReference(points);optimizer.initControlPoints(controls,true);
        optimized=optimizer.BsplineOptimizeTrajRebound(controls,ts);
      }
      if (!optimized) {
        throw std::runtime_error("SCAN_OPTIMIZATION_FAILED");
      }
      scan_planner::UniformBspline spline(controls,3,ts);
      duration=spline.getTimeSum();
      double factor=motionFactor(spline,ts,limits,false);
      // Cubic Hermite yaw is independent of XY heading for this holonomic ideal
      // point model. Its angular speed extrema are endpoints and one vertex.
      const double w0=motion.angular.z,w1=0;
      const double b=3*heading_delta/(duration*duration)-(2*w0+w1)/duration;
      const double a=-2*heading_delta/(duration*duration*duration)+(w0+w1)/(duration*duration);
      double peak=std::max(std::abs(w0),std::abs(w1));
      if (std::abs(a)>1e-12) {
        const double vertex=-b/(3*a);
        if (vertex>0 && vertex<duration) {peak=std::max(peak,std::abs(w0+2*b*vertex+3*a*vertex*vertex));}
      }
      factor=std::max(factor,peak/limits.yaw_rate);
      if (factor>1+1e-6) {
        // Upstream reparamBspline samples the optimized geometry and restores
        // the original boundary derivatives after reallocating time.
        reparameterize(points,controls,ts);ts*=factor*1.02;refining=true;continue;
      }
      auto velocity=spline.getDerivative();auto acceleration=velocity.getDerivative();
      if ((spline.evaluateDeBoorT(0)-points.front()).norm()>1e-7 ||
        (velocity.evaluateDeBoorT(0)-start_velocity).norm()>1e-7 ||
        (acceleration.evaluateDeBoorT(0)-start_acceleration).norm()>1e-6 ||
        (spline.evaluateDeBoorT(duration)-points.back()).norm()>1e-7 ||
        (velocity.evaluateDeBoorT(duration)-end_velocity).norm()>1e-7) {
        throw std::runtime_error("SCAN_MOTION_BOUNDARY_MISMATCH");
      }
      auto & trajectory=result.trajectory;
      trajectory.header.frame_id="world";trajectory.mode=request.segment.modes.front();
      // Include every knot so cubic Hermite interpolation by the executor
      // reproduces each SCAN polynomial, including its derivatives.
      const int spans=controls.cols()-3;
      const int subdivisions=std::max(1,static_cast<int>(std::ceil(ts/std::min(sample_dt_,.02))));
      if (static_cast<long long>(spans)*subdivisions>100000) {throw std::runtime_error("SCAN_SAMPLE_LIMIT");}
      Eigen::Vector3d previous=points.front();double output_length=0;
      double min_clearance=std::numeric_limits<double>::infinity();
      for (int i=0;i<=spans*subdivisions;++i) {
        budget(request);
        const double t=std::min(duration,ts*i/subdivisions);
        const Eigen::Vector3d p=spline.evaluateDeBoorT(t),v=velocity.evaluateDeBoorT(t),acc=acceleration.evaluateDeBoorT(t);
        const double heading=initial_yaw+w0*t+b*t*t+a*t*t*t;
        if (env->getInflateOccupancy(p,heading)) {throw std::runtime_error("SCAN_NO_TERRAIN_OR_COLLISION");}
        if (i) {
          const double dt=ts/subdivisions;
          // A curve with |acceleration| <= A deviates from its chord by at most
          // A*dt^2/8. Include that bound when checking the executed cubic.
          const double gap=map_.segmentClearance(previous,p)-limits.acceleration*dt*dt/8;
          min_clearance=std::min(min_clearance,gap);
          if (gap<margin_-1e-9) {throw std::runtime_error("SCAN_EXECUTED_CURVE_COLLISION at sample "+std::to_string(i));}
          output_length+=(p-previous).norm();
        }
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;geometry_msgs::msg::Transform transform;
        transform.translation.x=p.x();transform.translation.y=p.y();transform.translation.z=p.z();
        transform.rotation.z=std::sin(heading/2);transform.rotation.w=std::cos(heading/2);
        point.transforms.push_back(transform);
        geometry_msgs::msg::Twist derivative,second;
        derivative.linear.x=v.x();derivative.linear.y=v.y();derivative.linear.z=v.z();
        derivative.angular.z=w0+2*b*t+3*a*t*t;
        second.linear.x=acc.x();second.linear.y=acc.y();second.linear.z=acc.z();
        second.angular.z=2*b+6*a*t;
        point.velocities.push_back(derivative);point.accelerations.push_back(second);
        const auto ns=static_cast<int64_t>(std::llround(t*1e9));
        point.time_from_start.sec=static_cast<int32_t>(ns/1000000000);
        point.time_from_start.nanosec=static_cast<uint32_t>(ns%1000000000);
        trajectory.points.push_back(std::move(point));previous=p;
      }
      budget(request);result.success=true;
      result.reason="length="+std::to_string(output_length)+" min_clearance="+std::to_string(min_clearance);
      return result;
    }
    throw std::runtime_error(failure);
  } catch (const std::exception & e) {result.trajectory.points.clear();result.reason=e.what();}
  return result;
}
}  // namespace local_planning
