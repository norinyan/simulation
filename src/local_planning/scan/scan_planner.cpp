#include "scan_planner.h"
#include <bspline_opt/bspline_optimizer.h>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <cstdio>
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
double stamp(const builtin_interfaces::msg::Time & t) {return t.sec+t.nanosec*1e-9;}
double dynamicDistance(const LocalRequest & request,const Eigen::Vector3d & point,
  double absolute_time,Eigen::Vector3d & gradient) {
  double best=std::numeric_limits<double>::infinity();gradient.setZero();
  if (!request.dynamic.enabled) {return best;}
  for (const auto & obstacle:request.obstacles) {
    const double elapsed=absolute_time-stamp(obstacle.header.stamp);
    if (elapsed < -1e-6 || elapsed>request.dynamic.horizon+1e-6) {
      throw std::runtime_error("DYNAMIC_PREDICTION_HORIZON");
    }
    const auto & q=obstacle.pose.orientation;
    const Eigen::Quaterniond quaternion(q.w,q.x,q.y,q.z);
    const Eigen::Matrix3d rotation=quaternion.normalized().toRotationMatrix();
    const auto & v=obstacle.twist.linear;
    const Eigen::Vector3d center=position(obstacle.pose)+std::max(0.0,elapsed)*Eigen::Vector3d(v.x,v.y,v.z);
    const Eigen::Vector3d local=rotation.transpose()*(point-center);
    const auto & size=obstacle.size;
    const Eigen::Vector3d delta=local.cwiseAbs()-.5*Eigen::Vector3d(size.x,size.y,size.z);
    const Eigen::Vector3d outside=delta.cwiseMax(0);
    const double length=outside.norm();double distance;
    Eigen::Vector3d direction=Eigen::Vector3d::Zero();
    if (length>1e-12) {
      distance=length;
      for (int axis=0;axis<3;++axis) {direction[axis]=outside[axis]/length*(local[axis]>=0?1:-1);}
    } else {
      Eigen::Index axis;distance=delta.maxCoeff(&axis);direction[axis]=local[axis]>=0?1:-1;
    }
    distance-=request.dynamic.base_margin+request.dynamic.growth*std::max(0.0,elapsed);
    if (distance<best) {best=distance;gradient=rotation*direction;}
  }
  return best;
}
class Environment final : public GridMap {
public:
  Environment(const ScanMap & map,const LocalRequest & r,double margin,double resolution)
  : map_(map),request_(r),margin_(margin),resolution_(resolution) {}
  double getResolution() const override {return resolution_;}
  double signedDistance(const Eigen::Vector3d & p,Eigen::Vector3d & gradient) const override {
    budget(request_);return map_.signedDistance(p,gradient);
  }
  double signedDistanceAt(const Eigen::Vector3d & p,double time,Eigen::Vector3d & gradient) const override {
    double result=signedDistance(p,gradient);Eigen::Vector3d dynamic_gradient;
    const double distance=dynamicDistance(request_,p,stamp(request_.state.header.stamp)+time,dynamic_gradient);
    if (distance<result) {result=distance;gradient=dynamic_gradient;}
    return result;
  }
  int getInflateOccupancyAt(const Eigen::Vector3d & p,double heading,double time) const override {
    Eigen::Vector3d gradient;
    return getInflateOccupancy(p,heading) ||
      dynamicDistance(request_,p,stamp(request_.state.header.stamp)+time,gradient)<margin_;
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
double pointTime(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint & p)
{return p.time_from_start.sec+p.time_from_start.nanosec*1e-9;}
Eigen::Vector3d vector(const geometry_msgs::msg::Vector3 & p) {return {p.x,p.y,p.z};}
Eigen::Vector3d oldPosition(const interfaces::msg::LocalTrajectory & tr,double elapsed) {
  if (elapsed<=0) {return vector(tr.points.front().transforms[0].translation);}
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
        if (!controls.allFinite()) {
          throw std::runtime_error("SCAN_REFINE_DIAGNOSTIC_NONFINITE_CONTROLS");
        }
        for (std::size_t i=0;i<optimizer.ref_pts_.size();++i) {
          if (!optimizer.ref_pts_[i].allFinite()) {
            throw std::runtime_error("SCAN_REFINE_DIAGNOSTIC_NONFINITE_REFERENCE");
          }
          if (i>=1 && (optimizer.ref_pts_[i]-optimizer.ref_pts_[i-1]).norm()<1e-9) {
            throw std::runtime_error("SCAN_REFINE_DIAGNOSTIC_DUPLICATE_REFERENCE");
          }
          if (i>=2 && (optimizer.ref_pts_[i]-optimizer.ref_pts_[i-2]).norm()<1e-9) {
            throw std::runtime_error("SCAN_REFINE_DIAGNOSTIC_DEGENERATE_REFERENCE_TANGENT");
          }
        }
        static bool refine_diagnostic_reported=false;
        if (!refine_diagnostic_reported) {
          std::fprintf(stderr,"SCAN_REFINE_DIAGNOSTIC controls=%d refs=%zu ts=%.9g\n",
            static_cast<int>(controls.cols()),optimizer.ref_pts_.size(),ts);
          refine_diagnostic_reported=true;
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
        if (env->getInflateOccupancyAt(p,heading,t)) {throw std::runtime_error("SCAN_NO_TERRAIN_OR_COLLISION");}
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
      trajectory.header.stamp=request.state.header.stamp;
      if (request.dynamic.enabled) {
        std::string reason;
        const double start=stamp(trajectory.header.stamp);
        if (!trajectorySafe(trajectory,request,start,start+duration,reason)) {throw std::runtime_error(reason);}
      }
      budget(request);result.success=true;
      result.reason="length="+std::to_string(output_length)+" min_clearance="+std::to_string(min_clearance);
      return result;
    }
    throw std::runtime_error(failure);
  } catch (const std::exception & e) {result.trajectory.points.clear();result.reason=e.what();}
  return result;
}

bool ScanPlanner::trajectorySafe(const interfaces::msg::LocalTrajectory & trajectory,
  const LocalRequest & request,double from,double until,std::string & reason) const {
  try {
    budget(request);
    if (trajectory.points.empty() || !std::isfinite(from) || !std::isfinite(until) || until<from) {
      throw std::runtime_error("INVALID_SAFETY_INTERVAL");
    }
    const double start=stamp(trajectory.header.stamp);
    if (from<start-1e-6) {throw std::runtime_error("SAFETY_BEFORE_TRAJECTORY_START");}
    double relative_speed=request.limits.speed;
    for (const auto & obstacle:request.obstacles) {
      const auto & v=obstacle.twist.linear;
      relative_speed=std::max(relative_speed,request.limits.speed+Eigen::Vector3d(v.x,v.y,v.z).norm()+request.dynamic.growth);
    }
    const double step=std::min(.02,margin_/(4*std::max(.01,relative_speed)));
    const int count=std::max(1,static_cast<int>(std::ceil((until-from)/step)));
    if (count>100000) {throw std::runtime_error("SAFETY_SAMPLE_LIMIT");}
    Eigen::Vector3d previous=oldPosition(trajectory,from-start),gradient;
    const Environment environment(map_,request,margin_,resolution_);
    for (int i=0;i<=count;++i) {
      budget(request);
      const double time=from+(until-from)*i/count;
      const Eigen::Vector3d p=oldPosition(trajectory,time-start);
      if (environment.getInflateOccupancy(p,0)) {throw std::runtime_error("STATIC_TRAJECTORY_COLLISION_OR_NO_TERRAIN");}
      const double dt=(until-from)/count;
      if (i && map_.segmentClearance(previous,p)-request.limits.acceleration*dt*dt/8<margin_-1e-9) {
        throw std::runtime_error("STATIC_EXECUTED_CURVE_COLLISION");
      }
      const double gap=dynamicDistance(request,p,time,gradient);
      // Signed distance is 1-Lipschitz in relative position. Protect the gap
      // between samples as well as the samples themselves, including holds.
      if (gap-relative_speed*dt*.5<margin_-1e-9) {throw std::runtime_error("DYNAMIC_TRAJECTORY_COLLISION");}
      previous=p;
    }
    return true;
  } catch (const std::exception & error) {reason=error.what();return false;}
}

LocalResult ScanPlanner::brake(const LocalRequest & request) const {
  LocalResult result;
  try {
    budget(request);
    const auto & state=request.state;const double heading=yaw(state.pose.pose.orientation);
    const auto & twist=state.twist.twist;
    const double c=std::cos(heading),s=std::sin(heading);
    const Eigen::Vector3d v(c*twist.linear.x-s*twist.linear.y,s*twist.linear.x+c*twist.linear.y,twist.linear.z);
    const Eigen::Vector3d a=vector(request.acceleration),p=position(state.pose.pose);
    if (!v.allFinite() || !a.allFinite() || v.norm()>request.limits.speed+1e-6 ||
      a.norm()>request.limits.acceleration+1e-6) {throw std::runtime_error("INVALID_BRAKE_BOUNDARY");}
    // |v|/A is a physical lower bound; search upward instead of starting at
    // 3|v|/A, which doubles the required stopping distance when a(0)=0.
    double duration=std::max(.05,v.norm()/request.limits.acceleration);
    // Cubic velocity Bernstein controls preserve the initial acceleration and
    // end with zero velocity/acceleration. Their convex hull bounds motion.
    bool feasible=false;
    auto bounded=[](std::vector<Eigen::Vector3d> controls,double limit,auto && self,int depth)->bool {
      double peak=0;for (const auto & control:controls) {peak=std::max(peak,control.norm());}
      if (peak<=limit+1e-6) {return true;}
      if (depth==0 || controls.front().norm()>limit+1e-6 || controls.back().norm()>limit+1e-6) {return false;}
      // Subdivision tightens the Bernstein convex-hull bound without confusing
      // a conservative control polygon violation with an actual motion limit.
      std::vector<Eigen::Vector3d> left{controls.front()},right{controls.back()};
      while (controls.size()>1) {
        for (std::size_t i=0;i+1<controls.size();++i) {controls[i]=.5*(controls[i]+controls[i+1]);}
        controls.pop_back();left.push_back(controls.front());right.push_back(controls.back());
      }
      std::reverse(right.begin(),right.end());
      return self(left,limit,self,depth-1) && self(right,limit,self,depth-1);
    };
    auto motionFeasible=[&](double time) {
      const Eigen::Vector3d middle=v+a*time/3;
      if (!bounded({v,middle,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()},request.limits.speed,bounded,8) ||
        !bounded({a,-3*v/time-a,Eigen::Vector3d::Zero()},request.limits.acceleration,bounded,8)) {return false;}
      // Check the actual Hermite execution during the duration search too.
      // A feasible generating quartic alone does not bound its interpolant.
      const Eigen::Vector3d b=-3*v/(time*time)-2*a/time;
      const Eigen::Vector3d d=2*v/(time*time*time)+a/(time*time);
      const int count=std::max(1,static_cast<int>(std::ceil(time/.02)));
      auto positionAt=[&](double t)->Eigen::Vector3d {return v*t+.5*a*t*t+b*t*t*t/3+d*t*t*t*t/4;};
      auto velocityAt=[&](double t)->Eigen::Vector3d {return v+a*t+b*t*t+d*t*t*t;};
      for (int i=0;i<count;++i) {
        const double lo=time*i/count,hi=time*(i+1)/count,h=hi-lo;
        const Eigen::Vector3d va=velocityAt(lo),vb=velocityAt(hi);
        const Eigen::Vector3d mid=3*(positionAt(hi)-positionAt(lo))/h-va-vb;
        if (!bounded({va,mid,vb},request.limits.speed,bounded,8) ||
          std::max((2*(mid-va)/h).norm(),(2*(vb-mid)/h).norm())>request.limits.acceleration+1e-6) {return false;}
      }
      return true;
    };
    double rejected=duration;
    for (int attempt=0;attempt<32;++attempt) {
      budget(request);
      if (motionFeasible(duration)) {feasible=true;break;}
      rejected=duration;duration*=1.1;
    }
    if (!feasible) {throw std::runtime_error("BRAKE_MOTION_LIMITS");}
    for (int step=0;step<20 && duration-rejected>1e-6;++step) {
      const double middle=.5*(rejected+duration);
      if (motionFeasible(middle)) {duration=middle;} else {rejected=middle;}
    }
    // Keep a small time margin for the subsequently checked sampled Hermite
    // execution, while retaining every original speed/acceleration limit.
    if (motionFeasible(duration*1.001)) {duration*=1.001;}
    auto & tr=result.trajectory;tr.header=state.header;tr.header.frame_id="world";
    tr.mode=request.segment.modes.empty()?"wheel":request.segment.modes.front();
    const int count=std::max(1,static_cast<int>(std::ceil(duration/.02)));
    const Eigen::Vector3d b=-3*v/(duration*duration)-2*a/duration;
    const Eigen::Vector3d d=2*v/(duration*duration*duration)+a/(duration*duration);
    const double w=twist.angular.z;
    if (!std::isfinite(w) || std::abs(w)>request.limits.yaw_rate+1e-6) {throw std::runtime_error("BRAKE_YAW_LIMIT");}
    for (int i=0;i<=count;++i) {
      const double t=duration*i/count;
      const Eigen::Vector3d pos=p+v*t+.5*a*t*t+b*t*t*t/3+d*t*t*t*t/4;
      const Eigen::Vector3d velocity=v+a*t+b*t*t+d*t*t*t;
      const Eigen::Vector3d acceleration=a+2*b*t+3*d*t*t;
      const double u=t/duration;
      const double theta=heading+w*duration*(u-u*u*u+.5*u*u*u*u);
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;geometry_msgs::msg::Transform transform;
      transform.translation.x=pos.x();transform.translation.y=pos.y();transform.translation.z=pos.z();
      transform.rotation.z=std::sin(theta/2);transform.rotation.w=std::cos(theta/2);point.transforms.push_back(transform);
      geometry_msgs::msg::Twist first,second;
      first.linear.x=velocity.x();first.linear.y=velocity.y();first.linear.z=velocity.z();
      first.angular.z=w*(1-3*u*u+2*u*u*u);
      second.linear.x=acceleration.x();second.linear.y=acceleration.y();second.linear.z=acceleration.z();
      second.angular.z=w*(-6*u+6*u*u)/duration;
      point.velocities.push_back(first);point.accelerations.push_back(second);
      const auto ns=static_cast<int64_t>(std::llround(t*1e9));
      point.time_from_start.sec=static_cast<int32_t>(ns/1000000000);
      point.time_from_start.nanosec=static_cast<uint32_t>(ns%1000000000);tr.points.push_back(std::move(point));
    }
    // The executor uses cubic Hermite between samples, not the generating
    // quartic. Bound that actual interpolation through its derivative control
    // polygon and acceleration endpoints before accepting the brake.
    for (std::size_t i=1;i<tr.points.size();++i) {
      const auto & first=tr.points[i-1];const auto & last=tr.points[i];
      const double dt=pointTime(last)-pointTime(first);
      const Eigen::Vector3d pa=vector(first.transforms[0].translation),pb=vector(last.transforms[0].translation);
      const Eigen::Vector3d va=vector(first.velocities[0].linear),vb=vector(last.velocities[0].linear);
      const Eigen::Vector3d middle=3*(pb-pa)/dt-va-vb;
      if (!bounded({va,middle,vb},request.limits.speed,bounded,8) ||
        std::max((2*(middle-va)/dt).norm(),(2*(vb-middle)/dt).norm())>request.limits.acceleration+1e-6) {
        throw std::runtime_error("BRAKE_EXECUTED_MOTION_LIMITS speed_bound="+
          std::to_string(std::max({va.norm(),vb.norm(),middle.norm()}))+"/"+std::to_string(request.limits.speed)+
          " acceleration="+std::to_string(std::max((2*(middle-va)/dt).norm(),(2*(vb-middle)/dt).norm()))+
          "/"+std::to_string(request.limits.acceleration));
      }
    }
    std::string reason;const double start=stamp(tr.header.stamp);
    if (!trajectorySafe(tr,request,start,start+duration,reason)) {throw std::runtime_error(reason);}
    result.success=true;result.reason="SAFE_BRAKE";
  } catch (const std::exception & error) {result.trajectory.points.clear();result.reason=error.what();}
  return result;
}
}  // namespace local_planning
