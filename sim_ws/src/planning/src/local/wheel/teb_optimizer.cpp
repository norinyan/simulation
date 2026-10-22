#include "teb_optimizer.h"

#include <ceres/ceres.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace planning
{
namespace
{
constexpr double kTolerance = 1e-6;
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double distance(const Pose& a, const Pose& b) { return std::hypot(a.x-b.x, a.y-b.y); }

struct Config : TebOptions
{
  double length, width, margin;
  bool oriented{false};
};

bool footprintFree(const Pose& p, const PlanRequest& r, double extra = 0)
{
  return isPoseCollisionFree(p,r,extra);
}

bool sweptFree(const Pose& a, const Pose& b, const PlanRequest& r, const TebOptions& c)
{
  return isMotionCollisionFree(a,b,r,c.collision_translation_step,c.collision_rotation_step);
}

class BudgetCallback final : public ceres::IterationCallback
{
public:
  explicit BudgetCallback(const PlanControl& control) : control_(control) {}
  ceres::CallbackReturnType operator()(const ceres::IterationSummary&) override {
    return control_.failure().empty() ? ceres::SOLVER_CONTINUE : ceres::SOLVER_ABORT;
  }
private:
  const PlanControl& control_;
};

// Residual families/weights follow the old Demo: differential-drive motion,
// velocity/acceleration limits, initial velocity, reference and clearance.
template<typename T> T positive(T x) { return x > T(0) ? x : T(0); }
template<typename T> T angle(T x)
{
  using std::sin; using std::cos; using std::atan2;
  return atan2(sin(x), cos(x));
}

struct Motion
{
  Config c;
  template<typename T> bool operator()(const T* a, const T* b, const T* dt, T* r) const
  {
    using std::sin; using std::cos; using std::sqrt; using std::abs;
    const T dx = b[0]-a[0], dy = b[1]-a[1], mid = (a[2]+b[2])/T(2);
    const T v = (dx*cos(mid)+dy*sin(mid))/dt[0], w = angle(b[2]-a[2])/dt[0];
    r[0] = T(150)*(dx*sin(mid)-dy*cos(mid));
    r[1] = T(12)*positive(v-T(c.max_v)); r[2] = T(12)*positive(-v);
    r[3] = T(12)*positive(abs(w)-T(c.max_w));
    r[4] = T(.12)*sqrt(dt[0]);
    r[5] = T(.15)*sqrt(dx*dx+dy*dy+T(1e-9));
    return true;
  }
};

struct Accel
{
  Config c;
  template<typename T> bool operator()(const T* a, const T* b, const T* d,
                                      const T* t1, const T* t2, T* r) const
  {
    using std::cos; using std::sin; using std::abs;
    auto velocity = [](const T* x, const T* y, const T* dt) {
      return ((y[0]-x[0])*cos((x[2]+y[2])/T(2))+(y[1]-x[1])*sin((x[2]+y[2])/T(2)))/dt[0];
    };
    const T dt = (t1[0]+t2[0])/T(2), v1 = velocity(a,b,t1), v2 = velocity(b,d,t2);
    const T w1 = angle(b[2]-a[2])/t1[0], w2 = angle(d[2]-b[2])/t2[0];
    r[0] = T(8)*positive(abs((v2-v1)/dt)-T(c.max_a));
    r[1] = T(8)*positive(abs((w2-w1)/dt)-T(c.max_alpha));
    return true;
  }
};

struct InitialVelocity
{
  double v, w; Config c;
  template<typename T> bool operator()(const T* a, const T* b, const T* dt, T* r) const
  {
    using std::cos; using std::sin; using std::abs;
    const T mid = (a[2]+b[2])/T(2);
    const T speed = ((b[0]-a[0])*cos(mid)+(b[1]-a[1])*sin(mid))/dt[0];
    r[0] = T(8)*positive(abs(speed-T(v))-T(c.max_a)*dt[0]/T(2));
    r[1] = T(8)*positive(abs(angle(b[2]-a[2])/dt[0]-T(w))-T(c.max_alpha)*dt[0]/T(2));
    return true;
  }
};

struct Reference
{
  Pose p; double weight;
  template<typename T> bool operator()(const T* x, T* r) const
  {
    r[0] = T(weight)*(x[0]-T(p.x)); r[1] = T(weight)*(x[1]-T(p.y)); return true;
  }
};

struct Clearance
{
  BoxObstacle o; Config c;
  template<typename T> bool operator()(const T* p, T* r) const
  {
    using std::abs; using std::sin; using std::cos; using std::sqrt;
    const T x=p[0]-T(o.center_x),y=p[1]-T(o.center_y);
    if(!c.oriented) {
      const T dx=positive(abs(x)-T(o.size_x/2)),dy=positive(abs(y)-T(o.size_y/2));
      r[0]=T(45)*positive(T(std::hypot(c.length,c.width)/2+c.margin+.025)-sqrt(dx*dx+dy*dy+T(1e-8)));
      return true;
    }
    T gap=T(-1e6);
    for(const T axis:std::array<T,4>{p[2],p[2]+T(std::acos(-1.)/2),T(o.yaw),T(o.yaw+std::acos(-1.)/2)}) {
      const T separation=abs(x*cos(axis)+y*sin(axis))-
        T(c.length/2+c.margin)*abs(cos(p[2]-axis))-T(c.width/2+c.margin)*abs(sin(p[2]-axis))-
        T(o.size_x/2)*abs(cos(T(o.yaw)-axis))-T(o.size_y/2)*abs(sin(T(o.yaw)-axis));
      if(separation>gap)gap=separation;
    }
    r[0] = T(45)*positive(-gap);
    return true;
  }
};

struct Band
{
  std::vector<Pose> poses;
  std::vector<double> dt;
  double cost{0};
  bool usable{false};
};

Band optimizeBand(const std::vector<Pose>& seed, const LocalPlanRequest& request, const Config& c)
{
  Band band;
  std::vector<std::array<double,3>> poses;
  std::vector<double> dt;
  for (const auto& p : seed) poses.push_back({p.x,p.y,p.yaw});
  for (std::size_t i = 1; i < seed.size(); ++i) {
    dt.push_back(std::clamp(std::max(distance(seed[i-1],seed[i])/c.max_v,
                                   std::abs(wrap(seed[i].yaw-seed[i-1].yaw))/c.max_w),.08,1.2));
  }
  ceres::Problem problem;
  const auto& bounds = request.geometry.bounds;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    auto* p = poses[i].data();
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Reference,2,3>(new Reference{seed[i],c.oriented?2.:.25}),nullptr,p);
    problem.SetParameterLowerBound(p,0,bounds.min_x+c.length/2+c.margin);
    problem.SetParameterUpperBound(p,0,bounds.max_x-c.length/2-c.margin);
    problem.SetParameterLowerBound(p,1,bounds.min_y+c.width/2+c.margin);
    problem.SetParameterUpperBound(p,1,bounds.max_y-c.width/2-c.margin);
    for (const auto& o : request.geometry.obstacles) {
      if (o.center_z+o.size_z/2 <= request.geometry.start.z+kTolerance) continue;
      const double reach=std::hypot(c.length/2+c.margin,c.width/2+c.margin)+1.;
      const double ex=(std::abs(std::cos(o.yaw))*o.size_x+std::abs(std::sin(o.yaw))*o.size_y)/2;
      const double ey=(std::abs(std::sin(o.yaw))*o.size_x+std::abs(std::cos(o.yaw))*o.size_y)/2;
      if(std::abs(seed[i].x-o.center_x)>ex+reach || std::abs(seed[i].y-o.center_y)>ey+reach)continue;
      problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Clearance,1,3>(new Clearance{o,c}),nullptr,p);
    }
    if (i+1 < poses.size()) {
      problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Motion,6,3,3,1>(new Motion{c}),nullptr,p,poses[i+1].data(),&dt[i]);
      problem.SetParameterLowerBound(&dt[i],0,.05);
      problem.SetParameterUpperBound(&dt[i],0,2.5);
    }
    if (i+2 < poses.size()) {
      problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Accel,2,3,3,3,1,1>(new Accel{c}),nullptr,
                               p,poses[i+1].data(),poses[i+2].data(),&dt[i],&dt[i+1]);
    }
  }
  problem.SetParameterBlockConstant(poses.front().data());
  problem.SetParameterBlockConstant(poses.back().data());
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<InitialVelocity,2,3,3,1>(
    new InitialVelocity{request.initial_linear_velocity,request.initial_angular_velocity,c}),nullptr,
    poses[0].data(),poses[1].data(),&dt[0]);
  ceres::Solver::Options options;
  options.max_num_iterations = c.iterations;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.num_threads = 1;
  options.max_solver_time_in_seconds = std::max(1e-6,std::min(c.solver_time_limit,request.geometry.control.remaining()));
  BudgetCallback budget(request.geometry.control);
  options.callbacks.push_back(&budget);
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options,&problem,&summary);
  band.cost = summary.final_cost;
  band.usable = summary.IsSolutionUsable() && std::isfinite(band.cost);
  band.dt = std::move(dt);
  for (const auto& p : poses) band.poses.push_back({p[0],p[1],request.geometry.start.z,wrap(p[2])});
  return band;
}

double pathLength(const std::vector<Pose>& path)
{
  double result = 0;
  for (std::size_t i = 1; i < path.size(); ++i) result += distance(path[i-1],path[i]);
  return result;
}

Pose at(const std::vector<Pose>& path, double arc)
{
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double d = distance(path[i-1],path[i]);
    if (d < 1e-10) continue;
    if (arc <= d) {
      const double u = std::clamp(arc/d,0.0,1.0);
      return {path[i-1].x+u*(path[i].x-path[i-1].x),path[i-1].y+u*(path[i].y-path[i-1].y),
              path[i].z,std::atan2(path[i].y-path[i-1].y,path[i].x-path[i-1].x)};
    }
    arc -= d;
  }
  return path.back();
}

double closestArc(const std::vector<Pose>& path, const Pose& pose)
{
  double arc = 0, best = std::numeric_limits<double>::infinity(), result = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto& a = path[i-1]; const auto& b = path[i];
    const double dx = b.x-a.x, dy = b.y-a.y, d = std::hypot(dx,dy);
    if (d < 1e-10) continue;
    const double u = std::clamp(((pose.x-a.x)*dx+(pose.y-a.y)*dy)/(d*d),0.0,1.0);
    const double gap = std::hypot(pose.x-a.x-u*dx,pose.y-a.y-u*dy);
    if (gap < best) { best = gap; result = arc+u*d; }
    arc += d;
  }
  return result;
}

std::vector<Pose> seed(const LocalPlanRequest& request, const Config& c, double begin, double end, int side)
{
  std::vector<Pose> points{request.geometry.start};
  const int count = std::max(4,static_cast<int>(std::ceil((end-begin)/c.spacing)));
  for (int i = 1; i <= count; ++i) points.push_back(at(request.reference_path,begin+(end-begin)*i/count));
  if (side) for (const auto& o : request.geometry.obstacles) {
    if (o.center_z+o.size_z/2 <= request.geometry.start.z+kTolerance) continue;
    const double radius = std::hypot(c.length,c.width)/2+c.margin+.08;
    const bool relevant = std::any_of(points.begin(),points.end(),[&](const Pose& p) {
      return std::abs(p.x-o.center_x) < o.size_x/2+radius && std::abs(p.y-o.center_y) < o.size_y/2+radius;
    });
    if (!relevant) continue;
    const double target = o.center_y+side*(o.size_y/2+radius);
    for (std::size_t i = 1; i+1 < points.size(); ++i) {
      const double dx = std::abs(points[i].x-o.center_x), span = o.size_x/2+radius+.5;
      if (dx < span) points[i].y += (target-points[i].y)*std::min(1.0,(span-dx)/.5);
    }
  }
  for (std::size_t i = 1; i+1 < points.size(); ++i) {
    points[i].yaw = std::atan2(points[i+1].y-points[i-1].y,points[i+1].x-points[i-1].x);
  }
  // Preserve measured turning motion in the first seed knot. Projecting it
  // straight onto a reference corner can seed an impossible instantaneous
  // reversal of angular velocity; the optimizer still validates every result.
  if(points.size()>2 && request.initial_linear_velocity>.02 &&
     std::abs(request.initial_angular_velocity)>.05) {
    const double dt=std::clamp(c.spacing/request.initial_linear_velocity,.05,.5);
    const double turn=request.initial_angular_velocity*dt;
    const auto& start=request.geometry.start;
    points[1]={start.x+request.initial_linear_velocity*dt*std::cos(start.yaw+turn/2),
               start.y+request.initial_linear_velocity*dt*std::sin(start.yaw+turn/2),
               start.z,start.yaw+turn};
  }
  // Unwrap before Ceres so midpoint headings do not jump at +/-pi.
  for (std::size_t i = 1; i < points.size(); ++i) points[i].yaw = points[i-1].yaw+wrap(points[i].yaw-points[i-1].yaw);
  return points;
}

std::vector<TrajectoryPoint> timedPoints(const Band& band, const LocalPlanRequest& request)
{
  std::vector<TrajectoryPoint> result;
  result.push_back({band.poses.front(),0,request.initial_linear_velocity,request.initial_angular_velocity,0,0});
  for (std::size_t i = 0; i < band.dt.size(); ++i) {
    const auto& a = band.poses[i]; const auto& b = band.poses[i+1];
    const double mid = a.yaw+wrap(b.yaw-a.yaw)/2;
    const double v = ((b.x-a.x)*std::cos(mid)+(b.y-a.y)*std::sin(mid))/band.dt[i];
    const double w = wrap(b.yaw-a.yaw)/band.dt[i];
    const double interval = i == 0 ? band.dt[i]/2 : (band.dt[i-1]+band.dt[i])/2;
    result.push_back({b,result.back().time_from_start+band.dt[i],v,w,
                      (v-result.back().linear_velocity)/interval,(w-result.back().angular_velocity)/interval});
  }
  return result;
}

bool retime(Band& band, const LocalPlanRequest& request, const Config& c)
{
  // Ceres penalties are soft. Stretch optimized times until the discrete hard
  // motion bounds hold; do not clip velocities or substitute seed trajectories.
  for (int iteration = 0; iteration < 20; ++iteration) {
    const auto points = timedPoints(band,request);
    double scale = 1;
    for (std::size_t i = 1; i < points.size(); ++i) {
      const auto& p = points[i];
      if (!std::isfinite(p.linear_velocity) || p.linear_velocity < -kTolerance) return false;
      const auto& a = points[i-1].pose; const auto& b = p.pose;
      const double mid = a.yaw+wrap(b.yaw-a.yaw)/2;
      const double lateral = std::abs(-(b.x-a.x)*std::sin(mid)+(b.y-a.y)*std::cos(mid))/band.dt[i-1];
      scale = std::max({scale,p.linear_velocity/c.max_v,std::abs(p.angular_velocity)/c.max_w,
                        lateral/c.max_lateral_velocity,
                        std::sqrt(std::abs(p.linear_acceleration)/c.max_a),
                        std::sqrt(std::abs(p.angular_acceleration)/c.max_alpha)});
    }
    if (scale <= 1+kTolerance) return true;
    for (auto& dt : band.dt) dt *= scale*(1+1e-6);
  }
  return false;
}

Band prefix(const Band& band, double horizon, bool interpolate)
{
  Band result;
  result.poses.push_back(band.poses.front());
  result.cost = band.cost;
  double time = 0;
  for (std::size_t i = 0; i < band.dt.size(); ++i) {
    if (time+band.dt[i] <= horizon+kTolerance) {
      result.dt.push_back(band.dt[i]); result.poses.push_back(band.poses[i+1]); time += band.dt[i];
    } else {
      const double remaining = horizon-time;
      if (interpolate && remaining > 1e-5) {
        const double u = remaining/band.dt[i]; const auto& a = band.poses[i]; const auto& b = band.poses[i+1];
        result.dt.push_back(remaining);
        result.poses.push_back({a.x+u*(b.x-a.x),a.y+u*(b.y-a.y),a.z,wrap(a.yaw+u*wrap(b.yaw-a.yaw))});
      }
      break;
    }
  }
  return result;
}

bool validateBand(const Band& band, const LocalPlanRequest& request, const Config& c, LocalPlanResult& out)
{
  if (band.poses.size() < 2 || band.dt.size()+1 != band.poses.size()) return false;
  for (const auto dt : band.dt) if (!std::isfinite(dt) || dt <= 0) return false;
  const auto points = timedPoints(band,request);
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto& p = points[i]; const auto& a = points[i-1].pose;
    if (!std::isfinite(p.pose.x) || !std::isfinite(p.pose.y) || !std::isfinite(p.pose.yaw) ||
        !std::isfinite(p.linear_velocity) || !std::isfinite(p.angular_velocity) ||
        !std::isfinite(p.linear_acceleration) || !std::isfinite(p.angular_acceleration)) return false;
    const double mid = a.yaw+wrap(p.pose.yaw-a.yaw)/2;
    const double lateral = std::abs(-(p.pose.x-a.x)*std::sin(mid)+(p.pose.y-a.y)*std::cos(mid))/band.dt[i-1];
    if (p.linear_velocity < -kTolerance || p.linear_velocity > c.max_v+kTolerance ||
        std::abs(p.angular_velocity) > c.max_w+kTolerance ||
        std::abs(p.linear_acceleration) > c.max_a+kTolerance ||
        std::abs(p.angular_acceleration) > c.max_alpha+kTolerance ||
        lateral > c.max_lateral_velocity+kTolerance || !sweptFree(a,p.pose,request.geometry,c)) return false;
    out.max_linear_velocity = std::max(out.max_linear_velocity,std::abs(p.linear_velocity));
    out.max_angular_velocity = std::max(out.max_angular_velocity,std::abs(p.angular_velocity));
    out.max_linear_acceleration = std::max(out.max_linear_acceleration,std::abs(p.linear_acceleration));
    out.max_angular_acceleration = std::max(out.max_angular_acceleration,std::abs(p.angular_acceleration));
    out.max_lateral_velocity = std::max(out.max_lateral_velocity,lateral);
    out.path_length += distance(a,p.pose);
  }
  if (points.back().time_from_start > c.time_horizon+kTolerance) return false;
  out.trajectory = points; out.duration = points.back().time_from_start;
  out.optimization_cost = band.cost;
  return true;
}

bool finitePose(const Pose& p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(p.yaw);
}

void validateRequest(const LocalPlanRequest& r, const TebOptions& c)
{
  const auto& g = r.geometry; const auto& b = g.bounds;
  if (!finitePose(g.start) || !finitePose(g.goal) || !std::isfinite(g.safety_margin) || g.safety_margin < 0 ||
      !std::isfinite(g.footprint.length) || !std::isfinite(g.footprint.width) ||
      g.footprint.length <= 0 || g.footprint.width <= 0 ||
      !std::isfinite(b.min_x) || !std::isfinite(b.max_x) || !std::isfinite(b.min_y) || !std::isfinite(b.max_y) ||
      b.min_x >= b.max_x || b.min_y >= b.max_y ||
      !std::isfinite(r.initial_linear_velocity) || !std::isfinite(r.initial_angular_velocity) ||
      r.initial_linear_velocity < 0 || r.initial_linear_velocity > c.max_v ||
      std::abs(r.initial_angular_velocity) > c.max_w || r.reference_path.size() < 2) {
    throw std::invalid_argument("INVALID_LOCAL_REQUEST");
  }
  for (const auto& p : r.reference_path) {
    if (!finitePose(p) || std::abs(p.z-g.start.z) > kTolerance) throw std::invalid_argument("NON_PLANAR_WHEEL_REFERENCE");
  }
  if (std::abs(g.goal.z-g.start.z) > kTolerance ||
      distance(g.goal,r.reference_path.back()) > kTolerance ||
      std::abs(wrap(g.goal.yaw-r.reference_path.back().yaw)) > kTolerance) {
    throw std::invalid_argument("REFERENCE_SEGMENT_GOAL_MISMATCH");
  }
  for (const auto& o : g.obstacles) {
    if (!std::isfinite(o.center_x) || !std::isfinite(o.center_y) || !std::isfinite(o.center_z) ||
        !std::isfinite(o.size_x) || !std::isfinite(o.size_y) || !std::isfinite(o.size_z) ||
        o.size_x <= 0 || o.size_y <= 0 || o.size_z <= 0) throw std::invalid_argument("INVALID_OBSTACLE");
  }
  if (!footprintFree(g.start,g)) throw std::invalid_argument("START_FOOTPRINT_COLLISION");
}
}  // namespace

TebOptimizer::TebOptimizer(TebOptions options) : options_(options)
{
  for (double v : {options.max_v,options.max_w,options.max_a,options.max_alpha,options.time_horizon,
                   options.spacing,options.solver_time_limit,options.max_lateral_velocity,
                   options.collision_translation_step,options.collision_rotation_step}) {
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Invalid TEB options");
  }
  if (options.iterations <= 0 || options.max_v*options.time_horizon/options.spacing > 1000) {
    throw std::invalid_argument("Invalid TEB iteration/seed-size limit");
  }
}

LocalPlanResult TebOptimizer::plan(const LocalPlanRequest& request)
{
  const auto begin_time = std::chrono::steady_clock::now();
  LocalPlanResult best;
  int evaluated = 0;
  try {
    auto options = options_;
    if(request.limits) static_cast<WheelMotionLimits&>(options) = *request.limits;
    validateRequest(request,options);
    if(const auto reason=request.geometry.control.failure();!reason.empty()) throw std::runtime_error(reason);
    Config c; static_cast<TebOptions&>(c) = options;
    c.length = request.geometry.footprint.length; c.width = request.geometry.footprint.width;
    c.margin = request.geometry.safety_margin;
    c.oriented=request.geometry.require_support || std::any_of(request.geometry.obstacles.begin(),request.geometry.obstacles.end(),
      [](const auto& box){return std::abs(box.yaw)>1e-8;});
    const double begin = closestArc(request.reference_path,request.geometry.start);
    const auto closest = at(request.reference_path,begin);
    if (distance(closest,request.geometry.start) > std::max(c.spacing,1e-3)) throw std::invalid_argument("START_TOO_FAR_FROM_REFERENCE");
    const double accelerate = std::clamp((c.max_v-request.initial_linear_velocity)/c.max_a,0.0,c.time_horizon);
    const double reach = request.initial_linear_velocity*accelerate+.5*c.max_a*accelerate*accelerate+
                         c.max_v*(c.time_horizon-accelerate);
    const double end = std::min(pathLength(request.reference_path),begin+reach);
    if (end-begin < 1e-5) throw std::invalid_argument("NO_REFERENCE_PROGRESS");
    const double base_spacing=c.spacing;
    for(double spacing : {base_spacing,base_spacing*2/3,base_spacing/3}) {
      c.spacing=spacing;
      if(std::isfinite(request.geometry.control.remaining()))
        c.solver_time_limit=std::min(options.solver_time_limit,request.geometry.control.remaining()/9);
      for (int side : {0,1,-1}) {
        if(const auto reason=request.geometry.control.failure();!reason.empty()) throw std::runtime_error(reason);
        ++evaluated;
        auto band = optimizeBand(seed(request,c,begin,end,side),request,c);
        if (!band.usable || !retime(band,request,c)) continue;
        LocalPlanResult candidate;
        auto window = prefix(band,c.time_horizon,true);
        bool valid = validateBand(window,request,c,candidate);
        if (!valid) {
          candidate = {};
          window = prefix(band,c.time_horizon,false);
          valid = validateBand(window,request,c,candidate);
        }
        if (valid && candidate.duration < c.time_horizon-kTolerance &&
            end < pathLength(request.reference_path)-kTolerance) {
          // A partial final interval can tighten acceleration checks. If only a
          // whole-knot prefix is valid, slow that optimized prefix to fill the
          // requested prediction window, then check all constraints again.
          const double scale = c.time_horizon/candidate.duration;
          for (auto& dt : window.dt) dt *= scale;
          candidate = {};
          valid = validateBand(window,request,c,candidate);
        }
        if (valid && (!best.success || candidate.optimization_cost < best.optimization_cost)) {
          candidate.success = true;
          candidate.diagnostics["seed_spacing_m"]=spacing;
          best = std::move(candidate);
        }
        // Preserve Demo's preference for the reference seed if already feasible.
        if (side == 0 && best.success) break;
      }
      if(best.success) break;
    }
    if (!best.success) best.failure_reason = "NO_FEASIBLE_TIMED_BAND";
  } catch (const std::exception& error) {
    best = {}; best.failure_reason = error.what();
  }
  best.candidates_evaluated = evaluated;
  best.planning_ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin_time).count();
  return best;
}

}  // namespace planning
