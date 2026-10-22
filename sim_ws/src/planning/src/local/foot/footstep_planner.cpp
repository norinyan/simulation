#include "footstep_planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <utility>
#include <iostream>

namespace planning
{

namespace
{
double& planarAxis(std::array<Pose,3>& derivatives,int axis) {
  if(axis==0)return derivatives[0].x;
  if(axis==1)return derivatives[0].y;
  if(axis==2)return derivatives[0].z;
  return axis%2?derivatives[(axis-3)/2+1].x:derivatives[(axis-3)/2+1].z;
}
double planarAxis(const std::array<Pose,3>& derivatives,int axis) {auto copy=derivatives;return planarAxis(copy,axis);}

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
    const double dx=x-b.center_x,dy=footY(side,c)-b.center_y;
    if (std::abs(dx*std::cos(b.yaw)+dy*std::sin(b.yaw))<=b.size_x/2+kEpsilon &&
        std::abs(-dx*std::sin(b.yaw)+dy*std::cos(b.yaw))<=b.size_y/2+kEpsilon) {
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
  if(std::any_of(c.request.terrain.begin(),c.request.terrain.end(),[](const auto& b){return std::abs(b.yaw)>1e-8;})){
    // Intersect actual oriented treads/slabs with the complete sole-width band.
    // World-axis treads are slightly rotated in a measured-entry flight frame.
    double edge=std::numeric_limits<double>::infinity();
    for(double yy:{y-hy,y,y+hy}){
      std::vector<std::pair<double,double>> spans;
      for(const auto& b:c.request.terrain){
        if(std::abs(b.center_z+b.size_z/2-h)>kEpsilon)continue;
        double lo=bounds.min_x,hi=bounds.max_x;const double dy=yy-b.center_y,co=std::cos(b.yaw),si=std::sin(b.yaw);
        for(auto axis:{std::array<double,3>{co,si*dy,b.size_x/2},std::array<double,3>{-si,co*dy,b.size_y/2}}){
          if(std::abs(axis[0])<1e-12){if(std::abs(axis[1])>axis[2]+kEpsilon){lo=1;hi=0;break;}}
          else {double a=b.center_x+(-axis[2]-axis[1])/axis[0],z=b.center_x+(axis[2]-axis[1])/axis[0];lo=std::max(lo,std::min(a,z));hi=std::min(hi,std::max(a,z));}
        }
        if(lo<=hi)spans.emplace_back(lo,hi);
      }
      std::sort(spans.begin(),spans.end());bool found=false;double lo=0,hi=0;
      for(std::size_t i=0;i<spans.size();){lo=spans[i].first;hi=spans[i++].second;
        while(i<spans.size() && spans[i].first<=hi+kEpsilon)hi=std::max(hi,spans[i++].second);
        if(x>=lo && x<=hi){found=true;break;}
      }
      if(!found)return -1;
      for(const auto& b:c.request.terrain){
        const double ex=(std::abs(std::cos(b.yaw))*b.size_x+std::abs(std::sin(b.yaw))*b.size_y)/2;
        const double ey=(std::abs(std::sin(b.yaw))*b.size_x+std::abs(std::cos(b.yaw))*b.size_y)/2;
        if(b.center_z+b.size_z/2<=h+kEpsilon || std::abs(y-b.center_y)>=hy+ey)continue;
        if(b.center_x-ex>x)hi=std::min(hi,b.center_x-ex);else if(b.center_x+ex<x)lo=std::max(lo,b.center_x+ex);else return -1;
      }
      edge=std::min(edge,std::min(x-lo,hi-x)-c.request.robot.foot_size[0]/2);
    }
    return edge;
  }
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

// Full OBB SAT also handles rotated flights, tilted rails and hip-yaw turns.
using Vec=std::array<double,3>;
double dot(const Vec& a,const Vec& b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
Vec cross(const Vec& a,const Vec& b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
bool boxHit(const Part& a,const Part& b)
{
  const Vec d{a.box.center_x-b.box.center_x,a.box.center_y-b.box.center_y,a.box.center_z-b.box.center_z};
  const Vec ah{a.box.size_x/2,a.box.size_y/2,a.box.size_z/2},bh{b.box.size_x/2,b.box.size_y/2,b.box.size_z/2};
  // Most World solids are distant: reject them before constructing OBB axes.
  // This is the same conservative sphere test used below previously.
  if(dot(d,d)>std::pow(std::sqrt(dot(ah,ah))+std::sqrt(dot(bh,bh)),2))return false;
  auto axes=[](const Part& p){
    const double y=p.box.yaw,t=p.pitch+p.box.pitch,r=p.box.roll;
    const double cy=std::cos(y),sy=std::sin(y),ct=std::cos(t),st=std::sin(t),cr=std::cos(r),sr=std::sin(r);
    return std::array<Vec,3>{Vec{cy*ct,sy*ct,-st},Vec{cy*st*sr-sy*cr,sy*st*sr+cy*cr,ct*sr},Vec{cy*st*cr+sy*sr,sy*st*cr-cy*sr,ct*cr}};
  };
  const auto aa=axes(a),bb=axes(b);
  auto separated=[&](Vec axis){
    const double n=std::sqrt(dot(axis,axis));if(n<1e-8)return false;
    for(auto& v:axis)v/=n;
    double e=0;for(int i=0;i<3;++i)e+=ah[i]*std::abs(dot(axis,aa[i]))+bh[i]*std::abs(dot(axis,bb[i]));
    return std::abs(dot(d,axis))>=e-kEpsilon;
  };
  for(int i=0;i<3;++i){if(separated(aa[i])||separated(bb[i]))return false;
    for(int j=0;j<3;++j)if(separated(cross(aa[i],bb[j])))return false;}
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
      for (const auto& b : *boxes) if (boxHit(p, {b})) {
        if(why)*why="TERRAIN_OR_OBSTACLE_COLLISION part="+std::to_string(&p-parts.data())+" at="+std::to_string(s.time_from_start)+" solid="+std::to_string(b.center_x)+","+std::to_string(b.center_y)+","+std::to_string(b.center_z);
        return false;
      }
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
  const bool descending=c.request.geometry.goal.z<c.request.geometry.start.z-kEpsilon;
  // Descending places the body over the lower leading foot before lowering;
  // keeping it at the midpoint drives the trailing-bend shin into the riser.
  end.body.x = (descending && std::abs(end.feet[0].z-end.feet[1].z)>kEpsilon?std::max(end.feet[0].x,end.feet[1].x):(end.feet[0].x+end.feet[1].x)/2)-r.shoulder_offset[0]-r.foot_center_offset_x;
  end.body.z = (std::min(end.feet[0].z,end.feet[1].z)+(descending &&
    (std::abs(end.feet[0].x-c.request.geometry.goal.x-r.shoulder_offset[0]-r.foot_center_offset_x)>kEpsilon ||
     std::abs(end.feet[1].x-c.request.geometry.goal.x-r.shoulder_offset[0]-r.foot_center_offset_x)>kEpsilon)
      ?.25*std::min(c.max_rise,c.request.geometry.traversal.max_step_height):0.))-r.foot_size[2]/2+o.body_height;
  const double peak = std::max(start.feet[side].z, end.feet[side].z)+o.swing_clearance;
  const int n = std::max(4, 4*static_cast<int>(std::ceil(o.step_duration/o.sample_dt/4)));
  std::vector<Sample> samples;
  samples.reserve(n+1);
  for (int i = 0; i <= n; ++i) {
    const double u = static_cast<double>(i)/n;
    Sample s = start;
    s.spatial_velocity = {}; s.spatial_acceleration = {};
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
      planarAxis(s.spatial_velocity,axis) = (b-a)*footQd(phase)*scale/o.step_duration;
      planarAxis(s.spatial_acceleration,axis) = (b-a)*footQdd(phase)*scale*scale/(o.step_duration*o.step_duration);
    };
    derivatives(0, start.body.x, end.body.x, u, 1);
    derivatives(2, start.body.z, end.body.z, u, 1);
    if (u < .25) derivatives(4+side*2, start.feet[side].z, peak, u*4, 4);
    else if (u < .75) derivatives(3+side*2, start.feet[side].x, target, (u-.25)*2, 2);
    else derivatives(4+side*2, peak, end.feet[side].z, (u-.75)*4, 4);
    samples.push_back(s);
  }
  // Retime this existing quintic motion to the shared joint/Cartesian limits.
  // Keep the phase-aligned knot count; the public 50 ms bound still applies.
  double scale=0;std::array<double,13> previous{},previous_v{};
  for(std::size_t i=0;i<samples.size();++i){
    const auto& s=samples[i];const auto l=footJoints(s,0,c),rr=footJoints(s,1,c);
    for(int j=0;j<3;++j){const auto& v=s.spatial_velocity[j];const auto& a=s.spatial_acceleration[j];
      const double vmax=j?o.max_foot_v:o.max_body_v,amax=j?o.max_foot_a:o.max_body_a;
      scale=std::max({scale,std::abs(v.x)/vmax,std::abs(v.y)/vmax,std::abs(v.z)/vmax,
        std::sqrt(std::abs(a.x)/amax),std::sqrt(std::abs(a.y)/amax),std::sqrt(std::abs(a.z)/amax)});
    }
    std::array<double,13> p{s.body.x,s.body.y,s.body.z,s.feet[0].x,s.feet[0].z,s.feet[1].x,s.feet[1].z,l[0],l[1],l[2],rr[0],rr[1],rr[2]};
    if(i){const double dt=s.time_from_start-samples[i-1].time_from_start;
      for(int j=0;j<13;++j){const double v=(p[j]-previous[j])/dt,a=(v-previous_v[j])/dt;
        const double vmax=j<3?o.max_body_v:j<7?o.max_foot_v:o.max_joint_v,amax=j<3?o.max_body_a:j<7?o.max_foot_a:o.max_joint_a;
        if(std::isfinite(v))scale=std::max(scale,std::abs(v)/vmax);
        if(i>1 && std::isfinite(a))scale=std::max(scale,std::sqrt(std::abs(a)/amax));
        previous_v[j]=v;
      }
    }previous=p;
  }
  if(scale>0){scale*=1.01;for(auto& s:samples){s.time_from_start=start.time_from_start+(s.time_from_start-start.time_from_start)*scale;for(auto& v:s.spatial_velocity){v.x/=scale;v.y/=scale;v.z/=scale;v.yaw/=scale;}for(auto& a:s.spatial_acceleration){a.x/=scale*scale;a.y/=scale*scale;a.z/=scale*scale;a.yaw/=scale*scale;}}}
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
      if (std::abs(planarAxis(s.spatial_velocity,k)) > (k < 3 ? o.max_body_v : o.max_foot_v)+kEpsilon ||
          std::abs(planarAxis(s.spatial_acceleration,k)) > (k < 3 ? o.max_body_a : o.max_foot_a)+kEpsilon) {
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
          if (why) *why = "MOTION_LIMIT axis="+std::to_string(k)+" t="+std::to_string(s.time_from_start)+" v="+std::to_string(v)+" a="+std::to_string(a);
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

FootLocalPlanResult planFeet(const FootLocalPlanRequest& request, const FootstepOptions& options, bool spatial_seed=false)
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
  if(request.geometry.goal.z<request.geometry.start.z-kEpsilon){
    for(const auto& b:request.terrain){
      const double margin=b.size_x/2-r.foot_size[0]/2-options.edge_margin-(std::abs(b.yaw)>1e-8?.001:0.);
      if(margin<0)continue;
      for(double x:{b.center_x-margin,b.center_x,b.center_x+margin})
        if(x>start.feet[0].x+kEpsilon && x<goal-kEpsilon && footLanding(x,0,c)&&footLanding(x,1,c))candidates.push_back(x);
    }
    std::sort(candidates.begin(),candidates.end());
    candidates.erase(std::unique(candidates.begin(),candidates.end(),[](double a,double b){return std::abs(a-b)<1e-7;}),candidates.end());
  }
  candidates.push_back(goal);
  // Long straight flights first try paired forward placements on the actual
  // foothold candidates. Every transition uses the same kinematic/collision
  // checks; if the seed is infeasible the existing A* remains the fallback.
  if(spatial_seed){
    FootLocalPlanResult seeded;seeded.trajectory={start};
    bool possible=true;
    while(seeded.trajectory.back().feet[0].x<goal-kEpsilon){
      const auto current=seeded.trajectory.back();bool advanced=false;
      for(auto it=candidates.rbegin();it!=candidates.rend();++it){
        if(*it<=current.feet[0].x+kEpsilon || *it-current.feet[0].x>options.max_step+kEpsilon)continue;
        const double rise=footGround(*it,0,c)-footGround(current.feet[0].x,0,c);
        if(rise>c.max_rise+kEpsilon || rise < -c.max_rise-kEpsilon)continue;
        auto left=footTransition(current,0,*it,c);
        if(!footMotionValid(left,c,&seeded.failure_reason)){if(seeded.footsteps.empty())std::clog<<"Initial swing target="<<*it<<" rejected "<<seeded.failure_reason<<std::endl;continue;}
        auto right=footTransition(left.back(),1,*it,c);
        if(!footMotionValid(right,c,&seeded.failure_reason)){if(seeded.footsteps.empty())std::clog<<"Initial second swing target="<<*it<<" rejected "<<seeded.failure_reason<<std::endl;continue;}
        for(int side=0;side<2;++side){
          const auto& segment=side==0?left:right;
          seeded.footsteps.push_back({side,segment.front().feet[side],segment.back().feet[side],footEdge(*it,side,c),segment.back().time_from_start});
          seeded.trajectory.insert(seeded.trajectory.end(),segment.begin()+1,segment.end());
        }
        advanced=true;break;
      }
      if(!advanced){std::clog<<"Paired placement blocked at "<<current.feet[0].x<<" reason="<<seeded.failure_reason<<std::endl;possible=false;break;}
    }
    if(possible && footMotionValid(seeded.trajectory,c,&seeded.failure_reason)){seeded.success=true;return seeded;}
  }
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
    pose.body.x = (request.geometry.goal.z<request.geometry.start.z-kEpsilon && std::abs(pose.feet[0].z-pose.feet[1].z)>kEpsilon?std::max(pose.feet[0].x,pose.feet[1].x):(pose.feet[0].x+pose.feet[1].x)/2)-offset;
    pose.body.z = (std::min(pose.feet[0].z,pose.feet[1].z)+(request.geometry.goal.z<request.geometry.start.z-kEpsilon && (s.left!=0 || s.right!=0) && (s.left!=last || s.right!=last)?.25*std::min(c.max_rise,request.geometry.traversal.max_step_height):0.))-r.foot_size[2]/2+options.body_height;
    const int old = s.next == 0 ? s.left : s.right, other = s.next == 0 ? s.right : s.left;
    for (int target = old+1; target < count; ++target) {
      const double dx = candidates[target]-candidates[old];
      if (dx > options.max_step+kEpsilon) break;
      if (candidates[target] < candidates[other]-kEpsilon || std::abs(candidates[target]-candidates[other]) > options.max_step) continue;
      const double rise = footGround(candidates[target], s.next, c)-footGround(candidates[old], s.next, c);
      if (rise > c.max_rise+kEpsilon || rise < -c.max_rise-kEpsilon) continue;
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

namespace {
Pose rotatePose(Pose p,const Pose& frame,bool inverse=false){
 const double yaw=inverse?-frame.yaw:frame.yaw,c=std::cos(yaw),s=std::sin(yaw);
 if(inverse){p.x-=frame.x;p.y-=frame.y;p.z-=frame.z;}
 const double x=p.x;p.x=c*x-s*p.y;p.y=s*x+c*p.y;p.yaw+=yaw;
 if(!inverse){p.x+=frame.x;p.y+=frame.y;p.z+=frame.z;}
 return p;
}
Pose hipPose(const Pose& body,int side,const FootRobotGeometry& r){
 return rotatePose({r.shoulder_offset[0],(side? -1:1)*r.shoulder_offset[1],r.shoulder_offset[2],0},body);
}
Pose standingFoot(const Pose& body,int side,const FootRobotGeometry& r,double ground){
 auto p=hipPose(body,side,r);p.x+=r.foot_center_offset_x*std::cos(body.yaw);p.y+=r.foot_center_offset_x*std::sin(body.yaw);
 p.z=ground+r.foot_size[2]/2;return p;
}
bool supportedFoot(const Pose& p,const FootLocalPlanRequest& r){
 const auto& size=r.robot.foot_size;
 for(double x:{-size[0]/2-r.limits->edge_margin,0.,size[0]/2+r.limits->edge_margin})
  for(double y:{-size[1]/2-r.limits->edge_margin,0.,size[1]/2+r.limits->edge_margin}){
   const double xx=p.x+x*std::cos(p.yaw)-y*std::sin(p.yaw),yy=p.y+x*std::sin(p.yaw)+y*std::cos(p.yaw);
   bool ok=false;
   for(const auto& b:r.terrain)if(std::abs(b.roll)+std::abs(b.pitch)<1e-7 && std::abs(p.z-size[2]/2-b.center_z-b.size_z/2)<1e-7){
    const double dx=xx-b.center_x,dy=yy-b.center_y;
    if(std::abs(dx*std::cos(b.yaw)+dy*std::sin(b.yaw))<=b.size_x/2+1e-7 &&
       std::abs(-dx*std::sin(b.yaw)+dy*std::cos(b.yaw))<=b.size_y/2+1e-7){ok=true;break;}
   }
   if(!ok)return false;
  }
 return true;
}
std::vector<Part> spatialParts(const Sample& s,const FootLocalPlanRequest& req){
 const auto& r=req.robot;const auto& o=*req.limits;
 auto box=[](Pose p,double x,double y,double z,double pitch=0.){return Part{{p.x,p.y,p.z,x,y,z,p.yaw},pitch};};
 const auto bar=rotatePose({r.shoulder_offset[0],0,r.shoulder_offset[2],0},s.body);
 auto carrier=s.body;carrier.z+=-r.carrier_drop+o.wheel_lift;
 std::vector<Part> parts{box(s.body,r.body_size[0],r.body_size[1],r.body_size[2]),
  box(bar,r.leg_width,2*r.shoulder_offset[1],r.leg_width),box(carrier,2*r.wheel_radius,r.wheel_track+r.wheel_width,2*r.wheel_radius)};
 for(int side=0;side<2;++side){
  const auto k=footInverseKinematics(s.body,s.feet[side],side,r);
  auto mid=[](Pose a,Pose b){a.x=(a.x+b.x)/2;a.y=(a.y+b.y)/2;a.z=(a.z+b.z)/2;return a;};
  parts.push_back(box(mid(k.hip,k.knee),r.leg_width,r.leg_width,r.upper_leg_length,k.upper_pitch));
  parts.push_back(box(mid(k.knee,k.ankle),r.leg_width,r.leg_width,r.lower_leg_length,k.lower_pitch));
  parts.push_back(box(s.feet[side],r.foot_size[0],r.foot_size[1],r.foot_size[2]));
 }
 return parts;
}
void spatialPoseValid(const Sample& s,const FootLocalPlanRequest& r){
 if(!finitePose(s.body)||!finitePose(s.feet[0])||!finitePose(s.feet[1]))throw std::runtime_error("NONFINITE_SPATIAL_POSE");
 if(!supportedFoot(s.feet[0],r)&&!supportedFoot(s.feet[1],r))throw std::runtime_error("NO_PLANTED_SUPPORT_FOOT");
 const auto parts=spatialParts(s,r);
 for(std::size_t i=0;i<parts.size();++i){
  for(const auto& b:r.geometry.obstacles)if(boxHit(parts[i],{b})){
   throw std::runtime_error("SPATIAL_COLLISION part="+std::to_string(i)+" obstacle="+std::to_string(b.center_x)+","+std::to_string(b.center_y)+","+std::to_string(b.center_z));
  }
  for(std::size_t j=i+1;j<parts.size();++j){
   if((i<3 && j<3)||(i==1&&(j==3||j==6))||(i==3&&j==4)||(i==4&&j==5)||(i==6&&j==7)||(i==7&&j==8))continue;
   if(boxHit(parts[i],parts[j]))throw std::runtime_error("SPATIAL_SELF_COLLISION "+std::to_string(i)+","+std::to_string(j));
  }
 }
}
// A turn step first repositions its support foot along the chord of that hip's
// rotation, then pivots with the planted foot fixed. A small body translation
// keeps the hip in the support leg's plane and returns to the exact turn centre.
double spatialTimeScale(const std::vector<Sample>& knots,const FootLocalPlanRequest& r);
std::vector<Sample> turnSwing(const Sample& start,int swing,Pose target,double yaw_delta,const FootLocalPlanRequest& r){
 const auto& o=*r.limits;const auto& robot=r.robot;
 const int stance=1-swing;const auto planted=start.feet[stance];
 const auto ik=footInverseKinematics(start.body,start.feet[swing],swing,robot);
 const double initial_dx=(ik.ankle.x-ik.hip.x)*std::cos(start.feet[swing].yaw)+(ik.ankle.y-ik.hip.y)*std::sin(start.feet[swing].yaw);
 auto endBody=start.body;endBody.yaw+=yaw_delta;
 const auto endHip=hipPose(endBody,swing,robot);
 const double end_dx=(target.x-robot.foot_center_offset_x*std::cos(target.yaw)-endHip.x)*std::cos(target.yaw)+
  (target.y-robot.foot_center_offset_x*std::sin(target.yaw)-endHip.y)*std::sin(target.yaw);
 const int count=4*int(std::ceil(o.step_duration/o.sample_dt/4));
 std::vector<Sample> result;
 for(int i=0;i<=count;++i){
  const double u=double(i)/count;Sample s=start;s.time_from_start=start.time_from_start+u*o.step_duration;
  if(yaw_delta!=0){
   s.body.yaw=start.body.yaw+yaw_delta*footQ(std::clamp((u-.25)*2,0.,1.));
   const auto h=hipPose(s.body,stance,robot);
   const double nx=-std::sin(planted.yaw),ny=std::cos(planted.yaw);
   const double lateral=(h.x-planted.x)*nx+(h.y-planted.y)*ny;
   s.body.x-=lateral*nx;s.body.y-=lateral*ny;
  }
  const double lift=footQ(std::clamp(u*4,0.,1.)),lower=footQ(std::clamp((u-.75)*4,0.,1.));
  const double heading=yaw_delta!=0?s.body.yaw:start.feet[swing].yaw+std::atan2(std::sin(target.yaw-start.feet[swing].yaw),std::cos(target.yaw-start.feet[swing].yaw))*footQ(u);
  const auto hip=hipPose(s.body,swing,robot);
  const double dx=initial_dx+(end_dx-initial_dx)*footQ(std::min(1.,u*4));
  s.feet[swing]={hip.x+(dx+robot.foot_center_offset_x)*std::cos(heading),hip.y+(dx+robot.foot_center_offset_x)*std::sin(heading),
   start.feet[swing].z+o.swing_clearance*lift+(target.z-start.feet[swing].z-o.swing_clearance)*lower,heading};
  // Lift with a fixed body before following the yawing hip. Its initial sole
  // heading may differ from the body after a preceding pivot.
  if(yaw_delta!=0 && u<.25){
   const double a=start.feet[swing].yaw+(start.body.yaw-start.feet[swing].yaw)*footQ(u*4);
   s.feet[swing].x=hip.x+(dx+robot.foot_center_offset_x)*std::cos(a);
   s.feet[swing].y=hip.y+(dx+robot.foot_center_offset_x)*std::sin(a);s.feet[swing].yaw=a;
  }
  if(i==0)s=start;
  spatialPoseValid(s,r);result.push_back(s);
 }
 setSpatialFootDerivatives(result);
 for(std::size_t i=0;i<result.size();++i){
   const double u=double(i)/count;
   auto& sample=result[i];
   const double up=std::clamp(u*4,0.,1.),down=std::clamp((u-.75)*4,0.,1.);
   sample.spatial_velocity[swing+1].z=(o.swing_clearance*footQd(up)+(target.z-start.feet[swing].z-o.swing_clearance)*footQd(down))*4/o.step_duration;
   sample.spatial_acceleration[swing+1].z=(o.swing_clearance*footQdd(up)+(target.z-start.feet[swing].z-o.swing_clearance)*footQdd(down))*16/(o.step_duration*o.step_duration);
 }
 const double scale=spatialTimeScale(result,r)*1.01;
 for(auto& s:result){s.time_from_start=start.time_from_start+(s.time_from_start-start.time_from_start)*scale;
   for(auto& v:s.spatial_velocity){v.x/=scale;v.y/=scale;v.z/=scale;v.yaw/=scale;}
   for(auto& a:s.spatial_acceleration){a.x/=scale*scale;a.y/=scale*scale;a.z/=scale*scale;a.yaw/=scale*scale;}}
 return result;
}
double spatialTimeScale(const std::vector<Sample>& knots,const FootLocalPlanRequest& r){
 const auto& o=*r.limits;double scale=0,previous_time=0,previous_dt=0;bool first=true;int count=0;
 std::array<double,20> previous{},previous_v{};
 auto check=[&](const Sample& s){
   const auto left=footInverseKinematics(s.body,s.feet[0],0,r.robot),right=footInverseKinematics(s.body,s.feet[1],1,r.robot);
   std::array<double,20> p{s.body.x,s.body.y,s.body.z,s.body.yaw,s.feet[0].x,s.feet[0].y,s.feet[0].z,s.feet[0].yaw,s.feet[1].x,s.feet[1].y,s.feet[1].z,s.feet[1].yaw};
   for(int j=0;j<4;++j){p[12+j]=left.joints[j];p[16+j]=right.joints[j];}
   if(!first){const double dt=s.time_from_start-previous_time;
     for(int j=0;j<20;++j){
       const double v=(p[j]-previous[j])/dt,a=2*(v-previous_v[j])/(dt+previous_dt);
       const double vmax=j<3?o.max_body_v:j%4==3||j>=12?o.max_joint_v:o.max_foot_v;
       const double amax=j<3?o.max_body_a:j%4==3||j>=12?o.max_joint_a:o.max_foot_a;
       scale=std::max(scale,std::abs(v)/vmax);if(count>1)scale=std::max(scale,std::sqrt(std::abs(a)/amax));previous_v[j]=v;
     }previous_dt=dt;
   }
   for(int j=0;j<3;++j){const auto& v=s.spatial_velocity[j];const auto& a=s.spatial_acceleration[j];
     const double vmax=j?o.max_foot_v:o.max_body_v,amax=j?o.max_foot_a:o.max_body_a;
     scale=std::max({scale,std::abs(v.x)/vmax,std::abs(v.y)/vmax,std::abs(v.z)/vmax,std::abs(v.yaw)/o.max_joint_v,
       std::sqrt(std::abs(a.x)/amax),std::sqrt(std::abs(a.y)/amax),std::sqrt(std::abs(a.z)/amax),std::sqrt(std::abs(a.yaw)/o.max_joint_a)});
   }
   previous=p;previous_time=s.time_from_start;first=false;++count;
 };
 for(std::size_t i=1;i<knots.size();++i){
   if(const auto why=r.geometry.control.failure();!why.empty())throw std::runtime_error(why);
   const auto& a=knots[i-1];const auto& b=knots[i];const int n=std::max(4,int(std::ceil((b.time_from_start-a.time_from_start)/.005)));
   for(int j=0;j<n;++j)check(interpolateFootTrajectory(a,b,double(j)/n));
 }check(knots.back());return scale;
}
FootLocalPlanResult planSpatial(const FootLocalPlanRequest& request,const FootstepOptions& options){
 if(!request.limits || request.reference_path.size()<2)throw std::runtime_error("INVALID_SPATIAL_REQUEST");
 FootLocalPlanResult out;Sample initial;initial.body=request.geometry.start;initial.body.z+=options.body_height;
 for(int side=0;side<2;++side)initial.feet[side]=standingFoot(initial.body,side,request.robot,request.geometry.start.z);
 if(request.current_body){initial.body=*request.current_body;initial.feet=request.current_feet;}
 spatialPoseValid(initial,request);out.trajectory.push_back(initial);
 auto append=[&](const std::vector<Sample>& segment,int side){
  if(side>=0)out.footsteps.push_back({side,segment.front().feet[side],segment.back().feet[side],options.edge_margin,segment.back().time_from_start});
  out.trajectory.insert(out.trajectory.end(),segment.begin()+1,segment.end());
 };
 auto reference=request.reference_path;
 // The measured entry need not lie exactly on the discrete reference lane.
 // Reuse the existing planted-foot turn to align the first straight flight;
 // its endpoint then reaches the real platform centre without lateral snapping.
 if(std::hypot(reference[1].x-initial.body.x,reference[1].y-initial.body.y)>1e-5){
   const double heading=std::atan2(reference[1].y-initial.body.y,reference[1].x-initial.body.x);
   const double turn=std::atan2(std::sin(heading-initial.body.yaw),std::cos(heading-initial.body.yaw));
   if(std::abs(turn)>1e-8){
     if(reference.size()==2 || std::hypot(reference[2].x-reference[1].x,reference[2].y-reference[1].y)>1e-8)reference.insert(reference.begin()+2,reference[1]);
     Pose align=initial.body;align.z-=options.body_height;align.yaw=heading;
     reference.insert(reference.begin()+1,align);reference[2].yaw=heading;
   }
 }
 for(std::size_t k=1;k<reference.size();++k){
  if(const auto why=request.geometry.control.failure();!why.empty())throw std::runtime_error(why);
  std::clog<<"Spatial foot section "<<k<<" of "<<reference.size()-1<<std::endl;
  const auto target=reference[k];auto current=out.trajectory.back();
  const double turn=std::atan2(std::sin(target.yaw-current.body.yaw),std::cos(target.yaw-current.body.yaw));
  if(std::abs(turn)<=1e-8 && std::hypot(target.x-current.body.x,target.y-current.body.y)<1e-8 && std::abs(target.z-current.body.z+options.body_height)<1e-8)continue;
  if(std::abs(turn)>1e-8){
   if(std::hypot(target.x-current.body.x,target.y-current.body.y)>1e-5)throw std::runtime_error("TURN_CENTRE_MISMATCH");
   const int n=int(std::ceil(std::abs(turn)/(std::acos(-1.)/8)));const double delta=turn/n;
   for(int j=0;j<n;++j){
    const int stance=j%2;current=out.trajectory.back();auto nextBody=current.body;nextBody.yaw+=delta;
    const auto a=hipPose(current.body,stance,request.robot),b=hipPose(nextBody,stance,request.robot);
    double yaw=std::atan2(b.y-a.y,b.x-a.x);
    while(yaw-current.body.yaw>std::acos(-1.)/2)yaw-=std::acos(-1.);
    while(yaw-current.body.yaw<-std::acos(-1.)/2)yaw+=std::acos(-1.);
    Pose foot{a.x+request.robot.foot_center_offset_x*std::cos(yaw),a.y+request.robot.foot_center_offset_x*std::sin(yaw),current.feet[stance].z,yaw};
    append(turnSwing(current,stance,foot,0,request),stance);
    current=out.trajectory.back();
    append(turnSwing(current,1-stance,standingFoot(nextBody,1-stance,request.robot,target.z),delta,request),1-stance);
   }
   current=out.trajectory.back();const int last=(n-1)%2;
   append(turnSwing(current,last,standingFoot(current.body,last,request.robot,target.z),0,request),last);
  }else{
   auto local=request;local.spatial=false;local.geometry.control=request.geometry.control;
   Pose frame=current.body;frame.z-=options.body_height;
   local.geometry.start={0,0,0,0};local.geometry.goal=rotatePose(target,frame,true);
   if(std::abs(local.geometry.goal.y)>1e-5)throw std::runtime_error("FLIGHT_LANE_MISMATCH");
   local.geometry.goal.y=0;local.geometry.goal.yaw=0;local.geometry.bounds={-30,30,-30,30};
   local.terrain.clear();local.geometry.obstacles.clear();
   for(auto b:request.terrain){
    if(std::abs(b.roll)+std::abs(b.pitch)>1e-7 || b.center_z+b.size_z/2>std::max(target.z,frame.z)+1e-6)continue;
    auto p=rotatePose({b.center_x,b.center_y,b.center_z,b.yaw},frame,true);b.center_x=p.x;b.center_y=p.y;b.center_z=p.z;b.yaw=p.yaw;
    if(std::abs(std::sin(b.yaw))<1e-6)b.yaw=0;
    else if(std::abs(std::cos(b.yaw))<1e-6){std::swap(b.size_x,b.size_y);b.yaw=0;}
    local.terrain.push_back(b);
   }
   // Local search uses nearby actual solids; the complete world is independently
   // checked after transformation, including rails, higher floors and ceilings.
   for(auto b:request.geometry.obstacles){
    auto p=rotatePose({b.center_x,b.center_y,b.center_z,b.yaw},frame,true);b.center_x=p.x;b.center_y=p.y;b.center_z=p.z;b.yaw=p.yaw;
    if(std::abs(b.center_y)<1.2 && b.center_x+b.size_x/2>-.6 && b.center_x-b.size_x/2<local.geometry.goal.x+.6 && b.center_z-b.size_z/2<local.geometry.goal.z+1.)local.geometry.obstacles.push_back(b);
   }
   local.reference_path={local.geometry.start,local.geometry.goal};
   auto segment=planFeet(local,options,true);if(!segment.success)throw std::runtime_error("FLIGHT "+std::to_string(k)+": "+segment.failure_reason);
   out.expanded+=segment.expanded;
   for(auto& s:segment.trajectory){
    for(int j=0;j<3;++j){const int axis=j?3+2*(j-1):0;
      const double c=std::cos(frame.yaw),sn=std::sin(frame.yaw);
      const double vx=planarAxis(s.spatial_velocity,axis),vy=j?0:planarAxis(s.spatial_velocity,1),vz=planarAxis(s.spatial_velocity,j?axis+1:2);
      const double ax=planarAxis(s.spatial_acceleration,axis),ay=j?0:planarAxis(s.spatial_acceleration,1),az=planarAxis(s.spatial_acceleration,j?axis+1:2);
      s.spatial_velocity[j]={c*vx-sn*vy,sn*vx+c*vy,vz,0};s.spatial_acceleration[j]={c*ax-sn*ay,sn*ax+c*ay,az,0};
    }
    s.body=rotatePose(s.body,frame);for(auto& f:s.feet)f=rotatePose(f,frame);
    s.time_from_start+=current.time_from_start;spatialPoseValid(s,request);
   }
   for(auto step:segment.footsteps){step.from=rotatePose(step.from,frame);step.to=rotatePose(step.to,frame);step.touchdown_time+=current.time_from_start;out.footsteps.push_back(step);}
   append(segment.trajectory,-1);
  }
 }
 canonicalizeFootCoordinates(out.trajectory);
 // Pack long individual segments into the existing 20k message contract.
 // Preserve every coordinate's stationary/moving boundary and <=50 ms knots;
 // Preserve phase boundaries and the executor sampling interval.
 if(out.trajectory.size()>20000){
   std::vector<Sample> knots;knots.push_back(out.trajectory.front());
   for(std::size_t i=1;i+1<out.trajectory.size();++i){
     const auto& a=out.trajectory[i-1];const auto& b=out.trajectory[i];const auto& c=out.trajectory[i+1];
     bool boundary=false;
     for(int j=0;j<3;++j){
       const auto& p=j?a.feet[j-1]:a.body;const auto& q=j?b.feet[j-1]:b.body;const auto& r=j?c.feet[j-1]:c.body;
       auto changed=[](double x,double y,double z){return (std::abs(x-y)<1e-13)!=(std::abs(y-z)<1e-13);};
       boundary=boundary||changed(p.x,q.x,r.x)||changed(p.y,q.y,r.y)||changed(p.z,q.z,r.z)||changed(p.yaw,q.yaw,r.yaw);
     }
     if(boundary || c.time_from_start-knots.back().time_from_start>.05-1e-9)knots.push_back(b);
   }
   knots.push_back(out.trajectory.back());out.trajectory=std::move(knots);
 }
 for(auto& sample:out.trajectory)sample.time_from_start=std::round(sample.time_from_start*1e9)/1e9;
 out.success=true;return out;
}
} // namespace


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
    out = request.spatial ? planSpatial(request,options) : planFeet(request, options);
  }
  catch (const std::exception& e) { out.failure_reason = e.what(); }
  out.planning_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-begin).count();
  return out;
}

}  // namespace planning
