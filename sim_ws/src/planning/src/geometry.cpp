#include "global_planner.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
namespace planning {
namespace {
constexpr double kEpsilon=1e-9;
bool finite(double x){return std::isfinite(x);}
double distance(const Pose& a,const Pose& b){return std::hypot(a.x-b.x,a.y-b.y);}
bool poseFinite(const Pose& p){return finite(p.x)&&finite(p.y)&&finite(p.z)&&finite(p.yaw);}
bool validate(const PlanRequest& r)
{
  const auto& b = r.bounds;
  if (!poseFinite(r.start) || !poseFinite(r.goal) ||
      !finite(b.min_x) || !finite(b.max_x) || !finite(b.min_y) || !finite(b.max_y) ||
      b.min_x >= b.max_x || b.min_y >= b.max_y ||
      !finite(r.footprint.length) || !finite(r.footprint.width) ||
      r.footprint.length <= 0 || r.footprint.width <= 0 ||
      !finite(r.safety_margin) || r.safety_margin < 0) return false;
  for(const auto* collection:{&r.obstacles,&r.supports})for (const auto& o : *collection)
    if (!finite(o.center_x) || !finite(o.center_y) || !finite(o.center_z) ||
        !finite(o.size_x) || !finite(o.size_y) || !finite(o.size_z) ||
        o.size_x <= 0 || o.size_y <= 0 || o.size_z <= 0 || !finite(o.yaw)) return false;
  return finite(std::hypot(r.footprint.length, r.footprint.width)/2+r.safety_margin);
}

}
namespace {
double spatialDistance(const Pose& a,const Pose& b) {
 return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z);
}
bool samePose(const Pose& a,const Pose& b) {
 return poseFinite(a)&&poseFinite(b)&&spatialDistance(a,b)<1e-7 &&
  std::abs(std::atan2(std::sin(a.yaw-b.yaw),std::cos(a.yaw-b.yaw)))<1e-7;
}
const SupportRegion* findRegion(const SceneMap& map,const std::string& id) {
 for(const auto& region:map.regions)if(region.id==id)return &region;
 return nullptr;
}
}

PlanRequest regionRequest(const PlanRequest& r,const SupportRegion& region) {
  auto local=r;local.map.reset();local.bounds=region.bounds;local.supports=region.supports;
  local.obstacles=region.obstacles;local.require_support=true;
  local.start.z=local.goal.z=region.height;return local;
}

std::string validateConnection(const PlanRequest& r,const StairConnection& c) {
 if(!r.map || !c.enabled || c.reference.size()<2 || c.treads.empty() || c.platforms.empty())return "INVALID_CONNECTION";
 const auto* from=findRegion(*r.map,c.from);const auto* to=findRegion(*r.map,c.to);
 if(!from || !to)return "UNKNOWN_CONNECTION_REGION";
 if(!isPoseCollisionFree(c.reference.front(),regionRequest(r,*from)) ||
    !isPoseCollisionFree(c.reference.back(),regionRequest(r,*to)))return "UNSAFE_MODE_SWITCH";
 auto terrain=c.treads;terrain.insert(terrain.end(),c.platforms.begin(),c.platforms.end());
 terrain.insert(terrain.end(),from->supports.begin(),from->supports.end());
 terrain.insert(terrain.end(),to->supports.begin(),to->supports.end());
 for(const auto& tread:c.treads)
  if(tread.size_x+1e-8<r.traversal.min_tread || tread.size_y<r.footprint.width+2*r.safety_margin)
   return "STAIR_CAPABILITY_OR_WIDTH";
 // Check the complete reference interpolation. Vertical risers are supported by
 // adjacent tread edges; they are ground references, not executable foot motion.
 for(std::size_t i=0;i<c.reference.size();++i) {
  const auto& a=c.reference[i?i-1:i];const auto& b=c.reference[i];
  if(!poseFinite(b))return "INVALID_CONNECTION_POSE";
  const double dz=std::abs(b.z-a.z),d=distance(a,b);
  if(dz>1e-7 && (d>1e-7 || dz>r.traversal.max_step_height+1e-7))return "INVALID_STAIR_RISE";
  const double turn=std::atan2(std::sin(b.yaw-a.yaw),std::cos(b.yaw-a.yaw));
  if(dz<1e-7) {
   // Exact union of support intervals along this horizontal reference edge.
   // This detects arbitrarily thin unsupported gaps, including between samples.
   std::vector<std::pair<double,double>> intervals;
   for(const auto& box:terrain) {
    if(std::abs(box.center_z+box.size_z/2-a.z)>.003)continue;
    double lo=0,hi=1,cs=std::cos(box.yaw),sn=std::sin(box.yaw);
    const double x=cs*(a.x-box.center_x)+sn*(a.y-box.center_y);
    const double y=-sn*(a.x-box.center_x)+cs*(a.y-box.center_y);
    const double vx=cs*(b.x-a.x)+sn*(b.y-a.y),vy=-sn*(b.x-a.x)+cs*(b.y-a.y);
    for(auto axis:std::array<std::array<double,3>,2>{{{x,vx,box.size_x/2},{y,vy,box.size_y/2}}}) {
     if(std::abs(axis[1])<1e-12) {if(std::abs(axis[0])>axis[2]+1e-9){lo=1;hi=0;}}
     else {
      double t0=(-axis[2]-axis[0])/axis[1],t1=(axis[2]-axis[0])/axis[1];
      lo=std::max(lo,std::min(t0,t1));hi=std::min(hi,std::max(t0,t1));
     }
    }
    if(lo<=hi+1e-9)intervals.push_back({lo,hi});
   }
   std::sort(intervals.begin(),intervals.end());double covered=0;
   for(auto interval:intervals){if(interval.first>covered+1e-8)break;covered=std::max(covered,interval.second);}
   if(covered<1-1e-8)return "UNSUPPORTED_STAIR_REFERENCE";
  } else {
   // A vertical riser must join the two actual horizontal surfaces at its edge.
   for(const auto& p:{a,b}) {
    bool support=false;
    for(const auto& box:terrain) {
     const double cs=std::cos(box.yaw),sn=std::sin(box.yaw),x=p.x-box.center_x,y=p.y-box.center_y;
     if(std::abs(cs*x+sn*y)<=box.size_x/2+1e-8 && std::abs(-sn*x+cs*y)<=box.size_y/2+1e-8 &&
        std::abs(p.z-box.center_z-box.size_z/2)<.003)support=true;
    }
    if(!support)return "UNSUPPORTED_RISER_EDGE";
   }
  }
  // The swept body envelope covers the entire interpolation, including vertical
  // travel. This is a macro passage check, not a claim about a feasible gait.
  auto body=r;body.map.reset();body.require_support=false;body.supports.clear();body.obstacles.clear();
  body.bounds={-1e4,1e4,-1e4,1e4};
  const double low=std::min(a.z,b.z)+r.traversal.body_ground_clearance;
  const double high=std::max(a.z,b.z)+r.traversal.body_ground_clearance+r.traversal.body_height;
  for(const auto& box:r.map->solids) {
   const double cp=std::cos(box.pitch),sp=std::sin(box.pitch),cr=std::cos(box.roll),sr=std::sin(box.roll);
   const double extent=(std::abs(sp)*box.size_x+std::abs(cp*sr)*box.size_y+std::abs(cp*cr)*box.size_z)/2;
   if(box.center_z+extent>low+1e-6 && box.center_z-extent<high-1e-6) {
    auto projection=box;
    projection.size_x=std::abs(cp)*box.size_x+std::abs(sp*sr)*box.size_y+std::abs(sp*cr)*box.size_z;
    projection.size_y=std::abs(cr)*box.size_y+std::abs(sr)*box.size_z;
    projection.center_z=(low+high)/2;projection.size_z=high-low;
    body.obstacles.push_back(projection);
   }
  }
  Pose flat_b=b;flat_b.z=a.z;
  if(!isMotionCollisionFree(a,flat_b,body))return "STAIR_BODY_COLLISION";
  if(std::abs(turn)>1e-7) {
   auto flat=body;flat.obstacles.clear();flat.supports=c.platforms;flat.require_support=true;
   if(dz>1e-7 || !isMotionCollisionFree(a,b,flat))return "INSUFFICIENT_PLATFORM_TURN_SPACE";
  }

 }
 return {};
}

std::string validateGlobalPath(const PlanRequest& r,PlanResult& result) {
 if(!r.map || result.path.empty() || !samePose(result.path.front(),r.start) || !samePose(result.path.back(),r.goal))return "INVALID_GLOBAL_ENDPOINTS";
 if(result.segments.empty())return "MISSING_GLOBAL_SEGMENTS";
 std::size_t end=0;result.spatial_length=0;result.mode_switches=0;
 for(std::size_t i=0;i<result.segments.size();++i) {
  const auto& s=result.segments[i];
  if(s.begin!=end || s.end<s.begin || s.end>=result.path.size())return "INVALID_SEGMENT_COVERAGE";
  end=s.end;
  if(i && s.mode!=result.segments[i-1].mode)++result.mode_switches;
  if(s.mode==TravelMode::Wheel) {
   auto region=findRegion(*r.map,s.region_id);if(!region || !s.connection_id.empty())return "INVALID_WHEEL_REGION";
   auto local=regionRequest(r,*region);
   for(std::size_t j=s.begin;j<=s.end;++j)
    if(std::abs(result.path[j].z-region->height)>1e-7 || !isPoseCollisionFree(result.path[j],local) ||
       (j>s.begin && !isMotionCollisionFree(result.path[j-1],result.path[j],local)))return "INVALID_GLOBAL_GEOMETRY";
  } else {
   const StairConnection* c=nullptr;
   for(const auto& candidate:r.map->connections)if(candidate.id==s.connection_id)c=&candidate;
   if(!c || s.end-s.begin+1!=c->reference.size())return "INVALID_FOOT_CONNECTION";
   if(auto failure=validateConnection(r,*c);!failure.empty())return failure;
   for(std::size_t j=0;j<c->reference.size();++j)
    if(!samePose(result.path[s.begin+j],c->reference[j]))return "INVALID_CONNECTION_PATH";
   if(i==0 || i+1==result.segments.size() || result.segments[i-1].region_id!=c->from ||
      result.segments[i+1].region_id!=c->to)return "INVALID_CONNECTION_CONTINUITY";
  }
 }
 if(end+1!=result.path.size())return "INCOMPLETE_GLOBAL_PATH";
 for(std::size_t i=1;i<result.path.size();++i)result.spatial_length+=spatialDistance(result.path[i-1],result.path[i]);
 return {};
}

namespace {
struct Point {double x,y;};
using Polygon=std::vector<Point>;
Polygon clip(const Polygon& polygon,double nx,double ny,double offset) {
  Polygon out;
  if(polygon.empty())return out;
  auto previous=polygon.back();double pd=nx*previous.x+ny*previous.y-offset;
  for(const auto& current:polygon) {
    const double d=nx*current.x+ny*current.y-offset;
    if((pd<=0)!=(d<=0)) {
      const double t=pd/(pd-d);
      out.push_back({previous.x+t*(current.x-previous.x),previous.y+t*(current.y-previous.y)});
    }
    if(d<=0)out.push_back(current);
    previous=current;pd=d;
  }
  return out;
}
double area(const Polygon& polygon) {
  double value=0;
  for(std::size_t i=0;i<polygon.size();++i){const auto& a=polygon[i];const auto& b=polygon[(i+1)%polygon.size()];value+=a.x*b.y-a.y*b.x;}
  return std::abs(value)/2;
}
bool supported(const Pose& p,const PlanRequest& r,double padding) {
  if(!r.require_support)return true;
  const double hx=r.footprint.length/2+r.safety_margin+padding,hy=r.footprint.width/2+r.safety_margin+padding;
  const double c=std::cos(p.yaw),s=std::sin(p.yaw);
  Polygon footprint;
  for(auto v:std::array<Point,4>{{{-hx,-hy},{hx,-hy},{hx,hy},{-hx,hy}}})
    footprint.push_back({p.x+c*v.x-s*v.y,p.y+s*v.x+c*v.y});
  // Most poses lie on one slab. Avoid polygon subtraction in that common case;
  // retain the exact union test below for slab joins and floor openings.
  for(const auto& box:r.supports) {
    if(std::abs(box.center_z+box.size_z/2-p.z)>.003)continue;
    const double cs=std::cos(box.yaw),sn=std::sin(box.yaw);
    bool contained=true;
    for(const auto& v:footprint) {
      const double x=v.x-box.center_x,y=v.y-box.center_y;
      if(std::abs(cs*x+sn*y)>box.size_x/2 || std::abs(-sn*x+cs*y)>box.size_y/2){contained=false;break;}
    }
    if(contained)return true;
  }
  std::vector<Polygon> uncovered{footprint};
  for(const auto& support:r.supports) {
    if(!finite(support.yaw) || support.size_x<=0 || support.size_y<=0 ||
       std::abs(support.center_z+support.size_z/2-p.z)>.003)continue;
    const double cs=std::cos(support.yaw),sn=std::sin(support.yaw);
    std::vector<Polygon> next;
    for(auto inside:uncovered) {
      for(auto axis:std::array<std::array<double,3>,4>{{{cs,sn,support.size_x/2},{-cs,-sn,support.size_x/2},
                                                    {-sn,cs,support.size_y/2},{sn,-cs,support.size_y/2}}}) {
        const double edge=axis[0]*support.center_x+axis[1]*support.center_y+axis[2];
        auto outside=clip(inside,-axis[0],-axis[1],-edge);
        if(area(outside)>1e-10)next.push_back(std::move(outside));
        inside=clip(inside,axis[0],axis[1],edge);
        if(inside.empty())break;
      }
    }
    uncovered=std::move(next);
    if(uncovered.empty())return true;
    if(uncovered.size()>10000)return false;
  }
  return false;
}
}

bool isPoseCollisionFree(const Pose& p, const PlanRequest& r, double padding)
{
  if (!poseFinite(p) || !validate(r) || !std::isfinite(padding) || padding < 0) return false;
  const double hx = r.footprint.length/2+r.safety_margin+padding;
  const double hy = r.footprint.width/2+r.safety_margin+padding;
  const double c=std::cos(p.yaw),s=std::sin(p.yaw);
  const double ex=std::abs(c)*hx+std::abs(s)*hy,ey=std::abs(s)*hx+std::abs(c)*hy;
  if(p.x-ex<r.bounds.min_x-kEpsilon || p.x+ex>r.bounds.max_x+kEpsilon ||
     p.y-ey<r.bounds.min_y-kEpsilon || p.y+ey>r.bounds.max_y+kEpsilon)return false;
  for(const auto& box:r.obstacles) {
    if(box.center_z+box.size_z/2<=p.z+1e-6)continue;
    const double dx=box.center_x-p.x,dy=box.center_y-p.y;
    const double radius=(box.size_x+box.size_y)/2;
    if(std::abs(dx)>ex+radius || std::abs(dy)>ey+radius)continue;
    bool separated=false;
    for(double axis:{p.yaw,p.yaw+std::acos(-1.)/2,box.yaw,box.yaw+std::acos(-1.)/2}) {
      const double d=std::abs(dx*std::cos(axis)+dy*std::sin(axis));
      const double extent=hx*std::abs(std::cos(p.yaw-axis))+hy*std::abs(std::sin(p.yaw-axis))+
                          box.size_x/2*std::abs(std::cos(box.yaw-axis))+box.size_y/2*std::abs(std::sin(box.yaw-axis));
      if(d>extent){separated=true;break;}
    }
    if(!separated)return false;
  }
  return supported(p,r,padding);
}

bool isMotionCollisionFree(const Pose& a, const Pose& b, const PlanRequest& r,
                           double translation_step, double rotation_step)
{
  if (!poseFinite(a) || !poseFinite(b) || std::abs(a.z-b.z)>1e-6 ||
      !std::isfinite(translation_step) || !std::isfinite(rotation_step) ||
      translation_step<=0 || rotation_step<=0) return false;
  const double turn = std::atan2(std::sin(b.yaw-a.yaw),std::cos(b.yaw-a.yaw));
  const double d = distance(a,b);
  if(std::abs(turn)<1e-10) {
    // A fixed-heading translation has an exact swept rectangle when aligned
    // with the body. Its rectangle bound is conservative for lateral motion.
    auto swept=r;
    const double dx=b.x-a.x,dy=b.y-a.y,c=std::cos(a.yaw),s=std::sin(a.yaw);
    swept.footprint.length+=std::abs(c*dx+s*dy);
    swept.footprint.width+=std::abs(-s*dx+c*dy);
    return isPoseCollisionFree({(a.x+b.x)/2,(a.y+b.y)/2,a.z,a.yaw},swept);
  }
  const double count = std::ceil(std::max(d/translation_step,std::abs(turn)/rotation_step));
  if (!std::isfinite(count) || count>100000) return false;
  const int n = std::max(1,static_cast<int>(count));
  const double radius=std::hypot(r.footprint.length/2+r.safety_margin,r.footprint.width/2+r.safety_margin);
  const double padding=d/(2*n)+2*radius*std::sin(std::abs(turn)/(4*n));
  if(!validate(r))return false;
  auto nearby=r;nearby.obstacles.clear();
  const double reach=radius+std::sqrt(2.)*(r.safety_margin+padding);
  for(const auto& box:r.obstacles) {
    const double extent=(box.size_x+box.size_y)/2;
    if(box.center_x+extent<std::min(a.x,b.x)-reach || box.center_x-extent>std::max(a.x,b.x)+reach ||
       box.center_y+extent<std::min(a.y,b.y)-reach || box.center_y-extent>std::max(a.y,b.y)+reach)continue;
    nearby.obstacles.push_back(box);
  }
  auto pose=[&](double t){return Pose{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z,a.yaw+t*turn};};
  if(!isPoseCollisionFree(a,nearby)||!isPoseCollisionFree(b,nearby))return false;
  // A failed padded sample can be a conservative bound touching a nearby wall.
  // Refine only that interval; every accepted interval still certifies its full
  // translation and rotation, with the configured safety margin unchanged.
  auto clear=[&](auto&& self,double lo,double hi,int depth)->bool {
    const auto p=pose((lo+hi)/2);
    const double span=hi-lo;
    const double extra=d*span/2+2*radius*std::sin(std::abs(turn)*span/4);
    if(isPoseCollisionFree(p,nearby,extra))return true;
    if(depth==8||!isPoseCollisionFree(p,nearby))return false;
    const double mid=(lo+hi)/2;
    return self(self,lo,mid,depth+1)&&self(self,mid,hi,depth+1);
  };
  for(int i=0;i<n;i++)if(!clear(clear,double(i)/n,double(i+1)/n,0))return false;
  return true;
}

PlanResult runPlanner(GlobalPlanner& planner, const PlanRequest& request)
{
  try {
    if (const auto reason=request.control.failure(); !reason.empty()) return {false,{},reason};
    auto begin=std::chrono::steady_clock::now();
    auto result=planner.plan(request);
    result.algorithm_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    begin=std::chrono::steady_clock::now();
    if (const auto reason=request.control.failure(); !reason.empty()) return {false,{},reason};
    if(!result.success) return {false,{},result.failure_reason.empty()?"PLANNER_FAILED":result.failure_reason};
    if(request.map) {
      auto reason=validateGlobalPath(request,result);
      if(!reason.empty())return {false,{},reason};
      if(auto reason=request.control.failure();!reason.empty())return {false,{},reason};
      result.validation_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      return result;
    }
    auto matches=[](const Pose& a,const Pose& b) {
      return poseFinite(a) && distance(a,b)<1e-6 && std::abs(a.z-b.z)<1e-6 &&
        std::abs(std::atan2(std::sin(a.yaw-b.yaw),std::cos(a.yaw-b.yaw)))<1e-6;
    };
    if(!validate(request) || result.path.empty() || !matches(result.path.front(),request.start) ||
        !matches(result.path.back(),request.goal)) return {false,{},"INVALID_GLOBAL_ENDPOINTS"};
    for(std::size_t i=0;i<result.path.size();++i) {
      if(std::abs(result.path[i].z-request.start.z)>1e-6 || !isPoseCollisionFree(result.path[i],request) ||
          (i && !isMotionCollisionFree(result.path[i-1],result.path[i],request)))
        return {false,{},"INVALID_GLOBAL_GEOMETRY"};
    }
    return result;
  } catch(const std::exception& e) { return {false,{},e.what()}; }
}


}
