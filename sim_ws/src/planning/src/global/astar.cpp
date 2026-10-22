#include "astar.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <unordered_map>

namespace planning
{
namespace
{
constexpr double kEpsilon = 1e-9;

bool finite(double value) { return std::isfinite(value); }
double distance(const Pose& a, const Pose& b) { return std::hypot(a.x-b.x, a.y-b.y); }

bool poseFinite(const Pose& p)
{
  return finite(p.x) && finite(p.y) && finite(p.z) && finite(p.yaw);
}

bool intersectsBox(const Pose& a, const Pose& b, const BoxObstacle& box)
{
  double first = 0.0, last = 1.0;
  const std::array<double, 2> start{a.x, a.y}, end{b.x, b.y};
  const std::array<double, 2> center{box.center_x, box.center_y};
  const std::array<double, 2> size{box.size_x, box.size_y};
  for (std::size_t axis = 0; axis < 2; ++axis) {
    const double lo = center[axis]-size[axis]/2, hi = center[axis]+size[axis]/2;
    const double delta = end[axis]-start[axis];
    if (std::abs(delta) < kEpsilon) {
      if (start[axis] < lo || start[axis] > hi) return false;
    } else {
      const double t0 = (lo-start[axis])/delta, t1 = (hi-start[axis])/delta;
      first = std::max(first, std::min(t0, t1));
      last = std::min(last, std::max(t0, t1));
      if (first > last) return false;
    }
  }
  return true;
}

double pointSegmentSquared(double x, double y, const Pose& a, const Pose& b)
{
  const double dx = b.x-a.x, dy = b.y-a.y, length2 = dx*dx+dy*dy;
  const double t = length2 > 0 ? std::clamp(((x-a.x)*dx+(y-a.y)*dy)/length2, 0.0, 1.0) : 0;
  const double ex = x-a.x-t*dx, ey = y-a.y-t*dy;
  return ex*ex+ey*ey;
}

double segmentBoxSquared(const Pose& a, const Pose& b, const BoxObstacle& box)
{
  if (intersectsBox(a, b, box)) return 0.0;
  auto pointBoxSquared = [&](const Pose& p) {
    const double dx = std::max(std::abs(p.x-box.center_x)-box.size_x/2, 0.0);
    const double dy = std::max(std::abs(p.y-box.center_y)-box.size_y/2, 0.0);
    return dx*dx+dy*dy;
  };
  double result = std::min(pointBoxSquared(a), pointBoxSquared(b));
  for (double sx : {-1.0, 1.0}) for (double sy : {-1.0, 1.0}) {
    result = std::min(result, pointSegmentSquared(box.center_x+sx*box.size_x/2,
                                                box.center_y+sy*box.size_y/2, a, b));
  }
  return result;
}

class CollisionChecker
{
public:
  explicit CollisionChecker(const PlanRequest& request)
  : request_(request), radius_(std::hypot(request.footprint.length/2+request.safety_margin,
                                        request.footprint.width/2+request.safety_margin)) {}

  double radius() const { return radius_; }

  bool circleFree(const Pose& point) const { return straightFree(point, point); }

  bool straightFree(const Pose& a, const Pose& b) const
  {
    const auto& bounds = request_.bounds;
    for (const auto& p : {a, b}) {
      if (p.x-radius_ < bounds.min_x-kEpsilon || p.x+radius_ > bounds.max_x+kEpsilon ||
          p.y-radius_ < bounds.min_y-kEpsilon || p.y+radius_ > bounds.max_y+kEpsilon) return false;
    }
    for (const auto& box : request_.obstacles) {
      if (relevant(box) && segmentBoxSquared(a, b, box) <= radius_*radius_) return false;
    }
    return true;
  }

  // Exact swept OBB for a translation parallel to a fixed robot heading.
  bool dockingFree(const Pose& a, const Pose& b, double yaw) const
  {
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double dx = b.x-a.x, dy = b.y-a.y;
    if (std::abs(-s*dx+c*dy) > 1e-7 || c*dx+s*dy < -kEpsilon) return false;
    const double hx = request_.footprint.length/2+request_.safety_margin+distance(a, b)/2;
    const double hy = request_.footprint.width/2+request_.safety_margin;
    const double x = (a.x+b.x)/2, y = (a.y+b.y)/2;
    const double ex = std::abs(c)*hx+std::abs(s)*hy;
    const double ey = std::abs(s)*hx+std::abs(c)*hy;
    const auto& bounds = request_.bounds;
    if (x-ex < bounds.min_x-kEpsilon || x+ex > bounds.max_x+kEpsilon ||
        y-ey < bounds.min_y-kEpsilon || y+ey > bounds.max_y+kEpsilon) return false;
    for (const auto& box : request_.obstacles) {
      if (!relevant(box)) continue;
      const double ox = box.size_x/2, oy = box.size_y/2;
      const double tx = box.center_x-x, ty = box.center_y-y;
      // Separating axes: world X/Y and robot X/Y. Touching counts as collision.
      if (std::abs(tx) > ex+ox || std::abs(ty) > ey+oy ||
          std::abs(c*tx+s*ty) > hx+std::abs(c)*ox+std::abs(s)*oy ||
          std::abs(-s*tx+c*ty) > hy+std::abs(s)*ox+std::abs(c)*oy) continue;
      return false;
    }
    return true;
  }

private:
  bool relevant(const BoxObstacle& box) const
  {
    return box.center_z+box.size_z/2 > request_.start.z+1e-6;
  }
  const PlanRequest& request_;
  double radius_;
};

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

bool endpointAnchor(const Pose& endpoint, bool is_start, double resolution,
                    const CollisionChecker& checker, Pose& anchor)
{
  if (!checker.dockingFree(endpoint, endpoint, endpoint.yaw)) return false;
  anchor = endpoint;
  if (checker.circleFree(anchor)) return true;
  // Search only the short approach needed to transition to a turning envelope.
  // Bound the loop independently of an excessively small grid resolution.
  const double step = std::max(resolution/2, checker.radius()/128);
  for (double d = step; d <= 2*checker.radius()+step; d += step) {
    const double direction = is_start ? 1.0 : -1.0;
    anchor = endpoint;
    anchor.x += direction*d*std::cos(endpoint.yaw);
    anchor.y += direction*d*std::sin(endpoint.yaw);
    if (checker.circleFree(anchor) &&
        (is_start ? checker.dockingFree(endpoint, anchor, endpoint.yaw)
                  : checker.dockingFree(anchor, endpoint, endpoint.yaw))) return true;
  }
  return false;
}

void appendDistinct(std::vector<Pose>& path, const Pose& point)
{
  if (path.empty() || distance(path.back(), point) > kEpsilon) path.push_back(point);
}
// Position/yaw lattice for rectangular robots in doorways and on finite slabs.
// Rotations are real swept motions; no circular-envelope rejection at door throats.
PlanResult orientedPlan(const PlanRequest& r,const AStarOptions& options) {
  auto failure=[](const std::string& reason){return PlanResult{false,{},reason};};
  if(!isPoseCollisionFree(r.start,r))return failure("START_COLLISION_OR_UNSUPPORTED");
  if(!isPoseCollisionFree(r.goal,r))return failure("GOAL_COLLISION_OR_UNSUPPORTED");
  constexpr int headings=8;
  const double pi=std::acos(-1.),step=options.resolution;
  const double min_x=r.start.x-std::floor((r.start.x-r.bounds.min_x)/step)*step;
  const double min_y=r.start.y-std::floor((r.start.y-r.bounds.min_y)/step)*step;
  const int nx=static_cast<int>(std::floor((r.bounds.max_x-min_x)/step))+1;
  const int ny=static_cast<int>(std::floor((r.bounds.max_y-min_y)/step))+1;
  if(nx<=0 || ny<=0 || static_cast<double>(nx)*ny*headings>options.max_grid_cells)return failure("GRID_LIMIT_EXCEEDED");
  const std::size_t count=static_cast<std::size_t>(nx)*ny*headings,none=std::numeric_limits<std::size_t>::max();
  auto id=[&](int x,int y,int h){return (static_cast<std::size_t>(y)*nx+x)*headings+h;};
  auto point=[&](std::size_t i){const int h=i%headings; i/=headings;
    return Pose{min_x+(i%nx)*step,min_y+(i/nx)*step,r.start.z,h*pi/4};};
  auto heading=[&](double yaw){int h=static_cast<int>(std::llround(yaw/(pi/4)))%headings;return (h+headings)%headings;};
  auto connect=[&](const Pose& a,const Pose& b) {
    std::vector<Pose> path{a};
    if(distance(a,b)>1e-8) {
      const double yaw=std::atan2(b.y-a.y,b.x-a.x);
      path.push_back({a.x,a.y,a.z,yaw});path.push_back({b.x,b.y,b.z,yaw});
    }
    path.push_back(b);
    for(std::size_t i=1;i<path.size();++i)
      if(!isMotionCollisionFree(path[i-1],path[i],r,.01,.02))return std::vector<Pose>{};
    return path;
  };
  const int sx=std::llround((r.start.x-min_x)/step),sy=std::llround((r.start.y-min_y)/step);
  const auto source=id(sx,sy,heading(r.start.yaw));
  auto start_path=connect(r.start,point(source));
  if(start_path.empty())return failure("START_ORIENTATION_BLOCKED");
  const int gx=std::llround((r.goal.x-min_x)/step),gy=std::llround((r.goal.y-min_y)/step);
  std::size_t target=none;std::vector<Pose> goal_path;double gap=INFINITY;
  for(int y=gy-2;y<=gy+2;++y)for(int x=gx-2;x<=gx+2;++x) {
    if(x<0||y<0||x>=nx||y>=ny)continue;
    const auto candidate=id(x,y,heading(r.goal.yaw));const auto p=point(candidate);
    if(distance(p,r.goal)>=gap)continue;
    auto path=connect(p,r.goal);
    if(!path.empty()){target=candidate;gap=distance(p,r.goal);goal_path=std::move(path);}
  }
  if(target==none)return failure("GOAL_APPROACH_BLOCKED");
  std::vector<double> costs(count,INFINITY);std::vector<std::size_t> parent(count,none);
  std::vector<signed char> free(count,-1);std::vector<bool> closed(count,false);
  using Item=std::pair<double,std::size_t>;std::priority_queue<Item,std::vector<Item>,std::greater<Item>> open;
  costs[source]=0;open.push({distance(point(source),r.goal),source});
  const std::array<int,8> dx{{1,1,0,-1,-1,-1,0,1}},dy{{0,1,1,1,0,-1,-1,-1}};
  auto valid=[&](std::size_t i){
    if(free[i]<0)free[i]=!isPoseCollisionFree(point(i),r)?0:(isPoseCollisionFree(point(i),r,.04)?1:2);
    return free[i]>0;
  };
  while(!open.empty()) {
    if(const auto reason=r.control.failure();!reason.empty())return failure(reason);
    const auto current=open.top().second;open.pop();if(closed[current])continue;closed[current]=true;
    if(current==target)break;
    const int h=current%headings,x=(current/headings)%nx,y=(current/headings)/nx;
    for(int action:{0,-1,1}) {
      int xx=x,yy=y,hh=h;double edge;
      if(action==0){xx+=dx[h];yy+=dy[h];edge=step*std::hypot(dx[h],dy[h]);}
      else{hh=(h+action+headings)%headings;edge=std::hypot(r.footprint.length,r.footprint.width)/2*pi/4;}
      if(xx<0||yy<0||xx>=nx||yy>=ny)continue;
      const auto next=id(xx,yy,hh);
      if(closed[next] || !valid(next))continue;
      if(free[next]==2)edge*=3;  // Prefer tracking room; retain legal narrow doors.
      if(costs[current]+edge>=costs[next])continue;
      if(!isMotionCollisionFree(point(current),point(next),r,.01,.02))continue;
      costs[next]=costs[current]+edge;parent[next]=current;
      open.push({costs[next]+distance(point(next),r.goal),next});
    }
  }
  if(!closed[target])return failure("NO_PATH");
  std::vector<Pose> lattice;
  for(auto i=target;i!=none;i=parent[i])lattice.push_back(point(i));
  std::reverse(lattice.begin(),lattice.end());
  std::vector<Pose> path=start_path;
  for(const auto& p:lattice) {
    if(path.size()>=2) {
      auto& a=path[path.size()-2];auto& b=path.back();
      const bool same_yaw=std::cos(a.yaw-b.yaw)>1-1e-8 && std::cos(b.yaw-p.yaw)>1-1e-8;
      const bool same_position=distance(a,b)<1e-8 && distance(b,p)<1e-8;
      if((same_yaw || same_position) && isMotionCollisionFree(a,p,r,.01,.02)){b=p;continue;}
    }
    path.push_back(p);
  }
  path.insert(path.end(),goal_path.begin()+1,goal_path.end());
  path.front()=r.start;path.back()=r.goal;
  // Remove grid zigzags through open space. Every shortcut still consists of
  // swept rotation, forward translation and final rotation; doors remain checked.
  std::vector<Pose> smooth{path.front()};
  for(std::size_t begin=0;begin+1<path.size();) {
    bool joined=false;
    for(std::size_t end=path.size()-1;end>begin+1;--end) {
      if(!r.control.failure().empty())return failure(r.control.failure());
      auto shortcut=connect(path[begin],path[end]);
      if(shortcut.empty())continue;
      smooth.insert(smooth.end(),shortcut.begin()+1,shortcut.end());begin=end;joined=true;break;
    }
    if(!joined)smooth.push_back(path[++begin]);
  }
  std::vector<Pose> distinct;
  for(const auto& p:smooth)
    if(distinct.empty() || distance(distinct.back(),p)>1e-8 || std::cos(distinct.back().yaw-p.yaw)<1-1e-8)
      distinct.push_back(p);
  distinct.front()=r.start;distinct.back()=r.goal;
  return {true,distinct,{}};
}
}  // namespace

PlanRequest regionRequest(const PlanRequest& r,const SupportRegion& region) {
  auto local=r;local.map.reset();local.bounds=region.bounds;local.supports=region.supports;
  local.obstacles=region.obstacles;local.require_support=true;
  local.start.z=local.goal.z=region.height;return local;
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
PlanResult spatialPlan(const PlanRequest& r,const AStarOptions& options) {
 auto fail=[](const std::string& reason){return PlanResult{false,{},reason};};
 const auto& map=*r.map;const double step=options.resolution,pi=std::acos(-1.);
 const auto& ability=r.traversal;
 if(!finite(ability.wheel_height) || ability.wheel_height<=0 ||
    !finite(ability.body_ground_clearance) || ability.body_ground_clearance<=0 ||
    !finite(ability.body_height) || ability.body_height<=0 ||
    !finite(ability.max_step_height) || ability.max_step_height<=0 ||
    !finite(ability.min_tread) || ability.min_tread<=0)return fail("INVALID_TRAVERSAL_CONSTRAINTS");
 if(map.regions.empty() || map.frame_id!=r.frame_id)return fail("INVALID_SCENE_MAP");
 std::vector<PlanRequest> layers;
 for(const auto& region:map.regions)layers.push_back(regionRequest(r,region));
 auto layerOf=[&](const Pose& p){for(std::size_t f=0;f<layers.size();++f)
  if(std::abs(p.z-map.regions[f].height)<1e-7 && isPoseCollisionFree(p,layers[f]))return int(f);return -1;};
 int sf=layerOf(r.start),gf=layerOf(r.goal);if(sf<0 || gf<0)return fail("INVALID_GLOBAL_ENDPOINT_SUPPORT");
 // One position/yaw lattice per support height, plus exact endpoint/portal
 // vertices. A single A* frontier can choose any number/direction of stairs.
 const double minx=r.bounds.min_x,miny=r.bounds.min_y;
 const int nx=std::floor((r.bounds.max_x-minx)/step)+1,ny=std::floor((r.bounds.max_y-miny)/step)+1;
 if(nx<=0 || ny<=0)return fail("INVALID_MAP_BOUNDS");
 const std::size_t per=std::size_t(nx)*ny*8,total=per*layers.size();
 auto id=[&](int f,int x,int y,int h){return f*per+(std::size_t(y)*nx+x)*8+h;};
 auto point=[&](std::size_t i){int f=i/per;i%=per;int h=i%8;i/=8;return Pose{minx+(i%nx)*step,miny+(i/nx)*step,map.regions[f].height,h*pi/4};};
 auto connect=[&](const Pose& a,const Pose& b,int f){
  std::vector<Pose> path{a};if(distance(a,b)>1e-8) {
   double yaw=std::atan2(b.y-a.y,b.x-a.x);path.push_back({a.x,a.y,a.z,yaw});path.push_back({b.x,b.y,b.z,yaw});
  }path.push_back(b);
  for(std::size_t i=1;i<path.size();++i)if(!isMotionCollisionFree(path[i-1],path[i],layers[f]))return std::vector<Pose>{};
  return path;
 };
 struct Edge {std::size_t to;std::vector<Pose> path;int connection{-1};};
 std::unordered_map<std::size_t,std::vector<Edge>> extra;
 std::vector<Pose> terminals{r.start,r.goal};std::vector<int> terminal_layers{sf,gf};
 for(std::size_t c=0;c<map.connections.size();++c) {
  const auto& connection=map.connections[c];
  if(!connection.enabled || !validateConnection(r,connection).empty())continue;
  auto a=connection.reference.front(),b=connection.reference.back();int af=layerOf(a),bf=layerOf(b);
  if(af<0 || bf<0)continue;
  std::size_t ai=total+terminals.size();terminals.push_back(a);terminal_layers.push_back(af);
  std::size_t bi=total+terminals.size();terminals.push_back(b);terminal_layers.push_back(bf);
  extra[ai].push_back({bi,connection.reference,int(c)});
 }
 for(std::size_t t=0;t<terminals.size();++t) {
  const auto& p=terminals[t];int f=terminal_layers[t];
  int x=std::llround((p.x-minx)/step),y=std::llround((p.y-miny)/step);
  int h=(int(std::llround(p.yaw/(pi/4)))%8+8)%8;
  for(int xx=x-1;xx<=x+1;++xx)for(int yy=y-1;yy<=y+1;++yy) {
   if(xx<0||yy<0||xx>=nx||yy>=ny)continue;
   auto v=id(f,xx,yy,h);auto path=connect(p,point(v),f);
   if(!path.empty())extra[total+t].push_back({v,std::move(path)});
   path=connect(point(v),p,f);if(!path.empty())extra[v].push_back({total+t,std::move(path)});
  }
  for(std::size_t u=0;u<t;++u)if(terminal_layers[u]==f && distance(p,terminals[u])<1e-8) {
   auto path=connect(p,terminals[u],f);if(!path.empty())extra[total+t].push_back({total+u,std::move(path)});
   path=connect(terminals[u],p,f);if(!path.empty())extra[total+u].push_back({total+t,std::move(path)});
  }
 }
 struct State {double cost{INFINITY};std::size_t switches{0},parent{0};Edge edge;bool closed{false};};
 std::unordered_map<std::size_t,State> states;
 std::unordered_map<std::size_t,bool> free;
 auto pos=[&](std::size_t i){return i<total?point(i):terminals[i-total];};
 struct Item {double estimate,cost;std::size_t switches,id;bool operator<(const Item& b)const {
  if(estimate!=b.estimate)return estimate>b.estimate;if(switches!=b.switches)return switches>b.switches;return id>b.id;
 }};
 std::priority_queue<Item> open;states[total].cost=0;open.push({spatialDistance(r.start,r.goal),0,0,total});
 const int dx[8]={1,1,0,-1,-1,-1,0,1},dy[8]={0,1,1,1,0,-1,-1,-1};
 while(!open.empty()) {
  if(auto reason=r.control.failure();!reason.empty())return fail(reason);
  auto item=open.top();open.pop();auto current=item.id;
  if(states[current].closed || item.cost!=states[current].cost)continue;
  states[current].closed=true;if(current==total+1)break;
  if(states.size()>options.max_grid_cells)return fail("GRID_LIMIT_EXCEEDED");
  auto relax=[&](const Edge& e) {
   double length=0;for(std::size_t j=1;j<e.path.size();++j)length+=spatialDistance(e.path[j-1],e.path[j]);
   double cost=item.cost+length;std::size_t switches=item.switches+(e.connection>=0?2:0);
   auto& next=states[e.to];if(next.closed || cost>next.cost+1e-10 ||
     (std::abs(cost-next.cost)<=1e-10 && switches>=next.switches))return;
   next.cost=cost;next.switches=switches;next.parent=current;next.edge=e;
   open.push({cost+spatialDistance(pos(e.to),r.goal),cost,switches,e.to});
  };
  if(auto found=extra.find(current);found!=extra.end())for(const auto& e:found->second)relax(e);
  if(current>=total)continue;
  int f=current/per,h=current%8,x=(current%per/8)%nx,y=(current%per/8)/nx;
  for(int action:{0,-1,1}) {
   int xx=x,yy=y,hh=h;if(action==0){xx+=dx[h];yy+=dy[h];}else hh=(h+action+8)%8;
   if(xx<0||yy<0||xx>=nx||yy>=ny)continue;
   auto v=id(f,xx,yy,hh);if(states.count(v) && states[v].closed)continue;
   auto cached=free.find(v);bool valid;
   if(cached==free.end()){valid=isPoseCollisionFree(point(v),layers[f]);free[v]=valid;}else valid=cached->second;
   if(!valid || !isMotionCollisionFree(point(current),point(v),layers[f]))continue;
   relax({v,{point(current),point(v)}});
  }
 }
 if(!states[total+1].closed)return fail("NO_PATH");
 std::vector<Edge> edges;for(std::size_t i=total+1;i!=total;i=states[i].parent)edges.push_back(states[i].edge);
 std::reverse(edges.begin(),edges.end());PlanResult result;result.success=true;result.path={r.start};
 auto wheel=[&](int f){if(result.segments.empty() || result.segments.back().mode!=TravelMode::Wheel ||
   result.segments.back().region_id!=map.regions[f].id)
   result.segments.push_back({result.path.size()-1,result.path.size()-1,TravelMode::Wheel,map.regions[f].id,{}});};
 wheel(sf);
 for(const auto& edge:edges) {
  if(edge.connection>=0) {
   const auto& c=map.connections[edge.connection];
   result.segments.push_back({result.path.size()-1,result.path.size()-1,TravelMode::Foot,{},c.id});
  } else wheel(layerOf(edge.path.front()));
  result.path.insert(result.path.end(),edge.path.begin()+1,edge.path.end());
  result.segments.back().end=result.path.size()-1;
 }
 wheel(gf);return result;
}
}

AStar::AStar(AStarOptions options) : options_(options)
{
  if (!finite(options_.resolution) || options_.resolution <= 0 || options_.max_grid_cells == 0) {
    throw std::invalid_argument("Invalid AStar resolution/grid limit");
  }
}

PlanResult AStar::plan(const PlanRequest& request)
{
  auto failure = [](const std::string& reason) { return PlanResult{false, {}, reason}; };
  if (const auto reason = request.control.failure(); !reason.empty()) return failure(reason);
  if (!validate(request)) return failure("INVALID_REQUEST");
  if(request.map)return spatialPlan(request,options_);
  if (std::abs(request.start.z-request.goal.z) > 1e-6) return failure("NON_PLANAR_REQUEST");
  if(request.require_support || std::any_of(request.obstacles.begin(),request.obstacles.end(),
      [](const auto& box){return std::abs(box.yaw)>1e-8;})) return orientedPlan(request,options_);
  const CollisionChecker checker(request);
  Pose start, goal;
  if (!endpointAnchor(request.start, true, options_.resolution, checker, start)) {
    return failure("START_COLLISION_OR_NO_ALIGNED_APPROACH");
  }
  if (!endpointAnchor(request.goal, false, options_.resolution, checker, goal)) {
    return failure("GOAL_COLLISION_OR_NO_ALIGNED_APPROACH");
  }
  const auto& bounds = request.bounds;
  const double nx_value = std::floor((bounds.max_x-bounds.min_x)/options_.resolution)+1;
  const double ny_value = std::floor((bounds.max_y-bounds.min_y)/options_.resolution)+1;
  if (!finite(nx_value) || !finite(ny_value) || nx_value < 1 || ny_value < 1 ||
      nx_value > static_cast<double>(options_.max_grid_cells) ||
      ny_value > static_cast<double>(options_.max_grid_cells)/nx_value) {
    return failure("GRID_LIMIT_EXCEEDED");
  }
  const auto nx = static_cast<std::size_t>(nx_value), ny = static_cast<std::size_t>(ny_value);
  const auto count = nx*ny;
  const auto none = std::numeric_limits<std::size_t>::max();
  auto point = [&](std::size_t id) {
    return Pose{bounds.min_x+static_cast<double>(id%nx)*options_.resolution,
                bounds.min_y+static_cast<double>(id/nx)*options_.resolution, start.z, 0};
  };
  auto connector = [&](const Pose& p) {
    const auto ix = std::llround((p.x-bounds.min_x)/options_.resolution);
    const auto iy = std::llround((p.y-bounds.min_y)/options_.resolution);
    std::size_t best = none;
    double best_distance = std::numeric_limits<double>::infinity();
    for (long long dy = -1; dy <= 1; ++dy) for (long long dx = -1; dx <= 1; ++dx) {
      const auto x = ix+dx, y = iy+dy;
      if (x < 0 || y < 0 || x >= static_cast<long long>(nx) || y >= static_cast<long long>(ny)) continue;
      const auto id = static_cast<std::size_t>(y)*nx+static_cast<std::size_t>(x);
      const double d = distance(p, point(id));
      if (d < best_distance && checker.straightFree(p, point(id))) { best = id; best_distance = d; }
    }
    return best;
  };
  const auto source = connector(start), target = connector(goal);
  if (source == none || target == none) return failure("ENDPOINT_GRID_CONNECTOR_BLOCKED");
  std::vector<double> costs(count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parent(count, none);
  std::vector<bool> closed(count, false);
  using Item = std::pair<double, std::size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  costs[source] = 0;
  open.emplace(distance(point(source), point(target)), source);
  while (!open.empty()) {
    if (const auto reason = request.control.failure(); !reason.empty()) return failure(reason);
    const auto id = open.top().second;
    open.pop();
    if (closed[id]) continue;
    closed[id] = true;
    if (id == target) break;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;
      const auto x = static_cast<long long>(id%nx)+dx, y = static_cast<long long>(id/nx)+dy;
      if (x < 0 || y < 0 || x >= static_cast<long long>(nx) || y >= static_cast<long long>(ny)) continue;
      const auto next = static_cast<std::size_t>(y)*nx+static_cast<std::size_t>(x);
      if (closed[next] || !checker.straightFree(point(id), point(next))) continue;
      const double cost = costs[id]+options_.resolution*std::hypot(dx, dy);
      if (cost < costs[next]) {
        costs[next] = cost;
        parent[next] = id;
        open.emplace(cost+distance(point(next), point(target)), next);
      }
    }
  }
  if (!closed[target]) return failure("NO_PATH");
  std::vector<Pose> grid;
  for (auto id = target; id != none; id = parent[id]) grid.push_back(point(id));
  std::reverse(grid.begin(), grid.end());
  std::vector<Pose> raw{start};
  for (const auto& p : grid) appendDistinct(raw, p);
  appendDistinct(raw, goal);
  std::vector<Pose> simplified{raw.front()};
  for (std::size_t i = 0; i+1 < raw.size();) {
    std::size_t j = raw.size()-1;
    while (j > i+1 && !checker.straightFree(raw[i], raw[j])) --j;
    simplified.push_back(raw[j]);
    i = j;
  }
  std::vector<Pose> path{request.start};
  for (const auto& p : simplified) appendDistinct(path, p);
  appendDistinct(path, request.goal);
  for (std::size_t i = 0; i+1 < path.size(); ++i) {
    path[i].yaw = std::atan2(path[i+1].y-path[i].y, path[i+1].x-path[i].x);
  }
  path.front().yaw = request.start.yaw;
  if (path.size() == 1 && std::abs(request.start.yaw-request.goal.yaw) > kEpsilon) {
    path.push_back(request.goal);
  }
  path.back() = request.goal;
  return PlanResult{true, std::move(path), {}};
}

std::unique_ptr<GlobalPlanner> makeGlobalPlanner(const std::string& name, AStarOptions options)
{
  if (name == "astar") return std::make_unique<AStar>(options);
  throw std::invalid_argument("Unknown global planner: "+name);
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
  const double radius=std::hypot(r.footprint.length,r.footprint.width)/2;
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
  for(int i=0;i<=n;i++) {
    const double t=static_cast<double>(i)/n;
    if(!isPoseCollisionFree({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z,a.yaw+t*turn},nearby,padding)) return false;
  }
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


std::unique_ptr<GlobalPlanner> makeGlobalPlanner(const std::string& name,const PlannerParameters& parameters) {
  AStarOptions options;
  for(const auto& p:parameters) {
    if(!std::isfinite(p.second))throw std::invalid_argument("NONFINITE_ALGORITHM_PARAMETER: "+p.first);
    if(p.first=="resolution")options.resolution=static_cast<double>(p.second);
    else if(p.first=="max_grid_cells") {if(p.second<1 || p.second>double(std::numeric_limits<std::size_t>::max()/2) || std::floor(p.second)!=p.second)throw std::invalid_argument("INVALID_MAX_GRID_CELLS");options.max_grid_cells=static_cast<std::size_t>(p.second);}
    else throw std::invalid_argument("UNKNOWN_ALGORITHM_PARAMETER: "+p.first);
  }
  return makeGlobalPlanner(name,options);
}
}  // namespace planning
