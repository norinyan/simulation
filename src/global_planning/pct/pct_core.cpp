#include "pct_core.h"
#include "a_star/a_star_search.h"
#include "ele_planner/offline_ele_planner.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <sstream>

namespace pct_native {
namespace {
template<class T> T read(std::istream& in) { T value; in.read(reinterpret_cast<char*>(&value),sizeof(value)); if(!in)throw std::runtime_error("PCT_MAP_TRUNCATED"); return value; }
struct Cell {int l,x,y;};
Cell locate(const PctMap& m,const Pose& p,double threshold,double tolerance=.08) {
  const int x=std::lround((p.x-m.center_x)/m.resolution)+m.nx/2;
  const int y=std::lround((p.y-m.center_y)/m.resolution)+m.ny/2;
  if(x<0||y<0||x>=int(m.nx)||y>=int(m.ny))throw std::runtime_error("PCT_ENDPOINT_OUTSIDE_MAP");
  int layer=-1;double best=tolerance;
  for(int l=0;l<int(m.layers);++l) {
    const double g=m.at(3,l,x,y),c=m.at(4,l,x,y),cost=m.at(0,l,x,y),d=std::abs(g-p.z);
    if(std::isfinite(g)&&std::isfinite(cost)&&cost<=threshold&&(!std::isfinite(c)||c-g>=m.clearance-1e-5)&&d<best) {layer=l;best=d;}
  }
  if(layer<0){std::ostringstream why;why<<"PCT_ENDPOINT_UNREACHABLE_OR_WRONG_HEIGHT xyz="<<p.x<<","<<p.y<<","<<p.z<<" cell="<<x<<","<<y;
    for(int l=0;l<int(m.layers);++l)why<<" ["<<l<<":g="<<m.at(3,l,x,y)<<",c="<<m.at(4,l,x,y)<<",cost="<<m.at(0,l,x,y)<<"]";
    throw std::runtime_error(why.str());}
  return {layer,x,y};
}
Pose world(const PctMap& m,double x,double y,double z) {
  return {m.center_x+(x-int(m.nx/2))*m.resolution,m.center_y+(y-int(m.ny/2))*m.resolution,z,0};
}
}
std::shared_ptr<const PctMap> loadPctMap(const std::string& filename) {
  std::ifstream in(filename,std::ios::binary);if(!in)throw std::runtime_error("PCT_MAP_NOT_FOUND: "+filename);
  char magic[8]{};in.read(magic,8);if(!in)throw std::runtime_error("PCT_MAP_TRUNCATED");if(std::string(magic,8)!="PCTMAP02")throw std::runtime_error("PCT_MAP_BAD_FORMAT");
  auto m=std::make_shared<PctMap>();m->layers=read<uint32_t>(in);m->nx=read<uint32_t>(in);m->ny=read<uint32_t>(in);
  m->resolution=read<double>(in);m->center_x=read<double>(in);m->center_y=read<double>(in);m->max_step=read<double>(in);m->clearance=read<double>(in);m->inflation_radius=read<double>(in);
  if(!std::isfinite(m->center_x)||!std::isfinite(m->center_y)||!std::isfinite(m->max_step)||m->max_step<=0||!std::isfinite(m->clearance)||m->clearance<=0||!std::isfinite(m->inflation_radius)||m->inflation_radius<0||!m->layers||!m->nx||!m->ny||m->layers>256||m->nx>10000||m->ny>10000||m->cells()>50000000||!std::isfinite(m->resolution)||m->resolution<=0)throw std::runtime_error("PCT_MAP_INVALID_DIMENSIONS");
  m->data.resize(5*m->cells());in.read(reinterpret_cast<char*>(m->data.data()),m->data.size()*sizeof(float));if(!in)throw std::runtime_error("PCT_MAP_TRUNCATED");
  return m;
}
std::string validatePctPath(const PlanRequest& r,const std::vector<Pose>& path) {
  try {
    if(!r.pct_map||path.empty())return "PCT_EMPTY_MAP_OR_PATH";
    const auto& m=*r.pct_map;
    if(std::hypot(r.footprint.length,r.footprint.width)/2+r.safety_margin>m.inflation_radius+1e-5)return "PCT_MAP_FOOTPRINT_TOO_SMALL";
    for(std::size_t i=0;i<path.size();++i) {
      if(const auto reason=r.control.failure();!reason.empty())return reason;
      locate(m,path[i],r.cost_threshold);
      if(!i)continue;
      const auto& a=path[i-1];const auto& b=path[i];
      if(std::abs(a.z-b.z)>m.max_step+1e-6)return "PCT_HEIGHT_JUMP";
      const int n=std::max(1,int(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/(m.resolution*.4))));
      for(int j=1;j<n;++j) {
        const double t=double(j)/n;
        // Stair discontinuities are supported by either adjacent tread; do not
        // interpret the interpolated vertical gap as missing ground.
        Pose q{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z,0};
        try {locate(m,q,r.cost_threshold);} catch(const std::exception&) {q.z=b.z;locate(m,q,r.cost_threshold);}
      }
    }
    return {};
  } catch(const std::exception& e) {return std::string("PCT_VALIDATION: ")+e.what();}
}
PlanResult PctPlanner::plan(const PlanRequest& r) {
  auto fail=[](const std::string& why){return PlanResult{false,{},why,{}};};
  if(!r.pct_map)return fail("PCT_MAP_REQUIRED");
  for(const auto& p:{r.start,r.goal})if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(p.yaw))return fail("INVALID_REQUEST");
  if(!std::isfinite(r.footprint.length)||!std::isfinite(r.footprint.width)||r.footprint.length<=0||r.footprint.width<=0||!std::isfinite(r.safety_margin)||r.safety_margin<0)return fail("INVALID_REQUEST");
  if(const auto reason=r.control.failure();!reason.empty())return fail(reason);
  const auto& m=*r.pct_map;
  if(!std::isfinite(options_.reference_height)||options_.reference_height<0||!std::isfinite(options_.max_heading_rate)||options_.max_heading_rate<=0)return fail("PCT_INVALID_OPTIONS");
  if(std::hypot(r.footprint.length,r.footprint.width)/2+r.safety_margin>m.inflation_radius+1e-5)return fail("PCT_MAP_FOOTPRINT_TOO_SMALL");
  const auto start=locate(m,r.start,options_.cost_threshold),goal=locate(m,r.goal,options_.cost_threshold);
  const int rows=m.layers*m.nx,cols=m.ny;
  Eigen::MatrixXd cost(rows,cols),height(rows,cols),ceiling(rows,cols),gateway=Eigen::MatrixXd::Zero(rows,cols),gx(rows,cols),gy(rows,cols);
  for(int l=0;l<int(m.layers);++l)for(int x=0;x<int(m.nx);++x)for(int y=0;y<int(m.ny);++y) {
    const int row=l*m.nx+x;
    cost(row,y)=m.at(0,l,x,y);height(row,y)=std::isfinite(m.at(3,l,x,y))?m.at(3,l,x,y):-100;
    ceiling(row,y)=std::isfinite(m.at(4,l,x,y))?m.at(4,l,x,y):1e6;
    gx(row,y)=m.at(2,l,x,y);gy(row,y)=-m.at(1,l,x,y);
    if(!std::isfinite(cost(row,y)))cost(row,y)=50;
    // Upstream gateway rule: same actual surface, cheaper adjacent slice.
    if(l+1<int(m.layers)&&std::isfinite(m.at(3,l,x,y))&&std::isfinite(m.at(3,l+1,x,y))&&std::abs(m.at(3,l+1,x,y)-m.at(3,l,x,y))<.1) {
      const double difference=m.at(0,l+1,x,y)-m.at(0,l,x,y);
      if(difference < -8)gateway(row,y)=2;
      if(difference > 8)gateway(row+m.nx,y)=-2;
    }
  }
  // Missing or blocked surfaces never become gateways, including against NaN.
  OfflineElePlanner backend(options_.max_heading_rate,true);
  if(!std::isfinite(options_.optimizer_cost_margin)||options_.optimizer_cost_margin<0||options_.optimizer_cost_margin>=options_.cost_threshold||options_.optimizer_sample_interval<1||options_.optimizer_iterations<1)return fail("PCT_INVALID_OPTIMIZER_OPTIONS");
  backend.InitMap(options_.cost_threshold,options_.optimizer_cost_margin,m.resolution,m.layers,options_.step_cost_weight,cost,height,ceiling,gateway,gx,gy);
  if(!std::isfinite(options_.optimizer_jerk_sigma)||options_.optimizer_jerk_sigma<=0)return fail("PCT_INVALID_OPTIMIZER_OPTIONS");
  backend.ConfigureOptimizer(options_.optimizer_sample_interval,options_.optimizer_iterations,options_.optimizer_jerk_sigma);
  backend.SetReferenceHeight(options_.reference_height);
  backend.SetStopCheck([&r](){return !r.control.failure().empty();});
  backend.SetTransitionCheck([&r,&m](const Node& a,const Node& b){
    return validatePctPath(r,{world(m,a.idx[1],a.idx[2],a.height),world(m,b.idx[1],b.idx[2],b.height)}).empty();
  });
  if(!backend.Plan(Eigen::Vector3i(start.l,start.y,start.x),Eigen::Vector3i(goal.l,goal.y,goal.x),false))return fail(r.control.failure().empty()?"PCT_NO_PATH":r.control.failure());
  const auto& search=backend.get_path_finder();const auto points=search.GetPathPoints();
  PlanResult result;result.success=true;result.path.push_back(r.start);
  for(const auto& p:points) {
    auto q=world(m,p.y,p.x,p.height);
    if(std::hypot(q.x-result.path.back().x,q.y-result.path.back().y)>1e-8||std::abs(q.z-result.path.back().z)>1e-8)result.path.push_back(q);
  }
  if(std::hypot(result.path.back().x-r.goal.x,result.path.back().y-r.goal.y)>1e-8||std::abs(result.path.back().z-r.goal.z)>1e-8)result.path.push_back(r.goal);
  if(result.path.size()==1&&std::abs(r.start.yaw-r.goal.yaw)>1e-8)result.path.push_back(r.goal);
  result.path.front()=r.start;result.path.back()=r.goal;
  for(std::size_t i=1;i+1<result.path.size();++i)result.path[i].yaw=std::atan2(result.path[i+1].y-result.path[i].y,result.path[i+1].x-result.path[i].x);
  if(const auto reason=validatePctPath(r,result.path);!reason.empty())return fail(reason);
  if(options_.optimize&&points.size()>2) {
    if(!backend.OptimizePath())return fail("PCT_OPTIMIZATION_FAILED");
    const auto& optimizer=backend.get_trajectory_optimizer_wnoj();const auto traj=optimizer.GetResultMatrix();const auto z=optimizer.GetResultHeight();
    if(traj.rows()!=z.size()||traj.cols()<4||!traj.allFinite()||!z.allFinite())return fail("PCT_INVALID_OPTIMIZED_REFERENCE");
    for(int i=0;i<traj.rows();++i)result.body_reference.push_back(world(m,traj(i,3),traj(i,0),z(i)));
    // Reject an optimizer result outside the same free/support corridor; never
    // silently substitute the search route after an optimization failure.
    try {
      auto start_body=r.start;start_body.z+=options_.reference_height;
      auto goal_body=r.goal;goal_body.z+=options_.reference_height;
      result.body_reference.insert(result.body_reference.begin(),start_body);
      result.body_reference.push_back(goal_body);
      for(std::size_t i=0;i<result.body_reference.size();++i) {
        const auto& b=result.body_reference[i];const auto& a=i?result.body_reference[i-1]:b;
        const int count=std::max(1,int(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/(m.resolution*.4))));
        for(int k=0;k<=count;++k) {
          const double t=double(k)/count;Pose q{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)-options_.reference_height,0};
          const auto cell=locate(m,q,r.cost_threshold,m.max_step+.03);
          const double ceiling=m.at(4,cell.l,cell.x,cell.y);
          if(std::isfinite(ceiling)&&ceiling-(q.z+options_.reference_height)<.10)throw std::runtime_error("TOP_CLEARANCE");
        }
      }
      for(std::size_t i=1;i+1<result.body_reference.size();++i)result.body_reference[i].yaw=std::atan2(result.body_reference[i+1].y-result.body_reference[i].y,result.body_reference[i+1].x-result.body_reference[i].x);
    } catch(const std::exception& e) {return fail(std::string("PCT_OPTIMIZED_REFERENCE_INVALID: ")+e.what());}
  }
  if(const auto reason=r.control.failure();!reason.empty())return fail(reason);
  return result;
}
} // namespace pct_native
