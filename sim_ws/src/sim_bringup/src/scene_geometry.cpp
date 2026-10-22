#include "scene_geometry.h"
#include <sdf/sdf.hh>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <atomic>
#include <iostream>
#include <stdexcept>

namespace simulation {
namespace {
std::atomic_size_t parse_count{0};
using Pose = ignition::math::Pose3d;
using Vector = ignition::math::Vector3d;

Pose pose(const sdf::ElementPtr& element) {
  if(!element->HasElement("pose")) return Pose::Zero;
  auto p=element->GetElement("pose");
  if(p->HasAttribute("relative_to") && !p->Get<std::string>("relative_to").empty())
    throw std::runtime_error("Unsupported relative_to in collision geometry");
  return p->Get<Pose>();
}

planning::BoxObstacle projected(const Pose& p,const Vector& size,double low,double high) {
  // Intersect the oriented box with the robot's height slab, then conservatively
  // bound its XY projection. Horizontal boxes retain exact yaw below.
  if(std::abs(p.Rot().Roll())+std::abs(p.Rot().Pitch())<1e-7)
    return {p.Pos().X(),p.Pos().Y(),p.Pos().Z(),size.X(),size.Y(),size.Z(),p.Rot().Yaw()};
  Vector lo(std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity(),low);
  Vector hi(-lo.X(),-lo.Y(),high);
  std::vector<Vector> corners;
  for(int x:{-1,1})for(int y:{-1,1})for(int z:{-1,1})
    corners.push_back(p.Pos()+p.Rot().RotateVector(Vector(x*size.X()/2,y*size.Y()/2,z*size.Z()/2)));
  auto include=[&](const Vector& v){lo.X(std::min(lo.X(),v.X()));lo.Y(std::min(lo.Y(),v.Y()));
                                    hi.X(std::max(hi.X(),v.X()));hi.Y(std::max(hi.Y(),v.Y()));};
  for(const auto& c:corners)if(c.Z()>=low && c.Z()<=high)include(c);
  for(std::size_t i=0;i<8;++i)for(std::size_t j=i+1;j<8;++j) {
    const auto difference=i^j;
    if(difference!=1 && difference!=2 && difference!=4)continue;
    for(double z:{low,high})if((corners[i].Z()-z)*(corners[j].Z()-z)<=0 &&
                              std::abs(corners[i].Z()-corners[j].Z())>1e-10) {
      const double t=(z-corners[i].Z())/(corners[j].Z()-corners[i].Z());
      include(corners[i]+t*(corners[j]-corners[i]));
    }
  }
  if(!std::isfinite(lo.X()))throw std::runtime_error("Cannot project tilted collision");
  return {(lo.X()+hi.X())/2,(lo.Y()+hi.Y())/2,(low+high)/2,
          std::max(1e-6,hi.X()-lo.X()),std::max(1e-6,hi.Y()-lo.Y()),high-low,0};
}
}

static FloorGeometry extractFloorGeometry(const sdf::ElementPtr& world,
                                const planning::Pose& start,const planning::Pose& goal,
                                planning::RobotFootprint footprint,double height,double margin) {
  if(std::abs(start.z-goal.z)>.003)throw std::runtime_error("CROSS_FLOOR_NOT_SUPPORTED_IN_STAGE2");
  FloorGeometry result;auto& r=result.geometry;
  r.frame_id="world";r.start=start;r.goal=goal;r.footprint=footprint;r.safety_margin=margin;r.require_support=true;
  double xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY;
  for(auto model=world->GetElement("model");model;model=model->GetNextElement("model")) {
    const auto name=model->Get<std::string>("name");
    if(model->HasElement("model"))throw std::runtime_error("Nested model not supported: "+name);
    const auto mp=pose(model);
    bool pedestrian=false;
    sdf::ElementPtr patrol;
    if(model->HasElement("plugin"))for(auto plugin=model->GetElement("plugin");plugin;plugin=plugin->GetNextElement("plugin"))
      if(plugin->Get<std::string>("filename")=="libscene_controller.so"){pedestrian=true;patrol=plugin;}
    if(!model->HasElement("link"))continue;
    for(auto link=model->GetElement("link");link;link=link->GetNextElement("link")) {
      if(!link->HasElement("collision"))continue;
      for(auto collision=link->GetElement("collision");collision;collision=collision->GetNextElement("collision")) {
        auto geometry=collision->GetElement("geometry");
        // Gazebo's infinite ground plane only supports F1; building slabs provide
        // the actual finite navigation extent, including upper-floor cutouts.
        if(geometry->HasElement("plane") && name=="ground_plane")continue;
        if(!geometry->HasElement("box"))throw std::runtime_error("Unsupported collision shape: "+name);
        const auto size=geometry->GetElement("box")->Get<Vector>("size");
        const auto local=pose(link)*pose(collision),p=mp*local;
        if(pedestrian) {
          if(std::abs(mp.Pos().Z()-start.z)<.01) {
            MovingBox moving{name,local,size};
            const double speed=patrol->Get<double>("speed");
            if(!std::isfinite(speed) || speed<=0)throw std::runtime_error("Invalid pedestrian speed");
            moving.delay=patrol->HasElement("start_delay")?patrol->Get<double>("start_delay"):0;
            std::vector<std::pair<Vector,double>> points;
            for(auto w=patrol->GetElement("waypoint");w;w=w->GetNextElement("waypoint"))
              points.push_back({w->Get<Vector>("position"),w->HasElement("pause")?w->Get<double>("pause"):0});
            if(points.size()<2)throw std::runtime_error("Invalid pedestrian route");
            auto phase=[&](std::size_t a,std::size_t b) {
              const auto delta=points[b].first-points[a].first;
              const double yaw=std::atan2(delta.Y(),delta.X());
              if(points[a].second>0) {
                moving.phases.push_back({points[a].first,points[a].first,moving.period,points[a].second,yaw});
                moving.period+=points[a].second;
              }
              const double duration=delta.Length()/speed;
              if(duration<=0)throw std::runtime_error("Invalid pedestrian segment");
              moving.phases.push_back({points[a].first,points[b].first,moving.period,duration,yaw});
              moving.period+=duration;
            };
            const bool reverse=patrol->HasElement("initial_direction") && patrol->Get<std::string>("initial_direction")=="reverse";
            auto forward=[&](){for(std::size_t i=0;i+1<points.size();++i)phase(i,i+1);};
            auto backward=[&](){for(std::size_t i=points.size()-1;i>0;--i)phase(i,i-1);};
            if(reverse){backward();forward();}else{forward();backward();}
            if(patrol->HasElement("paused") && patrol->Get<bool>("paused")) {
              const auto initial=moving.phases.front();
              moving.delay=0;moving.period=1;
              moving.phases={{initial.from,initial.from,0,1,initial.yaw}};
            }
            result.pedestrians.push_back(std::move(moving));
          }
          continue;
        }
        const auto rotation=p.Rot();
        const double extent=std::abs(rotation.RotateVector(Vector(size.X()/2,0,0)).Z())+
                            std::abs(rotation.RotateVector(Vector(0,size.Y()/2,0)).Z())+
                            std::abs(rotation.RotateVector(Vector(0,0,size.Z()/2)).Z());
        const double bottom=p.Pos().Z()-extent,top=p.Pos().Z()+extent;
        const bool horizontal=std::abs(rotation.Roll())+std::abs(rotation.Pitch())<1e-7;
        if(horizontal && std::abs(top-start.z)<.003) {
          auto box=projected(p,size,bottom,top);r.supports.push_back(box);
          const double ex=std::abs(std::cos(box.yaw))*size.X()/2+std::abs(std::sin(box.yaw))*size.Y()/2;
          const double ey=std::abs(std::sin(box.yaw))*size.X()/2+std::abs(std::cos(box.yaw))*size.Y()/2;
          xmin=std::min(xmin,box.center_x-ex);xmax=std::max(xmax,box.center_x+ex);
          ymin=std::min(ymin,box.center_y-ey);ymax=std::max(ymax,box.center_y+ey);
        }
        const double low=std::max(bottom,start.z+.005),high=std::min(top,start.z+height);
        if(high>low+1e-6)r.obstacles.push_back(projected(p,size,low,high));
      }
    }
  }
  if(r.supports.empty())throw std::runtime_error("FLOOR_HAS_NO_SUPPORT_SURFACES");
  r.bounds={xmin,xmax,ymin,ymax};
  return result;
}

std::size_t worldParseCount() {return parse_count.load();}
SceneGeometry loadSceneGeometry(const std::string& filename,
 const std::string& world_name,planning::RobotFootprint footprint,double height,double margin) {
 SceneGeometry prepared;
 ++parse_count;std::clog<<"SCENE_WORLD_PARSE count="<<parse_count<<" file="<<filename<<std::endl;
 sdf::SDFPtr document(new sdf::SDF());sdf::init(document);
 if(!sdf::readFile(filename,document))throw std::runtime_error("Cannot read loaded World");
 auto world=document->Root()->GetElement("world");
 if(world->Get<std::string>("name")!=world_name)throw std::runtime_error("WORLD_NAME_MISMATCH");
 auto result=std::make_shared<planning::SceneMap>();result->frame_id="world";
 std::map<std::string,planning::BoxObstacle> named;
 for(auto model=world->GetElement("model");model;model=model->GetNextElement("model")) {
  const auto name=model->Get<std::string>("name");
  bool dynamic=false;
  if(model->HasElement("plugin"))for(auto p=model->GetElement("plugin");p;p=p->GetNextElement("plugin"))
   if(p->Get<std::string>("filename")=="libscene_controller.so")dynamic=true;
  if(dynamic || !model->HasElement("link"))continue;
  if(model->HasElement("model"))throw std::runtime_error("Nested model unsupported");
  for(auto link=model->GetElement("link");link;link=link->GetNextElement("link")) {
   if(!link->HasElement("collision"))continue;
   for(auto c=link->GetElement("collision");c;c=c->GetNextElement("collision")) {
    auto g=c->GetElement("geometry");
    if(g->HasElement("plane") && name=="ground_plane")continue;
    if(!g->HasElement("box"))throw std::runtime_error("Unsupported World collision: "+name);
    auto size=g->GetElement("box")->Get<Vector>("size");auto p=pose(model)*pose(link)*pose(c);
    planning::BoxObstacle b{p.Pos().X(),p.Pos().Y(),p.Pos().Z(),size.X(),size.Y(),size.Z(),p.Rot().Yaw(),p.Rot().Roll(),p.Rot().Pitch()};
    result->solids.push_back(b);named[name]=b;
   }
  }
 }
 if(world_name=="two_steps") {
  std::vector<double> levels;
  for(const auto& item:named)if(item.first.find("platform")!=std::string::npos || item.first.find("floor")!=std::string::npos) {
    const auto& b=item.second; const double z=b.center_z+b.size_z/2;
    if(std::none_of(levels.begin(),levels.end(),[&](double x){return std::abs(x-z)<.003;}))levels.push_back(z);
  }
  // The legacy scene has an infinite ground plane and explicit upper platform;
  // its existing task adapter retains bounded supports/config geometry.
  prepared.map=std::move(result);return prepared;
 }
 // Semantic slab/tread names identify topology; all dimensions and heights come
 // from the loaded World, never from preset task names or copied scene geometry.
 for(int floor=1;floor<=3;++floor) {
  auto slab=named.at("f"+std::to_string(floor)+"_slab_01");
  double z=slab.center_z+slab.size_z/2;
  auto floor_geometry=extractFloorGeometry(world,{0,0,z,0},{0,0,z,0},footprint,height,margin);
  const auto& geometry=floor_geometry.geometry;
  result->regions.push_back({"F"+std::to_string(floor),z,geometry.bounds,geometry.supports,geometry.obstacles});
  prepared.floors.push_back(std::move(floor_geometry));
 }
 for(int floor=1;floor<=2;++floor) {
  const auto prefix="f"+std::to_string(floor)+"_stair_";
  auto landing=named.at(prefix+"mid_landing");
  auto first=named.at(prefix+"south_tread_01"),last=named.at(prefix+"north_tread_10");
  const double lower=result->regions[floor-1].height,upper=result->regions[floor].height;
  planning::StairConnection up;
  up.id=prefix+"up";up.from=result->regions[floor-1].id;up.to=result->regions[floor].id;
  up.platforms={landing};
  up.reference.push_back({first.center_x-first.size_x/2-.45,first.center_y,lower,0});
  double z=lower;
  for(const auto& flight: {std::string("south"),std::string("north")}) {
   const double direction=flight=="south"?1:-1,yaw=direction>0?0:std::acos(-1.);
   if(flight=="north") {
    const double top=landing.center_z+landing.size_z/2;
    const double y=named.at(prefix+"north_tread_01").center_y;
    up.reference.push_back({landing.center_x,first.center_y,top,0});
    up.reference.push_back({landing.center_x,first.center_y,top,std::acos(-1.)/2});
    up.reference.push_back({landing.center_x,y,top,std::acos(-1.)/2});
    up.reference.push_back({landing.center_x,y,top,yaw});z=top;
   }
   for(int i=1;i<=10;++i) {
    auto b=named.at(prefix+flight+"_tread_"+(i<10?"0":"")+std::to_string(i));up.treads.push_back(b);
    const double x=b.center_x-direction*b.size_x/2,top=b.center_z+b.size_z/2;
    up.reference.push_back({x,b.center_y,z,yaw});
    up.reference.push_back({x,b.center_y,top,yaw});
    up.reference.push_back({b.center_x+direction*b.size_x/2,b.center_y,top,yaw});z=top;
   }
  }
  up.reference.push_back({last.center_x-last.size_x/2-.45,last.center_y,upper,std::acos(-1.)});
  result->connections.push_back(up);
  auto down=up;down.id=prefix+"down";std::swap(down.from,down.to);
  std::reverse(down.reference.begin(),down.reference.end());
  for(auto& p:down.reference)p.yaw=std::atan2(std::sin(p.yaw+std::acos(-1.)),std::cos(p.yaw+std::acos(-1.)));
  result->connections.push_back(down);
 }
 prepared.map=std::move(result);return prepared;
}
std::shared_ptr<const planning::SceneMap> loadSceneMap(const std::string& filename,
 const std::string& world_name,planning::RobotFootprint footprint,double height,double margin) {
 return loadSceneGeometry(filename,world_name,footprint,height,margin).map;
}
}  // namespace simulation
