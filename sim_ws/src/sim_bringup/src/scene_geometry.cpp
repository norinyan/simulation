#include "scene_geometry.h"
#include <sdf/sdf.hh>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace simulation {
namespace {
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

planning::BoxObstacle movingBox(const MovingBox& model,const Pose& p) {
  const auto world=p*model.local_pose;
  return {world.Pos().X(),world.Pos().Y(),world.Pos().Z(),model.size.X(),model.size.Y(),model.size.Z(),world.Rot().Yaw()};
}

Pose patrolPose(const MovingBox& model,double time) {
  if(model.phases.empty() || model.period<=0)throw std::runtime_error("Missing pedestrian patrol phases");
  const double elapsed=time-model.delay;
  const double t=elapsed<0?0:std::fmod(elapsed,model.period);
  for(const auto& phase:model.phases)if(t<phase.begin+phase.duration) {
    const auto p=phase.from+(phase.to-phase.from)*std::clamp((t-phase.begin)/phase.duration,0.,1.);
    return Pose(p,ignition::math::Quaterniond(0,0,phase.yaw));
  }
  throw std::runtime_error("Invalid pedestrian phase time");
}

FloorGeometry loadFloorGeometry(const std::string& filename,const std::string& world_name,
                                const planning::Pose& start,const planning::Pose& goal,
                                planning::RobotFootprint footprint,double height,double margin) {
  if(std::abs(start.z-goal.z)>.003)throw std::runtime_error("CROSS_FLOOR_NOT_SUPPORTED_IN_STAGE2");
  sdf::SDFPtr document(new sdf::SDF());sdf::init(document);
  if(!sdf::readFile(filename,document) || !document->Root()->HasElement("world"))
    throw std::runtime_error("Cannot read loaded World: "+filename);
  auto world=document->Root()->GetElement("world");
  if(world->Get<std::string>("name")!=world_name)throw std::runtime_error("WORLD_NAME_MISMATCH");
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
  if(!planning::isPoseCollisionFree(start,r))throw std::runtime_error("NAV_START_COLLISION_OR_UNSUPPORTED");
  if(!planning::isPoseCollisionFree(goal,r))throw std::runtime_error("NAV_GOAL_COLLISION_OR_UNSUPPORTED");
  return result;
}
}  // namespace simulation
