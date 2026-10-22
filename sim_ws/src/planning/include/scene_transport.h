#pragma once
// ROS transport adapter. The algorithms consume SceneMap, never ROS messages.
#include "scene_data.h"
#include "planning/msg/scene_map.hpp"
namespace simulation {
inline geometry_msgs::msg::Pose rosPose(const ignition::math::Pose3d& p) {
 geometry_msgs::msg::Pose out;out.position.x=p.Pos().X();out.position.y=p.Pos().Y();out.position.z=p.Pos().Z();
 out.orientation.x=p.Rot().X();out.orientation.y=p.Rot().Y();out.orientation.z=p.Rot().Z();out.orientation.w=p.Rot().W();return out;
}
inline ignition::math::Pose3d gazeboPose(const geometry_msgs::msg::Pose& p) {
 return {{p.position.x,p.position.y,p.position.z},{p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z}};
}
inline geometry_msgs::msg::Pose rosPose(const planning::Pose& p) {return rosPose(ignition::math::Pose3d(p.x,p.y,p.z,0,0,p.yaw));}
inline planning::Pose planPose(const geometry_msgs::msg::Pose& p) {const auto g=gazeboPose(p);return {g.Pos().X(),g.Pos().Y(),g.Pos().Z(),g.Rot().Yaw()};}
inline planning::msg::Box encode(const planning::BoxObstacle& b) {
 planning::msg::Box m;m.pose=rosPose(ignition::math::Pose3d(b.center_x,b.center_y,b.center_z,b.roll,b.pitch,b.yaw));m.size.x=b.size_x;m.size.y=b.size_y;m.size.z=b.size_z;return m;
}
inline planning::BoxObstacle decode(const planning::msg::Box& m) {
 const auto p=gazeboPose(m.pose);return {p.Pos().X(),p.Pos().Y(),p.Pos().Z(),m.size.x,m.size.y,m.size.z,p.Rot().Yaw(),p.Rot().Roll(),p.Rot().Pitch()};
}
template<class A,class B,class F> void convert(const A& a,B& b,F f) {for(const auto& item:a)b.push_back(f(item));}
inline planning::msg::SceneMap encode(const SceneGeometry& geometry) {
 planning::msg::SceneMap m;const auto& map=*geometry.map;m.header.frame_id=map.frame_id;
 convert(map.solids,m.solids,[](const auto& b){return encode(b);});
 for(const auto& r:map.regions) {planning::msg::SupportRegion x;x.id=r.id;x.height=r.height;x.bounds={r.bounds.min_x,r.bounds.max_x,r.bounds.min_y,r.bounds.max_y};convert(r.supports,x.supports,[](const auto& b){return encode(b);});convert(r.obstacles,x.obstacles,[](const auto& b){return encode(b);});m.regions.push_back(x);}
 for(const auto& c:map.connections) {planning::msg::StairConnection x;x.id=c.id;x.from_region=c.from;x.to_region=c.to;x.enabled=c.enabled;convert(c.reference,x.reference,[](const auto& b){return rosPose(b);});convert(c.treads,x.treads,[](const auto& b){return encode(b);});convert(c.platforms,x.platforms,[](const auto& b){return encode(b);});m.connections.push_back(x);}
 for(const auto& floor:geometry.floors)for(const auto& b:floor.pedestrians) {
  planning::msg::MovingObstacle x;x.name=b.name;x.local_pose=rosPose(b.local_pose);x.size.x=b.size.X();x.size.y=b.size.Y();x.size.z=b.size.Z();x.delay=b.delay;x.period=b.period;x.floor_height=floor.geometry.start.z;
  for(const auto& phase:b.phases){planning::msg::PatrolPhase q;q.from_position.x=phase.from.X();q.from_position.y=phase.from.Y();q.from_position.z=phase.from.Z();q.to_position.x=phase.to.X();q.to_position.y=phase.to.Y();q.to_position.z=phase.to.Z();q.begin=phase.begin;q.duration=phase.duration;q.yaw=phase.yaw;x.phases.push_back(q);}m.pedestrians.push_back(x);
 }
 return m;
}
inline SceneGeometry decode(const planning::msg::SceneMap& m) {
 SceneGeometry out;auto map=std::make_shared<planning::SceneMap>();map->frame_id=m.header.frame_id;convert(m.solids,map->solids,[](const auto& b){return decode(b);});
 for(const auto& r:m.regions){planning::SupportRegion x;x.id=r.id;x.height=r.height;x.bounds={r.bounds[0],r.bounds[1],r.bounds[2],r.bounds[3]};convert(r.supports,x.supports,[](const auto& b){return decode(b);});convert(r.obstacles,x.obstacles,[](const auto& b){return decode(b);});map->regions.push_back(x);FloorGeometry floor;floor.geometry.frame_id=map->frame_id;floor.geometry.start.z=floor.geometry.goal.z=r.height;floor.geometry.bounds=x.bounds;floor.geometry.supports=x.supports;floor.geometry.obstacles=x.obstacles;floor.geometry.require_support=true;out.floors.push_back(floor);}
 for(const auto& c:m.connections){planning::StairConnection x;x.id=c.id;x.from=c.from_region;x.to=c.to_region;x.enabled=c.enabled;convert(c.reference,x.reference,[](const auto& b){return planPose(b);});convert(c.treads,x.treads,[](const auto& b){return decode(b);});convert(c.platforms,x.platforms,[](const auto& b){return decode(b);});map->connections.push_back(x);}
 for(const auto& b:m.pedestrians){MovingBox x;x.name=b.name;x.local_pose=gazeboPose(b.local_pose);x.size={b.size.x,b.size.y,b.size.z};x.delay=b.delay;x.period=b.period;for(const auto& q:b.phases)x.phases.push_back({{q.from_position.x,q.from_position.y,q.from_position.z},{q.to_position.x,q.to_position.y,q.to_position.z},q.begin,q.duration,q.yaw});for(auto& floor:out.floors)if(std::abs(floor.geometry.start.z-b.floor_height)<.003)floor.pedestrians.push_back(x);}
 out.map=map;return out;
}
}
