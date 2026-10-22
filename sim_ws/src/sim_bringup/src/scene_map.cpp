#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// One WorldPlugin owns the immutable map, including scenes without pedestrians.
#include <gazebo_ros/node.hpp>
#include <yaml-cpp/yaml.h>
#include "scene_geometry.h"
#include "scene_transport.h"
namespace gazebo {
class SceneMapPublisher final : public WorldPlugin {
 gazebo_ros::Node::SharedPtr ros_;
 rclcpp::Publisher<planning::msg::SceneMap>::SharedPtr map_;
public:
 void Load(physics::WorldPtr world,sdf::ElementPtr sdf) override {
  ros_=gazebo_ros::Node::Get(sdf);
  const auto doc=YAML::LoadFile(sdf->Get<std::string>("scene_file"));const auto r=doc["robot"];
  const auto geometry=simulation::loadSceneGeometry(sdf->Get<std::string>("world_file"),world->Name(),
    {r["wheel_mode_length"].as<double>(),r["wheel_mode_width"].as<double>()},
    r["wheel_radius"].as<double>()+r["carrier_drop"].as<double>()+r["body_size"][2].as<double>()/2,.05);
  auto message=simulation::encode(geometry);message.scene_id=world->Name();message.version=sdf->Get<std::string>("version");message.world_parse_count=simulation::worldParseCount();
  map_=ros_->create_publisher<planning::msg::SceneMap>("/scene/map",rclcpp::QoS(1).reliable().transient_local());map_->publish(message);
  RCLCPP_INFO(ros_->get_logger(),"Scene map published: scene=%s version=%s world_parse_count=%zu",message.scene_id.c_str(),message.version.c_str(),simulation::worldParseCount());
 }
};
GZ_REGISTER_WORLD_PLUGIN(SceneMapPublisher)
}
