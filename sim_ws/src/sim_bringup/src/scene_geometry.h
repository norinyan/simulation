#ifndef SIM_BRINGUP_SCENE_GEOMETRY_H
#define SIM_BRINGUP_SCENE_GEOMETRY_H

#include "global_planner.h"
#include <ignition/math/Pose3.hh>
#include <string>
#include <vector>

namespace simulation {

struct MovingBox {
  struct Phase {
    ignition::math::Vector3d from, to;
    double begin, duration, yaw;
  };
  std::string name;
  ignition::math::Pose3d local_pose;
  ignition::math::Vector3d size;
  std::vector<Phase> phases;
  double delay{0}, period{0};
};

struct FloorGeometry {
  planning::PlanRequest geometry;
  std::vector<MovingBox> pedestrians;
};

struct SceneGeometry {
  std::shared_ptr<const planning::SceneMap> map;
  std::vector<FloorGeometry> floors;
};
// Parse the loaded World once; all floor projections and pedestrian descriptions
// share that document and remain immutable for the planning process lifetime.
SceneGeometry loadSceneGeometry(const std::string& world_file,
    const std::string& world_name, planning::RobotFootprint footprint, double height, double margin);
std::shared_ptr<const planning::SceneMap> loadSceneMap(const std::string& world_file,
    const std::string& world_name, planning::RobotFootprint footprint, double height, double margin);
std::size_t worldParseCount();
planning::BoxObstacle movingBox(const MovingBox& model, const ignition::math::Pose3d& pose);
ignition::math::Pose3d patrolPose(const MovingBox& model, double simulation_time);

}  // namespace simulation
#endif
