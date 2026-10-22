#pragma once
#include "scene_data.h"
namespace simulation {
// Parse the loaded World once; all floor projections and pedestrian descriptions
// share that document and remain immutable for the planning process lifetime.
SceneGeometry loadSceneGeometry(const std::string& world_file,
    const std::string& world_name, planning::RobotFootprint footprint, double height, double margin);
std::shared_ptr<const planning::SceneMap> loadSceneMap(const std::string& world_file,
    const std::string& world_name, planning::RobotFootprint footprint, double height, double margin);
std::size_t worldParseCount();
planning::BoxObstacle movingBox(const MovingBox& model, const ignition::math::Pose3d& pose);
ignition::math::Pose3d patrolPose(const MovingBox& model, double simulation_time);

}
