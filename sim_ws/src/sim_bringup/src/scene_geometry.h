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

// Read actual collision geometry, compose model/link/collision poses and clip
// solids to the wheel robot's height interval. No map or furniture copy is maintained.
FloorGeometry loadFloorGeometry(const std::string& world_file, const std::string& world_name,
                                const planning::Pose& start, const planning::Pose& goal,
                                planning::RobotFootprint footprint, double height, double margin);
planning::BoxObstacle movingBox(const MovingBox& model, const ignition::math::Pose3d& pose);
ignition::math::Pose3d patrolPose(const MovingBox& model, double simulation_time);

}  // namespace simulation
#endif
