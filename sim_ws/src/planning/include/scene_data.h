#pragma once
#include "global_planner.h"
#include <ignition/math/Pose3.hh>
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

planning::BoxObstacle movingBox(const MovingBox&,const ignition::math::Pose3d&);
ignition::math::Pose3d patrolPose(const MovingBox&,double);
}
