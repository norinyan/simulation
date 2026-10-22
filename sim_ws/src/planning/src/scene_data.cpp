#include "scene_data.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace simulation {
using Pose=ignition::math::Pose3d;
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

}
