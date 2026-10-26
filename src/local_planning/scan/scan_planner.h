#pragma once

#include "../local_node.h"
#include "scan_map.h"
#include <yaml-cpp/yaml.h>

namespace local_planning
{
class ScanPlanner final : public LocalPlanner
{
public:
  explicit ScanPlanner(const YAML::Node & config);
  void loadMap(const std::string & filename) override;
  bool supports(const std::string & mode) const override;
  LocalResult plan(const LocalRequest & request) override;

private:
  ScanMap map_;
  double safety_margin_, stair_speed_;
  double swing_lift_, feedback_period_;
  double resolution_, spacing_, sample_dt_, acceleration_, clearance_;
  double smooth_weight_, collision_weight_, feasibility_weight_;
  int iterations_;
};
}  // namespace local_planning
