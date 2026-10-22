#pragma once
#include "local_planner.h"

namespace planning {
struct ScanOptions {
  double resolution{.05}, spacing{.15}, sample_dt{.05};
  double smooth_weight{1.0}, collision_weight{1.0}, feasibility_weight{.1};
  double clearance{.10};
  int iterations{100};
};

// SCAN position-spline optimization adapted to the shared wheel trajectory contract.
// Each request owns its map and optimizer; no trajectory survives a task change.
class ScanLocalPlanner final : public WheelLocalPlanner {
public:
  explicit ScanLocalPlanner(ScanOptions options = {});
  LocalPlanResult plan(const LocalPlanRequest& request) override;
private:
  ScanOptions options_;
};
}
