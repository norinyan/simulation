#pragma once
#include "local_planner.h"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <memory>

// Wheel-only replacement for the upstream sensor/sliding-map backend. The input
// already contains the active floor's projected obstacles and explicit support.
// Query the shared yaw-aware rectangular footprint without a second World parser,
// synthetic point clouds, sensor fusion, or treating unobserved space as free.
class GridMap {
public:
  using Ptr = std::shared_ptr<GridMap>;
  GridMap(const planning::PlanRequest& geometry, double resolution)
      : geometry_(geometry), resolution_(resolution) {}
  double getResolution() const { return resolution_; }
  bool expired() const { return !geometry_.control.failure().empty(); }
  int getInflateOccupancy(const Eigen::Vector3d& p, double yaw) const {
    if (!p.allFinite() || !std::isfinite(yaw) || std::abs(p.z()-geometry_.start.z)>1e-5)
      return 1;
    return !planning::isPoseCollisionFree({p.x(),p.y(),p.z(),yaw},geometry_);
  }
private:
  planning::PlanRequest geometry_;
  double resolution_;
};
