#pragma once
#include <Eigen/Core>
#include <memory>

// Occupancy includes the current robot envelope; unknown/unsupported is blocked.
class GridMap {
public:
  using Ptr = std::shared_ptr<GridMap>;
  virtual ~GridMap() = default;
  virtual double getResolution() const = 0;
  virtual int getInflateOccupancy(const Eigen::Vector3d &position, double yaw) const = 0;
  virtual bool expired() const = 0;
};
