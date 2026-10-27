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
  // Signed distance to actual geometry and its world-coordinate gradient.
  virtual double signedDistance(const Eigen::Vector3d &position, Eigen::Vector3d &gradient) const = 0;
  virtual double signedDistanceAt(const Eigen::Vector3d &position,double,
    Eigen::Vector3d &gradient) const {return signedDistance(position,gradient);}
  virtual int getInflateOccupancyAt(const Eigen::Vector3d &position,double yaw,double) const
  {return getInflateOccupancy(position,yaw);}
  virtual bool expired() const = 0;
};
