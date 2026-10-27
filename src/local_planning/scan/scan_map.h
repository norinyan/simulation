#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace local_planning {
class ScanMap {
public:
  void load(const std::string & filename);
  double signedDistance(const Eigen::Vector3d & point, Eigen::Vector3d & gradient) const;
  double clearance(const Eigen::Vector3d & point) const;
  double segmentClearance(const Eigen::Vector3d & from,const Eigen::Vector3d & to) const;
  bool height(double x,double y,double ceiling,double depth,double & z) const;
  std::size_t size() const {return boxes_.size();}
private:
  struct Box {
    Eigen::Vector3d center,half,minimum,maximum;
    Eigen::Matrix3d rotation;
  };
  std::vector<Box> boxes_;
};
}  // namespace local_planning
