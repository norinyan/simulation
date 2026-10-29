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
  std::size_t size() const {return grid_.distance.empty()?boxes_.size():grid_.distance.size();}
private:
  struct Grid {
    double resolution{0};
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    std::size_t nx{0},ny{0},nz{0},layers{0};
    std::vector<float> distance,ground;
    std::vector<unsigned char> valid;
  };
  void loadNpz(const std::string & filename);
  Grid grid_;
  struct Box {
    Eigen::Vector3d center,half,minimum,maximum;
    Eigen::Matrix3d rotation;
  };
  std::vector<Box> boxes_;
};
}  // namespace local_planning
