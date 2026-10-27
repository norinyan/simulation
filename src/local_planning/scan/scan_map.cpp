#include "scan_map.h"
#include <tinyxml2.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace local_planning {
namespace {
using Element = tinyxml2::XMLElement;

std::vector<double> numbers(const char * text, std::size_t count)
{
  if (!text) {throw std::runtime_error("SCAN_WORLD_MISSING_NUMBERS");}
  std::istringstream input(text);
  std::vector<double> result(count);
  for (double & value : result) {
    if (!(input >> value) || !std::isfinite(value)) {
      throw std::runtime_error("SCAN_WORLD_INVALID_NUMBERS");
    }
  }
  std::string extra;
  if (input >> extra) {throw std::runtime_error("SCAN_WORLD_EXTRA_NUMBERS");}
  return result;
}

Eigen::Isometry3d pose(Element * parent)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const auto element = parent->FirstChildElement("pose");
  if (!element) {return transform;}
  // The current World uses parent-relative poses only. Refuse unsupported SDF
  // frames instead of silently moving an obstacle to a different location.
  if (element->FirstAttribute()) {throw std::runtime_error("SCAN_WORLD_UNSUPPORTED_POSE_FRAME");}
  const auto values = numbers(element->GetText(), 6);
  transform.translation() = Eigen::Vector3d(values[0], values[1], values[2]);
  transform.linear() = (Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
  return transform;
}

}  // namespace

void ScanMap::load(const std::string & filename)
{
  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("SCAN_WORLD_UNREADABLE: " + filename);
  }
  auto sdf = document.FirstChildElement("sdf");
  auto world = sdf ? sdf->FirstChildElement("world") : nullptr;
  if (!world || world->FirstChildElement("include") || world->FirstChildElement("frame")) {
    throw std::runtime_error("SCAN_WORLD_UNSUPPORTED_STRUCTURE");
  }
  std::vector<Box> parsed;
  for (auto model = world->FirstChildElement("model"); model;
    model = model->NextSiblingElement("model"))
  {
    const auto is_static = model->FirstChildElement("static");
    if (!is_static || !is_static->GetText() || std::string(is_static->GetText()) != "true" ||
      model->FirstChildElement("model") || model->FirstChildElement("include") ||
      model->FirstChildElement("frame"))
    {
      throw std::runtime_error("SCAN_WORLD_REQUIRES_STATIC_BOX_MODELS");
    }
    const auto model_pose = pose(model);
    for (auto link = model->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
      const auto link_pose = model_pose * pose(link);
      for (auto collision = link->FirstChildElement("collision"); collision;
        collision = collision->NextSiblingElement("collision"))
      {
        const auto geometry = collision->FirstChildElement("geometry");
        const auto box = geometry ? geometry->FirstChildElement("box") : nullptr;
        const auto size = box ? box->FirstChildElement("size") : nullptr;
        if (!size || geometry->FirstChildElement() != box || box->NextSiblingElement()) {
          throw std::runtime_error("SCAN_WORLD_REQUIRES_BOX_COLLISIONS");
        }
        const auto values = numbers(size->GetText(), 3);
        if (std::any_of(values.begin(), values.end(), [](double x) {return x <= 0;})) {
          throw std::runtime_error("SCAN_WORLD_INVALID_BOX_SIZE");
        }
        const auto transform = link_pose * pose(collision);
        Box result;
        result.center = transform.translation();
        result.rotation = transform.linear();
        result.half = Eigen::Vector3d(values[0], values[1], values[2])*0.5;
        const Eigen::Vector3d extent = result.rotation.cwiseAbs()*result.half;
        result.minimum = result.center - extent;
        result.maximum = result.center + extent;
        parsed.push_back(result);
      }
    }
  }
  if (parsed.empty()) {throw std::runtime_error("SCAN_WORLD_NO_COLLISIONS");}
  boxes_ = std::move(parsed);
}

bool ScanMap::height(double x, double y, double ceiling, double depth, double & z) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(ceiling) ||
    !std::isfinite(depth) || depth <= 0) {return false;}
  double best = std::numeric_limits<double>::infinity();
  const Eigen::Vector3d origin(x, y, ceiling), direction(0, 0, -1);
  for (const auto & box : boxes_) {
    if (x < box.minimum.x()-1e-9 || x > box.maximum.x()+1e-9 ||
      y < box.minimum.y()-1e-9 || y > box.maximum.y()+1e-9 ||
      box.minimum.z() > ceiling+1e-9 || box.maximum.z() < ceiling-depth-1e-9) {continue;}
    const Eigen::Vector3d local_origin = box.rotation.transpose()*(origin-box.center);
    const Eigen::Vector3d local_direction = box.rotation.transpose()*direction;
    double enter = 0, leave = depth;
    bool hit = true;
    for (int i = 0; i < 3; ++i) {
      if (std::abs(local_direction[i]) < 1e-12) {
        if (std::abs(local_origin[i]) > box.half[i]+1e-9) {hit = false; break;}
      } else {
        double first = (-box.half[i]-local_origin[i])/local_direction[i];
        double last = (box.half[i]-local_origin[i])/local_direction[i];
        if (first > last) {std::swap(first, last);}
        enter = std::max(enter, first);
        leave = std::min(leave, last);
        if (enter > leave+1e-9) {hit = false; break;}
      }
    }
    // If the ray starts inside geometry, its entry is not a supporting top.
    if (hit && (local_origin.cwiseAbs().array() < box.half.array()-1e-9).all()) {continue;}
    if (hit && enter < best) {best = enter;}
  }
  if (!std::isfinite(best)) {return false;}
  z = ceiling-best;
  return true;
}


// Signed distance to the union of World OBBs. The nearest box supplies the
// piecewise-smooth gradient used by SCAN's actual-curve collision objective.
double ScanMap::signedDistance(const Eigen::Vector3d & point,Eigen::Vector3d & gradient) const
{
  gradient.setZero();
  if (boxes_.empty() || !point.allFinite()) {throw std::runtime_error("SCAN_INVALID_DISTANCE_QUERY");}
  double best=std::numeric_limits<double>::infinity();
  for (const auto & box:boxes_) {
    const Eigen::Vector3d broad=(point-box.maximum).cwiseMax(box.minimum-point).cwiseMax(0);
    if (broad.squaredNorm()>0 && (best<=0 || broad.squaredNorm()>=best*best)) {continue;}
    const Eigen::Vector3d local=box.rotation.transpose()*(point-box.center);
    const Eigen::Vector3d delta=local.cwiseAbs()-box.half;
    const Eigen::Vector3d outside=delta.cwiseMax(0);
    const double length=outside.norm();
    double value;Eigen::Vector3d direction=Eigen::Vector3d::Zero();
    if (length>1e-12) {
      value=length;
      for (int i=0;i<3;++i) {direction[i]=outside[i]/length*(local[i]>=0?1:-1);}
    } else {
      Eigen::Index axis;value=delta.maxCoeff(&axis);direction[axis]=local[axis]>=0?1:-1;
    }
    if (value<best) {best=value;gradient=box.rotation*direction;}
  }
  return best;
}

double ScanMap::clearance(const Eigen::Vector3d & point) const
{
  return segmentClearance(point, point);
}

// Exact distance from the executed line segment to each oriented World box.
// Within each interval split by box-face crossings, squared distance is quadratic.
double ScanMap::segmentClearance(const Eigen::Vector3d & from, const Eigen::Vector3d & to) const
{
  if (boxes_.empty() || !from.allFinite() || !to.allFinite()) {return 0;}
  double best=std::numeric_limits<double>::infinity();
  const Eigen::Vector3d minimum=from.cwiseMin(to),maximum=from.cwiseMax(to);
  for (const auto & box:boxes_) {
    const Eigen::Vector3d broad=(minimum-box.maximum).cwiseMax(box.minimum-maximum).cwiseMax(0);
    if (broad.norm()>=best) {continue;}
    const Eigen::Vector3d a=box.rotation.transpose()*(from-box.center);
    const Eigen::Vector3d d=box.rotation.transpose()*(to-from);
    std::vector<double> cuts{0,1};
    for (int j=0;j<3;++j) {
      if (std::abs(d[j])<1e-14) {continue;}
      for (double face:{-box.half[j],box.half[j]}) {
        const double t=(face-a[j])/d[j];
        if (t>0 && t<1) {cuts.push_back(t);}
      }
    }
    std::sort(cuts.begin(),cuts.end());
    auto value=[&](double t)->Eigen::Vector3d {return (a+t*d).cwiseAbs().cwiseMax(box.half)-box.half;};
    for (std::size_t k=1;k<cuts.size();++k) {
      const double mid=(cuts[k-1]+cuts[k])/2;
      double numerator=0,denominator=0;
      for (int j=0;j<3;++j) {
        const double q=a[j]+mid*d[j];
        if (std::abs(q)<=box.half[j]) {continue;}
        const double c=a[j]-(q>0?box.half[j]:-box.half[j]);
        numerator+=c*d[j];denominator+=d[j]*d[j];
      }
      const double t=denominator>0?std::clamp(-numerator/denominator,cuts[k-1],cuts[k]):mid;
      best=std::min(best,value(t).norm());
      if (best<=1e-12) {return 0;}
    }
  }
  return best;
}
}  // namespace local_planning
