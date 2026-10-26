#pragma once
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// 保持参考 PCT 核心的数据语义；ROS 接口适配位于 pct_planner.cpp。
namespace pct_native {
struct Pose { double x{0}, y{0}, z{0}, yaw{0}; };
struct PlanControl {
  std::chrono::steady_clock::time_point deadline;
  std::string failure() const {
    return std::chrono::steady_clock::now() >= deadline ? "PLANNING_TIMEOUT" : "";
  }
};
struct PctMap {
  uint32_t layers{0}, nx{0}, ny{0};
  double resolution{0}, center_x{0}, center_y{0}, max_step{0}, clearance{0}, inflation_radius{0};
  std::vector<float> data;
  std::size_t cells() const { return static_cast<std::size_t>(layers)*nx*ny; }
  float at(int channel, int layer, int x, int y) const {
    return data.at(channel*cells()+(static_cast<std::size_t>(layer)*nx+x)*ny+y);
  }
};
struct PctOptions {
  double cost_threshold{20}, step_cost_weight{.2}, reference_height{.44};
  double max_heading_rate{.1}, optimizer_cost_margin{8}, optimizer_jerk_sigma{.03};
  int optimizer_sample_interval{5}, optimizer_iterations{200};
  bool optimize{false};
};
struct PlanRequest {
  Pose start, goal;
  struct Footprint {double length{0}, width{0};} footprint;
  double safety_margin{0}, cost_threshold{20};
  PlanControl control;
  std::shared_ptr<const PctMap> pct_map;
};
struct PlanResult {
  bool success{false};
  std::vector<Pose> path;
  std::string failure_reason;
  std::vector<Pose> body_reference;
};
std::shared_ptr<const PctMap> loadPctMap(const std::string & filename);
std::string validatePctPath(const PlanRequest &, const std::vector<Pose> &);
class PctPlanner {
public:
  explicit PctPlanner(PctOptions options) : options_(options) {}
  PlanResult plan(const PlanRequest &);
private:
  PctOptions options_;
};
}  // namespace pct_native
