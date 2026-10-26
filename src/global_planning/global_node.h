#ifndef GLOBAL_PLANNING__GLOBAL_NODE_H_
#define GLOBAL_PLANNING__GLOBAL_NODE_H_

#include <chrono>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>

namespace global_planning
{
struct TraversalConstraints
{
  double length{}, width{}, safety_margin{}, max_step{}, clearance{};
};

struct PlanRequest
{
  geometry_msgs::msg::Pose start, goal;  // world 下的地面位姿。
  TraversalConstraints constraints;
  std::chrono::steady_clock::time_point deadline;
};

enum class Terrain { Flat, Stair, StairLanding };
struct PlanResult
{
  bool success{false};
  std::vector<geometry_msgs::msg::Pose> points;
  std::vector<Terrain> terrain;  // 每条边一个地形标签，包含楼梯中间平台。
  std::string reason;
};

// 算法适配器拥有专属地图；此阶段不提供算法实现。
class GlobalPlanner
{
public:
  virtual ~GlobalPlanner() = default;
  virtual void loadMap(const std::string & file) = 0;
  virtual geometry_msgs::msg::Pose groundPose(
    const geometry_msgs::msg::Pose & body_pose) const = 0;
  virtual PlanResult plan(const PlanRequest & request) = 0;
};
}  // namespace global_planning
#endif
