#ifndef LOCAL_PLANNING__LOCAL_NODE_H_
#define LOCAL_PLANNING__LOCAL_NODE_H_

#include <chrono>
#include <string>

#include <interfaces/msg/global_path.hpp>
#include <interfaces/msg/local_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace local_planning
{
struct MotionLimits
{
  double speed{}, yaw_rate{}, acceleration{}, reference_height{};
  double position_tolerance{}, orientation_tolerance{};
};

struct LocalRequest
{
  nav_msgs::msg::Odometry state;
  interfaces::msg::GlobalPath segment;  // world 地面路径，模式仅作为地形标签。
  MotionLimits limits;
  std::chrono::steady_clock::time_point deadline;
};

struct LocalResult
{
  bool success{false};
  interfaces::msg::LocalTrajectory trajectory;  // world 下的带时间本体位姿。
  std::string reason;
};

// 各算法在自己的目录中适配；地图表示和碰撞检查由实现持有。
class LocalPlanner
{
public:
  virtual ~LocalPlanner() = default;
  virtual void loadMap(const std::string & filename) = 0;
  virtual bool supports(const std::string & mode) const = 0;
  virtual LocalResult plan(const LocalRequest & request) = 0;
};
}  // namespace local_planning
#endif
