#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <interfaces/msg/local_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <yaml-cpp/yaml.h>

namespace control
{
class IdealTrajectoryControl : public gazebo::ModelPlugin
{
public:
  ~IdealTrajectoryControl() override
  {
    update_.reset();
    std::lock_guard<std::mutex> lock(gate_->mutex);
    gate_->active = false;
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);
    try {
      const char * environment = std::getenv("IDEAL_CONTROL_CONFIG");
      const auto filename = sdf->HasElement("config_file") ? sdf->Get<std::string>("config_file") :
        (environment && *environment ? environment :
        ament_index_cpp::get_package_share_directory("control") + "/config/ideal.yaml");
      const auto cfg = YAML::LoadFile(filename);
      auto positive = [&cfg](const char * key) {
          const double value = cfg[key].as<double>();
          if (!std::isfinite(value) || value <= 0) {throw std::runtime_error(key);}
          return value;
        };
      feedback_period_ = positive("feedback_period");
      position_tolerance_ = positive("position_tolerance");
      orientation_tolerance_ = positive("orientation_tolerance");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "理想执行器配置无效：%s", e.what());
      return;
    }
    for (const auto & link : model_->GetLinks()) {
      link->SetGravityMode(false);
      link->SetKinematic(true);
      link->SetCollideMode("none");
    }
    pose_ = model_->WorldPose();
    last_time_ = last_publish_ = model_->GetWorld()->SimTime().Double();
    publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("/robot/odom", 10);
    subscription_ = node_->create_subscription<interfaces::msg::LocalTrajectory>(
      "/local_trajectory", 10,
      [this, gate = gate_](interfaces::msg::LocalTrajectory::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(gate->mutex);
        if (gate->active) {ReceiveTrajectory(*msg);}
      });
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this, gate = gate_](const gazebo::common::UpdateInfo & info) {
        std::lock_guard<std::mutex> lock(gate->mutex);
        if (gate->active) {Update(info.simTime.Double());}
      });
    RCLCPP_INFO(node_->get_logger(), "理想轨迹执行器就绪");
  }

private:
  struct Gate {std::mutex mutex; bool active{true};};
  struct Point {
    ignition::math::Vector3d position, velocity;
    double yaw{}, rate{}, time{};
  };
  struct State {ignition::math::Vector3d position, velocity; double yaw{}, rate{};};
  static State Sample(const std::vector<Point> & points, double elapsed)
  {
    if (elapsed >= points.back().time) {
      return {points.back().position, ignition::math::Vector3d::Zero, points.back().yaw, 0};
    }
    const auto end = std::upper_bound(points.begin(), points.end(), elapsed,
      [](double t, const Point & p) {return t < p.time;});
    const auto & b = *end; const auto & a = *(end - 1);
    const double h = b.time-a.time, u = std::clamp((elapsed-a.time)/h, 0.0, 1.0);
    const double u2=u*u, u3=u2*u;
    const double h00=2*u3-3*u2+1, h10=u3-2*u2+u, h01=-2*u3+3*u2, h11=u3-u2;
    const double d00=(6*u2-6*u)/h, d10=3*u2-4*u+1, d01=-d00, d11=3*u2-2*u;
    const double delta=Angle(b.yaw-a.yaw);
    return {h00*a.position+h*h10*a.velocity+h01*b.position+h*h11*b.velocity,
      d00*a.position+d10*a.velocity+d01*b.position+d11*b.velocity,
      a.yaw+h01*delta+h*h10*a.rate+h*h11*b.rate,
      d01*delta+d10*a.rate+d11*b.rate};
  }
  static double Angle(double value) {return std::atan2(std::sin(value), std::cos(value));}

  void ReceiveTrajectory(const interfaces::msg::LocalTrajectory & msg)
  {
    auto reject = [this](const char * reason) {
        RCLCPP_WARN(node_->get_logger(), "理想轨迹拒绝：%s", reason);
      };
    const double now = model_->GetWorld()->SimTime().Double();
    const double begin = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9;
    if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000u || begin <= now) {
      reject("轨迹生效时间已过期"); return;
    }
    if (!pending_.empty() && begin > pending_start_) {
      reject("替换预约的生效时间不能晚于原预约"); return;
    }
    if (msg.header.frame_id != "world" || msg.points.size() < 2) {
      reject("需要 world 坐标系及至少两个点"); return;
    }
    std::vector<Point> next;
    for (const auto & point : msg.points) {
      if (point.transforms.size() != 1 || point.velocities.size() != 1 || point.accelerations.size() != 1) {
        reject("每点需要位姿、速度和加速度"); return;
      }
      const auto & v = point.velocities.front(); const auto & acc = point.accelerations.front();
      auto finite = [](const geometry_msgs::msg::Twist & value) {
        return std::isfinite(value.linear.x) && std::isfinite(value.linear.y) && std::isfinite(value.linear.z) &&
          std::isfinite(value.angular.x) && std::isfinite(value.angular.y) && std::isfinite(value.angular.z) &&
          std::abs(value.angular.x) < 1e-8 && std::abs(value.angular.y) < 1e-8;
      };
      if (!finite(v) || !finite(acc)) {reject("运动导数格式无效"); return;}
      const auto & tr = point.transforms.front();
      const auto & p = tr.translation; const auto & q = tr.rotation;
      const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(norm) || norm < 1e-9 || std::abs(norm - 1.0) > 1e-3 ||
        point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u)
      {
        reject("位姿或时间格式无效"); return;
      }
      const ignition::math::Quaterniond rotation(q.w/norm, q.x/norm, q.y/norm, q.z/norm);
      if (std::abs(rotation.Roll()) > 1e-6 || std::abs(rotation.Pitch()) > 1e-6) {
        reject("仅支持水平本体及 yaw"); return;
      }
      const double time = point.time_from_start.sec + point.time_from_start.nanosec*1e-9;
      if ((next.empty() && time != 0) || (!next.empty() && time <= next.back().time)) {
        reject("时间必须从零严格递增"); return;
      }
      next.push_back({{p.x, p.y, p.z}, {v.linear.x,v.linear.y,v.linear.z}, rotation.Yaw(), v.angular.z, time});
    }
    State expected{pose_.Pos(), ignition::math::Vector3d::Zero, pose_.Rot().Yaw(), 0};
    if (!points_.empty()) {expected = Sample(points_, begin-start_);}
    if (next.front().position.Distance(expected.position) > position_tolerance_ ||
      std::abs(Angle(next.front().yaw-expected.yaw)) > orientation_tolerance_ ||
      next.front().velocity.Distance(expected.velocity) > 1e-3 ||
      std::abs(next.front().rate-expected.rate) > 1e-3)
    {
      reject("切换时刻位姿或速度不连续"); return;
    }
    if (next.back().velocity.Length() > 1e-6 || std::abs(next.back().rate) > 1e-6) {
      reject("轨迹必须保留停车末端"); return;
    }
    pending_ = std::move(next); pending_start_ = begin;
    RCLCPP_INFO(node_->get_logger(), "预约理想轨迹，%zu 点，生效 %.3f s", pending_.size(), begin);
  }

  void Update(double now)
  {
    if (now < last_time_) {
      points_.clear(); pending_.clear();
      linear_.Set(0, 0, 0); angular_ = 0;
      last_publish_ = now;
    }
    if (now <= last_time_) {last_time_ = now; return;}
    if (!pending_.empty() && now >= pending_start_) {
      points_ = std::move(pending_); pending_.clear(); start_ = pending_start_;
      RCLCPP_INFO(node_->get_logger(), "理想轨迹生效，sim=%.3f", now);
    }
    linear_.Set(0, 0, 0); angular_ = 0;
    if (!points_.empty()) {
      const auto state = Sample(points_, now-start_);
      pose_ = ignition::math::Pose3d(state.position, ignition::math::Quaterniond(0, 0, state.yaw));
      linear_ = pose_.Rot().RotateVectorReverse(state.velocity); angular_ = state.rate;
      if (now >= start_+points_.back().time) {
        points_.clear();
        RCLCPP_INFO(node_->get_logger(), "理想轨迹执行完成");
      }
    }
    model_->SetWorldPose(pose_);
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
    if (now-last_publish_ + 1e-9 >= feedback_period_) {PublishOdometry(now); last_publish_ = now;}
    last_time_ = now;
  }

  void PublishOdometry(double now)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(now*1e9));
    msg.header.frame_id = "world"; msg.child_frame_id = "base_link";
    const auto actual = model_->WorldPose();
    msg.pose.pose.position.x = actual.Pos().X();
    msg.pose.pose.position.y = actual.Pos().Y();
    msg.pose.pose.position.z = actual.Pos().Z();
    msg.pose.pose.orientation.x = actual.Rot().X();
    msg.pose.pose.orientation.y = actual.Rot().Y();
    msg.pose.pose.orientation.z = actual.Rot().Z();
    msg.pose.pose.orientation.w = actual.Rot().W();
    msg.twist.twist.linear.x = linear_.X(); msg.twist.twist.linear.y = linear_.Y();
    msg.twist.twist.linear.z = linear_.Z(); msg.twist.twist.angular.z = angular_;
    publisher_->publish(msg);
  }

  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_;
  rclcpp::Subscription<interfaces::msg::LocalTrajectory>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::shared_ptr<Gate> gate_{std::make_shared<Gate>()};
  std::vector<Point> points_, pending_;
  ignition::math::Pose3d pose_;
  ignition::math::Vector3d linear_;
  double feedback_period_{}, position_tolerance_{}, orientation_tolerance_{};
  double last_time_{}, last_publish_{}, start_{}, pending_start_{}, angular_{};
};
GZ_REGISTER_MODEL_PLUGIN(IdealTrajectoryControl)
}  // namespace control
