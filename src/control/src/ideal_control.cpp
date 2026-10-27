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
  struct Point {ignition::math::Vector3d position; double yaw{}, time{};};
  static double Angle(double value) {return std::atan2(std::sin(value), std::cos(value));}

  void ReceiveTrajectory(const interfaces::msg::LocalTrajectory & msg)
  {
    auto reject = [this](const char * reason) {
        RCLCPP_WARN(node_->get_logger(), "理想轨迹拒绝：%s", reason);
      };
    if (!points_.empty()) {reject("正在执行轨迹"); return;}
    if (msg.header.frame_id != "world" || msg.points.size() < 2) {
      reject("需要 world 坐标系及至少两个点"); return;
    }
    std::vector<Point> next;
    for (const auto & point : msg.points) {
      if (point.transforms.size() != 1) {reject("每点需要一个变换"); return;}
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
      next.push_back({{p.x, p.y, p.z}, rotation.Yaw(), time});
    }
    const auto actual = model_->WorldPose();
    if (next.front().position.Distance(actual.Pos()) > position_tolerance_ ||
      std::abs(Angle(next.front().yaw - actual.Rot().Yaw())) > orientation_tolerance_)
    {
      reject("首点未衔接当前位姿"); return;
    }
    points_ = std::move(next);
    started_ = false;
    index_ = 1;
    RCLCPP_INFO(node_->get_logger(), "接受理想轨迹，%zu 点，时长 %.3f s", points_.size(), points_.back().time);
  }

  void Update(double now)
  {
    if (now < last_time_) {
      points_.clear();
      started_ = false;
      linear_.Set(0, 0, 0); angular_ = 0;
      last_publish_ = now;
    }
    if (now <= last_time_) {last_time_ = now; return;}
    const auto previous = pose_;
    if (!points_.empty()) {
      if (!started_) {start_ = now; started_ = true;}
      const double elapsed = now - start_;
      while (index_ + 1 < points_.size() && elapsed >= points_[index_].time) {++index_;}
      const auto & a = points_[index_ - 1]; const auto & b = points_[index_];
      const double fraction = std::clamp((elapsed - a.time)/(b.time - a.time), 0.0, 1.0);
      const auto p = a.position + fraction*(b.position - a.position);
      pose_ = ignition::math::Pose3d(p,
        ignition::math::Quaterniond(0, 0, a.yaw + fraction*Angle(b.yaw-a.yaw)));
      if (elapsed >= points_.back().time) {
        points_.clear(); started_ = false;
        RCLCPP_INFO(node_->get_logger(), "理想轨迹执行完成");
      }
    }
    model_->SetWorldPose(pose_);
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
    const double dt = now-last_time_;
    linear_ = pose_.Rot().RotateVectorReverse((pose_.Pos()-previous.Pos())/dt);
    angular_ = Angle(pose_.Rot().Yaw()-previous.Rot().Yaw())/dt;
    if (points_.empty()) {linear_.Set(0, 0, 0); angular_ = 0;}
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
  std::vector<Point> points_;
  ignition::math::Pose3d pose_;
  ignition::math::Vector3d linear_;
  double feedback_period_{}, position_tolerance_{}, orientation_tolerance_{};
  double last_time_{}, last_publish_{}, start_{}, angular_{};
  std::size_t index_{1};
  bool started_{false};
};
GZ_REGISTER_MODEL_PLUGIN(IdealTrajectoryControl)
}  // namespace control
