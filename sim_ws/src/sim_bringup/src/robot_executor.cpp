// Gazebo Classic kinematic execution, adapted from wheel_leg_demo/demo_node.cpp.
// The static model is deliberate: no contact dynamics or balance controller.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <yaml-cpp/yaml.h>
#include "local_planner.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace gazebo {
class RobotExecutor final : public ModelPlugin {
  using Trajectory = trajectory_msgs::msg::MultiDOFJointTrajectory;
  using V = ignition::math::Vector3d;
  using P = ignition::math::Pose3d;
  physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  event::ConnectionPtr update_;
  std::mutex mutex_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_;
  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<Trajectory>::SharedPtr feedback_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  YAML::Node scene_, robot_, config_;
  std::string frame_, state_{"READY_WHEEL"};
  enum class Action {Idle, Deploy, Fold, Foot};
  Action action_{Action::Idle};
  bool leg_{false}, driving_{false}, failed_{false}, live_geometry_{false};
  mutable std::string collision_reason_;
  double x_, y_, z_, yaw_, lift_{0}, v_{0}, w_{0}, cv_{0}, cw_{0};
  double vy_{0}, vz_{0};
  double last_{-1}, report_{-1}, command_time_{-100}, begin_{0}, heartbeat_{-1};
  double radius_, track_, drop_, l1_, l2_, hx_, hy_, hz_, toe_, half_;
  double body_height_, lift_target_, morph_time_, command_timeout_, feedback_timeout_;
  double max_v_, max_w_, max_a_, max_alpha_, wheel_height_, ground_, from_z_, from_lift_;
  std::array<double,2> fx_, fz_, from_fx_, from_fz_, wheels_{};
  std::array<double,3> mount_;
  Trajectory planned_;
  planning::FootRobotGeometry foot_robot_;
  bool spatial_task_{false},spatial_foot_{false};
  std::array<planning::Pose,2> spatial_feet_{};
  std::chrono::steady_clock::time_point wall_tick_{std::chrono::steady_clock::now()};
  static double q(double u) {u=std::clamp(u,0.,1.);return u*u*u*(10+u*(-15+6*u));}
  static double mix(double a,double b,double u) {return a+(b-a)*q(u);}
  void fail(const std::string& why) {
    failed_=true; action_=Action::Idle; driving_=false;v_=w_=cv_=cw_=0;state_="FAILED: "+why;
    RCLCPP_ERROR(node_->get_logger(), "%s",state_.c_str());
  }
  double forward() const {return x_*std::cos(yaw_)+y_*std::sin(yaw_);}
  void folded() {
    fx_.fill(forward()+hx_+toe_-l1_*std::sin(mount_[0])-l2_*std::sin(mount_[0]+mount_[1]));
    fz_.fill(z_+hz_-l1_*std::cos(mount_[0])-l2_*std::cos(mount_[0]+mount_[1]));
  }
  bool ik(int side, double& a,double& b) const {
    double dx=fx_[side]-toe_-forward()-hx_, down=z_+hz_-fz_[side];
    double c=(dx*dx+down*down-l1_*l1_-l2_*l2_)/(2*l1_*l2_);
    if(!std::isfinite(c)||c < -1-1e-8||c > 1+1e-8)return false;
    double knee=-std::acos(std::clamp(c,-1.,1.));
    a=std::atan2(-dx,down)-std::atan2(l2_*std::sin(knee),l1_+l2_*std::cos(knee));b=a+knee;
    const auto limits=robot_["leg_pitch_limits"];
    for(double joint:{a-mount_[0],b-a-mount_[1],-b-mount_[2]})
      if(joint < limits[0].as<double>()-1e-6 || joint > limits[1].as<double>()+1e-6)return false;
    return true;
  }
  void setLink(const std::string& name,double dx,double dy,double zz,double pitch=0) {
    model_->GetLink(name)->SetWorldPose(P(x_+std::cos(yaw_)*dx-std::sin(yaw_)*dy,
      y_+std::sin(yaw_)*dx+std::cos(yaw_)*dy,zz,0,pitch,yaw_),false,false);
  }
  bool apply(double now) {
    std::array<double,2> a,b;
    if(!spatial_foot_)for(int i=0;i<2;++i)if(!ik(i,a[i],b[i]))return false;
    model_->SetWorldPose(P(x_,y_,z_,0,0,yaw_),false,false);
    setLink("wheel_carrier",0,0,z_-drop_+lift_);
    sensor_msgs::msg::JointState msg;msg.header.stamp=rclcpp::Time(int64_t(now*1e9));
    msg.name={"wheel_lift_joint","left_wheel_joint","right_wheel_joint"};msg.position={lift_,wheels_[0],wheels_[1]};
    try {for(int i=0;i<2;++i){
      const std::string s=i==0?"left":"right";const double sign=i==0?1:-1;
      setLink(s+"_wheel",0,sign*track_/2,z_-drop_+lift_,wheels_[i]);
      if(spatial_foot_){
        const auto k=planning::footInverseKinematics({x_,y_,z_,yaw_},spatial_feet_[i],i,foot_robot_);
        auto place=[&](const std::string& name,const planning::Pose& p,double pitch){
          model_->GetLink(name)->SetWorldPose(P(p.x,p.y,p.z,0,pitch,p.yaw),false,false);
        };
        place(s+"_hip_yaw",k.hip,0);place(s+"_upper_leg",k.hip,k.upper_pitch);
        place(s+"_lower_leg",k.knee,k.lower_pitch);place(s+"_foot",k.ankle,0);
        msg.name.insert(msg.name.end(),{s+"_hip_yaw_joint",s+"_shoulder_joint",s+"_knee_joint",s+"_ankle_joint"});
        msg.position.insert(msg.position.end(),k.joints.begin(),k.joints.end());
      }else{
        setLink(s+"_hip_yaw",hx_,sign*hy_,z_+hz_);
        setLink(s+"_upper_leg",hx_,sign*hy_,z_+hz_,a[i]);
        setLink(s+"_lower_leg",hx_-l1_*std::sin(a[i]),sign*hy_,z_+hz_-l1_*std::cos(a[i]),b[i]);
        setLink(s+"_foot",fx_[i]-forward()-toe_,sign*hy_,fz_[i]);
        msg.name.insert(msg.name.end(),{s+"_hip_yaw_joint",s+"_shoulder_joint",s+"_knee_joint",s+"_ankle_joint"});
        msg.position.insert(msg.position.end(),{0,a[i]-mount_[0],b[i]-a[i]-mount_[1],-b[i]-mount_[2]});
      }
    }}catch(const std::exception& e){collision_reason_=e.what();return false;}
    model_->GetWorld()->PublishModelPose(model_);joints_->publish(msg);return true;
  }
  // SAT for oriented robot boxes against the live Gazebo scene boxes.
  struct Box {V center, half; ignition::math::Quaterniond rotation;};
  static bool hit(const Box& a,const Box& b) {
    std::array<V,3> aa,bb;for(int i=0;i<3;i++) {V axis(0,0,0);axis[i]=1;aa[i]=a.rotation.RotateVector(axis);bb[i]=b.rotation.RotateVector(axis);}
    auto separated=[&](V axis) {
      if(axis.Length()<1e-9)return false;
      axis.Normalize();double extent=0;
      for(int i=0;i<3;i++)extent+=std::abs(axis.Dot(aa[i]))*a.half[i]+std::abs(axis.Dot(bb[i]))*b.half[i];
      return std::abs((a.center-b.center).Dot(axis))>=extent-1e-6;
    };
    for(int i=0;i<3;i++){if(separated(aa[i])||separated(bb[i]))return false;for(int j=0;j<3;j++)if(separated(aa[i].Cross(bb[j])))return false;}
    return true;
  }
  bool collisionFree() const {
    std::vector<Box> parts;
    auto part=[&](const std::string& name,V offset,V size) {
      const auto p=model_->GetLink(name)->WorldPose();parts.push_back({p.Pos()+p.Rot().RotateVector(offset),size/2,p.Rot()});
    };
    auto vec=[](const YAML::Node& n){return V(n[0].as<double>(),n[1].as<double>(),n[2].as<double>());};
    if(!leg_ && action_==Action::Idle && !live_geometry_) {
      const auto p=model_->WorldPose();
      parts.push_back({V(p.Pos().X(),p.Pos().Y(),ground_+.5),V(robot_["wheel_mode_length"].as<double>()/2+.02,robot_["wheel_mode_width"].as<double>()/2+.02,.49),p.Rot()});
      const auto b=scene_[ground_>scene_["ground_heights"]["lower"].as<double>()+.01?"upper_bounds":"lower_bounds"];
      const auto& box=parts[0];
      double ex=std::abs(std::cos(yaw_))*box.half.X()+std::abs(std::sin(yaw_))*box.half.Y();
      double ey=std::abs(std::sin(yaw_))*box.half.X()+std::abs(std::cos(yaw_))*box.half.Y();
      if(x_-ex<b[0].as<double>()||x_+ex>b[1].as<double>()||y_-ey<b[2].as<double>()||y_+ey>b[3].as<double>())return false;
    } else {
      part("base_link",V::Zero,vec(robot_["body_size"]));
      part("base_link",V(hx_,0,hz_),V(robot_["leg_width"].as<double>(),2*hy_,robot_["leg_width"].as<double>()));
      part("wheel_carrier",V::Zero,V(2*radius_,track_+robot_["wheel_width"].as<double>(),2*radius_));
      for(std::string s:{"left","right"}) {
        part(s+"_upper_leg",V(0,0,-l1_/2),V(robot_["leg_width"].as<double>(),robot_["leg_width"].as<double>(),l1_));
        part(s+"_lower_leg",V(0,0,-l2_/2),V(robot_["leg_width"].as<double>(),robot_["leg_width"].as<double>(),l2_));
        part(s+"_foot",V(toe_,0,0),vec(robot_["foot_size"]));
      }
    }
    for(auto m:model_->GetWorld()->Models()) {
      if(m==model_)continue;
      const std::string name=m->GetName();
      // Markers and path visuals have no collisions. Only test real solids.
      for(auto l:m->GetLinks())for(auto c:l->GetCollisions()) {
        auto shape=boost::dynamic_pointer_cast<physics::BoxShape>(c->GetShape());if(!shape)continue;
        // In wheel mode platform/steps are enforced by the configured surface bounds.
        if(!live_geometry_&&!leg_&&action_==Action::Idle&&name.find("obstacle")==std::string::npos&&name.find("wall")==std::string::npos)continue;
        const auto p=c->WorldPose();Box obstacle{p.Pos(),shape->Size()/2,p.Rot()};
        for(const auto& box:parts)if(hit(box,obstacle)){collision_reason_=name+"/"+l->GetName();return false;}
      }
    }
    if(live_geometry_ && !leg_ && action_==Action::Idle) {
      // Require actual horizontal solid support at the configured wheel ground;
      // do not infer storey height from X or skip rails/steps in collision checks.
      for(double dx:{-.27,0.,.27})for(double dy:{-.39,0.,.39}) {
        const double xx=x_+std::cos(yaw_)*dx-std::sin(yaw_)*dy, yy=y_+std::sin(yaw_)*dx+std::cos(yaw_)*dy;
        bool supported=false;
        for(auto m:model_->GetWorld()->Models())if(m!=model_)for(auto l:m->GetLinks())for(auto c:l->GetCollisions()) {
          auto sh=boost::dynamic_pointer_cast<physics::BoxShape>(c->GetShape());if(!sh)continue;
          const auto p=c->WorldPose();const auto r=p.Rot().Euler();if(std::abs(r.X())+std::abs(r.Y())>1e-5)continue;
          const auto local=p.Rot().RotateVectorReverse(V(xx,yy,ground_)-p.Pos());const auto size=sh->Size();
          if(std::abs(local.Z()-size.Z()/2)<.003 && std::abs(local.X())<=size.X()/2+1e-6 && std::abs(local.Y())<=size.Y()/2+1e-6)supported=true;
        }
        if(!supported){collision_reason_="WHEEL_SURFACE_SUPPORT";return false;}
      }
    }
    return true;
  }
  void commandMode(const std::string& command) {
    if(command=="STOP") {action_=Action::Idle;driving_=false;v_=w_=cv_=cw_=0;state_=failed_?state_:"STOPPED";return;}
    if(failed_)return;
    if(action_!=Action::Idle||std::abs(v_)>.001||std::abs(w_)>.001){fail("MODE_REQUIRES_STOP");return;}
    if(command!="LEG"&&command!="WHEEL"){fail("INVALID_MODE");return;}
    if((command=="LEG")==leg_)return;
    if(command=="WHEEL"&&std::abs(fz_[0]-fz_[1])>.005){fail("UNEVEN_FOLD_SUPPORT");return;}
    from_z_=z_;from_lift_=lift_;from_fx_=fx_;from_fz_=fz_;begin_=model_->GetWorld()->SimTime().Double();
    action_=command=="LEG"?Action::Deploy:Action::Fold;driving_=false;
    if(action_==Action::Fold){ground_=fz_[0]-half_;spatial_foot_=false;}
    state_=command=="LEG"?"MORPH_TO_LEG":"MORPH_TO_WHEEL";
  }
  void accept(const Trajectory& msg) {
    if(failed_)return;
    if(action_!=Action::Idle||!leg_){fail("FOOT_REQUIRES_IDLE_LEG_MODE");return;}
    if(msg.header.frame_id!=frame_ || msg.joint_names!=std::vector<std::string>{"body","left_foot_center","right_foot_center"} || msg.points.size()<2||msg.points.size()>20000) {fail("INVALID_FOOT_HEADER");return;}
    double prev=-1;
    for(const auto& p:msg.points) {
      double t=rclcpp::Duration(p.time_from_start).seconds();
      if(p.transforms.size()!=3||p.velocities.size()!=3||p.accelerations.size()!=3||t<=prev||(prev>=0&&t-prev>.051)){fail("INVALID_FOOT_SAMPLES");return;}
      if(spatial_task_) {
        auto pose=[](const auto& t){return planning::Pose{t.translation.x,t.translation.y,t.translation.z,
          2*std::atan2(t.rotation.z,t.rotation.w)};};
        try{for(int side=0;side<2;++side)planning::footInverseKinematics(pose(p.transforms[0]),pose(p.transforms[side+1]),side,foot_robot_);}
        catch(const std::exception& e){fail(std::string("INVALID_SPATIAL_FOOT: ")+e.what());return;}
      }
      for(size_t i=0;i<3;i++) {
        const auto& a=p.transforms[i];const auto& body=p.transforms[0];
        ignition::math::Quaterniond q(a.rotation.w,a.rotation.x,a.rotation.y,a.rotation.z);
        const auto r=q.Euler();
        const double lateral=-std::sin(yaw_)*(a.translation.x-body.translation.x)+std::cos(yaw_)*(a.translation.y-body.translation.y);
        const double angle=std::atan2(std::sin(r.Z()-yaw_),std::cos(r.Z()-yaw_));
        if(!std::isfinite(a.translation.x)||!std::isfinite(a.translation.y)||!std::isfinite(a.translation.z)||std::abs(r.X())+std::abs(r.Y())>1e-6||(!spatial_task_ && (std::abs(angle)>.001||std::abs(lateral-(i==0?0:(i==1?hy_:-hy_)))>.001))){fail("INVALID_FOOT_COORDINATES");return;}
      }
      prev=t;
    }
    const auto& first=msg.points.front();const auto& b=first.transforms[0].translation;
    double error=std::abs(b.x-x_)+std::abs(b.y-y_)+std::abs(b.z-z_);
    for(int i=0;i<2;i++)error+=std::abs(first.transforms[i+1].translation.x*std::cos(yaw_)+first.transforms[i+1].translation.y*std::sin(yaw_)-fx_[i])+std::abs(first.transforms[i+1].translation.z-fz_[i]);
    if(error>.012||std::abs(rclcpp::Duration(first.time_from_start).seconds())>1e-9){fail("FOOT_START_MISMATCH");return;}
    spatial_foot_=spatial_task_;
    planned_=msg;begin_=model_->GetWorld()->SimTime().Double();action_=Action::Foot;state_="EXECUTING_FOOT";
  }
  void footTick(double elapsed) {
    auto b=std::lower_bound(planned_.points.begin(),planned_.points.end(),elapsed,[](const auto& p,double t){return rclcpp::Duration(p.time_from_start).seconds()<t;});
    if(b==planned_.points.end())b=planned_.points.end()-1;
    auto a=b==planned_.points.begin()?b:b-1;double ta=rclcpp::Duration(a->time_from_start).seconds(),tb=rclcpp::Duration(b->time_from_start).seconds();
    auto value=[&](int i,int axis) {
      const auto& p=a->transforms[i].translation;const auto& e=b->transforms[i].translation;
      auto get=[axis](const auto& v){return axis==0?v.x:axis==1?v.y:v.z;};
      if(tb==ta)return get(e);
      return planning::footInterpolate(get(p),get(e),get(a->velocities[i].linear),get(b->velocities[i].linear),get(a->accelerations[i].linear),get(b->accelerations[i].linear),tb-ta,std::clamp((elapsed-ta)/(tb-ta),0.,1.));
    };
    x_=value(0,0);y_=value(0,1);z_=value(0,2);
    if(spatial_foot_) {
      auto rotation=[&](int i){
        const auto& p=a->transforms[i].rotation;const auto& q=b->transforms[i].rotation;
        const double from=2*std::atan2(p.z,p.w),to=from+std::atan2(std::sin(2*std::atan2(q.z,q.w)-from),std::cos(2*std::atan2(q.z,q.w)-from));
        if(tb==ta)return to;
        return planning::footInterpolate(from,to,a->velocities[i].angular.z,b->velocities[i].angular.z,a->accelerations[i].angular.z,b->accelerations[i].angular.z,tb-ta,std::clamp((elapsed-ta)/(tb-ta),0.,1.));
      };
      yaw_=rotation(0);
      for(int i=0;i<2;++i)spatial_feet_[i]={value(i+1,0),value(i+1,1),value(i+1,2),rotation(i+1)};
    }
    for(int i=0;i<2;i++){fx_[i]=value(i+1,0)*std::cos(yaw_)+value(i+1,1)*std::sin(yaw_);fz_[i]=value(i+1,2);}
    if(elapsed>=rclcpp::Duration(planned_.points.back().time_from_start).seconds()){action_=Action::Idle;state_="FOOT_DONE";}
  }
  void publish(double now) {
    auto stamp=rclcpp::Time(int64_t(now*1e9));const auto p=model_->WorldPose();
    nav_msgs::msg::Odometry od;od.header.stamp=stamp;od.header.frame_id=frame_;od.child_frame_id="base_link";
    od.pose.pose.position.x=p.Pos().X();od.pose.pose.position.y=p.Pos().Y();od.pose.pose.position.z=p.Pos().Z();
    od.pose.pose.orientation.x=p.Rot().X();od.pose.pose.orientation.y=p.Rot().Y();od.pose.pose.orientation.z=p.Rot().Z();od.pose.pose.orientation.w=p.Rot().W();
    // Pose and velocity are measured from the applied Gazebo model, not the plan.
    od.twist.twist.linear.x=v_;od.twist.twist.linear.y=vy_;od.twist.twist.linear.z=vz_;od.twist.twist.angular.z=w_;odom_->publish(od);
    Trajectory fb;fb.header=od.header;fb.joint_names={"body","left_foot_center","right_foot_center"};
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
    point.time_from_start=rclcpp::Duration::from_seconds(action_==Action::Foot||state_=="FOOT_DONE"?std::min(now-begin_,rclcpp::Duration(planned_.points.back().time_from_start).seconds()):0.);
    for(std::string name:{"base_link","left_foot","right_foot"}) {
      auto pp=model_->GetLink(name)->WorldPose();auto pos=pp.Pos();if(name!="base_link")pos+=pp.Rot().RotateVector(V(toe_,0,0));
      geometry_msgs::msg::Transform t;t.translation.x=pos.X();t.translation.y=pos.Y();t.translation.z=pos.Z();
      t.rotation.x=pp.Rot().X();t.rotation.y=pp.Rot().Y();t.rotation.z=pp.Rot().Z();t.rotation.w=pp.Rot().W();point.transforms.push_back(t);
    }
    fb.points.push_back(point);feedback_->publish(fb);
    std_msgs::msg::String status;status.data=state_+"; mode="+(leg_?"leg":"wheel");status_->publish(status);
  }
  void tick() {
    std::lock_guard<std::mutex> lock(mutex_);double now=model_->GetWorld()->SimTime().Double();
    if(last_>=0&&now<last_){fail("SIMULATION_TIME_RESET");last_=now;}
    if(last_>=0&&now-last_<.01)return;
    double dt=last_<0?.01:now-last_;last_=now;wall_tick_=std::chrono::steady_clock::now();
    if(failed_) {if(now-report_>=.05){publish(now);report_=now;}return;}
    const auto previous=model_->WorldPose();double previous_z=z_;
    const auto previous_spatial=spatial_feet_;
    const auto previous_fx=fx_,previous_fz=fz_,previous_wheels=wheels_;double previous_lift=lift_;
    std::vector<std::pair<physics::LinkPtr,P>> previous_links;
    for(auto link:model_->GetLinks())previous_links.emplace_back(link,link->WorldPose());
    if(!failed_) {
      if((driving_||action_!=Action::Idle)&&(now-heartbeat_>feedback_timeout_)){fail("PLANNER_HEARTBEAT_TIMEOUT");}
      if(driving_&&!leg_&&action_==Action::Idle) {
        if(now-command_time_>command_timeout_){fail("VELOCITY_COMMAND_TIMEOUT");}
        else {
          v_+=std::clamp(cv_-v_,-max_a_*dt,max_a_*dt);w_+=std::clamp(cw_-w_,-max_alpha_*dt,max_alpha_*dt);
          double heading=yaw_+w_*dt/2;x_+=v_*std::cos(heading)*dt;y_+=v_*std::sin(heading)*dt;yaw_=std::atan2(std::sin(yaw_+w_*dt),std::cos(yaw_+w_*dt));
          wheels_[0]+=(v_-w_*track_/2)*dt/radius_;wheels_[1]+=(v_+w_*track_/2)*dt/radius_;folded();
          if(std::abs(cv_)+std::abs(cw_)+std::abs(v_)+std::abs(w_)<1e-8)driving_=false;
        }
      }
      double u=(now-begin_)/morph_time_;
      if(action_==Action::Deploy) {
        if(u<.5)for(int i=0;i<2;i++){fx_[i]=mix(from_fx_[i],forward()+hx_+toe_,2*u);fz_[i]=mix(from_fz_[i],ground_+half_,2*u);}
        else {fx_.fill(forward()+hx_+toe_);fz_.fill(ground_+half_);z_=mix(from_z_,ground_+body_height_,2*u-1);lift_=mix(from_lift_,lift_target_,2*u-1);}
        if(u>=1){leg_=true;action_=Action::Idle;state_="LEG_READY";}
      } else if(action_==Action::Fold) {
        if(u<.5){z_=mix(from_z_,ground_+wheel_height_,2*u);lift_=mix(from_lift_,0,2*u);}
        else {z_=ground_+wheel_height_;lift_=0;folded();auto target_fx=fx_,target_fz=fz_;for(int i=0;i<2;i++){fx_[i]=mix(from_fx_[i],target_fx[i],2*u-1);fz_[i]=mix(from_fz_[i],target_fz[i],2*u-1);}}
        if(u>=1){leg_=false;action_=Action::Idle;state_="WHEEL_READY";}
      } else if(action_==Action::Foot)footTick(now-begin_);
    }
    if(!apply(now)){fail("IK_OR_JOINT_LIMIT");}
    else if(!failed_&&!collisionFree()){fail("LIVE_GAZEBO_COLLISION_GUARD: "+collision_reason_);}
    if(failed_) {
      // Hold the last safe pose, including links (kinematic rollback).
      x_=previous.Pos().X();y_=previous.Pos().Y();z_=previous_z;yaw_=previous.Rot().Yaw();
      spatial_feet_=previous_spatial;
      fx_=previous_fx;fz_=previous_fz;wheels_=previous_wheels;lift_=previous_lift;
      model_->SetWorldPose(previous,false,false);
      for(const auto& link:previous_links)link.first->SetWorldPose(link.second,false,false);
      model_->GetWorld()->PublishModelPose(model_);vy_=vz_=0;
    } else {
      const auto current=model_->WorldPose();v_=((current.Pos().X()-previous.Pos().X())*std::cos(yaw_)+(current.Pos().Y()-previous.Pos().Y())*std::sin(yaw_))/dt;
      w_=std::atan2(std::sin(current.Rot().Yaw()-previous.Rot().Yaw()),std::cos(current.Rot().Yaw()-previous.Rot().Yaw()))/dt;
      vy_=(-(current.Pos().X()-previous.Pos().X())*std::sin(yaw_)+(current.Pos().Y()-previous.Pos().Y())*std::cos(yaw_))/dt;
      vz_=(current.Pos().Z()-previous.Pos().Z())/dt;
    }
    if(now-report_>=.05||report_<0){publish(now);report_=now;}
  }
public:
  void Load(physics::ModelPtr model,sdf::ElementPtr sdf) override {
    model_=model;model_->SetStatic(true);node_=gazebo_ros::Node::Get(sdf);
    try {
      const auto doc=YAML::LoadFile(sdf->Get<std::string>("scene_file"));scene_=doc["scene"];robot_=doc["robot"];live_geometry_=scene_["live_geometry"] && scene_["live_geometry"].as<bool>();
      config_=YAML::LoadFile(sdf->Get<std::string>("planning_file"))["planning_node"]["ros__parameters"];
      auto r=[&](const char* key){return robot_[key].as<double>();};auto t=config_["task"];
      frame_=scene_["frame"].as<std::string>();radius_=r("wheel_radius");track_=r("wheel_track");drop_=r("carrier_drop");wheel_height_=drop_+radius_;
      l1_=r("upper_leg_length");l2_=r("lower_leg_length");toe_=r("foot_center_offset_x");half_=robot_["foot_size"][2].as<double>()/2;
      hx_=robot_["shoulder_offset"][0].as<double>();hy_=robot_["shoulder_offset"][1].as<double>();hz_=robot_["shoulder_offset"][2].as<double>();
      mount_={r("folded_shoulder_pitch"),r("folded_knee_pitch"),r("folded_ankle_pitch")};
      spatial_task_=scene_["name"].as<std::string>()=="three_floor";
      auto& g=foot_robot_;g.shoulder_offset={hx_,hy_,hz_};g.folded_pitch=mount_;
      g.upper_leg_length=l1_;g.lower_leg_length=l2_;g.foot_center_offset_x=toe_;
      g.joint_min=robot_["leg_pitch_limits"][0].as<double>();g.joint_max=robot_["leg_pitch_limits"][1].as<double>();
      g.hip_yaw_min=robot_["hip_yaw_limits"][0].as<double>();g.hip_yaw_max=robot_["hip_yaw_limits"][1].as<double>();
      body_height_=config_["foot"]["body_height"].as<double>();lift_target_=config_["foot"]["wheel_lift"].as<double>();
      morph_time_=t["morph_duration"].as<double>();command_timeout_=t["command_timeout"].as<double>();feedback_timeout_=t["feedback_timeout"].as<double>();
      max_v_=config_["motion"]["max_v"].as<double>();max_w_=config_["motion"]["max_w"].as<double>();max_a_=config_["motion"]["max_a"].as<double>();max_alpha_=config_["motion"]["max_alpha"].as<double>();
      const auto p=model_->WorldPose();x_=p.Pos().X();y_=p.Pos().Y();z_=p.Pos().Z();yaw_=p.Rot().Yaw();ground_=z_-wheel_height_;folded();
      odom_=node_->create_publisher<nav_msgs::msg::Odometry>("/robot/odom",10);
      joints_=node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
      feedback_=node_->create_publisher<Trajectory>("/robot/foot_feedback",10);
      status_=node_->create_publisher<std_msgs::msg::String>("/robot/execution_status",rclcpp::QoS(1).transient_local());
      cmd_=node_->create_subscription<geometry_msgs::msg::TwistStamped>("/robot/cmd_vel",1,[this](geometry_msgs::msg::TwistStamped::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);if(failed_||leg_||action_!=Action::Idle)return;
        double stamp=rclcpp::Time(msg->header.stamp).seconds(),now=model_->GetWorld()->SimTime().Double();
        if(msg->header.frame_id!="base_link"||now-stamp>command_timeout_||stamp-now>.1||!std::isfinite(msg->twist.linear.x)||!std::isfinite(msg->twist.angular.z)){fail("INVALID_VELOCITY_COMMAND");return;}
        cv_=std::clamp(msg->twist.linear.x,0.,max_v_);cw_=std::clamp(msg->twist.angular.z,-max_w_,max_w_);command_time_=stamp;
        driving_=std::abs(cv_)+std::abs(cw_)+std::abs(v_)+std::abs(w_)>1e-8;
        if(driving_)state_="EXECUTING_WHEEL";
      });
      mode_=node_->create_subscription<std_msgs::msg::String>("/robot/mode_command",10,[this](std_msgs::msg::String::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);heartbeat_=model_->GetWorld()->SimTime().Double();if(msg->data!="HEARTBEAT")commandMode(msg->data);
      });
      trajectory_sub_=node_->create_subscription<Trajectory>("/robot/foot_trajectory",rclcpp::QoS(1).reliable(),[this](Trajectory::SharedPtr msg){std::lock_guard<std::mutex> lock(mutex_);accept(*msg);});
      watchdog_=node_->create_wall_timer(std::chrono::milliseconds(100),[this](){
        std::lock_guard<std::mutex> lock(mutex_);
        if((driving_||action_!=Action::Idle)&&!failed_&&std::chrono::duration<double>(std::chrono::steady_clock::now()-wall_tick_).count()>feedback_timeout_){fail("GAZEBO_UPDATE_TIMEOUT");std_msgs::msg::String s;s.data=state_;status_->publish(s);}
      });
      update_=event::Events::ConnectWorldUpdateBegin(std::bind(&RobotExecutor::tick,this));
      RCLCPP_INFO(node_->get_logger(),"Kinematic executor loaded; frame=%s; body wheel height=%.3f",frame_.c_str(),wheel_height_);
    } catch(const std::exception& e){RCLCPP_FATAL(node_->get_logger(),"Executor load failed: %s",e.what());throw;}
  }
};
GZ_REGISTER_MODEL_PLUGIN(RobotExecutor)
} // namespace gazebo
