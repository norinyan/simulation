// Scene model control. Currently implements deterministic pedestrian patrols with real collisions.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/node.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>
namespace gazebo {
class SceneController final : public ModelPlugin {
  struct Waypoint { ignition::math::Vector3d p; double pause; };
  struct Phase { size_t from, to; double begin, duration; bool wait; int direction; };
  physics::ModelPtr model_;
  event::ConnectionPtr update_;
  transport::NodePtr node_;
  transport::PublisherPtr state_;
  gazebo_ros::Node::SharedPtr ros_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr feedback_;
  std::vector<Waypoint> points_;
  std::vector<Phase> phases_;
  double speed_{0}, delay_{0}, period_{0}, last_publish_{-1};
  double initial_yaw_{0};
  bool reverse_{false}, paused_{false};
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    try {
      ros_=gazebo_ros::Node::Get(sdf,model->GetName());
      feedback_=ros_->create_publisher<nav_msgs::msg::Odometry>("/scene/pedestrians",rclcpp::QoS(20).reliable());
      model_=model; speed_=sdf->Get<double>("speed");
      delay_=sdf->HasElement("start_delay") ? sdf->Get<double>("start_delay") : 0.;
      const std::string direction=sdf->HasElement("initial_direction") ? sdf->Get<std::string>("initial_direction") : "forward";
      if(direction!="forward" && direction!="reverse")throw std::runtime_error("initial_direction must be forward or reverse");
      reverse_=direction=="reverse";
      paused_=sdf->HasElement("paused") && sdf->Get<bool>("paused");
      if(!std::isfinite(speed_) || speed_<=0 || !std::isfinite(delay_) || delay_<0)throw std::runtime_error("invalid speed/start_delay");
      if(!sdf->HasElement("waypoint"))throw std::runtime_error("missing waypoints");
      for(auto p=sdf->GetElement("waypoint");p;p=p->GetNextElement("waypoint")) {
        Waypoint v{p->Get<ignition::math::Vector3d>("position"),p->HasElement("pause")?p->Get<double>("pause"):0.};
        if(!v.p.IsFinite() || !std::isfinite(v.pause) || v.pause<0)throw std::runtime_error("invalid waypoint");
        if(!points_.empty() && (std::abs(v.p.Z()-points_[0].p.Z())>1e-8 || v.p.Distance(points_.back().p)<1e-6))throw std::runtime_error("waypoints must be distinct, on one floor");
        points_.push_back(v);
      }
      if(points_.size()<2)throw std::runtime_error("need at least two waypoints");
      auto phase=[&](size_t a,size_t b,int dir){
        if(points_[a].pause>0){phases_.push_back({a,a,period_,points_[a].pause,true,dir});period_+=points_[a].pause;}
        const double duration=points_[a].p.Distance(points_[b].p)/speed_;
        phases_.push_back({a,b,period_,duration,false,dir});period_+=duration;
      };
      if(!reverse_){
        for(size_t i=0;i+1<points_.size();++i)phase(i,i+1,1);
        for(size_t i=points_.size()-1;i>0;--i)phase(i,i-1,-1);
      }else{
        for(size_t i=points_.size()-1;i>0;--i)phase(i,i-1,-1);
        for(size_t i=0;i+1<points_.size();++i)phase(i,i+1,1);
      }
      const size_t first=reverse_?points_.size()-1:0,next=reverse_?first-1:1;
      const auto d=points_[next].p-points_[first].p;initial_yaw_=std::atan2(d.Y(),d.X());
      node_.reset(new transport::Node());node_->Init(model_->GetWorld()->Name());
      state_=node_->Advertise<msgs::GzString>("~/pedestrians/state",100);
      update_=event::Events::ConnectWorldUpdateBegin(std::bind(&SceneController::Update,this,std::placeholders::_1));
      gzmsg<<"Pedestrian patrol loaded: "<<model_->GetName()<<" period="<<period_<<" speed="<<speed_<<"\n";
    }catch(const std::exception &e){gzerr<<"Pedestrian patrol rejected: "<<e.what()<<"\n";throw;}
  }
 private:
  void Update(const common::UpdateInfo &info){
    const double now=info.simTime.Double(), elapsed=now-delay_;
    size_t at=reverse_?points_.size()-1:0;auto pos=points_[at].p;
    double yaw=initial_yaw_,remaining=std::max(0.,-elapsed);int dir=reverse_?-1:1;
    std::string mode=paused_?"PAUSED":"DELAY";
    if(paused_)remaining=0;
    if(!paused_ && elapsed>=0){
      const double t=std::fmod(elapsed,period_);
      for(const auto &p:phases_)if(t<p.begin+p.duration){
        at=p.from;dir=p.direction;mode=p.wait?"PAUSE":(dir>0?"FORWARD":"RETURN");remaining=p.wait?p.begin+p.duration-t:0.;
        const auto delta=points_[p.to].p-points_[p.from].p;
        if(!p.wait){pos=points_[p.from].p+delta*((t-p.begin)/p.duration);yaw=std::atan2(delta.Y(),delta.X());}
        else{pos=points_[p.from].p;size_t next=dir>0?at+1:at-1;const auto heading=points_[next].p-pos;yaw=std::atan2(heading.Y(),heading.X());}
        break;
      }
    }
    model_->SetWorldPose(ignition::math::Pose3d(pos,ignition::math::Quaterniond(0,0,yaw)));
    if(now<last_publish_ || now-last_publish_>=.099999){
      const auto actual=model_->WorldPose();std::ostringstream out;out<<std::setprecision(12)
        <<"{\"name\":\""<<model_->GetName()<<"\",\"sim_s\":"<<now<<",\"x\":"<<actual.Pos().X()<<",\"y\":"<<actual.Pos().Y()<<",\"z\":"<<actual.Pos().Z()
        <<",\"yaw\":"<<actual.Rot().Yaw()<<",\"state\":\""<<mode<<"\",\"speed\":"<<(mode=="FORWARD"||mode=="RETURN"?speed_:0)
        <<",\"wait_remaining\":"<<remaining<<",\"waypoint\":"<<at<<",\"direction\":"<<dir<<",\"period\":"<<period_<<"}";
      nav_msgs::msg::Odometry feedback;feedback.header.stamp=rclcpp::Time(int64_t(now*1e9));feedback.header.frame_id="world";feedback.child_frame_id=model_->GetName();
      feedback.pose.pose.position.x=actual.Pos().X();feedback.pose.pose.position.y=actual.Pos().Y();feedback.pose.pose.position.z=actual.Pos().Z();feedback.pose.pose.orientation.z=actual.Rot().Z();feedback.pose.pose.orientation.w=actual.Rot().W();feedback.twist.twist.linear.x=mode=="FORWARD"||mode=="RETURN"?speed_:0;feedback_->publish(feedback);
      msgs::GzString message;message.set_data(out.str());state_->Publish(message);last_publish_=now;
    }
  }
};GZ_REGISTER_MODEL_PLUGIN(SceneController)
}
