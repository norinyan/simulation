// Scene visualization: building, isolated floors, cutaway interiors, and synchronized floor panels.
// All transformations below affect rendering only; the physics world is never changed.
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <yaml-cpp/yaml.h>
#include <QImage>
#include <QPainter>
#include <QLabel>
#include <QHBoxLayout>
#include <filesystem>
#include <mutex>
#include <map>
#include <chrono>
#include <cmath>
namespace gazebo {
class SceneVisualization final : public GUIPlugin {
 struct State {double t=0,x=0,y=0,z=0,yaw=0,speed=0,wait=0;std::string mode="WAITING";};
 transport::NodePtr node_;transport::SubscriberPtr sub_;event::ConnectionPtr render_;
 msgs::Scene data_;std::vector<rendering::VisualPtr> visuals_;std::map<std::string,State> states_;
 std::map<std::string,rendering::VisualPtr> people_;std::mutex mutex_;
 QLabel *labels_[3]{};std::string out_,world_,view_;int floor_=0;bool split_=false;bool ready_=false,saved_=false;double start_=-1,snapshot_after_=2;
 std::chrono::steady_clock::time_point previous_{};
 static int Floor(const std::string &s){return s.size()>1 && s[0]=='f' && s[1]>='1' && s[1]<='3'?s[1]-'0':0;}
 ignition::math::Pose3d DisplayPose(ignition::math::Pose3d p,int floor) const {
  if(split_){p.Pos().X()+=(floor-2)*22.;p.Pos().Z()-=(floor-1)*3.2;}
  return p;
 }
 public:void Load(sdf::ElementPtr)override{
  out_=std::getenv("SCENE_OUTPUT")?std::getenv("SCENE_OUTPUT"):"/tmp/scene_view";
  world_=std::getenv("SCENE_WORLD")?std::getenv("SCENE_WORLD"):"three_floor";
  if(std::getenv("SCENE_SNAPSHOT_AFTER"))snapshot_after_=std::stod(std::getenv("SCENE_SNAPSHOT_AFTER"));
  view_=std::getenv("SCENE_VIEW")?std::getenv("SCENE_VIEW"):"exterior";
  floor_=std::getenv("SCENE_FLOOR")?std::stoi(std::getenv("SCENE_FLOOR")):0;split_=view_=="split";
  std::filesystem::create_directories(out_);
  if(split_){
   setGeometry(0,0,1920,80);setStyleSheet("background-color:rgba(14,26,38,230);color:white;");
   auto layout=new QHBoxLayout(this);
   for(int i=0;i<3;++i){labels_[i]=new QLabel("Waiting for scene feedback",this);labels_[i]->setMinimumWidth(400);layout->addWidget(labels_[i],1);}
   show();
  }else{setGeometry(0,0,0,0);hide();}
  node_.reset(new transport::Node());node_->Init(world_);sub_=node_->Subscribe("~/pedestrians/state",&SceneVisualization::Feedback,this);
  render_=event::Events::ConnectPostRender(std::bind(&SceneVisualization::Render,this));
 }
 private:void Feedback(ConstGzStringPtr &message){
  try{const auto n=YAML::Load(message->data());State s;s.t=n["sim_s"].as<double>();s.x=n["x"].as<double>();s.y=n["y"].as<double>();s.z=n["z"].as<double>();s.yaw=n["yaw"].as<double>();s.wait=n["wait_remaining"].as<double>();s.mode=n["state"].as<std::string>();
   std::lock_guard<std::mutex> lock(mutex_);auto it=states_.find(n["name"].as<std::string>());
   if(it!=states_.end() && s.t>it->second.t)s.speed=std::hypot(s.x-it->second.x,s.y-it->second.y)/(s.t-it->second.t);
   states_[n["name"].as<std::string>()]=s;
  }catch(const std::exception &e){gzerr<<"Invalid pedestrian feedback: "<<e.what()<<"\n";}
 }
 // Preserve the established floor isolation and wall cutaway rules.
 bool Visible(const std::string &name,int floor) const {
  bool show=split_ || floor_==0 || floor==floor_;
  if(view_=="interior"){
   for(const auto *wall:{"wall_001","wall_006","wall_010","wall_004","wall_005","wall_009","wall_013"})
    if(name.find(wall)!=std::string::npos)show=false;
  }
  if((split_ || view_=="top") && name.find("lintel")!=std::string::npos)show=false;
  if(view_=="top" && name.find("_stair_")!=std::string::npos)show=floor==std::min(floor_,2);
  return show;
 }
 void SetCamera(const rendering::UserCameraPtr &camera){
  if(split_){camera->SetHFOV(ignition::math::Angle(M_PI/3));camera->SetWorldPose(ignition::math::Pose3d(0,0,57,0,M_PI/2,M_PI/2));return;}
  const double z=(std::max(floor_,1)-1)*3.2;
  ignition::math::Vector3d position(-20,-23,z+20),target(0,0,z+.5);
  if(view_=="top"){position.Set(0,0,z+28);target.Set(0,0,z);}
  if(floor_==0){position.Set(-23,-26,22);target.Set(0,0,4);}
  const auto delta=target-position;
  const double yaw=view_=="top"?M_PI/2:std::atan2(delta.Y(),delta.X());
  const double pitch=std::atan2(-delta.Z(),std::hypot(delta.X(),delta.Y()));
  camera->SetWorldPose(ignition::math::Pose3d(position,ignition::math::Quaterniond(0,pitch,yaw)));
 }
 void Build(const rendering::ScenePtr &scene){
  auto response=transport::request(world_,"scene_info");if(!response)return;if(!data_.ParseFromString(response->serialized_data()) || data_.model_size()==0)return;
  for(const auto &m:data_.model()){
   auto old=scene->GetVisual(m.name());if(old)old->SetVisible(false);
   for(const auto &l:m.link())for(const auto &v:l.visual()){auto original=scene->GetVisual(v.name());if(original)original->SetVisible(false);}
   const int floor=Floor(m.name());if(!floor)continue;
   if(!Visible(m.name(),floor))continue;
   const auto name="scene_view::"+m.name();rendering::VisualPtr root(new rendering::Visual(name,scene->WorldVisual()));root->Load();root->SetWorldPose(DisplayPose(msgs::ConvertIgn(m.pose()),floor));visuals_.push_back(root);
   if(m.name().find("_pedestrian")!=std::string::npos)people_[m.name()]=root;
   for(const auto &l:m.link()){
    rendering::VisualPtr link(new rendering::Visual("scene_view::"+l.name(),root));link->Load();link->SetPose(msgs::ConvertIgn(l.pose()));visuals_.push_back(link);
    for(const auto &original:l.visual())if(original.type()==msgs::Visual::VISUAL){msgs::Visual v=original;v.set_name("scene_view::"+original.name());v.set_parent_name(link->Name());v.clear_id();v.clear_parent_id();v.set_visible(true);rendering::VisualPtr leaf(new rendering::Visual(v.name(),link));ConstVisualPtr msg(new msgs::Visual(v));leaf->LoadFromMsg(msg);visuals_.push_back(leaf);}
   }
   if(split_)for(const auto &plugin:m.plugin())if(plugin.name()=="pedestrian_patrol"){
    const auto sdf=std::string("<sdf version='1.6'><model name='route'><link name='unused'/><plugin name='p' filename='unused'>")+plugin.innerxml()+"</plugin></model></sdf>";
    sdf::SDF parsed;parsed.SetFromString(sdf);auto p=parsed.Root()->GetElement("model")->GetElement("plugin");
    auto line=scene->WorldVisual()->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);line->setMaterial(floor==1?"Gazebo/Red":floor==2?"Gazebo/Blue":"Gazebo/Green");
    for(auto wp=p->GetElement("waypoint");wp;wp=wp->GetNextElement("waypoint")){auto v=wp->Get<ignition::math::Vector3d>("position");v.X()+=(floor-2)*22.;v.Z()=.035;line->AddPoint(v);}
    line->Update();
   }
  }
  ready_=true;
 }
 void Render(){
  auto wall=std::chrono::steady_clock::now();if(std::chrono::duration<double>(wall-previous_).count()<.045)return;previous_=wall;
  auto camera=gui::get_active_camera();if(!camera||!camera->GetScene())return;auto scene=camera->GetScene();
  camera->SetCaptureData(true);
  if(!ready_){Build(scene);if(ready_)SetCamera(camera);return;}
  std::map<std::string,State> states;{std::lock_guard<std::mutex> lock(mutex_);states=states_;}
  if(split_)setFixedWidth(camera->ImageWidth());
  if(split_)for(int f=1;f<=3;++f){auto it=states.find("f"+std::to_string(f)+"_pedestrian");if(it!=states.end()){auto &s=it->second;labels_[f-1]->setText(QString("F%1 | %2 | sim %3 s<br/>actual speed %4 m/s | pause %5 s").arg(f).arg(QString::fromStdString(s.mode)).arg(s.t,0,'f',1).arg(s.speed,0,'f',2).arg(s.wait,0,'f',1));}}
  double sim=0;for(const auto &s:states){sim=std::max(sim,s.second.t);auto p=people_.find(s.first);if(p!=people_.end())p->second->SetWorldPose(DisplayPose(ignition::math::Pose3d(s.second.x,s.second.y,s.second.z,0,0,s.second.yaw),Floor(s.first)));}
  if(states.size()!=3 || saved_)return;if(start_<0)start_=sim;
  if(sim-start_<snapshot_after_)return;
  const auto *bytes=camera->ImageData();if(!bytes)return;int w=camera->ImageWidth(),h=camera->ImageHeight();
  QImage image(bytes,w,h,w*3,QImage::Format_RGB888);
  if(!split_){saved_=image.save(QString::fromStdString(out_+"/scene_view.png"));return;}
  int crop=std::min(h,w*720/1920);QImage view=image.copy(0,(h-crop)/2,w,crop).scaled(1920,720,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
  QPainter painter(&view);painter.fillRect(0,0,1920,90,QColor(14,26,38));QFont font("DejaVu Sans",15);painter.setFont(font);
  for(int f=1;f<=3;++f){const auto &s=states.at("f"+std::to_string(f)+"_pedestrian");QColor color=f==1?QColor(240,95,70):f==2?QColor(80,170,250):QColor(85,220,120);painter.setPen(color);int x=(f-1)*640+18;
   painter.drawText(x,30,QString("F%1 | %2 | sim %3 s").arg(f).arg(QString::fromStdString(s.mode)).arg(s.t,0,'f',1));
   painter.setPen(Qt::white);painter.drawText(x,66,QString("actual speed %1 m/s | pause left %2 s").arg(s.speed,0,'f',2).arg(s.wait,0,'f',1));
   if(f>1){painter.setPen(QColor(60,80,100));painter.drawLine((f-1)*640,0,(f-1)*640,720);}
  }
  // Numbered waypoints projected into the common north-up top camera.
  const double scale=w/(2*57*std::tan(M_PI/6));
  painter.setPen(Qt::yellow);for(const auto &m:data_.model())for(const auto &plugin:m.plugin())if(plugin.name()=="pedestrian_patrol"){
   sdf::SDF sdf;sdf.SetFromString("<sdf version='1.6'><model name='m'><link name='l'/><plugin name='p' filename='x'>"+plugin.innerxml()+"</plugin></model></sdf>");auto p=sdf.Root()->GetElement("model")->GetElement("plugin");int k=0;
   for(auto wp=p->GetElement("waypoint");wp;wp=wp->GetNextElement("waypoint")){auto point=wp->Get<ignition::math::Vector3d>("position");double x=(w/2.+(point.X()+(Floor(m.name())-2)*22.)*scale)*1920/w;double y=(h/2.-point.Y()*scale-(h-crop)/2.)*720/crop;painter.drawEllipse(QPointF(x,y),5,5);painter.drawText(QPointF(x+7,y-7),QString::number(++k));}
  }
  painter.end();saved_=view.save(QString::fromStdString(out_+"/three_floors_top.png"));
 }
};GZ_REGISTER_GUI_PLUGIN(SceneVisualization)
}
