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
#include <fstream>
#include <mutex>
#include <map>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <deque>
namespace gazebo {
class SceneVisualization final : public GUIPlugin {
 struct State {double t=0,x=0,y=0,z=0,yaw=0,speed=0,wait=0;std::string mode="WAITING";};
 transport::NodePtr node_;transport::SubscriberPtr sub_,pose_sub_,overlay_sub_;event::ConnectionPtr render_,capture_;
 msgs::Scene data_;std::vector<rendering::VisualPtr> visuals_;std::map<std::string,State> states_;
 std::map<std::string,rendering::VisualPtr> people_,robot_links_,models_;
 struct Surface {rendering::VisualPtr visual;float transparency;};
 std::vector<Surface> occluders_;
 struct TimedPose {double wall,stamp;ignition::math::Pose3d pose;};
 using PoseHistory=std::deque<TimedPose>;
 std::map<std::string,PoseHistory> robot_poses_,pedestrian_poses_;std::mutex mutex_;
 struct Overlay {std::string text;std::vector<ignition::math::Vector3d> points;bool dirty=true;};
 std::map<std::string,Overlay> overlays_;
 std::map<std::string,rendering::DynamicLines*> overlay_lines_;
 bool refresh_overlays_=true;
 std::string foot_data_,drawn_foot_data_;
 uint64_t foot_version_{0};bool refresh_feet_=false;
 std::vector<rendering::VisualPtr> foot_markers_;
 std::string config_text_;bool follow_=false,capture_pending_=false;std::vector<rendering::DynamicLines*> lines_;
 std::chrono::steady_clock::time_point config_check_{};
 ignition::math::Vector3d marker_start_,marker_goal_;bool markers_=false;
 int robot_floor_=0;
 QLabel *labels_[3]{};std::string out_,world_,view_;int floor_=0;bool split_=false;bool ready_=false,saved_=false;double start_=-1,snapshot_after_=2;
 static double Wall(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
 static void Remember(PoseHistory& history,double stamp,const ignition::math::Pose3d& pose){
  if(!history.empty() && stamp<history.back().stamp)history.clear();
  if(!history.empty() && stamp==history.back().stamp){history.back().pose=pose;return;}
  history.push_back({Wall(),stamp,pose});
  while(history.size()>128)history.pop_front();
 }
 static ignition::math::Pose3d Sample(const PoseHistory& history,double wall){
  for(std::size_t i=1;i<history.size();++i)if(history[i].wall>=wall){
   const auto& a=history[i-1];const auto& b=history[i];
   const double t=std::clamp((wall-a.wall)/(b.wall-a.wall),0.,1.);
   return ignition::math::Pose3d(a.pose.Pos()+(b.pose.Pos()-a.pose.Pos())*t,
     ignition::math::Quaterniond::Slerp(t,a.pose.Rot(),b.pose.Rot(),true));
  }
  return history.back().pose;
 }
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
  pose_sub_=node_->Subscribe("~/pose/info",&SceneVisualization::Poses,this);
  overlay_sub_=node_->Subscribe("~/planning/path_overlay",&SceneVisualization::PathOverlay,this);
  render_=event::Events::ConnectPreRender(std::bind(&SceneVisualization::Render,this));
  capture_=event::Events::ConnectPostRender(std::bind(&SceneVisualization::Capture,this));
 }
 private:void Feedback(ConstGzStringPtr &message){
  try{const auto n=YAML::Load(message->data());State s;s.t=n["sim_s"].as<double>();s.x=n["x"].as<double>();s.y=n["y"].as<double>();s.z=n["z"].as<double>();s.yaw=n["yaw"].as<double>();s.wait=n["wait_remaining"].as<double>();s.mode=n["state"].as<std::string>();
   std::lock_guard<std::mutex> lock(mutex_);auto it=states_.find(n["name"].as<std::string>());
   if(it!=states_.end() && s.t>it->second.t)s.speed=std::hypot(s.x-it->second.x,s.y-it->second.y)/(s.t-it->second.t);
   states_[n["name"].as<std::string>()]=s;
   Remember(pedestrian_poses_[n["name"].as<std::string>()],s.t,ignition::math::Pose3d(s.x,s.y,s.z,0,0,s.yaw));
  }catch(const std::exception &e){gzerr<<"Invalid pedestrian feedback: "<<e.what()<<"\n";}
 }
 void Poses(ConstPosesStampedPtr &message){
  std::lock_guard<std::mutex> lock(mutex_);
  for(const auto &pose:message->pose())if(pose.name()=="wheel_leg_robot" || pose.name().find("wheel_leg_robot::")==0)
   Remember(robot_poses_[pose.name()],message->time().sec()+message->time().nsec()*1e-9,msgs::ConvertIgn(pose));
 }
 void PathOverlay(ConstGzStringPtr& message){
  try{
   const auto data=YAML::Load(message->data());const auto name=data["group"].as<std::string>();
   if(name=="footsteps"){std::lock_guard<std::mutex> lock(mutex_);
    const auto version=data["version"].as<uint64_t>(0);
    if(version>foot_version_){foot_version_=version;foot_data_=message->data();}return;}
   if(name.rfind("global_",0)!=0 && name!="local_0" && name!="local_1")return;
   std::lock_guard<std::mutex> lock(mutex_);auto& overlay=overlays_[name];
   if(overlay.text==message->data())return;
   overlay.text=message->data();overlay.points.clear();
   for(const auto& p:data["points"])overlay.points.emplace_back(p[0].as<double>(),p[1].as<double>(),p[2].as<double>());
   overlay.dirty=true;
  }catch(const std::exception& e){gzerr<<"Path overlay: "<<e.what()<<"\n";}
 }
 void DrawFootsteps(const rendering::ScenePtr& scene) {
  std::string text;{std::lock_guard<std::mutex> lock(mutex_);text=foot_data_;}
  if(text.empty() || (text==drawn_foot_data_ && !refresh_feet_))return;
  try {
   const auto data=YAML::Load(text),size=data["size"];
   const bool replace=text!=drawn_foot_data_;
   std::vector<rendering::VisualPtr> next;
   for(const auto& step:data["steps"]) {
    const auto id=next.size();const auto z=step[3].as<double>();
    int floor=world_=="three_floor"?int(std::round((z-size[2].as<double>()/2)/3.2))+1:0;
    rendering::VisualPtr marker;
    if(!replace)marker=foot_markers_.at(id);
    else {marker.reset(new rendering::Visual("planned_sole_"+data["version"].as<std::string>()+"_"+std::to_string(id),scene->WorldVisual()));marker->Load();}
    msgs::Visual visual;visual.set_name(marker->Name());visual.mutable_geometry()->set_type(msgs::Geometry::BOX);
    msgs::Set(visual.mutable_geometry()->mutable_box()->mutable_size(),ignition::math::Vector3d(size[0].as<double>(),size[1].as<double>(),.008));
    auto color=step[0].as<int>()==0?ignition::math::Color(.05,.8,1,1):ignition::math::Color(1,.45,.05,1);
    msgs::Set(visual.mutable_material()->mutable_ambient(),color);msgs::Set(visual.mutable_material()->mutable_diffuse(),color);
    marker->UpdateFromMsg(boost::make_shared<msgs::Visual>(visual));
    marker->SetPose(DisplayPose(ignition::math::Pose3d(step[1].as<double>(),step[2].as<double>(),z-size[2].as<double>()/2+.012,0,0,step[4].as<double>()),floor));
    marker->SetVisible(Visible("path",floor));next.push_back(marker);
   }
   if(replace)for(auto& old:foot_markers_)scene->RemoveVisual(old);
   foot_markers_=std::move(next);drawn_foot_data_=text;refresh_feet_=false;
  }catch(const std::exception& e){gzerr<<"Footstep display: "<<e.what()<<"\n";}
 }
 void DrawOverlays(const rendering::ScenePtr& scene){
  std::map<std::string,std::vector<ignition::math::Vector3d>> changes;
  {std::lock_guard<std::mutex> lock(mutex_);for(auto& entry:overlays_)if(entry.second.dirty || refresh_overlays_){
   changes[entry.first]=entry.second.points;entry.second.dirty=false;
  }}
  refresh_overlays_=false;
  for(const auto& entry:changes){
   auto& line=overlay_lines_[entry.first];
   if(!line){line=scene->WorldVisual()->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
    line->setMaterial(entry.first.find("local_")==0?"Gazebo/Green":entry.first=="global_1"?"Gazebo/Orange":"Gazebo/Blue");}
   line->Clear();
   for(auto point:entry.second){
    const int floor=world_=="three_floor"?int(std::round(point.Z()/3.2))+1:0;
    if(world_=="three_floor" && !Visible("path",floor))continue;
    line->AddPoint(DisplayPose(ignition::math::Pose3d(point,ignition::math::Quaterniond::Identity),floor).Pos());
   }
   line->Update();
  }
 }
 void Configure(const rendering::ScenePtr &scene){
  const auto now=std::chrono::steady_clock::now();
  if(std::chrono::duration<double>(now-config_check_).count()<.2)return;
  config_check_=now;std::ifstream file(out_+"/viewer.yaml");if(!file)return;
  const std::string text((std::istreambuf_iterator<char>(file)),std::istreambuf_iterator<char>());
  if(text==config_text_)return;
  try{
   auto config=YAML::Load(text);const auto view=config["view"].as<std::string>();
   const int floor=config["floor"].as<int>();
   if((view!="exterior"&&view!="top"&&view!="interior"&&view!="split")||floor<0||floor>3)return;
   const bool layout=view!=view_||floor!=floor_||config["robot_generation"].as<int>(0)!=generation_;
   if(generation_!=config["robot_generation"].as<int>(0)){
    for(auto& marker:foot_markers_)scene->RemoveVisual(marker);foot_markers_.clear();
    std::lock_guard<std::mutex> lock(mutex_);foot_data_.clear();drawn_foot_data_.clear();
   }
   generation_=config["robot_generation"].as<int>(0);
   view_=view;floor_=floor;follow_=config["follow"].as<bool>(false);
   markers_=config["start"] && config["goal"];
   if(markers_){auto a=config["start"],b=config["goal"];marker_start_.Set(a[0].as<double>(),a[1].as<double>(),a[2].as<double>());marker_goal_.Set(b[0].as<double>(),b[1].as<double>(),b[2].as<double>());}
   // Marker changes also rebuild the GUI hierarchy; physical models remain untouched.
   const bool changed=text!=config_text_;config_text_=text;
   if(layout||changed){
    for(auto line:lines_)scene->WorldVisual()->DeleteDynamicLine(line);lines_.clear();
    // Retain model/link visuals across view switches. Reusing their names after
    // removal races Gazebo's render cache and can produce an empty viewport.
    ready_=false;saved_=false;start_=-1;
    refresh_overlays_=true;refresh_feet_=true;
    const bool next_split=view_=="split";
    if(next_split && !labels_[0]){
     auto layout=new QHBoxLayout(this);
     for(int i=0;i<3;++i){labels_[i]=new QLabel(this);labels_[i]->setMinimumWidth(400);layout->addWidget(labels_[i],1);}
    }
    split_=next_split;
    if(split_){setGeometry(0,0,1920,80);setStyleSheet("background-color:rgba(14,26,38,230);color:white;");show();}
    else hide();
   }
  }catch(const std::exception &e){gzerr<<"Viewer configuration: "<<e.what()<<"\n";}
 }
 int generation_=-1;
 void Markers(const rendering::ScenePtr &scene){
  if(!markers_)return;
  for(int index=0;index<2;++index){auto p=index?marker_goal_:marker_start_;int floor=world_=="three_floor"?int(std::round(p.Z()/3.2))+1:0;
   if(world_=="three_floor" && !Visible("marker",floor))continue;
   auto display=DisplayPose(ignition::math::Pose3d(p,ignition::math::Quaterniond::Identity),floor).Pos();
   auto line=scene->WorldVisual()->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);lines_.push_back(line);
   line->setMaterial(index?"Gazebo/Red":"Gazebo/Green");
   for(int i=0;i<=32;++i){double a=2*M_PI*i/32;line->AddPoint(display+ignition::math::Vector3d(.3*std::cos(a),.3*std::sin(a),.05));}line->Update();
  }
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
  camera->SetRenderRate(60);
  if(split_){camera->SetHFOV(ignition::math::Angle(M_PI/3));camera->SetWorldPose(ignition::math::Pose3d(0,0,57,0,M_PI/2,M_PI/2));return;}
  camera->SetHFOV(ignition::math::Angle(follow_?80*M_PI/180:M_PI/3));
  const double z=(std::max(floor_,1)-1)*3.2;
  ignition::math::Vector3d position(-20,-23,z+20),target(0,0,z+.5);
  if(view_=="top"){position.Set(0,0,z+28);target.Set(0,0,z);}
  if(floor_==0){position.Set(-23,-26,22);target.Set(0,0,4);}
  if(world_=="two_steps"){
   target.Set(2.25,0,0);position.Set(13,-17,13);
   if(view_=="top")position.Set(2.25,0,22);
  }
  const auto delta=target-position;
  const double yaw=view_=="top"?M_PI/2:std::atan2(delta.Y(),delta.X());
  const double pitch=std::atan2(-delta.Z(),std::hypot(delta.X(),delta.Y()));
  camera->SetWorldPose(ignition::math::Pose3d(position,ignition::math::Quaterniond(0,pitch,yaw)));
 }
 void Build(const rendering::ScenePtr &scene){
  auto response=transport::request(world_,"scene_info");if(!response)return;if(!data_.ParseFromString(response->serialized_data()) || data_.model_size()==0)return;
  // Hide editor gizmos, while keeping every light and its illumination active.
  for(uint32_t i=0;i<scene->LightCount();++i)scene->LightByIndex(i)->ShowVisual(false);
  for(const auto &m:data_.model()){
   auto old=scene->GetVisual(m.name());if(old)old->SetVisible(false);
   for(const auto &l:m.link())for(const auto &v:l.visual()){auto original=scene->GetVisual(v.name());if(original)original->SetVisible(false);}
   const bool robot=m.name()=="wheel_leg_robot";
   const int floor=robot?int(std::round((m.pose().position().z()-.32)/3.2))+1:Floor(m.name());
   if(!robot && !floor && world_=="three_floor")continue;
   const bool show=world_!="three_floor" || Visible(m.name(),floor);
   if(robot)robot_floor_=floor;
   auto found=models_.find(m.name());
   if(found==models_.end()){
    const auto name="scene_view::"+m.name();rendering::VisualPtr root(new rendering::Visual(name,scene->WorldVisual()));root->Load();visuals_.push_back(root);models_[m.name()]=root;
    if(robot)robot_links_[m.name()]=root;
    if(m.name().find("_pedestrian")!=std::string::npos)people_[m.name()]=root;
    for(const auto &l:m.link()){
     rendering::VisualPtr link(new rendering::Visual("scene_view::"+l.name(),root));link->Load();link->SetPose(msgs::ConvertIgn(l.pose()));visuals_.push_back(link);
     if(robot)robot_links_[l.name()]=link;
     for(const auto &original:l.visual())if(original.type()==msgs::Visual::VISUAL){
      msgs::Visual v=original;v.set_name("scene_view::"+original.name());v.set_parent_name(link->Name());v.clear_id();v.clear_parent_id();v.set_visible(true);
      rendering::VisualPtr leaf(new rendering::Visual(v.name(),link));ConstVisualPtr msg(new msgs::Visual(v));leaf->LoadFromMsg(msg);visuals_.push_back(leaf);
      // Keep original transparency so leaving follow mode restores glass/materials.
      if(m.name().find("wall")!=std::string::npos || m.name().find("door")!=std::string::npos ||
         m.name().find("window")!=std::string::npos || m.name().find("lintel")!=std::string::npos)
       occluders_.push_back({leaf,leaf->GetTransparency()});
     }
    }
   }
   auto root=models_.at(m.name());root->SetWorldPose(DisplayPose(msgs::ConvertIgn(m.pose()),floor));root->SetVisible(show);
   if(split_)for(const auto &plugin:m.plugin())if(plugin.name()=="pedestrian_patrol"){
    const auto sdf=std::string("<sdf version='1.6'><model name='route'><link name='unused'/><plugin name='p' filename='unused'>")+plugin.innerxml()+"</plugin></model></sdf>";
    sdf::SDF parsed;parsed.SetFromString(sdf);auto p=parsed.Root()->GetElement("model")->GetElement("plugin");
    auto line=scene->WorldVisual()->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);lines_.push_back(line);line->setMaterial(floor==1?"Gazebo/Red":floor==2?"Gazebo/Blue":"Gazebo/Green");
    for(auto wp=p->GetElement("waypoint");wp;wp=wp->GetNextElement("waypoint")){auto v=wp->Get<ignition::math::Vector3d>("position");v.X()+=(floor-2)*22.;v.Z()=.035;line->AddPoint(v);}
    line->Update();
   }
  }
  // A removed robot has no physical model in the fresh Scene response.
  if(models_.count("wheel_leg_robot")){
   bool present=false;for(const auto &m:data_.model())if(m.name()=="wheel_leg_robot")present=true;
   if(!present)models_.at("wheel_leg_robot")->SetVisible(false);
  }
  // A wide follow camera may sit behind a room wall. Fade only render surfaces;
  // the physical walls, doors and windows retain their original collisions.
  for(const auto &surface:occluders_)
   surface.visual->SetTransparency(follow_?surface.transparency+(1-surface.transparency)*.72f:surface.transparency);
  Markers(scene);ready_=true;
 }
 bool SaveImage(const QImage &image,const std::string &name){
  const auto target=std::filesystem::path(out_)/name;
  const auto temporary=std::filesystem::path(out_)/("."+name);
  if(!image.save(QString::fromStdString(temporary.string()),"PNG"))return false;
  std::error_code error;std::filesystem::rename(temporary,target,error);
  return !error;
 }
 void Render(){
  auto camera=gui::get_active_camera();if(!camera||!camera->GetScene())return;auto scene=camera->GetScene();
  Configure(scene);
  if(saved_){camera->SetCaptureData(false);capture_pending_=false;}
  if(!ready_){Build(scene);if(ready_)SetCamera(camera);return;}
  std::map<std::string,ignition::math::Pose3d> robot;
  // A 100 ms display buffer absorbs transport jitter. It never changes physics,
  // collision checks or planning feedback, and never extrapolates stale poses.
  const double display_time=Wall()-.1;
  {std::lock_guard<std::mutex> lock(mutex_);for(const auto& entry:robot_poses_)robot[entry.first]=Sample(entry.second,display_time);}
  auto tracked=robot.find("wheel_leg_robot");
  if(world_=="three_floor" && tracked!=robot.end()){
   const int level=std::clamp(int(std::floor((tracked->second.Pos().Z()-.32+.005)/3.2))+1,1,3);
   if(robot_floor_!=level){robot_floor_=level;
    if(follow_ && floor_!=level){floor_=level;ready_=false;}
   }
   auto model=models_.find("wheel_leg_robot");
   if(model!=models_.end())model->second->SetVisible(Visible("wheel_leg_robot",robot_floor_));
  }
  for(const auto &pose:robot){
   auto visual=robot_links_.find(pose.first);if(visual==robot_links_.end())continue;
   // Gazebo pose/info model poses are world-relative; link poses are relative
   // to the model. Preserve that hierarchy instead of applying the offset twice.
   if(pose.first=="wheel_leg_robot")visual->second->SetWorldPose(DisplayPose(pose.second,robot_floor_));
   else visual->second->SetPose(pose.second);
  }
  auto body=robot.find("wheel_leg_robot");
  if(follow_ && body!=robot.end()){
   auto target=DisplayPose(body->second,robot_floor_).Pos();target.Z()+=.25;
   const double yaw=body->second.Rot().Yaw();auto position=target+ignition::math::Vector3d(-4.8*std::cos(yaw),-4.8*std::sin(yaw),4.8);
   auto delta=target-position;camera->SetWorldPose(ignition::math::Pose3d(position,ignition::math::Quaterniond(0,std::atan2(-delta.Z(),std::hypot(delta.X(),delta.Y())),std::atan2(delta.Y(),delta.X()))));
  }
  std::map<std::string,State> states;{std::lock_guard<std::mutex> lock(mutex_);states=states_;}
  DrawOverlays(scene);DrawFootsteps(scene);
  if(split_)setFixedWidth(camera->ImageWidth());
  if(split_)for(int f=1;f<=3;++f){auto it=states.find("f"+std::to_string(f)+"_pedestrian");if(it!=states.end()){auto &s=it->second;labels_[f-1]->setText(QString("F%1 | %2 | sim %3 s<br/>actual speed %4 m/s | pause %5 s").arg(f).arg(QString::fromStdString(s.mode)).arg(s.t,0,'f',1).arg(s.speed,0,'f',2).arg(s.wait,0,'f',1));}}
  {std::lock_guard<std::mutex> lock(mutex_);for(const auto& entry:pedestrian_poses_){
   auto p=people_.find(entry.first);if(p!=people_.end())p->second->SetWorldPose(DisplayPose(Sample(entry.second,display_time),Floor(entry.first)));
  }}
 }
 void Capture(){
  if(!ready_ || saved_)return;
  auto camera=gui::get_active_camera();if(!camera)return;
  const auto wall=std::chrono::steady_clock::now();
  std::map<std::string,State> states;{std::lock_guard<std::mutex> lock(mutex_);states=states_;}
  double sim=0;for(const auto& entry:states)sim=std::max(sim,entry.second.t);
  if(saved_)return;
  if(world_=="three_floor" && states.size()!=3)return;
  if(world_!="three_floor")sim=std::chrono::duration<double>(wall.time_since_epoch()).count();
  if(start_<0)start_=sim;
  if(sim-start_<snapshot_after_)return;
  // GPU-to-CPU readback is needed only for the requested snapshot, not every
  // subsequent frame of an interactive simulation.
  if(!capture_pending_){capture_pending_=true;camera->SetCaptureData(true);return;}
  const auto *bytes=camera->ImageData();if(!bytes)return;int w=camera->ImageWidth(),h=camera->ImageHeight();
  QImage image(bytes,w,h,w*3,QImage::Format_RGB888);
  if(!split_){saved_=SaveImage(image,"scene_view.png");if(saved_){camera->SetCaptureData(false);capture_pending_=false;}return;}
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
  painter.end();saved_=SaveImage(view,"three_floors_top.png");
  if(saved_){camera->SetCaptureData(false);capture_pending_=false;}
 }
};GZ_REGISTER_GUI_PLUGIN(SceneVisualization)
}
