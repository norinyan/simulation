#include "foot_acceptance.h"
#define SIMULATION_ACCEPTANCE
#include "planning_node.cpp"
#include <sstream>

namespace {
int checkMapReuse() {
 auto n=std::make_shared<PlanningNode>();const auto before=simulation::worldParseCount();
 require(before==1,"Expected one initialization parse");
 const auto& map=*n->prepared_geometry_.map;
 std::cout<<"static_solids="<<map.solids.size()<<'\n';
 for(std::size_t i=0;i<map.regions.size();++i){const auto& r=map.regions[i];
  std::cout<<r.id<<" height="<<r.height<<" supports="<<r.supports.size()<<" obstacles="<<r.obstacles.size()<<" pedestrians="<<n->prepared_geometry_.floors[i].pedestrians.size()<<'\n';}
 for(const auto& c:map.connections){std::cout<<c.id<<" reference=";for(const auto& p:c.reference)std::cout<<p.x<<','<<p.y<<','<<p.z<<','<<p.yaw<<';';std::cout<<'\n';}
 auto config=YAML::Clone(n->scene_config_);const auto waypoints=config["scene"]["waypoints"];
 auto route=config["scene"]["task_route"];
 config["scene"]["waypoints"][route[1].as<std::string>()]=config["scene"]["waypoints"][route[0].as<std::string>()];
 for(int i=0;i<2;++i){
   if(i==1){auto endpoint=config["scene"]["waypoints"][route[1].as<std::string>()];endpoint[0]=endpoint[0].as<double>()+.05;}
   const auto text=YAML::Dump(config);auto result=n->computePlan(text,n->budget(60,std::make_shared<std::atomic_bool>(false)));require(result.success,result.reason);}
 require(simulation::worldParseCount()==before,"Task reparsed World");
 std::cout<<"PASS two initialized requests; world_parse_count="<<simulation::worldParseCount()<<" nominal_foot_solves=0\n";return 0;
}
double footQ(double u){u=std::clamp(u,0.,1.);return u*u*u*(10+u*(-15+6*u));}
// Test-only exact-request memoization. The key excludes invocation deadline and
// cancellation pointers; every invocation retains the same wall budget/checks.
std::string footRequestKey(const planning::FootLocalPlanRequest& r) {
 std::ostringstream k;k<<std::hexfloat<<std::setprecision(17);
 auto number=[&](double v){k<<v<<'|';};auto point=[&](const Pose& p){number(p.x);number(p.y);number(p.z);number(p.yaw);};
 auto boxes=[&](const auto& items){k<<items.size()<<':';for(const auto& b:items)for(double v:{b.center_x,b.center_y,b.center_z,b.size_x,b.size_y,b.size_z,b.yaw,b.roll,b.pitch})number(v);};
 const auto& g=r.geometry;k<<g.frame_id<<'|';point(g.start);point(g.goal);
 for(double v:{g.bounds.min_x,g.bounds.max_x,g.bounds.min_y,g.bounds.max_y,g.footprint.length,g.footprint.width,g.safety_margin,g.traversal.wheel_height,g.traversal.body_height,g.traversal.body_ground_clearance,g.traversal.max_step_height,g.traversal.min_tread})number(v);
 k<<g.require_support<<r.spatial<<bool(r.current_body)<<r.supported[0]<<r.supported[1]<<'|';require(!g.map,"Foot key requires resolved geometry");
 boxes(g.obstacles);boxes(g.supports);boxes(r.terrain);k<<r.reference_path.size()<<':';for(const auto& p:r.reference_path)point(p);
 if(r.current_body)point(*r.current_body);for(const auto& p:r.current_feet)point(p);
 const auto& b=r.robot;for(const auto& a:{b.body_size,b.shoulder_offset,b.foot_size,b.folded_pitch})for(double v:a)number(v);
 for(double v:{b.upper_leg_length,b.lower_leg_length,b.leg_width,b.foot_center_offset_x,b.joint_min,b.joint_max,b.hip_yaw_min,b.hip_yaw_max,b.wheel_radius,b.wheel_width,b.wheel_track,b.carrier_drop,b.lift_min,b.lift_max})number(v);
 require(bool(r.limits),"Foot key requires limits");const auto& o=*r.limits;
 for(double v:{o.max_step,o.edge_margin,o.swing_clearance,o.step_duration,o.sample_dt,o.max_foot_v,o.max_foot_a,o.max_body_v,o.max_body_a,o.max_joint_v,o.max_joint_a,o.body_height,o.wheel_lift})number(v);
 return k.str();
}
struct ExactFootAcceptance final:planning::FootLocalPlanner {
 std::unique_ptr<planning::FootLocalPlanner> original;std::map<std::string,planning::FootLocalPlanResult> checked;
 std::filesystem::path directory;std::string parameters;std::size_t hits{0},misses{0};
 std::function<void(const planning::FootLocalPlanResult&,std::size_t)> save;
 planning::FootLocalPlanResult plan(const planning::FootLocalPlanRequest& request) override {
  const auto key=parameters+footRequestKey(request);const auto found=checked.find(key);
  if(found!=checked.end()){++hits;return found->second;}
  auto result=planning::validateFootTrajectory(request,planning::runPlanner(*original,request));require(result.success,result.failure_reason);
  ++misses;std::ofstream(directory/("request_"+std::to_string(misses)+".txt"))<<key;
  std::ofstream audit(directory/"independent.csv",std::ios::app);audit<<misses<<','<<result.trajectory.size()<<','<<std::setprecision(17)<<result.trajectory.back().time_from_start<<','<<result.planning_ms<<','<<result.validation_ms<<",passed\n";
  save(result,1000+misses);checked.emplace(key,result);return result;
 }
};
// Complete offline rollout uses production taskTick/drive, bounded-acceleration
// wheel integration and the executor's shared foot interpolation. It is never
// recorded as Gazebo arrival. Virtual time pauses for computation, not motion.
int checkWheelRollout(const std::string& presets,bool static_fixture) {
 auto n=std::make_shared<PlanningNode>();require(bool(n->timer_)&&bool(n->task_timer_),"Rollout requires interactive:=true or autonomous:=true");n->timer_->cancel();n->task_timer_->cancel();
 auto clock=n->get_clock();require(rcl_enable_ros_time_override(clock->get_clock_handle())==RCL_RET_OK,"Cannot enable offline clock");
 auto stamp=[&](double t){require(rcl_set_ros_time_override(clock->get_clock_handle(),int64_t(t*1e9))==RCL_RET_OK,"Offline clock failed");};stamp(1);
 const auto root=n->output_dir_;std::filesystem::create_directories(root);
 if(static_fixture)for(auto& floor:n->prepared_geometry_.floors)floor.pedestrians.clear();
 std::ofstream(root/"fixture.txt")<<(static_fixture?"static_fixture: all actual World static solids/supports, no dynamic patrol input; original World unchanged\n":"dynamic_fixture: deterministic World patrols at virtual time 1\n");
 auto memo=std::make_unique<ExactFootAcceptance>();auto* cache=memo.get();memo->original=std::move(n->foot_planner_);memo->directory=root;memo->parameters=YAML::Dump(n->scene_config_["robot"])+n->foot_name_+YAML::Dump(YAML::LoadFile(n->scene_file_)["scene"]["world_file"]);
 for(const auto& p:n->foot_parameters_)memo->parameters+=p.first+"="+std::to_string(p.second)+";";
 memo->save=[&](const auto& result,std::size_t id){n->saveFoot(result,id,true);};n->foot_planner_=std::move(memo);
 std::vector<std::tuple<std::string,std::string,Pose,Pose>> pairs;
 if(presets.empty()){auto scene=loadScene(n->scene_config_,n->margin_,n->prepared_geometry_.map);pairs.emplace_back("start","goal",scene.request.start,scene.request.goal);}
 else {auto points=YAML::LoadFile(presets)["scenes"]["three_floor"]["points"];for(auto a:points)for(auto b:points)if(a.first.as<std::string>()!=b.first.as<std::string>())pairs.emplace_back(a.first.as<std::string>(),b.first.as<std::string>(),pose(a.second["pose"]),pose(b.second["pose"]));}
 double cv=0,cw=0,v=0,w=0,t=1,morph_begin=0,foot_begin=0,morph_from_z=0;std::string executor="READY_WHEEL; mode=wheel",morph;
 planning::FootLocalPlanRequest robot_request;planning::FootTrajectoryPoint physical,morph_start;
 std::vector<planning::FootTrajectoryPoint> foot_motion;
 auto command=n->create_subscription<geometry_msgs::msg::TwistStamped>("/robot/cmd_vel",10,[&](geometry_msgs::msg::TwistStamped::SharedPtr msg){cv=msg->twist.linear.x;cw=msg->twist.angular.z;});
 auto mode=n->create_subscription<std_msgs::msg::String>("/robot/mode_command",10,[&](std_msgs::msg::String::SharedPtr msg){
  if(msg->data=="STOP")cv=cw=v=w=0;
  else if(msg->data=="LEG"||msg->data=="WHEEL"){morph=msg->data;morph_begin=t;morph_start=physical;morph_from_z=n->actual_pose_.z;executor="MORPH_TO_"+morph;}
 });
 auto execution=n->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>("/robot/foot_trajectory",10,[&](trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg){
  foot_motion.clear();for(const auto& knot:msg->points){planning::FootTrajectoryPoint p;p.time_from_start=rclcpp::Duration(knot.time_from_start).seconds();
   for(int j=0;j<3;++j){const auto& a=knot.transforms[j];Pose pose{a.translation.x,a.translation.y,a.translation.z,2*std::atan2(a.rotation.z,a.rotation.w)};if(j)p.feet[j-1]=pose;else p.body=pose;
    const auto& v=knot.velocities[j];const auto& z=knot.accelerations[j];p.spatial_velocity[j]={v.linear.x,v.linear.y,v.linear.z,v.angular.z};p.spatial_acceleration[j]={z.linear.x,z.linear.y,z.linear.z,z.angular.z};}
   foot_motion.push_back(p);}
  foot_begin=t;executor="EXECUTING_FOOT; mode=leg";
 });
 auto standing=[&](const Pose& body,int side,double ground,bool folded){const auto& r=robot_request.robot;const double a=folded?r.folded_pitch[0]:0,b=folded?r.folded_pitch[0]+r.folded_pitch[1]:0;
  const double x=r.shoulder_offset[0]+r.foot_center_offset_x-(folded?r.upper_leg_length*std::sin(a)+r.lower_leg_length*std::sin(b):0),y=(side==0?1:-1)*r.shoulder_offset[1];
  return Pose{body.x+std::cos(body.yaw)*x-std::sin(body.yaw)*y,body.y+std::sin(body.yaw)*x+std::cos(body.yaw)*y,folded?body.z+r.shoulder_offset[2]-r.upper_leg_length*std::cos(a)-r.lower_leg_length*std::cos(b):ground+r.foot_size[2]/2,body.yaw};};
 auto feedback=[&](){stamp(t);n->have_odom_=n->have_status_=n->have_feet_=true;n->executor_=executor;
  n->odom_stamp_=n->feet_stamp_=t;n->wall_odom_=n->wall_feet_=n->wall_status_=std::chrono::steady_clock::now();
  n->odom_.twist.twist=geometry_msgs::msg::Twist();n->odom_.twist.twist.linear.x=v;n->odom_.twist.twist.angular.z=w;
  auto message=makeFootCommand(planning::FootLocalPlanResult{true,{},std::vector<planning::FootTrajectoryPoint>{physical}},std_msgs::msg::Header{},true);n->feet_.points=message.points;
  for(const auto& floor:n->prepared_geometry_.floors)for(const auto& person:floor.pedestrians){PlanningNode::PedestrianSample p;p.pose=simulation::patrolPose(person,t);p.stamp=t;p.received=std::chrono::steady_clock::now();n->pedestrians_[person.name]=p;}
 };
 for(int i=0;i<20;++i){rclcpp::spin_some(n);std::this_thread::sleep_for(std::chrono::milliseconds(10));}
 std::ofstream results(root/"pairs.csv");results<<"start,goal,success,reason,segments,wheel_windows,foot_new,foot_reused,simulation_s,wall_s,position_error_m,yaw_error_rad,height_error_m\n";std::size_t passed=0;
 for(const auto& pair:pairs){
  const auto& start=std::get<0>(pair);const auto& goal=std::get<1>(pair);const auto before_miss=cache->misses,before_hit=cache->hits;
  n->discardFinishedJobs();for(auto* stream:{&n->actual_,&n->events_,&n->bands_,&n->pedestrians_log_}){if(stream->is_open())stream->close();stream->clear();}
  n->terminal_=n->task_ready_=false;n->active_segment_=0;n->navigation_segment_=static_cast<std::size_t>(-1);n->plan_id_=0;n->last_pedestrian_log_=-1;n->last_replan_=-100;
  n->output_dir_=root/(start+"__"+goal);auto config=YAML::Clone(n->scene_config_);auto scene=config["scene"];
  for(const auto& p:{std::pair<std::string,Pose>{"start",std::get<2>(pair)},{"goal",std::get<3>(pair)}}){YAML::Node node;for(double value:{p.second.x,p.second.y,p.second.z,p.second.yaw})node.push_back(value);scene["waypoints"][p.first]=node;}
  n->scene_snapshot_=YAML::Dump(config);n->plan_cancel_=std::make_shared<std::atomic_bool>(false);auto batch=n->computePlan(n->scene_snapshot_,n->budget(60,n->plan_cancel_));require(batch.success,batch.reason);
  const auto segments=batch.scene.task.size();n->commitPlan(std::move(batch));
  // Obtain the same configured robot geometry even for wheel-only pairs.
  robot_request=footRequest(config,n->task_scene_,{});robot_request.limits=n->foot_limits_;
  cv=cw=v=w=0;t=1;stamp(t);morph.clear();foot_motion.clear();executor="READY_WHEEL; mode=wheel";physical={};physical.body=std::get<2>(pair);physical.body.z+=n->wheelHeight();n->actual_pose_=physical.body;
  for(int side=0;side<2;++side)physical.feet[side]=standing(physical.body,side,std::get<2>(pair).z,true);
  feedback();n->beginTask();auto begin=std::chrono::steady_clock::now();std::string error;
  try {for(int tick=0;tick<24000&&!n->terminal_;++tick){
    feedback();rclcpp::spin_some(n);n->taskTick();
    if(n->wheel_future_.valid()){n->wheel_future_.wait();if(!n->terminal_&&n->task_state_==n->task_scene_.task[n->active_segment_].state)n->drive(false);}
    if(n->replan_future_.valid())n->replan_future_.wait();if(n->foot_future_.valid())n->foot_future_.wait();rclcpp::spin_some(n);
    for(int j=0;j<10&&!n->terminal_;++j){
     const double next=t+(j+1)*.01;const auto previous=n->actual_pose_;
     if(!morph.empty()){
      const double u=std::clamp((next-morph_begin)/n->get_parameter("task.morph_duration").as_double(),0.,1.);const bool deploy=morph=="LEG";const double ground=deploy?morph_from_z-n->wheelHeight():morph_start.feet[0].z-robot_request.robot.foot_size[2]/2;
      const double phase=deploy?std::clamp(2*u-1,0.,1.):std::clamp(2*u,0.,1.);const double target=deploy?ground+n->foot_limits_.body_height:ground+n->wheelHeight();
      physical.body.z=morph_from_z+(target-morph_from_z)*footQ(phase);
      for(int side=0;side<2;++side){const auto target_foot=standing(physical.body,side,ground,!deploy);const double q=footQ(deploy?std::min(1.,2*u):std::max(0.,2*u-1));const auto a=morph_start.feet[side];physical.feet[side]={a.x+(target_foot.x-a.x)*q,a.y+(target_foot.y-a.y)*q,a.z+(target_foot.z-a.z)*q,a.yaw};}
      if(u>=1){executor=deploy?"LEG_READY; mode=leg":"WHEEL_READY; mode=wheel";morph.clear();foot_motion.clear();}v=w=0;
     }else if(!foot_motion.empty()){
      const double elapsed=std::min(next-foot_begin,foot_motion.back().time_from_start);auto upper=std::upper_bound(foot_motion.begin(),foot_motion.end(),elapsed,[](double time,const auto& p){return time<p.time_from_start;});
      physical=upper==foot_motion.end()?foot_motion.back():planning::interpolateFootTrajectory(*(upper-1),*upper,(elapsed-(upper-1)->time_from_start)/(upper->time_from_start-(upper-1)->time_from_start));physical.time_from_start=elapsed;
      if(elapsed>=foot_motion.back().time_from_start)executor="FOOT_DONE; mode=leg";v=w=0;
     }else if(executor.find("wheel")!=std::string::npos){
      v+=std::clamp(cv-v,-n->wheel_limits_.max_a*.01,n->wheel_limits_.max_a*.01);w+=std::clamp(cw-w,-n->wheel_limits_.max_alpha*.01,n->wheel_limits_.max_alpha*.01);
      physical.body.x+=v*std::cos(previous.yaw+w*.005)*.01;physical.body.y+=v*std::sin(previous.yaw+w*.005)*.01;physical.body.yaw=PlanningNode::angle(previous.yaw+w*.01);
      auto geometry=n->active_floor_.geometry;auto ground=physical.body;ground.z=geometry.start.z;auto old=previous;old.z=geometry.start.z;
      for(const auto& person:n->active_floor_.pedestrians)geometry.obstacles.push_back(simulation::movingBox(person,simulation::patrolPose(person,next)));
      require(planning::isMotionCollisionFree(old,ground,geometry),"OFFLINE_TRACKING_COLLISION");for(int side=0;side<2;++side)physical.feet[side]=standing(physical.body,side,ground.z,true);
     }n->actual_pose_=physical.body;
    }t+=.1;require(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<120,"OFFLINE_ROLLOUT_WALL_LIMIT");
   }require(n->terminal_&&n->task_state_=="DONE","Offline rollout did not finish: "+n->task_state_);
  }catch(const std::exception& e){error=e.what();n->cancelPlanning();n->terminal_=true;}
  const auto expected=std::get<3>(pair);const double position=std::hypot(n->actual_pose_.x-expected.x,n->actual_pose_.y-expected.y),yaw=std::abs(PlanningNode::angle(n->actual_pose_.yaw-expected.yaw)),height=n->actual_pose_.z-expected.z-n->wheelHeight();
  const bool success=error.empty()&&position<=.05&&yaw<=5*std::acos(-1.)/180&&std::abs(height)<=.01&&n->stopped();passed+=success;
  results<<std::setprecision(17)<<start<<','<<goal<<','<<success<<','<<csvString(error)<<','<<segments<<','<<n->plan_id_<<','<<cache->misses-before_miss<<','<<cache->hits-before_hit<<','<<t-1<<','<<std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<<','<<position<<','<<yaw<<','<<height<<std::endl;
  if(std::filesystem::exists(n->output_dir_/"task_result.csv"))std::filesystem::rename(n->output_dir_/"task_result.csv",n->output_dir_/"offline_rollout_result.csv");
  std::ofstream(n->output_dir_/"OFFLINE_FEEDBACK.txt")<<(static_fixture?"STATIC_FIXTURE; no dynamic patrol input. ":"DYNAMIC_FIXTURE; World patrol input. ")<<"Production dispatcher, bounded wheel integration and shared foot interpolation; initial sim t=1; virtual time paused for computation, unchanged wall deadlines. No Gazebo arrival. Morphology uses original two-phase 4s contract; full morphology geometry remains covered by actual Gazebo runs.\n";
  std::cout<<(success?"PASS ":"FAIL ")<<start<<" -> "<<goal<<" sim_s="<<t-1<<" wall_s="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<<" windows="<<n->plan_id_<<" unique_foot="<<cache->misses<<" reused="<<cache->hits<<" reason="<<error<<std::endl;
  if(!success)break;
 }
 require(simulation::worldParseCount()==1,"Offline tasks reparsed World");std::cout<<passed<<'/'<<pairs.size()<<" offline navigation; unique_foot="<<cache->misses<<" reused="<<cache->hits<<" output="<<root<<std::endl;return passed==pairs.size()?0:1;
}
int checkAlgorithmSelection() {
 auto n=std::make_shared<PlanningNode>();
 const std::vector<rclcpp::Parameter> valid{{"planners.global","astar"},{"planners.wheel","teb"},{"planners.foot","footstep"}};
 auto invalid=valid;invalid[2]=rclcpp::Parameter("planners.foot","missing");
 require(!n->set_parameters_atomically(invalid).successful,"Unknown algorithm accepted");
 for(const auto& p:valid)require(n->get_parameter(p.get_name()).as_string()==p.as_string(),"Rejected atomic request partially changed configuration");
 require(n->set_parameters_atomically(valid).successful,"Valid idle selection rejected");
 n->task_ready_=true;
 require(!n->set_parameters_atomically(valid).successful,"Running task allowed an algorithm change");n->task_ready_=false;
 std::cout<<"PASS idle three-algorithm selection; atomic unknown-name rejection; busy freeze; unchanged configuration\n";return 0;
}
int checkLegacy() {
 auto n=std::make_shared<PlanningNode>();n->scene_snapshot_=n->sceneText();n->plan_cancel_=std::make_shared<std::atomic_bool>(false);
 auto batch=n->computePlan(n->scene_snapshot_,n->budget(n->plan_timeout_,n->plan_cancel_));require(batch.success,batch.reason);require(!batch.scene.unified,"Legacy check requires two_steps");
 for(const auto& local:batch.local)require(local.success,local.failure_reason);
 auto request=segmentFootRequest(n->scene_config_,batch.scene,1);request.limits=n->foot_limits_;request.geometry.control=n->budget(60,n->plan_cancel_);
 auto result=planning::validateFootTrajectory(request,planning::runPlanner(*n->foot_planner_,request));require(result.success,result.failure_reason);n->saveFoot(result,1,true);
 n->commitPlan(std::move(batch));require(n->plan_ready_,"Legacy plan not executable");
 std::cout<<"PASS legacy A-E-foot-X-B; no pre-solve; planar adapter retains original executor entry tolerance; samples="<<result.trajectory.size()<<" duration="<<result.trajectory.back().time_from_start<<" output="<<n->output_dir_<<std::endl;return 0;
}
int checkLocalSnapshot(const std::string& filename){
 auto n=std::make_shared<PlanningNode>();n->scene_snapshot_=n->sceneText();n->plan_cancel_=std::make_shared<std::atomic_bool>(false);
 auto batch=n->computePlan(n->scene_snapshot_,n->budget(n->plan_timeout_,n->plan_cancel_));require(batch.success,batch.reason);
 const auto snapshot=YAML::LoadFile(filename);
 if(snapshot["body"]){
   const auto index=snapshot["segment"].as<std::size_t>();auto request=segmentFootRequest(YAML::Load(n->scene_snapshot_),batch.scene,index);
   request.current_body=pose(snapshot["body"]);request.current_feet={pose(snapshot["feet"][0]),pose(snapshot["feet"][1])};
   request.geometry.start=*request.current_body;request.geometry.start.z-=n->foot_limits_.body_height;request.limits=n->foot_limits_;
   request.geometry.control=n->budget(n->foot_plan_timeout_,n->plan_cancel_);
   planning::FootLocalPlanResult generated;
   if(snapshot["trajectory_csv"]) {
     std::ifstream input(snapshot["trajectory_csv"].as<std::string>());std::string line;std::getline(input,line);
     std::vector<std::string> headers;std::stringstream header(line);std::string field;while(std::getline(header,field,','))headers.push_back(field);
     while(std::getline(input,line)){std::stringstream row(line);std::map<std::string,double> v;std::size_t k=0;while(std::getline(row,field,','))v[headers.at(k++)]=std::stod(field);
       planning::FootTrajectoryPoint sample;sample.time_from_start=v["time_from_start_s"];
       sample.body={v["body_x_m"],v["body_y_m"],v["body_z_m"],v["body_yaw_rad"]};
       sample.feet={Pose{v["left_x_m"],v["left_y_m"],v["left_z_m"],v["left_yaw_rad"]},Pose{v["right_x_m"],v["right_y_m"],v["right_z_m"],v["right_yaw_rad"]}};
       for(int j=0;j<3;++j)for(auto derivative:{std::string("v"),std::string("a")}){
         const auto prefix="spatial_"+std::to_string(j)+"_";Pose d{v[prefix+"x_"+derivative],v[prefix+"y_"+derivative],v[prefix+"z_"+derivative],v[prefix+"yaw_"+derivative]};
         (derivative=="v"?sample.spatial_velocity:sample.spatial_acceleration)[j]=d;
       }generated.trajectory.push_back(sample);
     }generated.success=true;
   } else generated=planning::runPlanner(*n->foot_planner_,request);
   auto result=planning::validateFootTrajectory(request,std::move(generated));n->saveFoot(result,index,true);
   std::cout<<"FOOT actual snapshot success="<<result.success<<" reason="<<result.failure_reason<<" steps="<<result.footsteps.size()<<" samples="<<result.trajectory.size()<<" duration="<<(result.trajectory.empty()?0:result.trajectory.back().time_from_start)<<" output="<<n->output_dir_<<std::endl;
   return result.success?0:1;
 }
 planning::LocalPlanRequest request;
 request.geometry=batch.scene.task.at(snapshot["segment"].as<std::size_t>()).geometry;
 request.geometry.start=pose(snapshot["start"]);request.initial_linear_velocity=scalar(snapshot["v"]);request.initial_angular_velocity=scalar(snapshot["w"]);
 for(auto p:snapshot["reference_path"])request.reference_path.push_back(pose(p));request.limits=n->wheel_limits_;
 const auto final=request.geometry.goal;Pose previous=final;
 for(auto it=request.reference_path.rbegin();it!=request.reference_path.rend();++it)if(std::hypot(it->x-final.x,it->y-final.y)>1e-5){previous=*it;break;}
 for(const auto& variant:{std::pair<std::string,double>{"final",final.yaw},{"tangent",std::atan2(final.y-previous.y,final.x-previous.x)},{"bearing",std::atan2(final.y-request.geometry.start.y,final.x-request.geometry.start.x)},{"aligned_tangent",std::atan2(final.y-previous.y,final.x-previous.x)}}){
   request.geometry.start=pose(snapshot["start"]);if(variant.first=="aligned_tangent")request.geometry.start.yaw=variant.second;
   request.geometry.goal.yaw=variant.second;request.reference_path.back().yaw=variant.second;
   request.geometry.control=n->budget(n->local_timeout_,n->plan_cancel_);
   auto result=planning::runPlanner(*n->wheel_planner_,request);
   std::cout<<std::setprecision(17)<<variant.first<<" goal_yaw="<<variant.second<<" success="<<result.success<<" reason="<<result.failure_reason<<" samples="<<result.trajectory.size()<<" duration="<<result.duration<<std::endl;
 }
 return 0;
}

// Exercise the production dispatcher with explicit synthetic endpoint feedback.
// No ROS executor or Gazebo robot is connected; this is a scheduling check only.
int checkStage2Schedule(){
  auto n=std::make_shared<PlanningNode>();
  require(n->interactive_ && !n->get_parameter("use_sim_time").as_bool(),"Schedule check requires interactive, wall clock, isolated ROS domain");
  n->scene_snapshot_=n->sceneText();n->plan_cancel_=std::make_shared<std::atomic_bool>(false);
  auto batch=n->computePlan(n->scene_snapshot_,n->budget(n->plan_timeout_,n->plan_cancel_));
  require(batch.success,batch.reason);require(batch.scene.task.size()==5,"Expected five segments");
  auto endpoint_request=segmentFootRequest(YAML::Load(n->scene_snapshot_),batch.scene,1);endpoint_request.limits=n->foot_limits_;endpoint_request.geometry.control=n->budget(n->foot_plan_timeout_,n->plan_cancel_);
  auto checked=planning::validateFootTrajectory(endpoint_request,planning::runPlanner(*n->foot_planner_,endpoint_request));require(checked.success,checked.failure_reason);
  auto invalid=checked;invalid.trajectory.back().feet[0].z+=.001;bool rejected=false;
  try{planning::validateSpatialFootTrajectory(endpoint_request,invalid);}catch(const std::exception& e){rejected=std::string(e.what())=="SPATIAL_FINAL_STANCE_MISMATCH";}
  require(rejected,"Public foot validator accepted a hovering final foot");
  invalid=checked;invalid.trajectory.back().spatial_velocity[0].x=.01;rejected=false;
  try{planning::validateSpatialFootTrajectory(endpoint_request,invalid);}catch(const std::exception& e){rejected=std::string(e.what())=="SPATIAL_ENDPOINT_DERIVATIVE";}
  require(rejected,"Public foot validator accepted a moving endpoint");
  n->commitPlan(std::move(batch));require(n->plan_ready_,"Schedule plan unavailable");n->beginTask();
  auto refresh=[&](){
    n->have_odom_=n->have_feet_=n->have_status_=true;
    n->odom_stamp_=n->feet_stamp_=n->now().seconds();
    n->wall_odom_=n->wall_feet_=n->wall_status_=std::chrono::steady_clock::now();
    n->odom_.twist.twist=geometry_msgs::msg::Twist();
  };
  n->active_segment_=0;n->segment_begin_=n->now().seconds();n->enterTask(n->task_scene_.task[0].state);
  std::ofstream audit(n->output_dir_/"schedule_check.csv");audit<<"segment,mode,connection,samples,motion_s,budget_s,synthetic_feedback\n";
  for(std::size_t i=0;i<n->task_scene_.task.size();++i){
    require(n->active_segment_==i,"Dispatcher skipped/reused a segment");const auto segment=n->task_scene_.task[i];
    const bool foot=segment.mode==TaskSegment::Mode::Foot;
    require(foot==(i%2==1),"Wrong schedule mode order");refresh();
    if(!foot){
      while(n->active_segment_==i && !n->terminal_){
        n->actual_pose_=n->task_scene_.task[i].geometry.goal;n->actual_pose_.z+=n->wheelHeight();n->executor_="WHEEL_READY; mode=wheel";n->drive(false);
      }
      audit<<i<<",wheel,"<<segment.label<<",0,0,"<<n->segment_budgets_[i]<<",true\n";
    }else{
      require(n->task_state_=="DEPLOY","Foot segment did not enter deploy");
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint first;
      const auto& robot=endpoint_request.robot;const auto& start=segment.geometry.start;
      for(int j=0;j<3;++j){geometry_msgs::msg::Transform p;double sign=j==1?1:-1;
        double x=j?robot.shoulder_offset[0]+robot.foot_center_offset_x:0,y=j?sign*robot.shoulder_offset[1]:0;
        p.translation.x=start.x+std::cos(start.yaw)*x-std::sin(start.yaw)*y;
        p.translation.y=start.y+std::sin(start.yaw)*x+std::cos(start.yaw)*y;
        p.translation.z=start.z+(j?robot.foot_size[2]/2:n->foot_limits_.body_height);
        p.rotation.z=std::sin(start.yaw/2);p.rotation.w=std::cos(start.yaw/2);first.transforms.push_back(p);
      }
      n->feet_.points={first};n->executor_="LEG_READY; mode=leg";
      n->actual_pose_=segment.geometry.start;n->actual_pose_.z+=n->foot_limits_.body_height;
      n->taskTick();
      while(n->task_state_=="DEPLOY"){
        std::this_thread::sleep_for(std::chrono::milliseconds(20));refresh();n->taskTick();
      }
      require(n->task_state_==segment.state,"Wrong foot execution state");
      const auto& command=n->foot_commands_[i];
      const double duration=rclcpp::Duration(command.points.back().time_from_start).seconds();
      require(command.points.size()<=20000 && duration>0 && n->segment_budgets_[i]>duration,"Wrong independent foot budget");
      n->feet_.points={command.points.back()};n->executor_="FOOT_DONE; mode=leg";
      n->actual_pose_=segment.geometry.goal;n->actual_pose_.z+=n->foot_limits_.body_height;refresh();n->taskTick();
      require(n->task_state_=="FOLD","Foot completion did not enter fold");
      n->executor_="WHEEL_READY; mode=wheel";n->actual_pose_.z=segment.geometry.goal.z+n->wheelHeight();refresh();n->taskTick();
      audit<<i<<",foot,"<<segment.label<<','<<command.points.size()<<','<<duration<<','<<n->segment_budgets_[i]<<",true\n";
    }
  }
  require(n->terminal_ && n->task_state_=="DONE","Five-segment dispatcher did not finish");
  std::filesystem::rename(n->output_dir_/"task_result.csv",n->output_dir_/"synthetic_schedule_result.csv");
  std::ofstream(n->output_dir_/"SYNTHETIC_FEEDBACK.txt")<<"Production dispatcher; synthetic endpoint acknowledgements. No robot motion, no cross-two-floor execution acceptance.\n";
  std::cout<<"PASS: five segments, two independent foot commands, measured-state handoff, per-segment budgets; SYNTHETIC feedback only; output="<<n->output_dir_<<std::endl;
  return 0;
}

// Offline first-stage acceptance uses the same map, algorithm and public validator
// as the node; it neither initializes ROS nor spawns/moves a robot.
int checkGlobalPairs(const std::string& world,const std::string& presets,
                     const std::string& scene_file,const std::string& planning_file,
                     const std::string& output) {
 auto config=YAML::LoadFile(scene_file),parameters=YAML::LoadFile(planning_file)["planning_node"]["ros__parameters"];
 auto robot=config["robot"];double margin=scalar(parameters["safety_margin"]);
 auto begin=std::chrono::steady_clock::now();
 auto map=simulation::loadSceneMap(world,"three_floor",{scalar(robot["wheel_mode_length"]),scalar(robot["wheel_mode_width"])},.41,margin);
 double map_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
 PlanRequest r;r.frame_id="world";r.map=map;r.bounds=map->regions.front().bounds;
 r.footprint={scalar(robot["wheel_mode_length"]),scalar(robot["wheel_mode_width"])};r.safety_margin=margin;
 r.traversal.body_height=scalar(robot["body_size"][2]);
 if(robot["traversal"]) {
  r.traversal.max_step_height=scalar(robot["traversal"]["max_step_height"]);
  r.traversal.min_tread=scalar(robot["traversal"]["min_tread"]);
  r.traversal.body_ground_clearance=scalar(robot["traversal"]["body_ground_clearance"]);
 }
 auto planner=planning::makeGlobalPlanner(parameters["planners"]["global"].as<std::string>(),
  planning::PlannerParameters{{"resolution",scalar(parameters["resolution"])},{"max_grid_cells",parameters["max_grid_cells"].as<double>()}});
 require(!std::filesystem::exists(output),"Evidence directory exists; choose a new output path");
 std::filesystem::create_directories(output);
 std::filesystem::copy_file(world,std::filesystem::path(output)/"world.sdf");
 std::filesystem::copy_file(presets,std::filesystem::path(output)/"presets.yaml");
 std::filesystem::copy_file(scene_file,std::filesystem::path(output)/"scene.yaml");
 std::filesystem::copy_file(planning_file,std::filesystem::path(output)/"planning.yaml");
 std::ofstream audit(std::filesystem::path(output)/"map.txt");
 audit<<std::setprecision(17)<<"map_preparation_ms="<<map_ms<<"\nstatic_solids="<<map->solids.size()<<'\n';
 for(const auto& region:map->regions)audit<<region.id<<" height="<<region.height<<" supports="<<region.supports.size()<<" obstacles="<<region.obstacles.size()<<'\n';
 for(const auto& c:map->connections) {
  const auto& a=c.reference.front();const auto& b=c.reference.back();
  auto reason=planning::validateConnection(r,c);
  audit<<c.id<<" "<<c.from<<" -> "<<c.to<<" entry="<<a.x<<","<<a.y<<","<<a.z<<","<<a.yaw<<" exit="<<b.x<<","<<b.y<<","<<b.z<<","<<b.yaw<<" validation="<<(reason.empty()?"PASS":reason)<<'\n';
  std::cout<<c.id<<": "<<(reason.empty()?"PASS":reason)<<std::endl;
 }
 std::ofstream gallery(std::filesystem::path(output)/"index.html");
 gallery<<"<!doctype html><meta charset=utf-8><title>Stage 1 global paths</title><style>body{font:15px sans-serif}li{margin:.4em}iframe{position:fixed;left:40%;top:0;width:59%;height:98vh;border:0}ul{width:37%}</style><h1>Stage 1 / global only</h1><p>Blue: wheel. Orange: foot. No local or execution acceptance.</p><iframe name=path></iframe><ul>";
 std::ofstream results(std::filesystem::path(output)/"pairs.csv");
 results<<"start,goal,success,reason,length_3d_m,mode_switches,segments,algorithm_ms,validation_ms\n";
 auto points=YAML::LoadFile(presets)["scenes"]["three_floor"]["points"];
 int passed=0,total=0;
 for(auto a:points)for(auto b:points) {
  auto start=a.first.as<std::string>(),goal=b.first.as<std::string>();if(start==goal)continue;
  r.start=pose(a.second["pose"]);r.goal=pose(b.second["pose"]);
  r.control.deadline=std::chrono::steady_clock::now()+std::chrono::seconds(60);
  auto result=planning::runPlanner(*planner,r);++total;passed+=result.success;
  results<<std::setprecision(17)<<start<<','<<goal<<','<<result.success<<','<<result.failure_reason<<','<<result.spatial_length<<','<<result.mode_switches<<','<<result.segments.size()<<','<<result.algorithm_ms<<','<<result.validation_ms<<std::endl;
  std::ofstream path(std::filesystem::path(output)/(start+"__"+goal+".csv"));
  path<<"segment,mode,region,connection,index,x,y,z,yaw\n";
  for(std::size_t s=0;s<result.segments.size();++s) {
   const auto& section=result.segments[s];
   for(std::size_t j=section.begin;j<=section.end;++j){const auto& p=result.path[j];
    path<<std::setprecision(17)<<s<<','<<(section.mode==planning::TravelMode::Wheel?"wheel":"foot")<<','<<section.region_id<<','<<section.connection_id<<','<<j<<','<<p.x<<','<<p.y<<','<<p.z<<','<<p.yaw<<'\n';}
  }
  gallery<<"<li><a target=path href=\""<<start<<"__"<<goal<<".svg\">"<<start<<" → "<<goal<<"</a> "<<(result.success?"PASS":result.failure_reason)<<"</li>";
  // Standalone SVG, oblique XYZ projection exposes both floor and stair geometry.
  std::ofstream svg(std::filesystem::path(output)/(start+"__"+goal+".svg"));
  svg<<"<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 1000 900\"><rect width=\"1000\" height=\"900\" fill=\"white\"/><text x=\"20\" y=\"30\">"<<start<<" to "<<goal<<" / global only</text>";
  auto xy=[](double x,double y,double z){return std::pair<double,double>{500+40*x+10*y,650-25*y-55*z};};
  for(const auto& region:map->regions)for(const auto& box:region.supports) {
   svg<<"<polygon fill=\"#eee\" fill-opacity=\"0.25\" stroke=\"#aaa\" stroke-width=\"0.6\" points=\"";
   for(auto corner:std::array<std::pair<int,int>,4>{{{-1,-1},{1,-1},{1,1},{-1,1}}}){
    auto p=xy(box.center_x+corner.first*box.size_x/2,box.center_y+corner.second*box.size_y/2,region.height);svg<<p.first<<','<<p.second<<' ';}svg<<"\"/>";
  }
  for(const auto& section:result.segments) {
   svg<<"<polyline fill=\"none\" stroke=\""<<(section.mode==planning::TravelMode::Wheel?"#1166cc":"#e65c00")<<"\" stroke-width=\"2\" points=\"";
   for(std::size_t j=section.begin;j<=section.end;++j){auto p=xy(result.path[j].x,result.path[j].y,result.path[j].z);svg<<p.first<<','<<p.second<<' ';}svg<<"\"/>";
  }svg<<"</svg>";
  std::cout<<total<<" "<<start<<" -> "<<goal<<" "<<(result.success?"PASS":result.failure_reason)<<" "<<result.algorithm_ms<<" ms"<<std::endl;
 }
 gallery<<"</ul>";
 std::cout<<passed<<"/"<<total<<" global checks passed; no local trajectories or execution tested"<<std::endl;
 return passed==72 && total==72?0:1;
}

}
int main(int argc, char** argv)
{
  if(argc==7 && std::string(argv[1])=="--check-global-pairs") {
    try {return checkGlobalPairs(argv[2],argv[3],argv[4],argv[5],argv[6]);}
    catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
  }
  if(argc>1 && std::string(argv[1])=="--check-algorithm-selection") {
    --argc;++argv;rclcpp::init(argc,argv);
    try{auto result=checkAlgorithmSelection();rclcpp::shutdown();return result;}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
  }
  if(argc>2 && (std::string(argv[1])=="--check-navigation-pairs" || std::string(argv[1])=="--check-static-navigation-pairs")) {
    const bool static_fixture=std::string(argv[1])=="--check-static-navigation-pairs";
    const std::string presets=argv[2];argc-=2;argv+=2;rclcpp::init(argc,argv);
    try{auto result=checkWheelRollout(presets,static_fixture);rclcpp::shutdown();return result;}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
  }
  if(argc>1 && std::string(argv[1])=="--check-wheel-rollout") {
    --argc;++argv;rclcpp::init(argc,argv);
    try{auto result=checkWheelRollout("",false);rclcpp::shutdown();return result;}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
  }
  if(argc>1 && std::string(argv[1])=="--check-legacy") {
    --argc;++argv;rclcpp::init(argc,argv);
    try{auto result=checkLegacy();rclcpp::shutdown();return result;}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
  }
  if(argc>1 && std::string(argv[1])=="--check-map-reuse") {
    --argc;++argv;rclcpp::init(argc,argv);
    try{auto result=checkMapReuse();rclcpp::shutdown();return result;}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
  }
  const bool local_snapshot=argc>2 && (std::string(argv[1])=="--check-wheel-snapshot" || std::string(argv[1])=="--check-local-snapshot");
  const std::string snapshot=local_snapshot?argv[2]:"";
  if(local_snapshot){argc-=2;argv+=2;}
  const bool schedule=argc>1 && std::string(argv[1])=="--check-stage2-schedule";
  if(schedule){--argc;++argv;}
  rclcpp::init(argc, argv);
  try {
    if(local_snapshot){const int result=checkLocalSnapshot(snapshot);rclcpp::shutdown();return result;}
    if(schedule){const int result=checkStage2Schedule();rclcpp::shutdown();return result;}
    rclcpp::spin(std::make_shared<PlanningNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("planning_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
