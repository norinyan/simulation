#include "scan_map.h"

#include <Eigen/Geometry>
#include <tinyxml2.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <array>
#include <unordered_map>

namespace local_planning
{
namespace
{
using Element = tinyxml2::XMLElement;

std::vector<double> numbers(const char * text, std::size_t count)
{
  if (!text) {throw std::runtime_error("SCAN_WORLD_MISSING_NUMBERS");}
  std::istringstream input(text);
  std::vector<double> result(count);
  for (double & value : result) {
    if (!(input >> value) || !std::isfinite(value)) {
      throw std::runtime_error("SCAN_WORLD_INVALID_NUMBERS");
    }
  }
  std::string extra;
  if (input >> extra) {throw std::runtime_error("SCAN_WORLD_EXTRA_NUMBERS");}
  return result;
}

Eigen::Isometry3d pose(Element * parent)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const auto element = parent->FirstChildElement("pose");
  if (!element) {return transform;}
  // The current World uses parent-relative poses only. Refuse unsupported SDF
  // frames instead of silently moving an obstacle to a different location.
  if (element->FirstAttribute()) {throw std::runtime_error("SCAN_WORLD_UNSUPPORTED_POSE_FRAME");}
  const auto values = numbers(element->GetText(), 6);
  transform.translation() = Eigen::Vector3d(values[0], values[1], values[2]);
  transform.linear() = (Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
  return transform;
}

bool separated(const Eigen::Vector3d & axis, const Eigen::Vector3d & delta,
  const Eigen::Matrix3d & a_rotation, const Eigen::Vector3d & a_half,
  const Eigen::Matrix3d & b_rotation, const Eigen::Vector3d & b_half)
{
  if (axis.squaredNorm() < 1e-18) {return false;}
  const double a_radius = (a_rotation.transpose()*axis).cwiseAbs().dot(a_half);
  const double b_radius = (b_rotation.transpose()*axis).cwiseAbs().dot(b_half);
  return std::abs(axis.dot(delta)) > a_radius + b_radius + 1e-9;
}

Eigen::Isometry3d urdfOrigin(Element * parent)
{
  Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
  const auto origin=parent->FirstChildElement("origin");
  if (!origin) {return result;}
  if (const char * xyz=origin->Attribute("xyz")) {
    const auto v=numbers(xyz,3);result.translation()=Eigen::Vector3d(v[0],v[1],v[2]);
  }
  if (const char * rpy=origin->Attribute("rpy")) {
    const auto v=numbers(rpy,3);
    result.linear()=(Eigen::AngleAxisd(v[2],Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(v[1],Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(v[0],Eigen::Vector3d::UnitX())).toRotationMatrix();
  }
  return result;
}

double segmentDistanceSquared(const Eigen::Vector3d & a,const Eigen::Vector3d & b,
  const Eigen::Vector3d & c,const Eigen::Vector3d & d)
{
  const Eigen::Vector3d u=b-a,v=d-c,w=a-c;
  const double aa=u.dot(u),bb=u.dot(v),cc=v.dot(v),dd=u.dot(w),ee=v.dot(w);
  const double denominator=aa*cc-bb*bb;
  double s=denominator>1e-18?std::clamp((bb*ee-cc*dd)/denominator,0.0,1.0):0.0;
  double t=(bb*s+ee)/cc;
  if (t<0) {t=0;s=std::clamp(-dd/aa,0.0,1.0);}
  else if (t>1) {t=1;s=std::clamp((bb-dd)/aa,0.0,1.0);}
  return (w+s*u-t*v).squaredNorm();
}

// Exact Euclidean separation of two boxes. Disjoint convex polyhedra attain
// their minimum at a vertex/face or edge/edge pair; SAT first handles overlap.
double boxDistance(const Eigen::Vector3d & ac,const Eigen::Matrix3d & ar,const Eigen::Vector3d & ah,
  const Eigen::Vector3d & bc,const Eigen::Matrix3d & br,const Eigen::Vector3d & bh)
{
  const Eigen::Vector3d delta=bc-ac;
  bool apart=false;
  for (int i=0;i<3&&!apart;++i) {
    apart=separated(ar.col(i),delta,ar,ah,br,bh)||separated(br.col(i),delta,ar,ah,br,bh);
  }
  for (int i=0;i<3&&!apart;++i) {
    for (int j=0;j<3&&!apart;++j) {apart=separated(ar.col(i).cross(br.col(j)),delta,ar,ah,br,bh);}
  }
  if (!apart) {return 0;}
  std::array<Eigen::Vector3d,8> av,bv;
  double best=std::numeric_limits<double>::infinity();
  for (int i=0;i<8;++i) {
    const Eigen::Vector3d signs((i&1)?1:-1,(i&2)?1:-1,(i&4)?1:-1);
    av[i]=ac+ar*(signs.cwiseProduct(ah));bv[i]=bc+br*(signs.cwiseProduct(bh));
    const Eigen::Vector3d ad=(br.transpose()*(av[i]-bc)).cwiseAbs()-bh;
    const Eigen::Vector3d bd=(ar.transpose()*(bv[i]-ac)).cwiseAbs()-ah;
    best=std::min({best,ad.cwiseMax(0).squaredNorm(),bd.cwiseMax(0).squaredNorm()});
  }
  for (int i=0;i<8;++i) {
    for (int axis=0;axis<3;++axis) {
      const int bit=1<<axis;if (i&bit) {continue;}
      for (int j=0;j<8;++j) {
        for (int other=0;other<3;++other) {
          const int obit=1<<other;if (j&obit) {continue;}
          best=std::min(best,segmentDistanceSquared(av[i],av[i|bit],bv[j],bv[j|obit]));
        }
      }
    }
  }
  return std::sqrt(best);
}
}  // namespace

void ScanMap::loadRobot(const std::string & filename,double wheel_lift)
{
  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str())!=tinyxml2::XML_SUCCESS) {throw std::runtime_error("SCAN_URDF_UNREADABLE");}
  auto robot=document.FirstChildElement("robot");
  if (!robot||!std::isfinite(wheel_lift)) {throw std::runtime_error("SCAN_INVALID_ROBOT");}
  std::vector<Joint> joints;std::vector<Shape> shapes;
  for (auto element=robot->FirstChildElement("joint");element;element=element->NextSiblingElement("joint")) {
    Joint joint;
    const auto parent=element->FirstChildElement("parent"),child=element->FirstChildElement("child");
    const char * name=element->Attribute("name"),*type=element->Attribute("type");
    if (!name||!type||!parent||!child||!parent->Attribute("link")||!child->Attribute("link")) {
      throw std::runtime_error("SCAN_INVALID_URDF_JOINT");
    }
    joint.name=name;joint.type=type;joint.parent=parent->Attribute("link");joint.child=child->Attribute("link");
    joint.origin=urdfOrigin(element);joint.axis=Eigen::Vector3d::UnitX();
    if (auto axis=element->FirstChildElement("axis")) {
      const auto v=numbers(axis->Attribute("xyz"),3);joint.axis=Eigen::Vector3d(v[0],v[1],v[2]);
    }
    if (joint.axis.norm()<1e-12) {throw std::runtime_error("SCAN_INVALID_JOINT_AXIS");}
    joint.axis.normalize();
    if (auto limit=element->FirstChildElement("limit")) {
      if (joint.type=="revolute"||joint.type=="prismatic") {
        if (limit->QueryDoubleAttribute("lower",&joint.lower)!=tinyxml2::XML_SUCCESS||
          limit->QueryDoubleAttribute("upper",&joint.upper)!=tinyxml2::XML_SUCCESS) {
          throw std::runtime_error("SCAN_INVALID_JOINT_LIMIT");
        }
      }
      limit->QueryDoubleAttribute("velocity",&joint.speed);
    }
    if (joint.type!="revolute"&&joint.type!="prismatic"&&joint.type!="continuous"&&joint.type!="fixed") {
      throw std::runtime_error("SCAN_UNSUPPORTED_JOINT");
    }
    joints.push_back(joint);
  }
  RobotGeometry geometry;
  for (auto link=robot->FirstChildElement("link");link;link=link->NextSiblingElement("link")) {
    const char * name=link->Attribute("name");if (!name) {throw std::runtime_error("SCAN_INVALID_LINK");}
    for (auto collision=link->FirstChildElement("collision");collision;collision=collision->NextSiblingElement("collision")) {
      Shape shape;shape.link=name;shape.origin=urdfOrigin(collision);
      auto g=collision->FirstChildElement("geometry");
      auto box=g?g->FirstChildElement("box"):nullptr;
      auto cylinder=g?g->FirstChildElement("cylinder"):nullptr;
      if (box) {
        const auto size=numbers(box->Attribute("size"),3);shape.half=Eigen::Vector3d(size[0],size[1],size[2])*0.5;
        if (shape.link=="left_foot") {geometry.foot_length=size[0];}
      } else if (cylinder&&(shape.link=="left_wheel"||shape.link=="right_wheel")) {
        double radius=0,length=0;cylinder->QueryDoubleAttribute("radius",&radius);cylinder->QueryDoubleAttribute("length",&length);
        shape.half=Eigen::Vector3d(radius,radius,length*0.5);geometry.wheel_radius=radius;
      } else {throw std::runtime_error("SCAN_UNSUPPORTED_ROBOT_COLLISION");}
      if (!shape.half.allFinite()||(shape.half.array()<=0).any()) {throw std::runtime_error("SCAN_INVALID_ROBOT_COLLISION");}
      shapes.push_back(shape);
    }
  }
  for (const auto & joint:joints) {
    if (joint.name=="left_hip_yaw_joint") {geometry.hip_radius=joint.origin.translation().head<2>().norm();}
    if (joint.name=="left_wheel_joint") {geometry.wheel_half_track=std::abs(joint.origin.translation().y());geometry.wheel_speed_limit=joint.speed;}
    if (joint.name=="wheel_lift_joint"&&(wheel_lift<joint.lower||wheel_lift>joint.upper)) {throw std::runtime_error("SCAN_INVALID_WHEEL_LIFT");}
  }
  if (geometry.foot_length<=0||geometry.hip_radius<=0||geometry.wheel_radius<=0||geometry.wheel_speed_limit<=0) {
    throw std::runtime_error("SCAN_INCOMPLETE_ROBOT_MODEL");
  }
  joints_=std::move(joints);shapes_=std::move(shapes);robot_geometry_=geometry;wheel_lift_=wheel_lift;
}

const ScanMap::Joint & ScanMap::jointByName(const std::string & name) const
{
  const auto it=std::find_if(joints_.begin(),joints_.end(),[&](const Joint & j){return j.name==name;});
  if (it==joints_.end()) {throw std::runtime_error("SCAN_MISSING_ROBOT_JOINT: "+name);}
  return *it;
}

bool ScanMap::legAngles(const Eigen::Vector3d & body,double yaw,const Eigen::Vector3d & foot,
  int side,std::array<double,4> & q) const
{
  const std::string prefix=side==0?"left_":"right_";
  const auto & hip=jointByName(prefix+"hip_yaw_joint");
  const auto & shoulder=jointByName(prefix+"shoulder_joint");
  const auto & knee=jointByName(prefix+"knee_joint");
  const auto & ankle=jointByName(prefix+"ankle_joint");
  const double upper=knee.origin.translation().norm(),lower=ankle.origin.translation().norm();
  const Eigen::Vector3d p=Eigen::AngleAxisd(-yaw,Eigen::Vector3d::UnitZ())*(foot-body)-hip.origin.translation();
  double hip_yaw=std::hypot(p.x(),p.y())<1e-8?0:std::atan2(p.y(),p.x());
  if (hip_yaw>M_PI_2) {hip_yaw-=M_PI;}if (hip_yaw<-M_PI_2) {hip_yaw+=M_PI;}
  const double x=p.x()*std::cos(hip_yaw)+p.y()*std::sin(hip_yaw);
  const double c=(x*x+p.z()*p.z()-upper*upper-lower*lower)/(2*upper*lower);
  if (c<-1||c>1) {return false;}
  const double k=-std::acos(c);
  const double s=std::atan2(-x,-p.z())-std::atan2(lower*std::sin(k),upper+lower*std::cos(k));
  const double shoulder_origin=std::atan2(shoulder.origin.linear()(0,2),shoulder.origin.linear()(2,2));
  const double knee_origin=std::atan2(knee.origin.linear()(0,2),knee.origin.linear()(2,2));
  q={hip_yaw,s-shoulder_origin,k-knee_origin,-(s-shoulder_origin)-(k-knee_origin)};
  const std::array<const Joint *,4> chain{&hip,&shoulder,&knee,&ankle};
  for (int i=0;i<4;++i) {
    if (!std::isfinite(q[i])||q[i]<chain[i]->lower||q[i]>chain[i]->upper) {return false;}
  }
  return true;
}

bool ScanMap::robotCollision(const Eigen::Vector3d & body,double yaw,bool foot,
  double margin,double tolerance,bool standing,
  const std::array<Eigen::Vector3d,2> * actual_feet,
  const std::array<std::array<double,2>,2> * contact_heights) const
{
  if (boxes_.empty()||joints_.empty()||!body.allFinite()||!std::isfinite(yaw)||
    !std::isfinite(margin)||margin<0||!std::isfinite(tolerance)||tolerance<0) {return true;}
  const Eigen::Matrix3d heading=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::unordered_map<std::string,double> positions;
  positions["wheel_lift_joint"]=foot?wheel_lift_:0;
  std::array<Eigen::Vector3d,2> feet;
  if (foot&&standing) {
    for (int side=0;side<2;++side) {
      const std::string prefix=side==0?"left_":"right_";
      const auto & hip=jointByName(prefix+"hip_yaw_joint");
      bool found=false;
      for (int n=0;n<=(actual_feet?0:48)&&!found;++n) {
        Eigen::Vector3d candidate;
        if (actual_feet) {candidate=(*actual_feet)[side];}
        else {
          const double offset=(n%2?1.0:-1.0)*((n+1)/2)*0.01;
          candidate=body+heading*Eigen::Vector3d(0.14+offset,hip.origin.translation().y(),0);
          double ground;
          if (!surface(candidate.x(),candidate.y(),yaw,body.z()-0.10,tolerance,ground)) {continue;}
          candidate.z()=ground+0.0175;
        }
        std::array<double,4> q;
        if (!legAngles(body,yaw,candidate,side,q)) {continue;}
        const std::array<std::string,4> names{prefix+"hip_yaw_joint",prefix+"shoulder_joint",prefix+"knee_joint",prefix+"ankle_joint"};
        for (int i=0;i<4;++i) {positions[names[i]]=q[i];}
        feet[side]=candidate;found=true;
      }
      if (!found) {return true;}
    }
  }
  std::unordered_map<std::string,Eigen::Isometry3d> transforms;
  Eigen::Isometry3d base=Eigen::Isometry3d::Identity();base.linear()=heading;base.translation()=body;
  transforms["base_link"]=base;
  for (std::size_t round=0;round<=joints_.size()&&transforms.size()<=joints_.size();++round) {
    bool progress=false;
    for (const auto & joint:joints_) {
      auto parent=transforms.find(joint.parent);
      if (parent==transforms.end()||transforms.count(joint.child)) {continue;}
      auto motion=Eigen::Isometry3d::Identity();const double q=positions[joint.name];
      if (joint.type=="prismatic") {motion.translation()=q*joint.axis;}
      else if (joint.type=="revolute"||joint.type=="continuous") {motion.linear()=Eigen::AngleAxisd(q,joint.axis).toRotationMatrix();}
      transforms[joint.child]=parent->second*joint.origin*motion;progress=true;
    }
    if (!progress) {break;}
  }
  for (const auto & shape:shapes_) {
    const bool rigid=shape.link=="base_link"||shape.link=="wheel_carrier"||
      shape.link=="left_wheel"||shape.link=="right_wheel";
    if (!standing&&!rigid) {continue;}
    const auto transform_it=transforms.find(shape.link);if (transform_it==transforms.end()) {return true;}
    const auto transform=transform_it->second*shape.origin;
    const Eigen::Vector3d center=transform.translation(),half=shape.half;
    const Eigen::Matrix3d rotation=transform.linear();
    const Eigen::Vector3d extent=rotation.cwiseAbs()*half;
    const Eigen::Vector3d minimum=center-extent,maximum=center+extent;
    bool contact=false;Eigen::Vector3d contact_point=Eigen::Vector3d::Zero();
    if (foot&&standing&&(shape.link.find("_foot")!=std::string::npos||shape.link.find("_lower_leg")!=std::string::npos)) {
      contact=true;contact_point=feet[shape.link.rfind("left_",0)==0?0:1];contact_point.z()-=0.0175;
    } else if (!foot&&(shape.link=="left_wheel"||shape.link=="right_wheel")) {
      contact=true;contact_point=center;contact_point.z()-=robot_geometry_.wheel_radius;
    }
    for (const auto & box:boxes_) {
      const Eigen::Vector3d gap=(minimum-box.maximum).cwiseMax(box.minimum-maximum).cwiseMax(0);
      if (gap.squaredNorm()>margin*margin+1e-12) {continue;}
      // Allow only the actual support top under a contact part. This does not
      // exempt raised steps, risers, nearby walls or any genuine penetration.
      bool supporting=false;
      bool contact_height=contact&&std::abs(box.maximum.z()-contact_point.z())<=tolerance;
      if (contact&&contact_heights&&foot) {
        const auto & heights=(*contact_heights)[shape.link.rfind("left_",0)==0?0:1];
        contact_height=std::abs(box.maximum.z()-heights[0])<=tolerance||
          std::abs(box.maximum.z()-heights[1])<=tolerance;
      }
      if (contact_height&&std::abs(std::abs(box.rotation(2,2))-1)<1e-9) {
        // Adjacent floor/tread boxes may form one horizontal support plane.
        // The contact centre need not lie in every box under a foot's extent.
        // Only a matching top below the whole part is allowed, never a riser
        // above that support or any actual penetration.
        supporting=true;
      }
      if (supporting&&minimum.z()>=box.maximum.z()-tolerance) {continue;}
      const double distance=boxDistance(center,rotation,half,box.center,box.rotation,box.half);
      if (distance<=1e-9||distance<margin-1e-9) {return true;}
    }
  }
  return false;
}

void ScanMap::validateFootTrajectory(const std::vector<FootPose> & poses,
  double swing_duration,double swing_lift,double feedback_period,
  double margin,double tolerance,std::chrono::steady_clock::time_point deadline) const
{
  if (poses.size()<2||swing_duration<=0||swing_lift<=0||feedback_period<=0||poses.front().time!=0) {
    throw std::runtime_error("SCAN_INVALID_FOOT_TIMING");
  }
  for (std::size_t i=0;i<poses.size();++i) {
    if (!poses[i].body.allFinite()||!std::isfinite(poses[i].yaw)||!std::isfinite(poses[i].time)||
      (i&&poses[i].time<=poses[i-1].time)) {throw std::runtime_error("SCAN_INVALID_FOOT_POSE");}
  }
  auto angle=[](double v){return std::atan2(std::sin(v),std::cos(v));};
  auto sample=[&](double time)->FootPose {
      auto end=std::lower_bound(poses.begin()+1,poses.end(),time,
        [](const FootPose & p,double t){return p.time<t;});
      if (end==poses.end()) {return poses.back();}
      const auto & a=*(end-1);const double t=std::clamp((time-a.time)/(end->time-a.time),0.0,1.0);
      return {a.body+t*(end->body-a.body),a.yaw+t*angle(end->yaw-a.yaw),time};
    };
  auto landing=[&](const FootPose & future,const FootPose & current,int side,Eigen::Vector3d & result) {
      const auto & hip=jointByName(side==0?"left_hip_yaw_joint":"right_hip_yaw_joint");
      const auto heading=Eigen::AngleAxisd(future.yaw,Eigen::Vector3d::UnitZ());
      for (int n=0;n<=48;++n) {
        const double offset=(n%2?1.0:-1.0)*((n+1)/2)*0.01;
        Eigen::Vector3d candidate=future.body+heading*Eigen::Vector3d(0.14+offset,hip.origin.translation().y(),0);
        double ground;
        if (!surface(candidate.x(),candidate.y(),future.yaw,future.body.z()-0.10,tolerance,ground)) {continue;}
        candidate.z()=ground+0.0175;std::array<double,4> q;
        if (legAngles(future.body,future.yaw,candidate,side,q)&&legAngles(current.body,current.yaw,candidate,side,q)) {
          result=candidate;return true;
        }
      }
      return false;
    };
  auto smooth=[](double t){return t*t*(3-2*t);};
  std::array<Eigen::Vector3d,2> feet;
  std::array<std::array<double,4>,2> previous_q;
  std::array<std::array<double,2>,2> contact_heights;
  const auto first_heading=Eigen::AngleAxisd(poses.front().yaw,Eigen::Vector3d::UnitZ());
  for (int side=0;side<2;++side) {
    const auto & hip=jointByName(side==0?"left_hip_yaw_joint":"right_hip_yaw_joint");
    feet[side]=poses.front().body+first_heading*Eigen::Vector3d(0.14,hip.origin.translation().y(),0);
    double ground;
    if (!surface(feet[side].x(),feet[side].y(),poses.front().yaw,poses.front().body.z()-0.10,tolerance,ground)) {
      throw std::runtime_error("SCAN_INITIAL_FOOT_SUPPORT");
    }
    feet[side].z()=ground+0.0175;
    if (!legAngles(poses.front().body,poses.front().yaw,feet[side],side,previous_q[side])) {
      throw std::runtime_error("SCAN_INITIAL_FOOT_IK");
    }
    contact_heights[side]={ground,ground};
  }
  bool swinging=false;int swing_side=0;
  double next_query=0,swing_start=0,actual_swing_duration=swing_duration,clearance=0;
  Eigen::Vector3d from,to;
  const double dt=0.001;
  const std::size_t count=static_cast<std::size_t>(std::ceil(poses.back().time/dt));
  for (std::size_t i=0;i<=count;++i) {
    if (std::chrono::steady_clock::now()>=deadline) {throw std::runtime_error("SCAN_TIMEOUT_FOOT_VALIDATION");}
    const double time=std::min(i*dt,poses.back().time);const auto current=sample(time);
    if (!swinging&&time>=next_query) {
      next_query=time+feedback_period;
      auto future=sample(time+swing_duration*2);Eigen::Vector3d target;
      if (!landing(future,current,swing_side,target)) {throw std::runtime_error("SCAN_NO_FUTURE_LANDING at t="+std::to_string(time));}
      if ((target-feet[swing_side]).norm()>0.015) {
        actual_swing_duration=swing_duration+4.5*std::abs(target.z()-feet[swing_side].z());
        future=sample(time+actual_swing_duration*1.5);
        if (!landing(future,current,swing_side,target)) {throw std::runtime_error("SCAN_NO_RETREIVED_LANDING at t="+std::to_string(time));}
        actual_swing_duration=std::max(actual_swing_duration,swing_duration+4.5*std::abs(target.z()-feet[swing_side].z()));
        swinging=true;swing_start=time;from=feet[swing_side];to=target;
        clearance=std::max(from.z(),to.z())+swing_lift;
        contact_heights[swing_side]={from.z()-0.0175,to.z()-0.0175};
        for (int j=0;j<=20;++j) {
          const auto p=from+(to-from)*(j/20.0);
          for (double x:{-0.0175,0.0725,0.1625}) {
            double ground;
            if (height(p.x()+x*std::cos(future.yaw),p.y()+x*std::sin(future.yaw),
              std::max(current.body.z(),future.body.z())-0.10,0.9,ground)) {
              clearance=std::max(clearance,ground+0.0175+swing_lift);
            }
          }
        }
      } else {swing_side=1-swing_side;}
    }
    if (swinging) {
      const double t=std::clamp((time-swing_start)/actual_swing_duration,0.0,1.0);
      if (std::abs(from.z()-to.z())<tolerance&&clearance<=from.z()+swing_lift+1e-6) {
        feet[swing_side]=from+(to-from)*smooth(t);feet[swing_side].z()+=swing_lift*std::sin(M_PI*t);
      } else if (t<0.35) {
        feet[swing_side]=from;feet[swing_side].z()+=(clearance-from.z())*smooth(t/0.35);
      } else if (t<0.65) {
        feet[swing_side]=from+(to-from)*smooth((t-0.35)/0.30);feet[swing_side].z()=clearance;
      } else {
        feet[swing_side]=to;feet[swing_side].z()=clearance+(to.z()-clearance)*smooth((t-0.65)/0.35);
      }
      if (t>=1) {contact_heights[swing_side]={to.z()-0.0175,to.z()-0.0175};swinging=false;swing_side=1-swing_side;}
    }
    for (int side=0;side<2;++side) {
      std::array<double,4> q;
      if (!legAngles(current.body,current.yaw,feet[side],side,q)) {
        throw std::runtime_error("SCAN_FOOT_SWING_IK at t="+std::to_string(time));
      }
      const std::string prefix=side==0?"left_":"right_";
      const std::array<std::string,4> names{prefix+"hip_yaw_joint",prefix+"shoulder_joint",prefix+"knee_joint",prefix+"ankle_joint"};
      for (int j=0;j<4;++j) {
        const double speed=i?std::abs(q[j]-previous_q[side][j])/std::min(dt,time-(i-1)*dt):0;
        if (speed>jointByName(names[j]).speed+1e-9) {
          throw std::runtime_error("SCAN_FOOT_JOINT_SPEED "+names[j]+" speed="+std::to_string(speed)+
            " limit="+std::to_string(jointByName(names[j]).speed)+" at t="+std::to_string(time));
        }
      }
      previous_q[side]=q;
    }
    if (robotCollision(current.body,current.yaw,true,margin,tolerance,true,&feet,&contact_heights)) {
      throw std::runtime_error("SCAN_FOOT_SWING_COLLISION at t="+std::to_string(time));
    }
  }
}

void ScanMap::load(const std::string & filename)
{
  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("SCAN_WORLD_UNREADABLE: " + filename);
  }
  auto sdf = document.FirstChildElement("sdf");
  auto world = sdf ? sdf->FirstChildElement("world") : nullptr;
  if (!world || world->FirstChildElement("include") || world->FirstChildElement("frame")) {
    throw std::runtime_error("SCAN_WORLD_UNSUPPORTED_STRUCTURE");
  }
  std::vector<Box> parsed;
  for (auto model = world->FirstChildElement("model"); model;
    model = model->NextSiblingElement("model"))
  {
    const auto is_static = model->FirstChildElement("static");
    if (!is_static || !is_static->GetText() || std::string(is_static->GetText()) != "true" ||
      model->FirstChildElement("model") || model->FirstChildElement("include") ||
      model->FirstChildElement("frame"))
    {
      throw std::runtime_error("SCAN_WORLD_REQUIRES_STATIC_BOX_MODELS");
    }
    const auto model_pose = pose(model);
    for (auto link = model->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
      const auto link_pose = model_pose * pose(link);
      for (auto collision = link->FirstChildElement("collision"); collision;
        collision = collision->NextSiblingElement("collision"))
      {
        const auto geometry = collision->FirstChildElement("geometry");
        const auto box = geometry ? geometry->FirstChildElement("box") : nullptr;
        const auto size = box ? box->FirstChildElement("size") : nullptr;
        if (!size || geometry->FirstChildElement() != box || box->NextSiblingElement()) {
          throw std::runtime_error("SCAN_WORLD_REQUIRES_BOX_COLLISIONS");
        }
        const auto values = numbers(size->GetText(), 3);
        if (std::any_of(values.begin(), values.end(), [](double x) {return x <= 0;})) {
          throw std::runtime_error("SCAN_WORLD_INVALID_BOX_SIZE");
        }
        const auto transform = link_pose * pose(collision);
        Box result;
        result.center = transform.translation();
        result.rotation = transform.linear();
        result.half = Eigen::Vector3d(values[0], values[1], values[2])*0.5;
        const Eigen::Vector3d extent = result.rotation.cwiseAbs()*result.half;
        result.minimum = result.center - extent;
        result.maximum = result.center + extent;
        parsed.push_back(result);
      }
    }
  }
  if (parsed.empty()) {throw std::runtime_error("SCAN_WORLD_NO_COLLISIONS");}
  boxes_ = std::move(parsed);
}

bool ScanMap::height(double x, double y, double ceiling, double depth, double & z) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(ceiling) ||
    !std::isfinite(depth) || depth <= 0) {return false;}
  double best = std::numeric_limits<double>::infinity();
  const Eigen::Vector3d origin(x, y, ceiling), direction(0, 0, -1);
  for (const auto & box : boxes_) {
    if (x < box.minimum.x()-1e-9 || x > box.maximum.x()+1e-9 ||
      y < box.minimum.y()-1e-9 || y > box.maximum.y()+1e-9 ||
      box.minimum.z() > ceiling+1e-9 || box.maximum.z() < ceiling-depth-1e-9) {continue;}
    const Eigen::Vector3d local_origin = box.rotation.transpose()*(origin-box.center);
    const Eigen::Vector3d local_direction = box.rotation.transpose()*direction;
    double enter = 0, leave = depth;
    bool hit = true;
    for (int i = 0; i < 3; ++i) {
      if (std::abs(local_direction[i]) < 1e-12) {
        if (std::abs(local_origin[i]) > box.half[i]+1e-9) {hit = false; break;}
      } else {
        double first = (-box.half[i]-local_origin[i])/local_direction[i];
        double last = (box.half[i]-local_origin[i])/local_direction[i];
        if (first > last) {std::swap(first, last);}
        enter = std::max(enter, first);
        leave = std::min(leave, last);
        if (enter > leave+1e-9) {hit = false; break;}
      }
    }
    // If the ray starts inside geometry, its entry is not a supporting top.
    if (hit && (local_origin.cwiseAbs().array() < box.half.array()-1e-9).all()) {continue;}
    if (hit && enter < best) {best = enter;}
  }
  if (!std::isfinite(best)) {return false;}
  z = ceiling-best;
  return true;
}

bool ScanMap::support(const Eigen::Vector3d & body, double expected_ground,
  double tolerance) const
{
  if (!body.allFinite() || !std::isfinite(expected_ground) ||
    !std::isfinite(tolerance) || tolerance < 0) {return false;}
  double actual;
  return height(body.x(), body.y(), body.z()-0.10, 0.9, actual) &&
    std::abs(actual-expected_ground) <= tolerance+1e-9;
}

bool ScanMap::surface(double x, double y, double yaw, double ceiling,
  double tolerance, double & z) const
{
  if (!std::isfinite(yaw) || !std::isfinite(tolerance) || tolerance < 0 ||
    !height(x, y, ceiling, 0.9, z)) {return false;}
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (double dx : {-0.0175, 0.1625}) {
    for (double dy : {-0.05, 0.05}) {
      double corner;
      if (!height(x+dx*c-dy*s, y+dx*s+dy*c, ceiling, 0.9, corner) ||
        std::abs(corner-z) > tolerance+1e-9) {return false;}
    }
  }
  return true;
}

bool ScanMap::footLanding(const Eigen::Vector3d & body, double yaw, int side,
  double tolerance, Eigen::Vector3d & foot) const
{
  if (!body.allFinite() || !std::isfinite(yaw) || (side != 0 && side != 1)) {return false;}
  const double c = std::cos(yaw), s = std::sin(yaw), dy = side == 0 ? 0.34 : -0.34;
  for (int n = 0; n <= 48; ++n) {
    const double offset = (n % 2 ? 1.0 : -1.0)*((n+1)/2)*0.01;
    const double dx = 0.14+offset;
    Eigen::Vector3d candidate(body.x()+dx*c-dy*s, body.y()+dx*s+dy*c, 0);
    double ground;
    if (surface(candidate.x(), candidate.y(), yaw, body.z()-0.10, tolerance, ground)) {
      candidate.z() = ground+0.0175;
      foot = candidate;
      return true;
    }
  }
  return false;
}
}  // namespace local_planning
