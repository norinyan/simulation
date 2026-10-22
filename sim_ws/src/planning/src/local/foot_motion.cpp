#include "local_planner.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace planning {
namespace {
std::array<double,3> footCoordinate(double p,double end,double v,double ev,double a,double ea,double dt,double u){
  const double c1=v*dt,c2=a*dt*dt/2,d=end-p-c1-c2,e=ev*dt-c1-2*c2,f=ea*dt*dt-2*c2;
  const double c3=10*d-4*e+f/2,c4=-15*d+7*e-f,c5=6*d-3*e+f/2;
  return {p+u*(c1+u*(c2+u*(c3+u*(c4+u*c5)))),
    (c1+u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5))))/dt,
    (2*c2+u*(6*c3+u*(12*c4+u*20*c5)))/(dt*dt)};
}
}
double footInterpolate(double p,double end,double v,double ev,double a,double ea,double dt,double u) {
  return footCoordinate(p,end,v,ev,a,ea,dt,u)[0];
}
FootTrajectoryPoint interpolateFootTrajectory(const FootTrajectoryPoint& a,const FootTrajectoryPoint& b,double u) {
  auto out=a;const double dt=b.time_from_start-a.time_from_start;out.time_from_start+=u*dt;
  for(int j=0;j<3;++j){
    auto& p=j==0?out.body:out.feet[j-1];const auto& end=j==0?b.body:b.feet[j-1];
    const auto& v=a.spatial_velocity[j];const auto& w=b.spatial_velocity[j];
    const auto& ac=a.spatial_acceleration[j];const auto& bc=b.spatial_acceleration[j];
    const auto x=footCoordinate(p.x,end.x,v.x,w.x,ac.x,bc.x,dt,u),y=footCoordinate(p.y,end.y,v.y,w.y,ac.y,bc.y,dt,u),
      z=footCoordinate(p.z,end.z,v.z,w.z,ac.z,bc.z,dt,u),
      yaw=footCoordinate(p.yaw,p.yaw+std::atan2(std::sin(end.yaw-p.yaw),std::cos(end.yaw-p.yaw)),v.yaw,w.yaw,ac.yaw,bc.yaw,dt,u);
    p={x[0],y[0],z[0],yaw[0]};out.spatial_velocity[j]={x[1],y[1],z[1],yaw[1]};out.spatial_acceleration[j]={x[2],y[2],z[2],yaw[2]};
  }return out;
}
void canonicalizeFootCoordinates(std::vector<FootTrajectoryPoint>& samples) {
  // Canonicalize roundoff from adjacent world-frame transforms before taking
  // derivatives; identical planted coordinates must yield a constant polynomial.
  for(std::size_t i=1;i<samples.size();++i)for(int j=0;j<3;++j){
    const auto& a=j? samples[i-1].feet[j-1]:samples[i-1].body;auto& b=j?samples[i].feet[j-1]:samples[i].body;
    auto same=[](double a,double& b){if(std::abs(a-b)<1e-13)b=a;};same(a.x,b.x);same(a.y,b.y);same(a.z,b.z);same(a.yaw,b.yaw);
  }
}
void setSpatialFootDerivatives(std::vector<FootTrajectoryPoint>& samples) {
  canonicalizeFootCoordinates(samples);
  for(auto& s:samples){s.spatial_velocity={};s.spatial_acceleration={};}
  for(std::size_t i=1;i+1<samples.size();++i){
    auto& s=samples[i];const auto& a=samples[i-1];const auto& b=samples[i+1];
    const double h=s.time_from_start-a.time_from_start,k=b.time_from_start-s.time_from_start;
    for(int j=0;j<3;++j){
      const auto& pa=j==0?a.body:a.feet[j-1];const auto& pc=j==0?s.body:s.feet[j-1];const auto& pb=j==0?b.body:b.feet[j-1];
      auto derivative=[&](double x,double y,double z){
        // A coordinate stationary on either adjacent interval has zero boundary
        // derivatives. Central differences otherwise leak swing motion into contact.
        if(std::abs(x-y)<1e-13 || std::abs(y-z)<1e-13)return std::pair<double,double>{0,0};
        return std::pair<double,double>{((y-x)*k/h+(z-y)*h/k)/(h+k),2*((z-y)/k-(y-x)/h)/(h+k)};
      };
      auto x=derivative(pa.x,pc.x,pb.x),y=derivative(pa.y,pc.y,pb.y),z=derivative(pa.z,pc.z,pb.z),yaw=derivative(pa.yaw,pc.yaw,pb.yaw);
      s.spatial_velocity[j]={x.first,y.first,z.first,yaw.first};s.spatial_acceleration[j]={x.second,y.second,z.second,yaw.second};
    }
  }

}

FootJointPose footInverseKinematics(const Pose& body,const Pose& foot,int side,const FootRobotGeometry& r)
{
  const double c=std::cos(body.yaw),s=std::sin(body.yaw),sign=side==0?1:-1;
  FootJointPose out;
  out.hip={body.x+c*r.shoulder_offset[0]-s*sign*r.shoulder_offset[1],
           body.y+s*r.shoulder_offset[0]+c*sign*r.shoulder_offset[1],body.z+r.shoulder_offset[2],foot.yaw};
  out.ankle={foot.x-r.foot_center_offset_x*std::cos(foot.yaw),
             foot.y-r.foot_center_offset_x*std::sin(foot.yaw),foot.z,foot.yaw};
  const double x=out.ankle.x-out.hip.x,y=out.ankle.y-out.hip.y;
  const double dx=x*std::cos(foot.yaw)+y*std::sin(foot.yaw),down=out.hip.z-foot.z;
  if(std::abs(-x*std::sin(foot.yaw)+y*std::cos(foot.yaw))>1e-5)
    throw std::runtime_error("HIP_YAW_LEG_PLANE");
  const double d=std::hypot(dx,down),a=r.upper_leg_length,b=r.lower_leg_length;
  if(d>=a+b-1e-5 || d<=std::abs(a-b)+1e-5)throw std::runtime_error("IK_UNREACHABLE");
  const double alpha=std::acos(std::clamp((d*d+a*a-b*b)/(2*d*a),-1.,1.));
  const double beta=std::acos(std::clamp((d*d+b*b-a*a)/(2*d*b),-1.,1.));
  const double theta=std::atan2(-dx,down);
  out.upper_pitch=theta+alpha;out.lower_pitch=theta-beta;
  out.joints={std::atan2(std::sin(foot.yaw-body.yaw),std::cos(foot.yaw-body.yaw)),
    out.upper_pitch-r.folded_pitch[0],out.lower_pitch-out.upper_pitch-r.folded_pitch[1],-out.lower_pitch-r.folded_pitch[2]};
  for(int i=0;i<4;++i)if(!std::isfinite(out.joints[i]) ||
    out.joints[i]<(i?r.joint_min:r.hip_yaw_min)-1e-6 || out.joints[i]>(i?r.joint_max:r.hip_yaw_max)+1e-6)
      throw std::runtime_error("FOOT_JOINT_LIMIT");
  out.knee={out.hip.x-a*std::sin(out.upper_pitch)*std::cos(foot.yaw),
    out.hip.y-a*std::sin(out.upper_pitch)*std::sin(foot.yaw),out.hip.z-a*std::cos(out.upper_pitch),foot.yaw};
  return out;
}

FootLocalPlanResult runPlanner(FootLocalPlanner& planner,const FootLocalPlanRequest& request) {
  FootLocalPlanResult result;
  try {
    auto check_control=[&](){if(auto reason=request.geometry.control.failure();!reason.empty())throw std::runtime_error(reason);};
    check_control();if(!request.limits)throw std::invalid_argument("FOOT_MOTION_LIMITS_REQUIRED");
    for(double v:{request.limits->max_step,request.limits->swing_clearance,request.limits->step_duration,request.limits->sample_dt,
      request.limits->max_foot_v,request.limits->max_foot_a,request.limits->max_body_v,request.limits->max_body_a,
      request.limits->max_joint_v,request.limits->max_joint_a,request.limits->body_height})
      if(!std::isfinite(v)||v<=0)throw std::invalid_argument("INVALID_FOOT_LIMITS");
    if(!std::isfinite(request.limits->edge_margin)||request.limits->edge_margin<0||!std::isfinite(request.limits->wheel_lift)||request.limits->wheel_lift<0||request.limits->sample_dt>.05)throw std::invalid_argument("INVALID_FOOT_LIMITS");
    if(request.current_body && (!request.supported[0] || !request.supported[1]))throw std::invalid_argument("INVALID_FOOT_ENTRY_SUPPORT");
    result=planner.plan(request);check_control();
    if(!result.success)throw std::runtime_error(result.failure_reason.empty()?"PLANNER_FAILED":result.failure_reason);
    const auto begin=std::chrono::steady_clock::now();
    if(result.trajectory.size()<2 || result.trajectory.size()>20000)throw std::runtime_error("INVALID_FOOT_SAMPLE_COUNT");
    auto finite=[](const Pose& p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::isfinite(p.yaw);};
    const auto& first=result.trajectory.front();const auto& last=result.trajectory.back();
    auto matches=[](const Pose& a,const Pose& b){return std::hypot(a.x-b.x,a.y-b.y)<1e-6&&std::abs(a.z-b.z)<1e-6&&std::abs(std::atan2(std::sin(a.yaw-b.yaw),std::cos(a.yaw-b.yaw)))<1e-6;};
    auto start=request.geometry.start,goal=request.geometry.goal;start.z+=request.limits->body_height;goal.z+=request.limits->body_height;
    if(first.time_from_start!=0 || !matches(first.body,request.current_body.value_or(start)) || !matches(last.body,goal))throw std::runtime_error("INVALID_FOOT_ENDPOINTS");
    for(const auto* endpoint:{&first,&last})for(const auto* values:{&endpoint->spatial_velocity,&endpoint->spatial_acceleration})for(const auto& d:*values)
      if(!finite(d)||std::max({std::abs(d.x),std::abs(d.y),std::abs(d.z),std::abs(d.yaw)})>1e-6)throw std::runtime_error("FOOT_ENDPOINT_DERIVATIVE");
    if(request.current_body)for(int side=0;side<2;++side)if(!matches(first.feet[side],request.current_feet[side]))throw std::runtime_error("INVALID_FOOT_INITIAL_FEET");
    for(std::size_t i=0;i<result.trajectory.size();++i){const auto& s=result.trajectory[i];
      if(!std::isfinite(s.time_from_start)||(i&&(s.time_from_start<=result.trajectory[i-1].time_from_start || s.time_from_start-result.trajectory[i-1].time_from_start>.05+1e-9)))throw std::runtime_error("INVALID_FOOT_TIME");
      if(!finite(s.body)||!finite(s.feet[0])||!finite(s.feet[1]))throw std::runtime_error("NONFINITE_FOOT_POSE");
      for(const auto* values:{&s.spatial_velocity,&s.spatial_acceleration})for(const auto& d:*values)if(!finite(d))throw std::runtime_error("NONFINITE_FOOT_DERIVATIVE");
    }
    for(const auto& step:result.footsteps)if((step.side!=0&&step.side!=1)||!finite(step.from)||!finite(step.to)||!std::isfinite(step.touchdown_time)||step.touchdown_time<0||step.touchdown_time>last.time_from_start+1e-6)throw std::runtime_error("INVALID_FOOTSTEP_FORMAT");
    result.validation_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();check_control();
  }catch(const std::exception& e){result={};result.failure_reason=e.what();}
  return result;
}
} // namespace planning
