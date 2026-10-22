#include "local_planner.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <limits>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>
void expect(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
struct Stub : planning::FootLocalPlanner {planning::FootLocalPlanResult result;bool called=false;planning::FootLocalPlanResult plan(const planning::FootLocalPlanRequest&)override{called=true;return result;}};
int main(int argc,char** argv){try {
  if(argc==2) {
    std::ifstream input(argv[1]);expect(input.good(),"Missing historical contact CSV");std::string line;std::getline(input,line);std::vector<planning::FootTrajectoryPoint> old;
    while(std::getline(input,line)) {std::replace(line.begin(),line.end(),',',' ');std::istringstream in(line);planning::FootTrajectoryPoint p;
      in>>p.time_from_start>>p.body.x>>p.body.y>>p.body.z;for(auto& foot:p.feet)in>>foot.x>>foot.y>>foot.z;
      std::array<double,7> v{},a{};for(auto& x:v)in>>x;for(auto& x:a)in>>x;
      for(int j=0;j<3;++j){const int k=j?3+2*(j-1):0;p.spatial_velocity[j]={v[k],j?0:v[1],v[j?k+1:2],0};p.spatial_acceleration[j]={a[k],j?0:a[1],a[j?k+1:2],0};}old.push_back(p);
    }
    auto fixed=old;planning::setSpatialFootDerivatives(fixed);double before=1,after=1,drift=0;int intervals=0;
    for(std::size_t i=1;i<old.size();++i)for(int side=0;side<2;++side){
      if(std::abs(old[i-1].feet[side].z-.0175)>1e-9||std::abs(old[i].feet[side].z-.0175)>1e-9)continue;++intervals;
      for(int k=0;k<=1000;++k){auto a=planning::interpolateFootTrajectory(old[i-1],old[i],k/1000.);auto b=planning::interpolateFootTrajectory(fixed[i-1],fixed[i],k/1000.);
        before=std::min(before,a.feet[side].z-.0175);after=std::min(after,b.feet[side].z-.0175);drift=std::max(drift,std::hypot(b.feet[side].x-fixed[i-1].feet[side].x,b.feet[side].y-fixed[i-1].feet[side].y));
      }
    }
    expect(intervals>0&&before < -1e-6&&after>=-1e-12&&drift<1e-12,"Historical contact reproduction failed");
    std::cout<<std::setprecision(17)<<"PASS historical contact samples="<<old.size()<<" intervals="<<intervals<<" old_min_sole_m="<<before<<" fixed_min_sole_m="<<after<<" planted_drift_m="<<drift<<'\n';return 0;
  }

    // Contact boundary from the historical 9 s failure: central differences
    // give a stationary sole nonzero derivatives and Hermite penetrates the slab.
    std::vector<planning::FootTrajectoryPoint> contact(4);
    for(int i=0;i<4;++i){contact[i].time_from_start=i*.019736842105263;contact[i].feet[0].z=.0175;}
    contact[3].feet[0].z=.017512620808122121;
    planning::setSpatialFootDerivatives(contact);
    for(int i=1;i<3;++i)for(int k=0;k<=1000;++k){auto p=planning::interpolateFootTrajectory(contact[i-1],contact[i],k/1000.);expect(p.feet[0].z==.0175,"Contact interpolator moved a planted sole");}
    // Last 11.25 ms of a 24 cm analytic descent: the interpolated sole
    // must stay above contact through touchdown, not only at its knots.
    for(int k=0;k<=1000;++k){double z=planning::footInterpolate(.01750236414400006,.0175,-.000627264,0,.1103872,0,.01125,k/1000.);expect(z>=.0175-1e-12,"Descending interpolation penetrated the support surface");}

  planning::FootLocalPlanRequest r;r.limits=planning::FootMotionLimits{};
  r.geometry.start={0,0,0,0};r.geometry.goal={.2,0,0,0};
  Stub stub;stub.result.success=true;
  planning::FootTrajectoryPoint a,b;a.body={0,0,r.limits->body_height,0};b=a;b.body.x=.2;b.time_from_start=.02;
  stub.result.trajectory={a,b};
  expect(planning::runPlanner(stub,r).success && stub.called,"Public foot dispatch failed");
  stub.result.trajectory[1].time_from_start=0;expect(!planning::runPlanner(stub,r).success,"Accepted duplicate time");
  stub.result.trajectory[1]=b;stub.result.trajectory[1].spatial_velocity[0].x=std::numeric_limits<double>::quiet_NaN();expect(!planning::runPlanner(stub,r).success,"Accepted NaN");
  stub.result.trajectory[1]=b;stub.result.trajectory[1].body.x=.3;expect(!planning::runPlanner(stub,r).success,"Accepted wrong endpoint");
  stub.result.trajectory[1]=b;stub.result.trajectory[1].spatial_velocity[0].x=.01;expect(!planning::runPlanner(stub,r).success,"Accepted moving endpoint");
  r.geometry.control.cancelled=std::make_shared<std::atomic_bool>(true);stub.called=false;expect(!planning::runPlanner(stub,r).success&&!stub.called,"Called cancelled planner");
  std::cout<<"PASS contact interpolation and public lightweight foot boundary\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
