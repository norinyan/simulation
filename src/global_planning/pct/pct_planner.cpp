#include "pct_planner.h"
#include "pct_core.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace global_planning {
namespace {
constexpr double height_epsilon = 1e-6;  // float32 台阶高差的数值误差，不增加通行能力。
pct_native::Pose native(const geometry_msgs::msg::Pose & p) {
  return {p.position.x, p.position.y, p.position.z,
    2*std::atan2(p.orientation.z,p.orientation.w)};
}
geometry_msgs::msg::Pose rosPose(const pct_native::Pose & p) {
  geometry_msgs::msg::Pose q;
  q.position.x=p.x; q.position.y=p.y; q.position.z=p.z;
  q.orientation.z=std::sin(p.yaw/2); q.orientation.w=std::cos(p.yaw/2);
  return q;
}
struct Region {
  Terrain terrain;
  double x,y,z,sx,sy,sz,yaw;
  bool contains(double px,double py,double pz,double padding,double step) const {
    const double dx=px-x,dy=py-y;
    return std::abs(std::cos(yaw)*dx+std::sin(yaw)*dy)<=sx/2+padding &&
      std::abs(-std::sin(yaw)*dx+std::cos(yaw)*dy)<=sy/2+padding &&
      pz>=z-sz/2-step-height_epsilon && pz<=z+sz/2+step+height_epsilon;
  }
};
class PctAdapter final : public GlobalPlanner {
public:
  explicit PctAdapter(const std::string & filename) {
    const auto config=YAML::LoadFile(filename);
    map_settings_=YAML::Clone(config["pct"]["map"]);
    const auto c=config["pct"]; const auto o=c["search"];
    options_.cost_threshold=o["cost_threshold"].as<double>();
    options_.step_cost_weight=o["step_cost_weight"].as<double>();
    const auto opt=c["optimizer"];
    options_.optimize=opt["enabled"].as<bool>();

    options_.max_heading_rate=opt["max_heading_rate"].as<double>();
    options_.optimizer_cost_margin=opt["cost_margin"].as<double>();
    options_.optimizer_jerk_sigma=opt["jerk_sigma"].as<double>();
    options_.optimizer_sample_interval=opt["sample_interval"].as<int>();
    options_.optimizer_iterations=opt["iterations"].as<int>();
    for (double v:{options_.cost_threshold,options_.max_heading_rate,
      options_.optimizer_jerk_sigma}) {
      if(!std::isfinite(v)||v<=0) throw std::runtime_error("PCT_INVALID_OPTIONS");
    }
    if(!std::isfinite(options_.step_cost_weight)||options_.step_cost_weight<0||
      !std::isfinite(options_.optimizer_cost_margin)||options_.optimizer_cost_margin<0||
      options_.optimizer_cost_margin>=options_.cost_threshold||
      options_.optimizer_sample_interval<1||options_.optimizer_iterations<1) {
      throw std::runtime_error("PCT_INVALID_OPTIONS");
    }
    const auto limits=config["constraints"];
    expected_={limits["length"].as<double>(),limits["width"].as<double>(),
      limits["safety_margin"].as<double>(),limits["max_step"].as<double>(),
      limits["clearance"].as<double>()};
    auto path=std::filesystem::path(c["ideal_config"].as<std::string>());
    if(path.is_relative())path=std::filesystem::path(filename).parent_path()/path;
    const auto ideal=YAML::LoadFile(path.string());
    reference_height_=ideal["reference_height"].as<double>();
    options_.reference_height=reference_height_;
    position_tolerance_=ideal["position_tolerance"].as<double>();
    for(double value:{reference_height_,position_tolerance_}) {
      if(!std::isfinite(value)||value<=0)throw std::runtime_error("PCT_INVALID_REFERENCE_HEIGHT");
    }
  }

  void loadMap(const std::string & filename) override {
    map_=pct_native::loadPctMap(filename);
    const auto metadata=YAML::LoadFile(std::filesystem::path(filename).replace_extension(".json").string());
    const auto parameters=metadata["parameters"];
    const auto shape=metadata["shape"];
    if(shape.size()!=4 || shape[0].as<int>()!=5 || shape[1].as<unsigned>()!=map_->layers ||
      shape[2].as<unsigned>()!=map_->nx || shape[3].as<unsigned>()!=map_->ny) {
      throw std::runtime_error("PCT_METADATA_SHAPE_MISMATCH");
    }
    const auto equal=[](double a,double b){return std::isfinite(a)&&std::abs(a-b)<1e-8;};
    if(!equal(parameters["length"].as<double>(),expected_.length)||
      !equal(parameters["width"].as<double>(),expected_.width)||
      !equal(parameters["safety_margin"].as<double>(),expected_.safety_margin)||
      !equal(map_->max_step,expected_.max_step)||!equal(map_->clearance,expected_.clearance)||
      std::hypot(expected_.length,expected_.width)/2+expected_.safety_margin>map_->inflation_radius+1e-8) {
      throw std::runtime_error("PCT_MAP_CONSTRAINT_MISMATCH_REGENERATE_OFFLINE");
    }
    if(!equal(parameters["cost_threshold"].as<double>(),options_.cost_threshold)) {
      throw std::runtime_error("PCT_MAP_COST_THRESHOLD_MISMATCH_REGENERATE_OFFLINE");
    }
    for(const auto & setting:map_settings_) {
      const auto key=setting.first.as<std::string>();
      if(!equal(parameters[key].as<double>(),setting.second.as<double>())) {
        throw std::runtime_error("PCT_MAP_PARAMETER_MISMATCH_REGENERATE_OFFLINE: "+key);
      }
    }
    regions_.clear();
    if(!metadata["regions"].IsSequence())throw std::runtime_error("PCT_MISSING_TERRAIN_REGIONS");
    for(const auto & item:metadata["regions"]) {
      const auto kind=item["kind"].as<std::string>();
      if(kind!="stair"&&kind!="stair_landing")throw std::runtime_error("PCT_BAD_TERRAIN_KIND");
      const auto center=item["center"],size=item["size"];
      if(center.size()!=3||size.size()!=3)throw std::runtime_error("PCT_BAD_TERRAIN_REGION");
      Region r{kind=="stair"?Terrain::Stair:Terrain::StairLanding,
        center[0].as<double>(),center[1].as<double>(),center[2].as<double>(),
        size[0].as<double>(),size[1].as<double>(),size[2].as<double>(),item["yaw"].as<double>()};
      for(double v:{r.x,r.y,r.z,r.sx,r.sy,r.sz,r.yaw}) {
        if(!std::isfinite(v))throw std::runtime_error("PCT_BAD_TERRAIN_REGION");
      }
      if(r.sx<=0||r.sy<=0||r.sz<=0)throw std::runtime_error("PCT_BAD_TERRAIN_REGION");
      regions_.push_back(r);
    }
  }

  geometry_msgs::msg::Pose groundPose(const geometry_msgs::msg::Pose & body) const override {
    if(!map_)throw std::runtime_error("PCT_MAP_REQUIRED");
    const auto & m=*map_;
    const int x=std::lround((body.position.x-m.center_x)/m.resolution)+m.nx/2;
    const int y=std::lround((body.position.y-m.center_y)/m.resolution)+m.ny/2;
    if(x<0||y<0||x>=int(m.nx)||y>=int(m.ny))throw std::runtime_error("PCT_ENDPOINT_OUTSIDE_MAP");
    std::vector<double> candidates;
    for(int l=0;l<int(m.layers);++l) {
      const double g=m.at(3,l,x,y),cost=m.at(0,l,x,y),ceiling=m.at(4,l,x,y);
      if(!std::isfinite(g)||!std::isfinite(cost)||cost>options_.cost_threshold||
        (std::isfinite(ceiling)&&ceiling-g<m.clearance-height_epsilon))continue;
      const double height=body.position.z-g;
      if(std::abs(height-reference_height_)>
        position_tolerance_+height_epsilon)continue;
      if(std::none_of(candidates.begin(),candidates.end(),[g](double v){return std::abs(g-v)<height_epsilon;})) {
        candidates.push_back(g);
      }
    }
    if(candidates.size()!=1)throw std::runtime_error("PCT_GROUND_START_MISSING_OR_AMBIGUOUS");
    auto result=body;result.position.z=candidates.front();return result;
  }

  PlanResult plan(const PlanRequest & request) override {
    try {
      if(!map_)throw std::runtime_error("PCT_MAP_REQUIRED");
      if(request.constraints.length!=expected_.length||request.constraints.width!=expected_.width||
        request.constraints.safety_margin!=expected_.safety_margin||
        request.constraints.max_step!=expected_.max_step||request.constraints.clearance!=expected_.clearance) {
        throw std::runtime_error("PCT_REQUEST_CONSTRAINT_MISMATCH");
      }
      pct_native::PlanRequest input;
      input.start=native(request.start); input.goal=native(request.goal);
      input.footprint={request.constraints.length,request.constraints.width};
      input.safety_margin=request.constraints.safety_margin;
      input.cost_threshold=options_.cost_threshold;
      input.control.deadline=request.deadline; input.pct_map=map_;
      pct_native::PctPlanner core(options_);
      auto raw=core.plan(input);
      if(!raw.success)return {false,{}, {},raw.failure_reason};
      PlanResult result;result.success=true;
      for(const auto & p:raw.path)result.points.push_back(rosPose(p));
      result.terrain=classifyTerrain(raw.path);
      optimized_reference_=std::move(raw.body_reference); // 不替换原始地面搜索路线。
      return result;
    } catch(const std::exception & e) {return {false,{}, {},e.what()};}
  }

private:
  std::vector<Terrain> classifyTerrain(const std::vector<pct_native::Pose> & path) const {
    std::vector<Terrain> terrain;
    const double padding=std::hypot(expected_.length,expected_.width)/2+expected_.safety_margin;
    for(std::size_t i=1;i<path.size();++i) {
      const auto & a=path[i-1];const auto & b=path[i];
      const int samples=std::max(1,int(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/(map_->resolution*.4))));
      Terrain kind=Terrain::Flat;
      for(int j=0;j<=samples;++j) {
        const double t=double(j)/samples;
        for(const auto & r:regions_) {
          if(r.contains(a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z),padding,map_->max_step)) {
            if(r.terrain==Terrain::StairLanding)kind=Terrain::StairLanding;
            else if(kind==Terrain::Flat)kind=Terrain::Stair;
          }
        }
      }
      if(kind==Terrain::Flat&&std::abs(a.z-b.z)>height_epsilon) {
        throw std::runtime_error("PCT_HEIGHT_CHANGE_OUTSIDE_STAIR_REGION");
      }
      terrain.push_back(kind);
    }
    return terrain;
  }
  std::shared_ptr<const pct_native::PctMap> map_;
  pct_native::PctOptions options_;
  YAML::Node map_settings_;
  TraversalConstraints expected_;
  std::vector<Region> regions_;
  std::vector<pct_native::Pose> optimized_reference_;
  double reference_height_{0},position_tolerance_{0};
};
}
std::unique_ptr<GlobalPlanner> makePctPlanner(const std::string & config_file) {
  return std::make_unique<PctAdapter>(config_file);
}
}  // namespace global_planning
