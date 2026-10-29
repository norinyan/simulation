#include "scan_map.h"
#include <tinyxml2.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <minizip/unzip.h>
#include <array>
#include <cstring>
#include <map>
#include <regex>
#include <cstdint>

namespace local_planning {
namespace {
using Element = tinyxml2::XMLElement;

struct Array {
  std::string type;
  std::vector<std::size_t> shape;
  std::vector<unsigned char> data;
};
std::map<std::string,Array> arrays(const std::string & filename)
{
  const auto zip=unzOpen64(filename.c_str());
  if(!zip)throw std::runtime_error("SCAN_NPZ_UNREADABLE: "+filename);
  std::map<std::string,Array> result;
  try {
    int status=unzGoToFirstFile(zip);
    std::size_t total=0;
    while(status==UNZ_OK) {
      unz_file_info64 info{};std::array<char,256> name{};
      if(unzGetCurrentFileInfo64(zip,&info,name.data(),name.size(),nullptr,0,nullptr,0)!=UNZ_OK ||
        info.size_filename>=name.size() || info.uncompressed_size>512ULL*1024*1024 ||
        (total+=info.uncompressed_size)>1024ULL*1024*1024) {
        throw std::runtime_error("SCAN_NPZ_INVALID_MEMBER_SIZE");
      }
      const std::string key(name.data());
      if(key.size()<5 || key.substr(key.size()-4)!=".npy" || key.find('/')!=std::string::npos || result.count(key)) {
        throw std::runtime_error("SCAN_NPZ_INVALID_MEMBER_NAME");
      }
      if(unzOpenCurrentFile(zip)!=UNZ_OK)throw std::runtime_error("SCAN_NPZ_MEMBER_UNREADABLE");
      std::vector<unsigned char> bytes(info.uncompressed_size);std::size_t read=0;
      while(read<bytes.size()) {
        const int n=unzReadCurrentFile(zip,bytes.data()+read,std::min<std::size_t>(bytes.size()-read,1<<20));
        if(n<=0) {unzCloseCurrentFile(zip);throw std::runtime_error("SCAN_NPZ_TRUNCATED");}
        read+=n;
      }
      if(unzCloseCurrentFile(zip)!=UNZ_OK)throw std::runtime_error("SCAN_NPZ_CRC_ERROR");
      if(bytes.size()<10 || std::memcmp(bytes.data(),"\x93NUMPY",6)!=0 || bytes[7]!=0 ||
        (bytes[6]!=1 && bytes[6]!=2))throw std::runtime_error("SCAN_NPY_BAD_FORMAT");
      const std::size_t prefix=bytes[6]==1?10:12;
      if(bytes.size()<prefix)throw std::runtime_error("SCAN_NPY_TRUNCATED_HEADER");
      std::size_t header_size=0;
      for(std::size_t j=8;j<prefix;++j)header_size|=std::size_t(bytes[j])<<(8*(j-8));
      if(header_size>65536 || prefix+header_size>bytes.size())throw std::runtime_error("SCAN_NPY_BAD_HEADER_SIZE");
      const std::string header(reinterpret_cast<const char*>(bytes.data()+prefix),header_size);
      std::smatch match;Array a;
      if(!std::regex_search(header,match,std::regex("'descr'\\s*:\\s*'([^']+)'")))throw std::runtime_error("SCAN_NPY_MISSING_DTYPE");
      a.type=match[1];
      if(!std::regex_search(header,std::regex("'fortran_order'\\s*:\\s*False")))throw std::runtime_error("SCAN_NPY_REQUIRES_C_ORDER");
      if(!std::regex_search(header,match,std::regex("'shape'\\s*:\\s*\\(([^)]*)\\)")))throw std::runtime_error("SCAN_NPY_MISSING_SHAPE");
      std::istringstream dims(match[1]);std::string dim;
      while(std::getline(dims,dim,',')) {
        if(dim.find_first_not_of(" ")==std::string::npos)continue;
        std::istringstream value(dim);std::size_t n;std::string extra;
        if(!(value>>n) || n==0 || n>50000000 || (value>>extra))throw std::runtime_error("SCAN_NPY_BAD_SHAPE");
        a.shape.push_back(n);
      }
      a.data.assign(bytes.begin()+prefix+header_size,bytes.end());
      result.emplace(key,std::move(a));status=unzGoToNextFile(zip);
    }
    if(status!=UNZ_END_OF_LIST_OF_FILE)throw std::runtime_error("SCAN_NPZ_BAD_DIRECTORY");
    unzClose(zip);return result;
  } catch(...) {unzClose(zip);throw;}
}
template<class T> std::vector<T> values(const std::map<std::string,Array> & a,
  const std::string & key,const std::string & type,const std::vector<std::size_t> & shape)
{
  const auto found=a.find(key+".npy");
  if(found==a.end() || found->second.type!=type || found->second.shape!=shape) {
    throw std::runtime_error("SCAN_NPZ_DTYPE_OR_SHAPE: "+key);
  }
  std::size_t count=1;
  for(auto n:shape) {if(count>50000000/n)throw std::runtime_error("SCAN_NPZ_TOO_MANY_CELLS");count*=n;}
  if(found->second.data.size()!=count*sizeof(T))throw std::runtime_error("SCAN_NPZ_PAYLOAD_SIZE: "+key);
  std::vector<T> out(count);std::memcpy(out.data(),found->second.data.data(),out.size()*sizeof(T));return out;
}

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

}  // namespace

void ScanMap::loadNpz(const std::string & filename)
{
  const auto a=arrays(filename);Grid parsed;
  if(values<int32_t>(a,"version","<i4",{})[0]!=1)throw std::runtime_error("SCAN_NPZ_UNSUPPORTED_VERSION");
  parsed.resolution=values<double>(a,"resolution","<f8",{})[0];
  const auto origin=values<double>(a,"origin","<f8",{3});
  parsed.origin=Eigen::Vector3d(origin[0],origin[1],origin[2]);
  if(!parsed.origin.allFinite() || !std::isfinite(parsed.resolution) || parsed.resolution<=0)throw std::runtime_error("SCAN_NPZ_INVALID_GEOMETRY");
  const auto order=values<unsigned char>(a,"axis_order","|u1",{3});
  if(std::string(order.begin(),order.end())!="xyz")throw std::runtime_error("SCAN_NPZ_BAD_AXIS_ORDER");
  for(const auto & key:{"source_sha256","geometry_fingerprint"}) {
    const auto fingerprint=values<unsigned char>(a,key,"|u1",{64});
    for(auto ch:fingerprint)if(!((ch>='0'&&ch<='9')||(ch>='a'&&ch<='f')))throw std::runtime_error("SCAN_NPZ_BAD_FINGERPRINT");
  }
  const auto member=a.find("distance.npy");
  if(member==a.end() || member->second.shape.size()!=3)throw std::runtime_error("SCAN_NPZ_BAD_DISTANCE_SHAPE");
  const auto shape=member->second.shape;parsed.nx=shape[0];parsed.ny=shape[1];parsed.nz=shape[2];
  if(parsed.nx<2||parsed.ny<2||parsed.nz<2)throw std::runtime_error("SCAN_NPZ_GRID_TOO_SMALL");
  parsed.distance=values<float>(a,"distance","<f4",shape);
  const auto occupancy=values<unsigned char>(a,"occupancy","|u1",shape);
  const auto ground=a.find("ground.npy");
  if(ground==a.end() || ground->second.shape.size()!=3 || ground->second.shape[1]!=parsed.nx || ground->second.shape[2]!=parsed.ny)throw std::runtime_error("SCAN_NPZ_BAD_GROUND_SHAPE");
  parsed.layers=ground->second.shape[0];parsed.ground=values<float>(a,"ground","<f4",ground->second.shape);
  parsed.valid=values<unsigned char>(a,"valid","|u1",ground->second.shape);
  const auto layer_z=values<double>(a,"layer_z","<f8",{parsed.layers});
  for(std::size_t i=0;i<layer_z.size();++i)if(!std::isfinite(layer_z[i]) || (i&&layer_z[i]<=layer_z[i-1]))throw std::runtime_error("SCAN_NPZ_INVALID_LAYERS");
  const double margin=values<double>(a,"conservative_margin","<f8",{})[0];
  if(!std::isfinite(margin)||margin<0)throw std::runtime_error("SCAN_NPZ_INVALID_DISTANCE_MARGIN");
  for(std::size_t i=0;i<parsed.distance.size();++i) {
    if(!std::isfinite(parsed.distance[i])||occupancy[i]>1 || (occupancy[i]&&parsed.distance[i]>0))throw std::runtime_error("SCAN_NPZ_INVALID_DISTANCE");
  }
  for(std::size_t i=0;i<parsed.ground.size();++i) {
    if(parsed.valid[i]>1 || (parsed.valid[i]?!std::isfinite(parsed.ground[i]):!std::isnan(parsed.ground[i])))throw std::runtime_error("SCAN_NPZ_INVALID_GROUND");
  }
  for(std::size_t xy=0;xy<parsed.nx*parsed.ny;++xy) {
    bool known=false;
    for(std::size_t layer=0;layer<parsed.layers;++layer)known=known||parsed.valid[layer*parsed.nx*parsed.ny+xy];
    if(!known)for(std::size_t z=0;z<parsed.nz;++z) {
      if(!occupancy[xy*parsed.nz+z])throw std::runtime_error("SCAN_NPZ_UNKNOWN_COLUMN_IS_FREE");
    }
  }
  boxes_.clear();grid_=std::move(parsed);
}

void ScanMap::load(const std::string & filename)
{
  if(filename.size()>=4 && filename.substr(filename.size()-4)==".npz") {loadNpz(filename);return;}
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
  grid_=Grid{};
}

bool ScanMap::height(double x, double y, double ceiling, double depth, double & z) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(ceiling) ||
    !std::isfinite(depth) || depth <= 0) {return false;}
  if(!grid_.distance.empty()) {
    const auto & g=grid_;const double qx=(x-g.origin.x())/g.resolution,qy=(y-g.origin.y())/g.resolution;
    if(qx<0||qy<0||qx>=double(g.nx)||qy>=double(g.ny))return false;
    const std::size_t ix=std::floor(qx),iy=std::floor(qy);
    bool found=false;double best=-std::numeric_limits<double>::infinity();
    for(std::size_t layer=0;layer<g.layers;++layer) {
      const auto i=(layer*g.nx+ix)*g.ny+iy;
      if(!g.valid[i])continue;
      const double h=g.ground[i];
      if(h<=ceiling+1e-9&&h>=ceiling-depth-1e-9&&h>best) {best=h;found=true;}
    }
    if(found)z=best;
    return found;
  }
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


// Signed distance to the union of World OBBs. The nearest box supplies the
// piecewise-smooth gradient used by SCAN's actual-curve collision objective.
double ScanMap::signedDistance(const Eigen::Vector3d & point,Eigen::Vector3d & gradient) const
{
  gradient.setZero();
  if(!grid_.distance.empty()) {
    if(!point.allFinite())throw std::runtime_error("SCAN_INVALID_DISTANCE_QUERY");
    const auto & g=grid_;const Eigen::Vector3d q=(point-g.origin)/g.resolution-Eigen::Vector3d::Constant(.5);
    const std::array<std::size_t,3> dims{g.nx,g.ny,g.nz};
    std::array<std::size_t,3> base;Eigen::Vector3d t;
    for(int j=0;j<3;++j) {
      if(q[j]<0||q[j]>double(dims[j]-1))return -g.resolution;
      base[j]=std::min<std::size_t>(std::floor(q[j]),dims[j]-2);t[j]=q[j]-base[j];
    }
    double result=0;
    for(int x=0;x<2;++x)for(int y=0;y<2;++y)for(int z=0;z<2;++z) {
      const double v=g.distance[((base[0]+x)*g.ny+base[1]+y)*g.nz+base[2]+z];
      const double wx=x?t.x():1-t.x(),wy=y?t.y():1-t.y(),wz=z?t.z():1-t.z();
      result+=v*wx*wy*wz;
      gradient.x()+=v*(x?1:-1)*wy*wz/g.resolution;
      gradient.y()+=v*wx*(y?1:-1)*wz/g.resolution;
      gradient.z()+=v*wx*wy*(z?1:-1)/g.resolution;
    }
    return result;
  }
  if (boxes_.empty() || !point.allFinite()) {throw std::runtime_error("SCAN_INVALID_DISTANCE_QUERY");}
  double best=std::numeric_limits<double>::infinity();
  for (const auto & box:boxes_) {
    const Eigen::Vector3d broad=(point-box.maximum).cwiseMax(box.minimum-point).cwiseMax(0);
    if (broad.squaredNorm()>0 && (best<=0 || broad.squaredNorm()>=best*best)) {continue;}
    const Eigen::Vector3d local=box.rotation.transpose()*(point-box.center);
    const Eigen::Vector3d delta=local.cwiseAbs()-box.half;
    const Eigen::Vector3d outside=delta.cwiseMax(0);
    const double length=outside.norm();
    double value;Eigen::Vector3d direction=Eigen::Vector3d::Zero();
    if (length>1e-12) {
      value=length;
      for (int i=0;i<3;++i) {direction[i]=outside[i]/length*(local[i]>=0?1:-1);}
    } else {
      Eigen::Index axis;value=delta.maxCoeff(&axis);direction[axis]=local[axis]>=0?1:-1;
    }
    if (value<best) {best=value;gradient=box.rotation*direction;}
  }
  return best;
}

double ScanMap::clearance(const Eigen::Vector3d & point) const
{
  if(!grid_.distance.empty()) {Eigen::Vector3d gradient;return point.allFinite()?std::max(0.,signedDistance(point,gradient)):0.;}
  return segmentClearance(point, point);
}

// Exact distance from the executed line segment to each oriented World box.
// Within each interval split by box-face crossings, squared distance is quadratic.
double ScanMap::segmentClearance(const Eigen::Vector3d & from, const Eigen::Vector3d & to) const
{
  if(!grid_.distance.empty()) {
    if(!from.allFinite()||!to.allFinite())return 0;
    const double length=(to-from).norm();
    if(!std::isfinite(length)||length/(grid_.resolution*.5)>1000000)return 0;
    const std::size_t count=std::max<std::size_t>(1,std::ceil(length/(grid_.resolution*.5)));
    if(count>1000000)return 0;
    double best=std::numeric_limits<double>::infinity();Eigen::Vector3d gradient;
    for(std::size_t i=0;i<=count;++i)best=std::min(best,signedDistance(from+(to-from)*(double(i)/count),gradient));
    // Each sampled field value is a lower bound on true geometry distance.
    // True geometry distance is 1-Lipschitz, even if interpolation is not.
    return std::max(0.,best-length/(2*count));
  }
  if (boxes_.empty() || !from.allFinite() || !to.allFinite()) {return 0;}
  double best=std::numeric_limits<double>::infinity();
  const Eigen::Vector3d minimum=from.cwiseMin(to),maximum=from.cwiseMax(to);
  for (const auto & box:boxes_) {
    const Eigen::Vector3d broad=(minimum-box.maximum).cwiseMax(box.minimum-maximum).cwiseMax(0);
    if (broad.norm()>=best) {continue;}
    const Eigen::Vector3d a=box.rotation.transpose()*(from-box.center);
    const Eigen::Vector3d d=box.rotation.transpose()*(to-from);
    std::vector<double> cuts{0,1};
    for (int j=0;j<3;++j) {
      if (std::abs(d[j])<1e-14) {continue;}
      for (double face:{-box.half[j],box.half[j]}) {
        const double t=(face-a[j])/d[j];
        if (t>0 && t<1) {cuts.push_back(t);}
      }
    }
    std::sort(cuts.begin(),cuts.end());
    auto value=[&](double t)->Eigen::Vector3d {return (a+t*d).cwiseAbs().cwiseMax(box.half)-box.half;};
    for (std::size_t k=1;k<cuts.size();++k) {
      const double mid=(cuts[k-1]+cuts[k])/2;
      double numerator=0,denominator=0;
      for (int j=0;j<3;++j) {
        const double q=a[j]+mid*d[j];
        if (std::abs(q)<=box.half[j]) {continue;}
        const double c=a[j]-(q>0?box.half[j]:-box.half[j]);
        numerator+=c*d[j];denominator+=d[j]*d[j];
      }
      const double t=denominator>0?std::clamp(-numerator/denominator,cuts[k-1],cuts[k]):mid;
      best=std::min(best,value(t).norm());
      if (best<=1e-12) {return 0;}
    }
  }
  return best;
}
}  // namespace local_planning
