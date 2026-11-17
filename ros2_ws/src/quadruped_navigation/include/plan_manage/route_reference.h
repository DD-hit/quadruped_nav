#ifndef QUADRUPED_ROUTE_REFERENCE_H
#define QUADRUPED_ROUTE_REFERENCE_H
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
namespace scan_planner {
inline Eigen::Vector3d nearestRoutePoint(const std::vector<Eigen::Vector3d>& route,
                                       const Eigen::Vector3d& point, double* arc=nullptr) {
  double best=std::numeric_limits<double>::infinity(), cumulative=0., best_arc=0.;
  Eigen::Vector3d nearest=point;
  for(size_t i=1;i<route.size();++i) {
    const Eigen::Vector3d v=route[i]-route[i-1]; const double length=v.norm();
    if(length<1e-9) continue;
    double f=std::clamp((point-route[i-1]).dot(v)/(length*length),0.,1.);
    Eigen::Vector3d q=route[i-1]+f*v; double d=(q-point).squaredNorm();
    if(d<best) {best=d; nearest=q; best_arc=cumulative+f*length;}
    cumulative+=length;
  }
  if(arc) *arc=best_arc;
  return nearest;
}
inline std::vector<Eigen::Vector3d> sampleRemainingRoute(
    const std::vector<Eigen::Vector3d>& route,const Eigen::Vector3d& start,
    const Eigen::Vector3d& target,double spacing) {
  if(route.size()<2 || spacing<=0.) return {};
  double first,last; nearestRoutePoint(route,start,&first); nearestRoutePoint(route,target,&last);
  if(last<=first+1e-8) return {};
  std::vector<Eigen::Vector3d> trimmed{start}; double arc=0.;
  for(size_t i=1;i<route.size();++i) {
    arc+=(route[i]-route[i-1]).norm();
    if(arc>first+1e-7 && arc<last-1e-7 && (route[i]-trimmed.back()).norm()>1e-7) trimmed.push_back(route[i]);
  }
  if((target-trimmed.back()).norm()>1e-7) trimmed.push_back(target);
  std::vector<double> lengths{0.};
  for(size_t i=1;i<trimmed.size();++i) lengths.push_back(lengths.back()+(trimmed[i]-trimmed[i-1]).norm());
  if(lengths.back()<1e-8) return {};
  const size_t count=std::max<size_t>(8,static_cast<size_t>(std::ceil(lengths.back()/spacing))+1);
  std::vector<Eigen::Vector3d> points; size_t segment=0;
  for(size_t i=0;i<count;++i) {
    double s=lengths.back()*i/(count-1);
    while(segment+2<lengths.size() && lengths[segment+1]<s) ++segment;
    double f=(s-lengths[segment])/(lengths[segment+1]-lengths[segment]);
    points.push_back((1.-f)*trimmed[segment]+f*trimmed[segment+1]);
  }
  points.front()=start; points.back()=target; return points;
}
}
#endif
