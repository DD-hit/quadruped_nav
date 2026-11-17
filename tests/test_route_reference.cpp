#include <cassert>
#include <plan_manage/route_reference.h>
using namespace scan_planner;
int main() {
  using V=Eigen::Vector3d;
  std::vector<V> route{V(0,0,0),V(0,2,0),V(2,2,0),V(2,0,0)};
  auto p=sampleRemainingRoute(route,V(0,0,0),V(2,0,0),.1);
  assert(p.size()>=60); assert(p.front().norm()<1e-9); assert((p.back()-V(2,0,0)).norm()<1e-9);
  for(auto q:p) assert((q-nearestRoutePoint(route,q)).norm()<1e-9);
  auto remaining=sampleRemainingRoute(route,V(.5,2,0),V(2,0,0),.1);
  for(auto q:remaining) assert(q.x()>=.5-1e-9);
  assert(sampleRemainingRoute({},V(0,0,0),V(1,0,0),.1).empty());
  assert(sampleRemainingRoute(route,V(0,0,0),V(2,0,0),0).empty());
  route.insert(route.begin()+1,route.front());
  auto duplicate=sampleRemainingRoute(route,V(0,0,0),V(2,0,0),.1);
  assert(duplicate.size()==p.size());
  for(auto q:duplicate) assert(q.allFinite());
  std::vector<V> ramp{V(0,0,0),V(1,0,.2),V(2,0,1)};
  auto stairs=sampleRemainingRoute(ramp,ramp.front(),ramp.back(),.1);
  assert(stairs.back().z()==1);for(auto q:stairs) assert((q-nearestRoutePoint(ramp,q)).norm()<1e-9);
}
