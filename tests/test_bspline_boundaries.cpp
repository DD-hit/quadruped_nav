#include <bspline_opt/uniform_bspline.h>
#include <cassert>
using namespace scan_planner;
int main(){
  using V=Eigen::Vector3d;
  for(int k: {4,8,31}) for(double dt:{.15,.6,1.8}) {
    std::vector<V> p;
    for(int i=0;i<k;++i) p.emplace_back(.1*i,.6*std::sin(i*.7),.03*i);
    for(bool rest:{true,false}){
      std::vector<V> d{rest?V::Zero().eval():V(.12,-.04,.02),V(-.03,.06,0),V(.02,.01,-.01),V(-.01,.02,0)};
      Eigen::MatrixXd q;UniformBspline::parameterizeToBspline(dt,p,d,q);
      UniformBspline s(q,3,dt);auto v=s.getDerivative();auto a=v.getDerivative();double T=s.getTimeSum();
      assert((s.evaluateDeBoorT(0)-p.front()).norm()<1e-9);
      assert((s.evaluateDeBoorT(T)-p.back()).norm()<1e-9);
      assert((v.evaluateDeBoorT(0)-d[0]).norm()<1e-9);
      assert((v.evaluateDeBoorT(T)-d[1]).norm()<1e-9);
      assert((a.evaluateDeBoorT(0)-d[2]).norm()<1e-8);
      assert((a.evaluateDeBoorT(T)-d[3]).norm()<1e-8);
      assert(q.allFinite());
    }
  }
  bool rejected=false;Eigen::MatrixXd q;
  try{UniformBspline::parameterizeToBspline(0,std::vector<V>(8,V::Zero()),std::vector<V>(4,V::Zero()),q);}catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);std::cout<<"PASS: exact spline boundary states across 18 curved/short route cases and invalid interval\n";
}
