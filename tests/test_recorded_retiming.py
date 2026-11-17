#!/usr/bin/env python3
"""Exercise native B-spline retiming on the recorded failing trajectories."""
import argparse,json,re,subprocess
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--session',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
src=a.root/'ros2_ws/src/quadruped_navigation'
cpp=r'''#include <bspline_opt/uniform_bspline.h>
#include <iomanip>
#include <cmath>
using scan_planner::UniformBspline;
int main(){int n;double duration;std::cin>>n>>duration;Eigen::MatrixXd points(3,n);for(int i=0;i<n;i++)for(int j=0;j<3;j++)std::cin>>points(j,i);
UniformBspline original(points,3,duration/(n-3)),scaled=original;scaled.setPhysicalLimits(.2,.5,0.);double ratio;bool before=scaled.checkFeasibility(ratio,false);double factor=ratio*1.001;scaled.scaleTime(factor);double after_ratio;bool after=scaled.checkFeasibility(after_ratio,false);
auto velocity=scaled.getDerivative(),acceleration=velocity.getDerivative();auto old_velocity=original.getDerivative();double error=0.,velocity_error=0.,max_v=0.,max_a=0.;
for(int i=0;i<=10000;i++){double t=duration*i/10000.;error=std::max(error,(original.evaluateDeBoorT(t)-scaled.evaluateDeBoorT(t*factor)).norm());velocity_error=std::max(velocity_error,(old_velocity.evaluateDeBoorT(t)/factor-velocity.evaluateDeBoorT(t*factor)).norm());max_v=std::max(max_v,velocity.evaluateDeBoorT(t*factor).norm());max_a=std::max(max_a,acceleration.evaluateDeBoorT(t*factor).norm());}
std::cout<<std::setprecision(16)<<"{\"before_feasible\":"<<before<<",\"after_feasible\":"<<after<<",\"scale\":"<<factor<<",\"geometry_error_m\":"<<error<<",\"velocity_scale_error\":"<<velocity_error<<",\"maximum_speed_mps\":"<<max_v<<",\"maximum_acceleration_mps2\":"<<max_a<<"}\n";
return before||!after||error>1e-9||velocity_error>1e-9||max_v>.2001||max_a>.5001;}
'''
(a.output/'retiming_probe.cpp').write_text(cpp)
subprocess.run(['g++','-O1','-std=c++17','-I/usr/include/eigen3','-I'+str(src/'include'),str(a.output/'retiming_probe.cpp'),str(src/'src/uniform_bspline.cpp'),'-o',str(a.output/'retiming_probe')],check=True)
events=[json.loads(s) for s in (a.session/'events.jsonl').open()];log=(a.session/'navigation.log').read_text();checks=[]
for identifier in [18,19]:
    event=next(v for v in events if v['kind']=='spline' and v['data']['id']==identifier)
    points=np.array(event['data']['points']);duration=float(re.search('Received trajectory '+str(identifier)+', duration ([0-9.]+)s',log)[1])
    data=f'{len(points)} {duration}\n'+'\n'.join(' '.join(map(str,row)) for row in points)+'\n'
    result=subprocess.run([str(a.output/'retiming_probe')],input=data,text=True,capture_output=True,check=True)
    value=json.loads(result.stdout);value['trajectory_id']=identifier;checks.append(value)
(a.output/'verification.json').write_text(json.dumps(dict(result='PASS',scope='native B-spline tests using recorded failures, not an Isaac acceptance',checks=checks),indent=2)+'\n')
print(json.dumps(checks,indent=2))
