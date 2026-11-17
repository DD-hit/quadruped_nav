#!/usr/bin/env python3
"""Offline failure evidence, including a native reference-generator probe without Isaac/ROS."""
import argparse,json,math,subprocess,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import BSpline
import re
import yaml
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--session',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
s=a.session;out=a.output;src=a.root/'ros2_ws/src/quadruped_navigation'
events=[json.loads(v) for v in (s/'events.jsonl').open()];rows=[json.loads(v) for v in (s/'simulation/states.jsonl').open()]
source=(src/'src/planner_manager.cpp').read_text()
start=source.index('  bool SCANPlannerManager::planGlobalTraj(');end=source.index('  bool SCANPlannerManager::refineTrajAlgo(',start)
function=source[start:end].replace('SCANPlannerManager::','ProbeManager::')
cpp='''#include <iostream>
#include <algorithm>
#include <traj_utils/polynomial_traj.h>
using namespace std;
struct Node {double now(){return 0.;}};
struct Data {PolynomialTraj trajectory;void setGlobalTraj(PolynomialTraj t,double){trajectory=t;trajectory.init();}};
struct ProbeManager {struct Params{double max_vel_=.2;} pp_;Node node;Node* node_=&node;Data global_data_;
bool planGlobalTraj(const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&);
};
'''+function+'''
int main(){ProbeManager p;Eigen::Vector3d start(-2.6,-.741,-.0134),vel(.239,.221,.0000712),target(-.05,.05,.00431),zero=Eigen::Vector3d::Zero();p.planGlobalTraj(start,vel,zero,target,zero,zero);auto& t=p.global_data_.trajectory;double duration=t.getTimeSum();for(int i=0;i<=1000;i++){auto point=t.evaluate(duration*i/1000.);cout<<point.transpose()<<"\\n";}}
'''
(out/'native_reference_probe.cpp').write_text(cpp)
subprocess.run(['g++','-O1','-std=c++17','-I/usr/include/eigen3','-I'+str(src/'include'),str(out/'native_reference_probe.cpp'),str(src/'src/polynomial_traj.cpp'),'-o',str(out/'native_reference_probe')],check=True)
with (out/'regenerated_reference.xyz').open('w') as f:subprocess.run([str(out/'native_reference_probe')],stdout=f,check=True)
regenerated=np.loadtxt(out/'regenerated_reference.xyz')
path=np.array([e['data'] for e in events if e['kind']=='global_path' and e['data']][-1])
np.savetxt(out/'jie_global_path.xyz',path)
gauge=json.loads((Path('/data0/rl/quadruped_nav/runs/d1_full_room_survey_20261109_02')/'contract.json').read_text())['world_from_odom_initial_placement'];R=np.array(gauge['rotation']);tr=np.array(gauge['translation'])
ob=yaml.safe_load((s/'simulation/scene_config.yaml').read_text())['room']['pillar'];center=np.array(ob['center'][:2]);half=np.array(ob['size'][:2])/2
def gap(points):
    dense=np.concatenate([np.linspace(p,q,max(2,int(np.linalg.norm(q-p)/.005))) for p,q in zip(points[:-1],points[1:])])
    direction=np.gradient(dense[:,:2],axis=0);direction/=np.maximum(np.linalg.norm(direction,axis=1)[:,None],1e-12)
    centers=np.concatenate([dense[:,:2]+.18*direction,dense[:,:2]-.18*direction])@R.T+tr
    return float((np.linalg.norm(np.maximum(abs(centers-center)-half,0.),axis=1)-.25).min())
physical_time=np.array([r['t']-2 for r in rows]);physical_pos=np.array([r['pos'] for r in rows]);physical_q=np.array([r['quat'] for r in rows]);physical_rot=Rotation.from_quat(physical_q[:,[1,2,3,0]])
window=[e for e in events if e['kind']=='localization_status' and 117<=e['t']<=124.4]
samples=[]
for t in [118,120,121,122,123,124]:
    i=int(np.argmin(abs(physical_time-t)));r=rows[i]
    est=min((e for e in events if e['kind']=='body_pose'),key=lambda e:abs(e['t']-t))
    cmd=min((e for e in events if e['kind']=='cmd_vel'),key=lambda e:abs(e['t']-t))
    world_velocity=physical_rot[i].apply(r['body_velocity'])[:2]@R
    command_world=Rotation.from_euler('z',est['data'][3]).apply([*cmd['data'][:2],0])[:2]
    truth_map=(physical_pos[i,:2]-tr)@R
    samples.append(dict(t=t,localized_pose=est['data'],physical_pose_in_map=truth_map.tolist(),pose_xy_error_m=float(np.linalg.norm(np.array(est['data'][:2])-truth_map)),command_body=cmd['data'],command_speed_mps=float(np.linalg.norm(cmd['data'][:2])),actual_body_speed_mps=float(np.linalg.norm(r['body_velocity'][:2])),command_map_xy=command_world.tolist(),actual_map_xy=world_velocity.tolist(),instantaneous_map_velocity_error_mps=float(np.linalg.norm(command_world-world_velocity))))
report=dict(scope='offline root-cause diagnosis; no production code changed',
    native_reference_probe=dict(Isaac_used=False,ROS_used=False,native_generator_sha256=hashlib.sha256(source[start:end].encode()).hexdigest(),input_source='recorded replan log at ROS t approximately 121.375; start acceleration set to zero for controlled probe, not exact failed trajectory reconstruction',jie_path_min_native_cylinder_gap_to_pillar_m=gap(path),regenerated_reference_min_native_cylinder_gap_to_pillar_m=gap(regenerated),note='Reference trajectory is not the final collision-checked local B-spline'),
    localization_window=dict(phases=sorted(set(e['data']['phase'] for e in window)),fitness_min=min(e['data']['quality']['fitness'] for e in window),rmse_max_m=max(e['data']['quality']['rmse_m'] for e in window)),motion_samples=samples)
log=(s/'navigation.log').read_text()
v=next(e for e in events if e['kind']=='spline' and e['data']['id']==18)
pts=np.array(v['data']['points']);duration=float(re.search('Received trajectory 18, duration ([0-9.]+)s',log)[1]);dt=duration/(len(pts)-3)
curve=BSpline(np.arange(-3,len(pts)+1)*dt,pts,3)
t=np.linspace(0,duration,10001);velocity=curve.derivative()(t);speed=np.linalg.norm(velocity[:,:2],axis=1)
failure_t=124.355;estimate=min((e for e in events if e['kind']=='body_pose'),key=lambda e:abs(e['t']-failure_t))
report['planning_control_consistency']=dict(trajectory_id=18,uniform_knot_interval_s=dt,logged_duration_s=duration,nominal_planner_speed_mps=.2,planner_component_limit_with_tolerance_mps=.3001,final_norm_check_limit_mps=1.2,controller_translation_norm_limit_mps=.3,trajectory_initial_speed_mps=float(speed[0]),trajectory_peak_speed_mps=float(speed.max()),trajectory_peak_abs_components_mps=np.max(abs(velocity),axis=0).tolist(),fraction_trajectory_above_controller_norm_cap=float(np.mean(speed>.3)),tracking_error_at_failure_m=float(np.linalg.norm(curve(failure_t-v['t'])[:2]-estimate['data'][:2])),knot_reconstruction='Uniform knots verified from native constructor; interval inferred from control count and logged duration. Not a full saved Bspline message.')
(out/'diagnosis.json').write_text(json.dumps(report,indent=2)+'\n')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Polygon
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
fig,axes=plt.subplots(1,2,figsize=(14,5),dpi=150)
ax=axes[0];ax.plot(path[:,0],path[:,1],label='jie 全局路线',color='#3388cc')
poly=curve(np.linspace(0,duration,500));ax.plot(poly[:,0],poly[:,1],label='局部轨迹 18',color='#e69f00')
body=np.array([e['data'][:2] for e in events if e['kind']=='body_pose' and 108<=e['t']<=124.36]);ax.plot(body[:,0],body[:,1],color='#c93c3c',label='实际定位轨迹')
corners=(np.array([center+[-half[0],-half[1]],center+[half[0],-half[1]],center+[half[0],half[1]],center+[-half[0],half[1]]])-tr)@R
ax.add_patch(Polygon(corners,color='#777777',label='柱体几何（映射到地图坐标）'))
ax.scatter(*estimate['data'][:2],marker='x',color='red',s=65,label='急停位置')
ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('x / m');ax.set_ylabel('y / m');ax.set_title('最后一段：路线、局部轨迹与定位',fontproperties=font);ax.legend(prop=font,fontsize=9)
ax=axes[1];window=np.linspace(0,2.98,200);ax.plot(window+v['t'],np.linalg.norm(curve.derivative()(window)[:,:2],axis=1),color='#e69f00',label='参考轨迹要求速度')
cmds=[e for e in events if e['kind']=='cmd_vel' and v['t']<=e['t']<=failure_t];ax.plot([e['t'] for e in cmds],[np.linalg.norm(e['data'][:2]) for e in cmds],color='#3388cc',label='控制器输出速度')
selected=[r for r in rows if v['t']<=r['t']-2<=failure_t];tt=np.array([r['t']-2 for r in selected]);vv=np.array([np.linalg.norm(r['body_velocity'][:2]) for r in selected]);ax.plot(tt[::20],vv[::20],color='#555555',alpha=.7,label='仿真实际速度（离线核对）')
ax.axhline(.3,color='#c93c3c',ls='--',label='控制器上限 0.3 m/s');ax.set_ylim(0,.4);ax.grid(alpha=.2);ax.set_xlabel('ROS time / s');ax.set_ylabel('m/s');ax.set_title('参考轨迹超过控制器限速',fontproperties=font);ax.legend(prop=font)
fig.tight_layout();fig.savefig(out/'failure_analysis.png');plt.close(fig)
print(json.dumps(report,indent=2))
