import json,hashlib
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Polygon
r=Path('/data0/rl/quadruped_nav/runs/refined_map_closed_loop_20261117_01');e=[json.loads(x) for x in (r/'events.jsonl').open()];d=json.loads((r/'failure_geometry.json').read_text());report=json.loads((r/'independent_evaluation.json').read_text());g=json.loads(Path('/data0/rl/quadruped_nav/runs/d1_full_room_survey_20261109_02/contract.json').read_text())['world_from_odom_initial_placement'];R=np.array(g['rotation']);tr=np.array(g['translation']);seed=np.loadtxt(r/'native_seed.xyz');delta=(seed[:,:2]@R.T+tr)-[-1.5,-.8];d['controlled_native_seed_probe']=dict(Isaac_used=False,ROS_used=False,input='Recorded initial replan start/velocity, zero start acceleration; not an exact replay of later failed trajectory',min_center_gap_to_physical_pillar_m=float(np.min(np.linalg.norm(np.maximum(abs(delta)-.2,0),axis=1))-.25),role='Shows endpoint polynomial seed may cross obstacle despite safe jie route; optimizer still runs afterward, so not proof final trajectory is unchecked.')
source=Path('/home/rl/workspace/quadruped_nav/ros2_ws/src/quadruped_navigation/src/planner_manager.cpp');d['planner_source_sha256']=hashlib.sha256(source.read_bytes()).hexdigest();(r/'failure_geometry.json').write_text(json.dumps(d,indent=2))
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']=font.get_name();plt.rcParams['axes.unicode_minus']=False
fig,ax=plt.subplots(figsize=(10,7));path=np.array([x['data'] for x in e if x['kind']=='global_path' and x['data']][-1]);ax.plot(path[:,0],path[:,1],c='#2783be',lw=2,label='全局路径')
for x in e:
 if x['kind']!='spline' or x['data']['id'] not in (11,12,13):continue
 v=x['data'];k=np.array(v['knots']);curve=BSpline(k,np.array(v['points']),v['order']);q=curve(np.linspace(0,k[-v['order']-1],600));ax.plot(q[:,0],q[:,1],lw=1.8,label='局部轨迹 '+str(v['id']))
xy=np.array([x['data'][:2] for x in e if x['kind']=='body_pose' and x['t']>=95.665]);ax.plot(xy[:,0],xy[:,1],c='#3e3f43',lw=2,label='定位轨迹');ax.scatter(*xy[-1],c='#c44f4f',s=75,marker='x',label='急停位置')
corners=(np.array([[-1.7,-1.],[-1.3,-1.],[-1.3,-.6],[-1.7,-.6]])-tr)@R;ax.add_patch(Polygon(corners,color='#aaa49e',label='柱体（固定场景参考）'));ax.set(xlim=(-3.3,-.35),ylim=(-1.15,.5),xlabel='地图 X / m',ylabel='地图 Y / m',title='失败路段：全局路径与局部轨迹的绕柱余量');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(prop=font);fig.tight_layout();fig.savefig(r/'绕柱失败分析.png',dpi=160);plt.close(fig)
print(json.dumps(dict(arrivals=len(report['arrivals']),initializations=report['initializations'],path_m=report['physics_path_length_m'],native_probe=d['controlled_native_seed_probe']),ensure_ascii=False,indent=2))
