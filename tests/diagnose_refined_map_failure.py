import json,math
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline
from scipy.spatial import cKDTree
r=Path('/data0/rl/quadruped_nav/runs/refined_map_closed_loop_20261117_01');e=[json.loads(x) for x in (r/'events.jsonl').open()];g=json.loads(Path('/data0/rl/quadruped_nav/runs/d1_full_room_survey_20261109_02/contract.json').read_text())['world_from_odom_initial_placement'];R=np.array(g['rotation']);tr=np.array(g['translation']);center=np.array([-1.5,-.8]);half=np.array([.2,.2])
def gap(p,yaw=None):
 if yaw is None:
  v=np.gradient(p[:,:2],axis=0);yaw=np.arctan2(v[:,1],v[:,0])
 f=np.c_[np.cos(yaw),np.sin(yaw)];centers=np.concatenate([p[:,:2]+.18*f,p[:,:2]-.18*f])@R.T+tr
 return float(np.min(np.linalg.norm(np.maximum(abs(centers-center)-half,0),axis=1))-.25)
path=np.array([x['data'] for x in e if x['kind']=='global_path' and x['data']][-1]);dense=np.concatenate([np.linspace(a,b,20) for a,b in zip(path[:-1],path[1:])]);print('GLOBAL GAP',gap(dense));rows=[]
for x in e:
 if x['kind']!='spline' or x['t']<95:continue
 d=x['data'];knots=np.array(d['knots']);pts=np.array(d['points']);curve=BSpline(knots,pts,d['order']);dur=knots[-d['order']-1];t=np.linspace(0,dur,1000);points=curve(t);vel=curve.derivative()(t);yaw=np.arctan2(vel[:,1],vel[:,0]);dis=cKDTree(dense[:,:2]).query(points[:,:2])[0];row=dict(id=d['id'],published=x['t'],start=d['start_time_s'],duration=dur,min_native_gap_m=gap(points,yaw),max_distance_from_global_path_m=float(dis.max()),peak_speed=float(np.linalg.norm(vel[:,:2],axis=1).max()));rows.append(row)
print(rows)
poses=np.array([x['data'][:2] for x in e if x['kind']=='body_pose' and x['t']>95]);print('actual global path deviation',np.percentile(cKDTree(dense[:,:2]).query(poses)[0],[50,95,100]));(r/'failure_geometry.json').write_text(json.dumps(dict(global_min_native_gap_m=gap(dense),splines=rows,estimated_body_global_path_deviation_m=np.percentile(cKDTree(dense[:,:2]).query(poses)[0],[50,95,100]).tolist()),indent=2))
