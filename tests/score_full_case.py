#!/usr/bin/env python3
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
p=argparse.ArgumentParser();p.add_argument('case',type=Path);a=p.parse_args();out=a.case
r=json.loads((out/'result.json').read_text());poses=json.loads((out/'poses.json').read_text());states=[json.loads(s) for s in (out/'simulation/states.jsonl').read_text().splitlines()];g=json.loads(Path('/data0/rl/quadruped_nav/runs/d1_full_room_survey_20261109_02/contract.json').read_text())['world_from_odom_initial_placement'];stand=json.loads(Path('/home/rl/workspace/quadruped_nav/ros2_ws/src/quadruped_sim/config/d1_standby_initial_state.json').read_text())
wm=np.eye(4);wm[:2,:2]=g['rotation'];wm[:3,3]=[*g['translation'],stand['base_position_world_m'][2]];mw=np.linalg.inv(wm)
t=np.array([s['t'] for s in states]);xyz=np.array([s['pos'] for s in states]);quats=np.array([s['quat'] for s in states]);slerp=Slerp(t,Rotation.from_quat(quats[:,[1,2,3,0]]));rows=[]
for p in poses:
 stamp=p['stamp']+2.
 if not t[0]<=stamp<=t[-1]:continue
 wt=np.eye(4);wt[:3,3]=[np.interp(stamp,t,xyz[:,k]) for k in range(3)];wt[:3,:3]=slerp(stamp).as_matrix();truth=mw@wt
 est=np.eye(4);est[:3,3]=p['position'];est[:3,:3]=Rotation.from_quat(p['quat_xyzw']).as_matrix()
 rows.append(dict(stamp=p['stamp'],position_error_m=float(np.linalg.norm(est[:3,3]-truth[:3,3])),rotation_error_deg=float(Rotation.from_matrix(est[:3,:3]@truth[:3,:3].T).magnitude()*180/np.pi),yaw_error_deg=float(abs((np.degrees(np.arctan2(est[1,0],est[0,0])-np.arctan2(truth[1,0],truth[0,0]))+180)%360-180)),truth_position=truth[:3,3].tolist(),estimated_position=p['position']))
r['truth_comparison_samples']=len(rows)
if rows:
 r['first_pose_errors']={k:rows[0][k] for k in ['position_error_m','rotation_error_deg','yaw_error_deg']}
 for k in ['position_error_m','rotation_error_deg','yaw_error_deg']:r[k]=dict(zip(['median','p95','max'],map(float,np.percentile([x[k] for x in rows],[50,95,100]))))
 r['within_test_limits']=r['result']=='TRACKING_OBSERVED' and r['position_error_m']['p95']<=.18 and r['rotation_error_deg']['p95']<=5
 r['first_pose_within_limits']=rows[0]['position_error_m']<=.18 and rows[0]['rotation_error_deg']<=5
(out/'scored.json').write_text(json.dumps(r,indent=2)+'\n');(out/'truth_errors.json').write_text(json.dumps(rows)+'\n');print(json.dumps(r,indent=2))
