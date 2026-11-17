import json,collections
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
run=Path('/data0/rl/quadruped_nav/runs/navigation_core_isaac_demo_20261112_01');out=Path('/data0/rl/quadruped_nav/runs/moving_localization_review_20261113_01');out.mkdir(exist_ok=False)
e=[json.loads(x) for x in (run/'events.jsonl').open()];s=[json.loads(x) for x in (run/'simulation/states.jsonl').open()];p=[x for x in e if x['kind']=='body_pose'];status=[x for x in e if x['kind']=='localization_status'];g=json.loads(Path('/data0/rl/quadruped_nav/runs/d1_full_room_survey_20261109_02/contract.json').read_text())['world_from_odom_initial_placement']
t=np.array([x['t']-2 for x in s]);xyz=np.array([x['pos'] for x in s]);v=np.array([x['body_velocity'] for x in s]);yaw=np.unwrap(Rotation.from_quat(np.array([x['quat'] for x in s])[:,[1,2,3,0]]).as_euler('xyz')[:,2]);R=np.array(g['rotation']);g_yaw=np.arctan2(R[1,0],R[0,0]);truth_xy=(xyz[:,:2]-g['translation'])@R
stat=lambda x:dict(count=len(x),mean=float(np.mean(x)),median=float(np.median(x)),p95=float(np.percentile(x,95)),max=float(np.max(x))) if len(x) else None
sensitivity={};series=[]
for lag in [0,.05,.1,.2]:
 rows=[]
 for x in p:
  stamp=x['t']-lag
  if not t[0]<=stamp<=t[-1]:continue
  xy=np.array([np.interp(stamp,t,truth_xy[:,i]) for i in range(2)]);truth_yaw=np.interp(stamp,t,yaw)-g_yaw;pose=x['data'];d=np.array(pose[:2])-xy;dyaw=abs((pose[3]-truth_yaw+np.pi)%(2*np.pi)-np.pi)*180/np.pi;speed=np.interp(stamp,t,np.linalg.norm(v[:,:2],axis=1))
  rows.append(dict(t=x['t'],xy_error_m=float(np.linalg.norm(d)),yaw_error_deg=float(dyaw),speed_mps=float(speed),error_xy=d.tolist()))
 move=[r for r in rows if r['speed_mps']>.05]
 sensitivity[str(lag)]=dict(all_xy_error_m=stat([r['xy_error_m'] for r in rows]),moving_xy_error_m=stat([r['xy_error_m'] for r in move]),moving_yaw_error_deg=stat([r['yaw_error_deg'] for r in move]),first_10s_xy_error_m=stat([r['xy_error_m'] for r in rows if r['t']<17]),last_10s_xy_error_m=stat([r['xy_error_m'] for r in rows if r['t']>146]),worst=max(rows,key=lambda x:x['xy_error_m']))
 if lag==0:series=rows
unique={}
for x in status:
 q=x['data'].get('quality')
 if q and q.get('scan_ns') is not None:unique[q['scan_ns']]=q
last=status[-1]['data'];active=[x for x in status if x['t']>=8]
r=dict(scope='offline review of existing 156 s / 18.64 m simulation; not a new run or hardware test',timing_limitation='body_pose header stamps not recorded; callback-clock estimates only. Lag variants are sensitivity checks, not fitted time alignment. No precise localization accuracy certification.',fixed_world_map_gauge=g,no_fitted_alignment=True,pose_samples=len(p),status_phase_counts=dict(collections.Counter(x['data']['phase'] for x in active)),last_counts={k:last[k] for k in ['accepted_matches','failed_matches','discarded_results','rejected_inputs']},unique_registration_results=len(unique),registration_fitness=stat([q['fitness'] for q in unique.values()]),registration_rmse_m=stat([q['rmse_m'] for q in unique.values() if q.get('rmse_m') is not None]),time_lag_sensitivity=sensitivity,standalone_fastlio_drift_measurable=False)
(out/'review.json').write_text(json.dumps(r,indent=2)+'\n');(out/'callback_time_errors.json').write_text(json.dumps(series)+'\n');print(json.dumps(r,indent=2))
