import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import BSpline
s=Path(sys.argv[1]);rows=[json.loads(x) for x in (s/'simulation/states.jsonl').open()];ev=[json.loads(x) for x in (s/'events.jsonl').open()]
t=np.array([r['t']-2 for r in rows]);p=np.array([r['pos'] for r in rows]);q=Rotation.from_quat(np.array([r['quat'] for r in rows])[:,[1,2,3,0]]);r=q.as_matrix();cmd=np.array([v['command'] for v in rows]);v=np.array([v['body_velocity'] for v in rows]);angular=np.array([v['angular_velocity'] for v in rows]);b=json.loads((s/'simulation/obstacle_events.json').read_text())[0];apply=b['sim_time_s'];box=b['boxes'][0];center=np.array(box['center_m'][:2]);half=np.array(box['size_m'][:2])/2
if any(abs(x)>1e-9 for x in box.get('velocity_mps',[0,0,0])):raise ValueError('This diagnostic accepts a stationary runtime obstacle only; use the dynamic evaluator for moving obstacles')
c=np.stack([p[:,:2]+.18*r[:,:2,0],p[:,:2]-.18*r[:,:2,0]],axis=1);gap=np.linalg.norm(np.maximum(abs(c-center)-half,0),axis=2)-.25;gap[t<apply]=999;ii,jj=np.unravel_index(gap.argmin(),gap.shape);dt=np.diff(t,append=t[-1]);worldcmd=np.einsum('nij,nj->ni',r[:,:2,:2],cmd[:,:2]);w=(t>=max(apply,t[ii]-4))&(t<=t[ii]);sel=np.flatnonzero(w)
active=(t>apply+1)&(np.linalg.norm(cmd[:,:2],axis=1)>.05)
g=json.loads(Path('/data0/rl/quadruped_nav/runs/d1_full_room_survey_20261109_02/contract.json').read_text())['world_from_odom_initial_placement'];R=np.array(g['rotation']);tr=np.array(g['translation']);routes=[]
for e in ev:
 if e['kind']!='spline' or e['t']<apply:continue
 d=e['data'];cp=np.array(d['points']);k=np.array(d['knots']);degree=d['order']
 if np.ptp(cp,axis=0).max()<1e-5:continue
 curve=BSpline(k,cp,degree);tt=np.linspace(k[degree],k[-degree-1],2001);xy=curve(tt)[:,:2];vel=curve.derivative()(tt)[:,:2];norm=np.linalg.norm(vel,axis=1);heading=vel/np.maximum(norm[:,None],1e-12)
 for z in np.flatnonzero(norm<1e-7):
  near=np.clip(z+(1 if z<len(tt)//2 else -1),0,len(tt)-1);delta=xy[near]-xy[z];heading[z]=delta/(np.linalg.norm(delta)+1e-12)
 xy=xy@R.T+tr;heading=heading@R.T;cent=np.stack([xy+.18*heading,xy-.18*heading],axis=1);gg=np.linalg.norm(np.maximum(abs(cent-center)-half,0),axis=2)-.25
 routes.append(dict(received_t=e['t'],trajectory_id=d['id'],min_nominal_gap_m=float(gg.min())))
out=dict(scope='offline simulator diagnostic; no truth provided to navigation',minimum_actual_gap_m=float(gap[ii,jj]),minimum_time_s=float(t[ii]),actual_pose_xy=p[ii,:2].tolist(),window_start_s=float(t[sel[0]]),window_end_s=float(t[sel[-1]]),command_integral_world_xy_m=np.sum(worldcmd[w]*dt[w,None],axis=0).tolist(),actual_displacement_world_xy_m=(p[sel[-1],:2]-p[sel[0],:2]).tolist(),active_command_samples=int(active.sum()),body_velocity_tracking_error_mean_mps=float(np.linalg.norm(v[active,:2]-cmd[active,:2],axis=1).mean()),body_velocity_tracking_error_p95_mps=float(np.percentile(np.linalg.norm(v[active,:2]-cmd[active,:2],axis=1),95)),nominal_trajectories=routes,minimum_nominal_gap_m=min((x['min_nominal_gap_m'] for x in routes),default=None),limitation='Offline nominal tangent footprint and measured velocity diagnostic; cannot prove all faults belong exclusively to the gait simulator')
(s/'tracking_diagnosis.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
