#!/usr/bin/env python3
"""Independent dynamic-obstacle evaluation; simulation geometry is never planner input."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
import argparse,json,math
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('session',type=Path);a=p.parse_args();s=a.session
core=json.loads((s/'verification.json').read_text());sim=json.loads((s/'simulation/verification.json').read_text());rows=[json.loads(v) for v in (s/'simulation/states.jsonl').open()];events=[json.loads(v) for v in (s/'events.jsonl').open()];obevents=json.loads((s/'simulation/obstacle_events.json').read_text());scene=yaml.safe_load((s/'simulation/scene_config.yaml').read_text())['room'];cfg=yaml.safe_load((s/'navigation.yaml').read_text())['/navigation/local_planner']['ros__parameters'];g=json.loads(Path('/data0/rl/quadruped_nav/runs/d1_full_room_survey_20261109_02/contract.json').read_text())['world_from_odom_initial_placement'];R=np.array(g['rotation']);tr=np.array(g['translation']);time=np.array([r['t']-2 for r in rows]);pos=np.array([r['pos'] for r in rows]);rot=Rotation.from_quat(np.array([r['quat'] for r in rows])[:,[1,2,3,0]]);forward=rot.as_matrix()[:,:2,0];radius=cfg['grid_map.double_cylinder_radius'];offset=cfg['grid_map.double_cylinder_offset'];centers=np.stack([pos[:,:2]+offset*forward,pos[:,:2]-offset*forward],axis=1)
def gap(c,half,mask):
 return float(np.min(np.linalg.norm(np.maximum(abs(centers[mask]-c[:,None,:])-half,0),axis=2)-radius))
static={}
for name,b in scene.items():
 if name in ('floor','ceiling'):continue
 static[name]=gap(np.tile(b['center'][:2],(len(time),1)),np.array(b['size_m'][:2])/2 if 'size_m' in b else np.array(b['size'][:2])/2,np.ones(len(time),bool))
motion_file=s/'simulation/obstacle_motion.jsonl'
motion=[json.loads(x) for x in motion_file.open()] if motion_file.exists() else []
dynamic=[];applied=next(e for e in obevents if e['boxes']);apply_t=applied['sim_time_s'];clear=[e for e in obevents if e['sim_time_s']>apply_t and not e['boxes']];clear_t=clear[0]['sim_time_s'] if clear else float(time[-1]+1)
for b in applied['boxes']:
 mask=(time>=apply_t)&(time<clear_t);elapsed=np.clip(time[mask]-apply_t,0,b.get('motion_duration_s',0));c=np.array(b['center_m'][:2])+elapsed[:,None]*np.array(b.get('velocity_mps',[0,0,0])[:2])
 if motion:
  records=[(m['sim_time_s'],box['center_m'][:2]) for m in motion for box in m['boxes'] if box['id']==b['id']]
  if records:
   mt=np.array([apply_t]+[v[0] for v in records]);mc=np.array([b['center_m'][:2]]+[v[1] for v in records]);idx=np.searchsorted(mt,time[mask],side='right')-1;c=mc[np.maximum(0,idx)]
 dynamic.append(dict(id=b['id'],minimum_gap_m=gap(c,np.array(b['size_m'][:2])/2,mask),center_motion_m=float(np.linalg.norm(c[-1]-c[0]))))
hits=0;scans=0
for line in (s/'simulation/lidar_scans.jsonl').open():
 d=json.loads(line);v=sum(d.get('scene_obstacle_hits',{}).values());hits+=v;scans+=int(v>0)
truth=(pos[:,:2]-tr)@R;active=(time>=apply_t)&(time<clear_t);motion_response=float(np.max(abs(truth[active,1]))) if np.any(active) else 0.
goal_result_t=next((e['t'] for e in reversed(events) if e['kind']=='result'),float(time[-1]))
commands=[e for e in events if np.linalg.norm([np.interp(e['t'],time,truth[:,0])-3.,np.interp(e['t'],time,truth[:,1])])>.5 and e['t']<goal_result_t-.1 and e['kind']=='cmd_vel' and e['t']>=apply_t+.5 and e['t']<clear_t];stopped=[];start=None
for e in commands:
 if np.linalg.norm(e['data'] if core.get('scenario',next(e['data']['scenario'] for e in events if e['kind']=='obstacle_applied'))=='blocked_wall' else e['data'][:2])<.01:
  if start is None:start=e['t']
 else:
  if start is not None:stopped.append(e['t']-start)
  start=None
if start is not None and commands:stopped.append(commands[-1]['t']-start)
stop_duration=max(stopped,default=0.);checks=dict(action_succeeded=core['result']=='PASS',no_fall=sim['failure'] in (None,'user_exit'),no_teleport=not sim['base_teleported_after_initialization'],no_rejected_commands=sim['cmd_vel_rejected']==0,real_raycast_obstacle_hits=hits>20,dynamic_native_footprint_clearance=all(b['minimum_gap_m']>=0 for b in dynamic),static_native_footprint_clearance=min(static.values())>=0,obstacle_response_observed=motion_response>.25 or stop_duration>.5)
scenario=core.get('scenario',next(e['data']['scenario'] for e in events if e['kind']=='obstacle_applied'))
if scenario=='moving_box':checks['box_really_moved']=any(b['center_motion_m']>1 for b in dynamic)
if scenario=='blocked_wall':
 checks['wall_removed']=bool(clear);checks['same_goal_resumed_after_removal']=bool(clear) and core['result']=='PASS';checks['stopped_while_blocked']=stop_duration>=1.
standstill={}
if scenario=='blocked_wall' and clear:
 window=(time>=clear_t-.5)&(time<clear_t)
 speeds=np.linalg.norm(np.array([r['body_velocity'][:2] for r in rows])[window],axis=1)
 yaw_speeds=np.abs(np.array([r['angular_velocity'][2] for r in rows])[window])
 standstill=dict(window_s=.5,samples=int(window.sum()),translation_p95_mps=float(np.percentile(speeds,95)) if len(speeds) else None,yaw_p95_radps=float(np.percentile(yaw_speeds,95)) if len(yaw_speeds) else None)
 standstill['below_reference_threshold']=bool(len(speeds) and standstill['translation_p95_mps']<.03 and standstill['yaw_p95_radps']<.05)
 standstill['scope']='Simulator execution observation, separate from navigation zero-command acceptance; user previously deferred gait stop drift'
result=dict(result='PASS' if all(checks.values()) else 'FAIL',scenario=scenario,checks={k:bool(v) for k,v in checks.items()},standstill_before_removal=standstill,dynamic_clearance=dynamic,static_clearance_m=static,raycast_hits=hits,scans_with_runtime_obstacle_hits=scans,longest_zero_translation_s=stop_duration,maximum_lateral_response_m=motion_response,apply_time_s=apply_t,clear_time_s=clear_t if clear else None,physics_path_length_m=float(np.linalg.norm(np.diff(pos[:,:2],axis=0),axis=1).sum()),hardware_verified=False,truth_used_by_navigation=False,clearance_scope='offline true scene geometry against configured physical double-cylinder; planning margin is not counted as robot size')
(s/'independent_evaluation.json').write_text(json.dumps(result,indent=2));fig,ax=plt.subplots(figsize=(10,6));ax.plot(pos[:,0],pos[:,1],c='#2b8a73',label='Physics trajectory');ax.scatter([pos[0,0]],[pos[0,1]],c='#3985c6',label='Start')
for b in applied['boxes']:
 elapsed=np.linspace(0,b.get('motion_duration_s',0),30);c=np.array(b['center_m'][:2])+elapsed[:,None]*np.array(b.get('velocity_mps',[0,0,0])[:2]);ax.plot(c[:,0],c[:,1],c='#d48638',lw=4,label='Runtime obstacle motion');ax.scatter(c[0,0],c[0,1],c='#d48638',s=70,marker='s')
ax.axis('equal');ax.grid(alpha=.2);ax.set(xlabel='World X / m',ylabel='World Y / m',title=scenario+' : dynamic obstacle regression');ax.legend();fig.tight_layout();fig.savefig(s/'动态避障轨迹.png',dpi=160);plt.close(fig);print(json.dumps(result,indent=2));raise SystemExit(0 if result['result']=='PASS' else 1)
