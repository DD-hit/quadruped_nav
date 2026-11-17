#!/usr/bin/env python3
"""Offline evaluation only. No simulator truth reaches localization or navigation."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
import argparse,json,math,collections
from pathlib import Path
import numpy as np,yaml,open3d as o3d
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('session',type=Path);a=p.parse_args();s=a.session;data=Path('/data0/rl/quadruped_nav');mapdir=data/'maps/d1_full_room_web_20261117_01'
events=[json.loads(v) for v in (s/'events.jsonl').open()];states=[json.loads(v) for v in (s/'simulation/states.jsonl').open()];core=json.loads((s/'verification.json').read_text());sim=json.loads((s/'simulation/verification.json').read_text());scene=yaml.safe_load((s/'simulation/scene_config.yaml').read_text())['room'];cfg=yaml.safe_load((s/'navigation.yaml').read_text())
gauge=json.loads((data/'runs/d1_full_room_survey_20261109_02/contract.json').read_text())['world_from_odom_initial_placement'];Rxy=np.array(gauge['rotation']);txy=np.array(gauge['translation']);gyaw=math.atan2(Rxy[1,0],Rxy[0,0]);times=np.array([x['t']-2 for x in states]);positions=np.array([x['pos'] for x in states]);quat=np.array([x['quat'] for x in states]);forward=Rotation.from_quat(quat[:,[1,2,3,0]]).as_matrix()[:,:2,0];truth_xy=(positions[:,:2]-txy)@Rxy;truth_yaw=np.unwrap(Rotation.from_quat(quat[:,[1,2,3,0]]).as_euler('xyz')[:,2])-gyaw
speed=np.linalg.norm(np.array([x['body_velocity'][:2] for x in states]),axis=1)
def truth_at(t):return np.array([np.interp(t,times,truth_xy[:,0]),np.interp(t,times,truth_xy[:,1]),np.interp(t,times,truth_yaw)])
def stats(v):
 v=np.array(v);return dict(count=len(v),median=float(np.median(v)),p95=float(np.percentile(v,95)),max=float(np.max(v))) if len(v) else None
poses=[]
for e in events:
 if e['kind']!='body_pose_stamped':continue
 d=e['data'];t=d['stamp_s']
 if not times[0]<=t<=times[-1]:continue
 q=d['quaternion_xyzw'];yaw=Rotation.from_quat(q).as_euler('xyz')[2];truth=truth_at(t);pos=d['position'];dyaw=math.atan2(math.sin(yaw-truth[2]),math.cos(yaw-truth[2]));poses.append(dict(t=t,position=pos,yaw=yaw,error_xy_m=float(np.linalg.norm(np.array(pos[:2])-truth[:2])),error_yaw_deg=abs(math.degrees(dyaw)),moving=bool(np.interp(t,times,speed)>.05)))
initials=[]
for e in events:
 if e['kind']!='initial_pose_confirmed':continue
 t=e['t'];d=e['data'];true=truth_at(t);pose=d['pose'];initials.append(dict(case=d.get('case','initial_spawn'),t=t,pose=pose,xy_error_m=float(np.linalg.norm(np.array(pose[:2])-true[:2])),yaw_error_deg=abs(math.degrees(math.atan2(math.sin(pose[3]-true[2]),math.cos(pose[3]-true[2])))),quality=d['quality']))
# This is an approximate fixed survey gauge, not trajectory-fitted alignment.
# Small initial origin/heading differences remain part of the reported error.
all_targets=[target for e in events if e['kind']=='request' for target in e['data']['targets']];arrivals=[];seen=set()
for e in events:
 if e['kind']!='controller_state' or e['data']['state']!='SUCCEEDED' or e['data']['task_id'] in seen:continue
 seen.add(e['data']['task_id']);idx=len(arrivals)
 if idx>=len(all_targets):continue
 target=all_targets[idx];true=truth_at(e['t']);near=min(poses,key=lambda x:abs(x['t']-e['t']));arrivals.append(dict(t=e['t'],target=target,truth_xy_error_m=float(np.linalg.norm(true[:2]-target[:2])),localized_xy_error_m=float(np.linalg.norm(np.array(near['position'][:2])-target[:2]))))
params=cfg['/navigation/local_planner']['ros__parameters'];radius=params['grid_map.double_cylinder_radius'];offset=params['grid_map.double_cylinder_offset'];centers=np.concatenate([positions[:,:2]+offset*forward,positions[:,:2]-offset*forward]);gaps={}
for name,ob in scene.items():
 if name in ('floor','ceiling'):continue
 d=np.maximum(abs(centers-np.array(ob['center'][:2]))-np.array(ob['size'][:2])/2,0);gaps[name]=float(np.min(np.linalg.norm(d,axis=1))-radius)
# Match independently sampled map-frame clouds against the saved localization PCD.
cloud=np.load(s/'cloud_samples.npz');reference=np.asarray(o3d.io.read_point_cloud(str(mapdir/'localization.pcd')).points);tree=cKDTree(reference);offsets=np.r_[0,np.cumsum(cloud['counts'])];cloudmetrics=[]
for i,t in enumerate(cloud['stamp_s']):
 pts=cloud['points'][offsets[i]:offsets[i+1]];distance=tree.query(pts)[0];cloudmetrics.append(dict(t=float(t),p95_m=float(np.percentile(distance,95)),fraction_within_10cm=float(np.mean(distance<=.1))))
quality={}
for e in events:
 if e['kind']=='localization_status':
  q=e['data'].get('quality')
  if q and q.get('scan_ns') is not None:quality[q['scan_ns']]=q
reviewed=np.load(mapdir.parent/'d1_full_room_web_20261116_02/layers.npz')['traversable_points'];reviewed=reviewed[(reviewed[:,2]>-.2)&(reviewed[:,2]<0)];coverage=[]
for c in reviewed:coverage.append(dict(cell=c.tolist(),min_truth_body_center_distance_m=float(np.min(np.linalg.norm(truth_xy-c[:2],axis=1)))))
moving=[p for p in poses if p['moving']];xy=stats([p['error_xy_m'] for p in moving]);yaw=stats([p['error_yaw_deg'] for p in moving]);checks=dict(core_actions_passed=core['result']=='PASS',three_initializations_verified=len(initials)==3 and all(p['xy_error_m']<.2 and p['yaw_error_deg']<10 for p in initials),seven_native_arrivals=len(arrivals)==7,arrival_truth_error_under_25cm=len(arrivals)==7 and all(p['truth_xy_error_m']<.25 for p in arrivals),moving_xy_p95_under_15cm=bool(xy and xy['p95']<.15),moving_yaw_p95_under_10deg=bool(yaw and yaw['p95']<10),cloud_match_p95_under_12cm=bool(cloudmetrics and np.percentile([x['p95_m'] for x in cloudmetrics],95)<.12),all_seven_edited_regions_visited=all(x['min_truth_body_center_distance_m']<.25 for x in coverage),no_fall=sim['failure'] in (None,'user_exit','interrupted'),no_teleport=sim['base_teleported_after_initialization'] is False,commands_not_rejected=sim['cmd_vel_rejected']==0,native_double_cylinder_no_overlap=min(gaps.values())>=0)
phase_counts=collections.Counter(e['data'].get('phase') for e in events if e['kind']=='localization_status' and e['t']>=initials[0]['t'])
checks['no_localization_lost_after_initialization']=phase_counts.get('lost',0)==0
report=dict(result='PASS' if all(checks.values()) else 'FAIL',checks={k:bool(v) for k,v in checks.items()},map_id=mapdir.name,localization_phase_counts=dict(phase_counts),scope='Isaac sensor + FAST-LIO + map registration + native global/local planning + velocity controller',initializations=initials,arrivals=arrivals,moving_xy_error_m=xy,moving_yaw_error_deg=yaw,cloud_sample_p95_distance_m=stats([x['p95_m'] for x in cloudmetrics]),registration_rmse_m=stats([q['rmse_m'] for q in quality.values() if q.get('rmse_m') is not None]),registration_fitness=stats([q['fitness'] for q in quality.values()]),clearance_gaps_m=gaps,edited_region_coverage=coverage,physics_path_length_m=float(np.linalg.norm(np.diff(positions[:,:2],axis=0),axis=1).sum()),simulation_duration_s=float(times[-1]-times[0]),hardware_verified=False,truth_used_by_navigation=False,reference_limitation='Fixed historical survey horizontal placement; exact ROS message stamps and physics time minus 2 s. No fitted pose/time alignment; not metrology-grade accuracy or full-body collision certification.')
(s/'independent_evaluation.json').write_text(json.dumps(report,indent=2));(s/'pose_error_series.json').write_text(json.dumps(poses));(s/'cloud_match_series.json').write_text(json.dumps(cloudmetrics))
fig,ax=plt.subplots(figsize=(12,7));walk=np.load(mapdir/'layers.npz')['traversable_points'];floor=walk[walk[:,2]<0];ax.scatter(floor[:,0],floor[:,1],s=3,c='#dceee5',marker='s');ax.plot(truth_xy[:,0],truth_xy[:,1],c='#d69537',lw=2,label='Physics trajectory (evaluation only)');pp=np.array([p['position'][:2] for p in poses]);ax.plot(pp[:,0],pp[:,1],c='#3985c6',lw=1,label='Localized trajectory');ax.scatter(reviewed[:,0],reviewed[:,1],s=60,facecolors='none',edgecolors='#c95353',label='Reviewed ground cells');ax.scatter(np.array(all_targets)[:,0],np.array(all_targets)[:,1],s=30,c='#28694c',marker='x',label='Waypoints');ax.axis('equal');ax.set(xlabel='Map X / m',ylabel='Map Y / m',title='Refined map: closed-loop navigation regression');ax.legend();fig.tight_layout();fig.savefig(s/'闭环导航轨迹.png',dpi=160);plt.close(fig)
print(json.dumps(report,indent=2));raise SystemExit(0 if report['result']=='PASS' else 1)
