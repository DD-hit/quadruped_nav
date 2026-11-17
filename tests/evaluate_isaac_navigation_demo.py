#!/usr/bin/env python3
"""Independent recorded-motion checks; physical truth is used only offline."""
import argparse,json,math
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser(description=__doc__);p.add_argument('session',type=Path);a=p.parse_args();s=a.session
core=json.loads((s/'verification.json').read_text())
sim=json.loads((s/'simulation/verification.json').read_text())
events=[json.loads(v) for v in (s/'events.jsonl').open()]
rows=[json.loads(v) for v in (s/'simulation/states.jsonl').open()]
config=yaml.safe_load((s/'navigation.yaml').read_text())
params=config['/navigation/local_planner']['ros__parameters']
radius=params['grid_map.double_cylinder_radius'];offset=params['grid_map.double_cylinder_offset']
scene=yaml.safe_load((s/'simulation/scene_config.yaml').read_text())
positions=np.array([r['pos'][:2] for r in rows])
quats=np.array([r['quat'] for r in rows]);forward=Rotation.from_quat(quats[:,[1,2,3,0]]).as_matrix()[:,:2,0]
centers=np.concatenate([positions+offset*forward,positions-offset*forward])
gaps={}
for name,ob in scene['room'].items():
    if name in ('floor','ceiling'):continue
    distance=np.maximum(np.abs(centers-np.array(ob['center'][:2]))-np.array(ob['size'][:2])/2,0.)
    gaps[name]=float(np.min(np.linalg.norm(distance,axis=1))-radius)
arrivals=[];seen=set();poses=[e for e in events if e['kind']=='body_pose']
targets=[v for e in events if e['kind']=='request' for v in e['data']['targets']]
for e in events:
    if e['kind']!='controller_state' or e['data']['state']!='SUCCEEDED' or e['data']['task_id'] in seen:continue
    seen.add(e['data']['task_id']);target=targets[len(arrivals)]
    nearest=min(poses,key=lambda v:abs(v['t']-e['t']));pose=nearest['data']
    arrivals.append(dict(t=e['t'],target=target,recorded_pose=pose,
        recorded_position_error_m=math.hypot(pose[0]-target[0],pose[1]-target[1]),
        recorded_yaw_error_rad=abs(math.atan2(math.sin(pose[3]-target[2]),math.cos(pose[3]-target[2]))),
        note='Nearest recorder sample; native controller owns exact arrival predicate'))
check=dict(core_passed=core['result']=='PASS',
    no_fall=sim['failure'] in (None,'user_exit'),
    no_teleport=sim['base_teleported_after_initialization'] is False,
    receiver_rejected_no_commands=sim['cmd_vel_rejected']==0,
    four_native_arrivals=len(arrivals)==4,
    native_double_cylinder_clearance=min(gaps.values())>=0.)
report=dict(result='PASS' if all(check.values()) else 'FAIL',checks=check,
    clearance_scope='native double-cylinder geometry only; not full-body clearance certification',
    minimum_native_double_cylinder_gap_m=min(gaps.values()),gaps_by_obstacle_m=gaps,
    native_arrivals=arrivals,physics_rows=len(rows),physics_trajectory_length_m=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()),
    truth_usage='offline evaluation only; no truth feedback into navigation/localization')
(s/'independent_evaluation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2));raise SystemExit(0 if report['result']=='PASS' else 1)
