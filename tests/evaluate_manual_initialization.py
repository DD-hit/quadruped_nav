"""Manual-initial-pose tolerance regression; no global search or truth feedback."""
import os
os.environ.update(OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
import collections
import importlib.util
import json
from pathlib import Path
import sys
import time
import numpy as np
import open3d as o3d
import yaml
from scipy.spatial.transform import Rotation

root=Path('/home/rl/workspace/quadruped_nav')
out=Path('/data0/rl/quadruped_nav/evaluations/manual_localization_20261117')
data=Path('/data0/rl/quadruped_nav/evaluations/localization_backends_20261117/data')
sys.path.insert(0,str(root/'ros2_ws/src/quadruped_localization/scripts'))
import map_registration as current
path=out/'before/ros2_ws/src/quadruped_localization/scripts/map_registration.py'
spec=importlib.util.spec_from_file_location('previous_registration',path)
previous=importlib.util.module_from_spec(spec);sys.modules[spec.name]=previous;spec.loader.exec_module(previous)
cfg=yaml.safe_load((root/'ros2_ws/src/quadruped_localization/config/localization.yaml').read_text())['registration']
points=np.asarray(o3d.io.read_point_cloud(str(data/'map.pcd')).points).copy()
new=current.MapRegistration(points,current.RegistrationConfig.from_dict(cfg))
old=previous.MapRegistration(points,previous.RegistrationConfig.from_dict(cfg))
truths=json.loads((data/'truth_for_evaluator.json').read_text())
mount=np.asarray(json.loads((data/'contract.json').read_text())['base_from_lidar'])
rows=[]
scenarios=[('accurate',0.,0.,0.),('coarse10',.35,-.2,10.),('coarse15',.5,0.,15.),('boundary30',1.,0.,30.)]
with (out/'manual_tolerance.jsonl').open('w',buffering=1) as log:
    for c in truths:
        cloud=np.asarray(o3d.io.read_point_cloud(str(data/f"scans/{c['id']:03d}.pcd")).points).copy()
        truth=np.asarray(c['map_from_base_truth'])
        for label,dx,dy,dyaw in scenarios:
            seed=truth.copy();seed[:2,3]+=[dx,dy]
            seed[:3,:3]=Rotation.from_euler('z',dyaw,degrees=True).as_matrix()@seed[:3,:3]
            start=time.monotonic();result=new.register(cloud,seed,mount,mode='initialize')
            row=dict(case=c['id'],scenario=label,accepted=result.accepted,reason=result.reason,wall_s=time.monotonic()-start)
            candidate=result.candidate_map_from_scan
            if candidate is not None:
                row['position_error_m']=float(np.linalg.norm(candidate[:3,3]-truth[:3,3]))
                row['rotation_error_deg']=float(np.degrees(Rotation.from_matrix(candidate[:3,:3]@truth[:3,:3].T).magnitude()))
            row['outcome']=('correct_accept' if row.get('position_error_m',1e9)<=.18 and row.get('rotation_error_deg',1e9)<=5 else 'false_accept') if result.accepted else 'rejected'
            # Same-data old/new regression on every case with the expected user error range.
            if label in ['accurate','coarse10']:
                baseline=old.register(cloud,seed,mount,mode='initialize')
                row['matches_previous_acceptance']=result.accepted==baseline.accepted
                row['matches_previous_pose']=((candidate is None and baseline.candidate_map_from_scan is None) or
                    (candidate is not None and baseline.candidate_map_from_scan is not None and np.allclose(candidate,baseline.candidate_map_from_scan,atol=1e-10)))
            rows.append(row);log.write(json.dumps(row)+'\n')
        if c['id']%16==15:print('completed',c['id']+1,flush=True)
summary={label:dict(collections.Counter(r['outcome'] for r in rows if r['scenario']==label)) for label,*_ in scenarios}
summary['baseline_comparison']=dict(compared=sum('matches_previous_acceptance' in r for r in rows),
    acceptance_identical=all(r.get('matches_previous_acceptance',True) for r in rows),
    poses_identical=all(r.get('matches_previous_pose',True) for r in rows))
(out/'manual_tolerance_summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2),flush=True)
