#!/usr/bin/env python3
"""One-shot initial ICP coverage; truth is used only to set known seed errors and score."""
import os
os.environ.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
import argparse,json,sys,time,hashlib,collections
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--scans',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False)
scripts=a.root/'ros2_ws/src/quadruped_localization/scripts';sys.path.insert(0,str(scripts));sys.path.insert(0,str(a.root/'ros2_ws/src/quadruped_mapping/scripts'))
from map_registration import MapRegistration,RegistrationConfig
from localization_map_source import load_localization_source
profile=yaml.safe_load((a.root/'ros2_ws/src/quadruped_localization/config/localization.yaml').read_text())
map_info,points=load_localization_source('/data0/rl/quadruped_nav/maps/d1_full_room_20261109_02')
registration=MapRegistration(points,RegistrationConfig.from_dict(profile['registration']))
cases=json.loads((a.scans/'cases.json').read_text());capture=json.loads((a.scans/'capture_report.json').read_text());mount=np.array(capture['base_from_lidar']);inv_mount=np.linalg.inv(mount);raw=np.load(a.scans/'lidar_10hz.npz')
scenarios=[('accurate',0,0,0),('coarse_10deg',.35,-.2,10),('coarse_20deg',-.5,.4,-20),('wrong_yaw_90deg',0,0,90),('wrong_yaw_180deg',0,0,180),('wrong_position_2m',2,0,0),('fixed_origin',None,None,None)]
contract=dict(scope='single static raw scan + production ICP core; not full LIO/ROS acceptance',position_limit_m=.18,rotation_limit_deg=5.,seed_roll_pitch_deg=[0,0],seed_z_m=0,scenarios=scenarios,map=map_info,registration_config=profile['registration'],registration_source_sha256=hashlib.sha256((scripts/'map_registration.py').read_bytes()).hexdigest(),self_occlusion_modeled=False,manufacturer_scan_pattern=False)
(a.output/'contract.json').write_text(json.dumps(contract,indent=2)+'\n')
rows=[];start=time.monotonic()
with (a.output/'results.jsonl').open('w',buffering=1) as log:
 for c in cases:
  truth=np.array(c['map_from_base_truth']);scan=raw['xyz_lidar_m'][raw['scan']==c['scan']].astype(float)
  for name,dx,dy,dyaw in scenarios:
   seed=np.eye(4)
   if name!='fixed_origin':
    seed[:2,3]=truth[:2,3]+[dx,dy];seed[:3,:3]=Rotation.from_euler('z',c['yaw_deg']+dyaw,degrees=True).as_matrix()
   t=time.monotonic();r=registration.register(scan,seed@mount,np.eye(4),mode='initialize')
   row=dict(case=c['id'],location=c['location'],xy=c['map_xy'],yaw_deg=c['yaw_deg'],scenario=name,seed_map_from_base=seed.tolist(),accepted=r.accepted,reason=r.reason,fitness=r.fitness,rmse_m=r.rmse_m,correspondences=r.correspondences,correction_shift_m=r.shift_m,correction_rotation_rad=r.rotation_rad,wall_s=time.monotonic()-t)
   if r.candidate_map_from_scan is not None:
    estimate=r.candidate_map_from_scan@inv_mount;err=estimate[:3,:3]@truth[:3,:3].T
    row.update(candidate_map_from_base=estimate.tolist(),position_error_m=float(np.linalg.norm(estimate[:3,3]-truth[:3,3])),rotation_error_deg=float(Rotation.from_matrix(err).magnitude()*180/np.pi),yaw_error_deg=float(abs((np.degrees(np.arctan2(estimate[1,0],estimate[0,0]))-c['yaw_deg']+180)%360-180)))
   accurate=row.get('position_error_m',1e9)<=.18 and row.get('rotation_error_deg',1e9)<=5
   row['outcome']='correct_accept' if r.accepted and accurate else 'false_accept' if r.accepted else 'rejected'
   rows.append(row);log.write(json.dumps(row)+'\n')
  if (c['id']+1)%8==0:print(json.dumps(dict(scans=c['id']+1,trials=len(rows),wall_s=time.monotonic()-start)),flush=True)
summary={}
for name,*_ in scenarios:
 group=[r for r in rows if r['scenario']==name];correct=[r for r in group if r['outcome']=='correct_accept'];accepted=[r for r in group if r['accepted']]
 summary[name]=dict(total=len(group),outcomes=dict(collections.Counter(r['outcome'] for r in group)),reasons=dict(collections.Counter(r['reason'] for r in group)),accepted_position_error_m={k:float(v) for k,v in zip(['median','p95','max'],np.percentile([r['position_error_m'] for r in accepted],[50,95,100]))} if accepted else None)
(a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
