#!/usr/bin/env python3
"""Create a new paired map using saved estimated gravity; never edit source maps."""
import os
os.environ.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',ROS_DOMAIN_ID='90',ROS_LOCALHOST_ONLY='1')
import json,sys,hashlib,shutil,subprocess,time,signal
from pathlib import Path
from datetime import datetime,timezone
import numpy as np,yaml,open3d as o3d
root=Path('/home/rl/workspace/quadruped_nav');data=Path('/data0/rl/quadruped_nav');sys.path.insert(0,str(root/'ros2_ws/src/quadruped_localization/scripts'))
from map_bundle import load_bundle,commit_bundle
from map_alignment import final_alignment
old=data/'maps/d1_full_room_20261109_02';oldnav=data/'maps/d1_full_room_web_20261113_01';new=data/'maps/d1_full_room_20261116_01';nav=data/'maps/d1_full_room_web_20261116_02';run=data/'runs/ground_map_repair_20261116_02'
run.mkdir(parents=True,exist_ok=True)
if nav.joinpath('manifest.yaml').exists():
 from localization_map_source import load_localization_source
 load_localization_source(nav);print(nav);raise SystemExit(0)
if not new.exists():
 manifest,points=load_bundle(old);source=Path(manifest['source']['runtime_directory'])/'generation_000/algorithm/Log/mat_pre.txt';logs=np.loadtxt(source);samples=np.c_[logs[:,0],logs[:,-3:]]
 # mat_pre timestamps are relative to first lidar; gravity is an estimated prior,
 # not a truth orientation. Source and stability are recorded explicitly.
 align,dR,dt,report=final_alignment(manifest['map_from_camera_init'],samples,int(samples[-1,0]*1e9),1.,.1)
 points[:,:3]=points[:,:3]@dR.T+dt;new.mkdir();m=dict(manifest);m.update(map_id=new.name,map_from_camera_init=align,parent_map_id=old.name,created_utc=datetime.now(timezone.utc).isoformat(),export_alignment=dict(report,source=str(source),state_kind='saved_prior_gravity',source_map_transform=manifest['map_from_camera_init']))
 snaps={f.name:yaml.safe_load(f.read_text()) if f.suffix=='.yaml' else json.loads(f.read_text()) for f in (old/'configuration').iterdir() if f.suffix in ('.yaml','.json')}
 commit_bundle(new,points,m,snaps)
 (run/'correction.json').write_text(json.dumps(dict(rotation=dR.tolist(),translation_m=dt.tolist(),**report),indent=2))
 pc=o3d.io.read_point_cloud(str(oldnav/'navigation.pcd'));pc.points=o3d.utility.Vector3dVector(np.asarray(pc.points)@dR.T+dt);o3d.io.write_point_cloud(str(run/'navigation.pcd'),pc)
if not (run/'navigation.pcd').exists():
 m=yaml.safe_load((new/'manifest.yaml').read_text());oldm=yaml.safe_load((old/'manifest.yaml').read_text());dR=np.array(m['map_from_camera_init']['rotation'])@np.array(oldm['map_from_camera_init']['rotation']).T;dt=np.array(m['map_from_camera_init']['translation_m'])-dR@np.array(oldm['map_from_camera_init']['translation_m']);pc=o3d.io.read_point_cloud(str(oldnav/'navigation.pcd'));pc.points=o3d.utility.Vector3dVector(np.asarray(pc.points)@dR.T+dt);o3d.io.write_point_cloud(str(run/'navigation.pcd'),pc)
import rclpy
from rclpy.node import Node
from quadruped_interfaces.srv import SaveNavigationMapPackage
rclpy.init();n=Node('rebuild_corrected_navigation_map');children=[];handles=[]
def launch(name,args):
 h=(run/(name+'.log')).open('w');handles.append(h);c=subprocess.Popen(args,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);children.append(c)
try:
 launch('import',['ros2','launch','quadruped_navigation','import_map.launch.py','pcd_file:='+str(run/'navigation.pcd'),'min_points_per_voxel:=3','min_cluster_voxels:=4'])
 c=n.create_client(SaveNavigationMapPackage,'/navigation/map_package_manager/save_package')
 if not c.wait_for_service(timeout_sec=30):raise RuntimeError('map save service missing')
 deadline=time.monotonic()+120
 while True:
  req=SaveNavigationMapPackage.Request();req.package_path=str(nav);req.overwrite=False;f=c.call_async(req)
  rclpy.spin_until_future_complete(n,f,timeout_sec=20)
  if not f.done():raise RuntimeError('save timeout')
  if f.result().success:break
  if time.monotonic()>deadline:raise RuntimeError(f.result().message)
  time.sleep(1)
finally:
 for c in reversed(children):
  if c.poll() is None:
   os.killpg(c.pid,signal.SIGINT)
   try:c.wait(timeout=15)
   except subprocess.TimeoutExpired:os.killpg(c.pid,signal.SIGTERM);c.wait(timeout=10)
 for h in handles:h.close()
 n.destroy_node();rclpy.shutdown()
shutil.copy(new/'localization.pcd',nav/'localization.pcd');shutil.copy(new/'manifest.yaml',nav/'source_manifest.yaml');shutil.copy(run/'navigation.pcd',nav/'navigation.pcd');(nav/'configuration').mkdir(exist_ok=True)
shutil.copy(root/'ros2_ws/src/quadruped_navigation/config/navigation.yaml',nav/'configuration/navigation.yaml')
(nav/'configuration/provenance.json').write_text(json.dumps(dict(parent_navigation=str(oldnav),parent_localization=str(old),repair=str(run),min_points_per_voxel=3,min_cluster_voxels=4,flattened=False,hardware_verified=False),indent=2))
meta=yaml.safe_load((nav/'meta.yaml').read_text());meta['map_id']=nav.name;(nav/'configuration/map_tools.yaml').write_text(yaml.safe_dump(dict(resolution=.1,min_points_per_voxel=3,min_cluster_voxels=4,minimum_support_fraction=.8),sort_keys=False));(nav/'meta.yaml').write_text(yaml.safe_dump(meta,sort_keys=False));files={str(f.relative_to(nav)):hashlib.sha256(f.read_bytes()).hexdigest() for f in nav.rglob('*') if f.is_file()}
parent=yaml.safe_load((new/'manifest.yaml').read_text());manifest=dict(schema_version=1,kind='navigation_map_bundle',status='complete',map_id=nav.name,frame_id='map',parent_map_id=new.name,parent_manifest_sha256=files['source_manifest.yaml'],parent_localization_sha256=files['localization.pcd'],navigation_from_parent=np.eye(4).tolist(),map_from_camera_init=parent['map_from_camera_init'],files_sha256=files,navigation_ready=False,simulation_workflow_verified=False,real_hardware_validation_complete=False)
(nav/'manifest.yaml').write_text(yaml.safe_dump(manifest,sort_keys=False));print(nav)
