#!/usr/bin/env python3
"""Apply seven reviewed ground edits to navigation geometry, then rebuild native layers."""
import os
os.environ.update(ROS_DOMAIN_ID='92',ROS_LOCALHOST_ONLY='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
import sys,json,hashlib,shutil,subprocess,time,signal
from pathlib import Path
import numpy as np,yaml,open3d as o3d
root=Path('/home/rl/workspace/quadruped_nav');data=Path('/data0/rl/quadruped_nav');old=data/'maps/d1_full_room_web_20261116_02';nav=data/'maps/d1_full_room_web_20261117_01';run=data/'runs/ground_manual_edit_20261117_01';run.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(root/'ros2_ws/src/quadruped_localization/scripts'))
from localization_map_source import load_localization_source
load_localization_source(old)
if (nav/'manifest.yaml').exists():load_localization_source(nav);print(nav);raise SystemExit(0)
layers=np.load(old/'layers.npz');walk=layers['traversable_points'];cells=walk[(walk[:,2]>-.2)&(walk[:,2]<0)];assert len(cells)==7
pc=o3d.io.read_point_cloud(str(old/'navigation.pcd'));points=np.asarray(pc.points).copy();original=points.copy();edits=[]
# Confirm these source-map cells correspond to scene floor, using recorded
# coordinate transforms for review only; the edited heights come from neighbors.
parent=yaml.safe_load((old/'source_manifest.yaml').read_text());initial=parent['export_alignment']['source_map_transform'];current=parent['map_from_camera_init'];dR=np.array(current['rotation'])@np.array(initial['rotation']).T
survey=data/'runs/d1_full_room_survey_20261109_02';gauge=json.loads((survey/'contract.json').read_text())['world_from_odom_initial_placement'];Rxy=np.array(gauge['rotation']);txy=np.array(gauge['translation']);scene=yaml.safe_load((survey/'scene_config.yaml').read_text())['room']
for c in cells:
 oldp=dR.T@np.r_[c[:2],-.325];worldxy=Rxy@oldp[:2]+txy
 assert abs(worldxy[0])<5.5 and abs(worldxy[1])<2.5
 for name in ['box','pillar']:
  ob=scene[name];delta=abs(worldxy-np.array(ob['center'][:2]))-np.array(ob['size'][:2])/2
  assert np.linalg.norm(np.maximum(delta,0))>.5,(c,name)
 target=(abs(points[:,0]-c[0])<.05)&(abs(points[:,1]-c[1])<.05)&(points[:,2]>=-.3)&(points[:,2]<-.2)
 ids=np.flatnonzero(target);assert len(ids)==1,(c,ids)
 near=(np.linalg.norm(points[:,:2]-c[:2],axis=1)<.25)&(points[:,2]>-.36)&(points[:,2]<-.3)
 patch=points[near];assert len(patch)>30
 A=np.c_[patch[:,:2],np.ones(len(patch))];coef=np.linalg.lstsq(A,patch[:,2],rcond=None)[0];res=patch[:,2]-A@coef
 assert np.percentile(abs(res),95)<.005
 z=np.c_[points[ids,:2],np.ones(len(ids))]@coef;assert np.all((points[ids,2]-z>.02)&(points[ids,2]-z<.06))
 edits.append(dict(cell=c.tolist(),world_xy_for_scene_review=worldxy.tolist(),point_indices=ids.tolist(),before=points[ids].tolist(),after_z=z.tolist(),neighbor_count=len(patch),neighbor_plane_residual_p95_m=float(np.percentile(abs(res),95))))
 points[ids,2]=z
assert np.count_nonzero(np.any(points!=original,axis=1))==7
assert np.array_equal(points[:,:2],original[:,:2])
pc.points=o3d.utility.Vector3dVector(points);o3d.io.write_point_cloud(str(run/'navigation.pcd'),pc)
(run/'edits.json').write_text(json.dumps(dict(source_map=old.name,target_map=nav.name,edits=edits,modified_points=7,localization_pcd_changed=False,scene_truth_used_only_for_review=True),indent=2))
import rclpy
from rclpy.node import Node
from quadruped_interfaces.srv import SaveNavigationMapPackage
rclpy.init();n=Node('reviewed_ground_edits');h=(run/'import.log').open('w');child=subprocess.Popen(['ros2','launch','quadruped_navigation','import_map.launch.py','pcd_file:='+str(run/'navigation.pcd')],stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
try:
 client=n.create_client(SaveNavigationMapPackage,'/navigation/map_package_manager/save_package');assert client.wait_for_service(timeout_sec=30)
 deadline=time.monotonic()+120
 while True:
  req=SaveNavigationMapPackage.Request();req.package_path=str(nav);req.overwrite=False;f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=25)
  if not f.done():raise RuntimeError('save timeout')
  if f.result().success:break
  if time.monotonic()>deadline:raise RuntimeError(f.result().message)
  time.sleep(1)
finally:
 if child.poll() is None:
  os.killpg(child.pid,signal.SIGINT)
  try:child.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
 h.close();n.destroy_node();rclpy.shutdown()
for name in ['localization.pcd','source_manifest.yaml']:shutil.copyfile(old/name,nav/name)
shutil.copyfile(run/'navigation.pcd',nav/'navigation.pcd');shutil.copytree(old/'configuration',nav/'configuration');shutil.copyfile(run/'edits.json',nav/'configuration/reviewed_ground_edits.json')
meta=yaml.safe_load((nav/'meta.yaml').read_text());meta['map_id']=nav.name;(nav/'meta.yaml').write_text(yaml.safe_dump(meta,sort_keys=False));m=yaml.safe_load((old/'manifest.yaml').read_text());m.update(map_id=nav.name,previous_navigation_map_id=old.name,manual_edit_record='configuration/reviewed_ground_edits.json',simulation_workflow_verified=False,navigation_ready=False);m['files_sha256']={str(f.relative_to(nav)):hashlib.sha256(f.read_bytes()).hexdigest() for f in nav.rglob('*') if f.is_file() and f.name!='manifest.yaml'};(nav/'manifest.yaml').write_text(yaml.safe_dump(m,sort_keys=False));load_localization_source(nav);print(nav)
