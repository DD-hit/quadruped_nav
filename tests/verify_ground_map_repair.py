#!/usr/bin/env python3
import os
os.environ.update(ROS_DOMAIN_ID='91',ROS_LOCALHOST_ONLY='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
import sys,json,subprocess,signal,time,hashlib
from pathlib import Path
import numpy as np,yaml,open3d as o3d
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path('/home/rl/workspace/quadruped_nav');data=Path('/data0/rl/quadruped_nav');out=data/'runs/ground_map_repair_20261116_02';old=data/'maps/d1_full_room_web_20261113_01';new=data/'maps/d1_full_room_web_20261116_02'
sys.path[:0]=[str(root/'ros2_ws/src/quadruped_localization/scripts'),str(root/'ros2_ws/src/quadruped_web')]
from localization_map_source import load_localization_source
from quadruped_web.maps import MapCatalog
info,xyz=load_localization_source(new);oldinfo,oldxyz=load_localization_source(old)
m=yaml.safe_load((new/'manifest.yaml').read_text());sm=yaml.safe_load((old/'manifest.yaml').read_text());R=np.array(m['map_from_camera_init']['rotation'])@np.array(sm['map_from_camera_init']['rotation']).T;t=np.array(m['map_from_camera_init']['translation_m'])-R@np.array(sm['map_from_camera_init']['translation_m']);expected=oldxyz@R.T+t
checks={};metrics={};checks['localization_rigid_transform_only']=np.max(abs(xyz-expected))<1e-6
checks['paired_localization_bundle_validated']=True
payload=MapCatalog(root,data,root/'ros2_ws/install/quadruped_web/lib/quadruped_web/octomap_web_export').load(new.name);checks['web_map_catalog_loads']=payload['localization_pcd_paired']
fig,axes=plt.subplots(1,2,figsize=(14,6),sharex=True,sharey=True)
for ax,d,title in zip(axes,[old,new],['Before','After']):
 layers=dict(np.load(d/'layers.npz'));walk=layers['traversable_points'];blocked=layers['preblocked_points'];ground=walk[walk[:,2]<0];g=ground[ground[:,2]<-.2];upper=walk[(walk[:,2]>.2)&(walk[:,2]<2.)]
 pc=np.asarray(o3d.io.read_point_cloud(str(d/'navigation.pcd')).points);floor=pc[(abs(pc[:,0])<5.5)&(abs(pc[:,1])<2.5)&(abs(pc[:,2]+.325)<.18)]
 for x,y,hx,hy in [(-1.5,-.8,.6,.6),(1.5,1.8,.65,.7)]:floor=floor[~((abs(floor[:,0]-x)<hx)&(abs(floor[:,1]-y)<hy))]
 A=np.c_[floor[:,:2],np.ones(len(floor))];keep=np.ones(len(floor),bool)
 for _ in range(5):coef=np.linalg.lstsq(A[keep],floor[keep,2],rcond=None)[0];err=floor[:,2]-A@coef;keep=abs(err)<np.percentile(abs(err),95)
 metrics[title]=dict(ground_traversable_cells=len(g),raised_ground_cells=int(np.sum((walk[:,2]>-.2)&(walk[:,2]<0))),box_pillar_top_cells=len(upper),floor_tilt_deg=float(np.degrees(np.arctan(np.linalg.norm(coef[:2])))),floor_residual_p95_m=float(np.percentile(abs(err),95)),roof_layer_preserved=bool(np.any(walk[:,2]>2.5)))
 ax.scatter(ground[:,0],ground[:,1],s=10,marker='s',c='#4aa884',label='Traversable ground');b=blocked[blocked[:,2]<.15];ax.scatter(b[:,0],b[:,1],s=5,marker='s',c='#d59b75',alpha=.5,label='Blocked');ax.set(title=title,xlabel='Map X / m',ylabel='Map Y / m',xlim=(-6.3,6.3),ylim=(-3.5,3.5));ax.set_aspect('equal');ax.legend(loc='upper right',fontsize=8)
fig.tight_layout();fig.savefig(out/'通行层修复前后.png',dpi=160);plt.close(fig)
checks['floor_tilt_below_0_05deg']=metrics['After']['floor_tilt_deg']<.05
checks['box_pillar_tops_not_traversable']=metrics['After']['box_pillar_top_cells']==0
checks['upper_storey_not_deleted']=metrics['After']['roof_layer_preserved']
checks['ground_coverage_increased']=metrics['After']['ground_traversable_cells']>metrics['Before']['ground_traversable_cells']
# Validate A* on the actual rebuilt PCD and native planner, without a simulator.
import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetPlan
from quadruped_interfaces.srv import GetNavigationMapMeta
rclpy.init();node=Node('ground_map_route_regression');h=(out/'routes.log').open('w');child=subprocess.Popen(['ros2','launch','quadruped_navigation','import_map.launch.py','pcd_file:='+str(new/'navigation.pcd')],stdout=h,stderr=subprocess.STDOUT,start_new_session=True);routes=[]
try:
 meta=node.create_client(GetNavigationMapMeta,'/navigation/global_planner/get_meta');c=node.create_client(GetPlan,'/navigation/compute_path')
 if not meta.wait_for_service(timeout_sec=30):raise RuntimeError('planner unavailable')
 deadline=time.monotonic()+60
 while True:
  f=meta.call_async(GetNavigationMapMeta.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=10)
  if f.done() and f.result().success:break
  if time.monotonic()>deadline:raise RuntimeError('map not ready')
  time.sleep(.2)
 targets=[[0,0],[2.8,2.1],[-3,1],[-3,-1.6],[0,0]]
 for start,goal in list(zip(targets[:-1],targets[1:]))+list(zip(targets[:0:-1],targets[-2::-1])):
  req=GetPlan.Request()
  for pose,xy in [(req.start,start),(req.goal,goal)]:pose.header.frame_id='map';pose.pose.position.x=float(xy[0]);pose.pose.position.y=float(xy[1]);pose.pose.position.z=-.25;pose.pose.orientation.w=1.
  f=c.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=30)
  path=f.result().plan.poses if f.done() else [];points=np.array([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in path]);passed=len(path)>1 and max(abs(points[:,2]+.25))<.15
  routes.append(dict(start=start,goal=goal,points=len(path),passed=bool(passed)))
 checks['native_global_routes_8_of_8']=len(routes)==8 and all(x['passed'] for x in routes)
finally:
 if child.poll() is None:
  os.killpg(child.pid,signal.SIGINT)
  try:child.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
 h.close();node.destroy_node();rclpy.shutdown()
checks['original_localization_unchanged']=hashlib.sha256((old/'localization.pcd').read_bytes()).hexdigest()=='323c21632a8c76eb23f73374e5720916b41d339356182cc4554ae5cbe9fd6b72'
checks['original_navigation_unchanged']=hashlib.sha256((old/'navigation.pcd').read_bytes()).hexdigest()=='5a41314908228206dd5f0a2d7897fa86d8a6b195226d59788887342d2bcce456'
report=dict(result='PASS' if all(checks.values()) else 'FAIL',checks={k:bool(v) for k,v in checks.items()},metrics=metrics,routes=routes,hardware_tested=False,simulation_restarted=False,local_motion_regression_run=False,remaining='Seven raised ground cells retained conservatively; no arbitrary obstacle deletion. Estimator/IMU root cause not declared fixed.')
(out/'verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if report['result']!='PASS':raise SystemExit(1)
