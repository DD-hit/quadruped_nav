#!/usr/bin/env python3
"""Read-only ground diagnostics. Physics truth is used solely for offline evaluation."""
import os
os.environ.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np,yaml,open3d as o3d
from scipy.spatial.transform import Rotation,Slerp
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
run=Path('/data0/rl/quadruped_nav/runs/d1_full_room_survey_20261109_02');rawdir=run/'isaac_gait';start=time.monotonic()
imu=np.load(rawdir/'imu_200hz.npz');times=imu['time_s'];positions=imu['base_position_world_m'].astype(float);quats=imu['base_quat_wxyz'].astype(float);rot=Rotation.from_quat(quats[:,[1,2,3,0]]);interp=Slerp(times,rot);velocity=np.gradient(positions,times,axis=0);speed=np.linalg.norm(velocity,axis=1);gyro=np.linalg.norm(imu['angular_velocity_radps'],axis=1)
raw=np.load(rawdir/'lidar_10hz.npz');xyz=raw['xyz_lidar_m'];scan=raw['scan'];offsets=raw['offset_time_ns'];stamp=raw['scan_stamp_s'];mount=np.array(json.loads((rawdir/'lidar_scans.jsonl').open().readline())['mount_matrix']);scene=yaml.safe_load((run/'scene_config.yaml').read_text());room=scene['room'];floor_z=room['floor']['center'][2]+room['floor']['size'][2]/2
xlo=room['west']['center'][0]+room['west']['size'][0]/2;xhi=room['east']['center'][0]-room['east']['size'][0]/2;ylo=room['south']['center'][1]+room['south']['size'][1]/2;yhi=room['north']['center'][1]-room['north']['size'][1]/2
starts=np.searchsorted(scan,np.arange(len(stamp)+1));classes=[]
for i,t in enumerate(stamp):
 mask=(times>=t)&(times<=t+.1);v=speed[mask];g=gyro[mask]
 kind='stationary' if len(v) and v.max()<=.01 and g.max()<=.03 else 'moving' if len(v) and (np.median(v)>=.05 or g.max()>=.15) else 'mixed'
 classes.append(kind)
selected=[]
for kind,maximum in [('stationary',30),('moving',180),('mixed',20)]:
 ids=np.flatnonzero(np.array(classes)==kind)
 selected.extend(ids[np.linspace(0,len(ids)-1,min(maximum,len(ids)),dtype=int)].tolist() if len(ids) else [])
selected=sorted(set(selected));rows=[];samples={k:[] for k in ['stationary_truth','moving_truth','stationary_raw_plane','moving_raw_plane','stationary_frozen_end','moving_frozen_end']}
def stats(values):
 x=np.asarray(values,dtype=float)
 return dict(count=len(x),mean=float(x.mean()),std=float(x.std()),rms=float(np.sqrt(np.mean(x*x))),p95_abs=float(np.percentile(abs(x),95)),p99_abs=float(np.percentile(abs(x),99)),max_abs=float(abs(x).max()),p01=float(np.percentile(x,1)),p99=float(np.percentile(x,99))) if len(x) else None

def expected_floor(origins,directions):
 dz=directions[:,2];distance=np.divide(floor_z-origins[:,2],dz,out=np.full(len(dz),np.inf),where=dz< -1e-9);points=origins+directions*distance[:,None]
 keep=(dz< -1e-9)&(distance>=scene['near_range_m'])&(distance<=scene['far_range_m'])&(points[:,0]>xlo+.03)&(points[:,0]<xhi-.03)&(points[:,1]>ylo+.03)&(points[:,1]<yhi-.03)
 for name in ['box','pillar']:
  ob=room[name];lo=np.array(ob['center'])-np.array(ob['size'])/2-1e-5;hi=np.array(ob['center'])+np.array(ob['size'])/2+1e-5
  inv=np.divide(1.,directions,out=np.full_like(directions,np.inf),where=abs(directions)>1e-12);t0=(lo-origins)*inv;t1=(hi-origins)*inv;enter=np.maximum(np.min([t0,t1],axis=0).max(axis=1),0);leave=np.maximum(t0,t1).min(axis=1)
  keep &= ~((leave>=enter)&(enter<distance-1e-5))
 return keep

for n,i in enumerate(selected):
 lo,hi=starts[i:i+2];local=xyz[lo:hi].astype(float);t=stamp[i]+offsets[lo:hi]*1e-9;inside=(t>=times[0])&(t<=times[-1]);local=local[inside];t=t[inside]
 base_rot=interp(t);base_pos=np.column_stack([np.interp(t,times,positions[:,k]) for k in range(3)]);base_points=local@mount[:3,:3].T+mount[:3,3];world=base_rot.apply(base_points)+base_pos
 origins=base_pos+base_rot.apply(np.broadcast_to(mount[:3,3],base_pos.shape));directions=base_rot.apply(local@mount[:3,:3].T);directions/=np.linalg.norm(directions,axis=1)[:,None]
 mask=expected_floor(origins,directions);floor_local=local[mask];world_floor=world[mask];truth_error=world_floor[:,2]-floor_z
 if len(floor_local)<50:raise RuntimeError(f'Too few expected floor rays scan {i}: {len(floor_local)}')
 centered=floor_local-floor_local.mean(axis=0);_,vectors=np.linalg.eigh(centered.T@centered/len(centered));normal=vectors[:,0];plane_error=centered@normal
 end=min(float(stamp[i]+.1),float(times[-1]));end_position=np.array([np.interp(end,times,positions[:,k]) for k in range(3)]);frozen=interp(end).apply(base_points[mask])+end_position;frozen_error=frozen[:,2]-floor_z
 kind=classes[i];rows.append(dict(scan=i,stamp_s=float(stamp[i]),kind=kind,points=len(local),expected_floor_points=int(mask.sum()),truth_reconstructed_world_z_error_m=stats(truth_error),raw_scan_best_plane_residual_m=stats(plane_error),no_deskew_end_pose_z_error_m=stats(frozen_error)))
 if kind in ('stationary','moving'):
  for suffix,array in [('truth',truth_error),('raw_plane',plane_error),('frozen_end',frozen_error)]:samples[kind+'_'+suffix].append(array)
 if n%30==0:print(json.dumps(dict(processed=n+1,total=len(selected),wall_s=time.monotonic()-start)),flush=True)

summaries={k:stats(np.concatenate(v)) for k,v in samples.items() if v};np.savez_compressed(a.output/'scan_error_samples.npz',**{k:np.concatenate(v)[::max(1,len(np.concatenate(v))//100000)] for k,v in samples.items() if v})
gauge=json.loads((run/'contract.json').read_text())['world_from_odom_initial_placement'];R=np.array(gauge['rotation']);translation=np.array(gauge['translation']);base_z=json.loads((run/'initial_state.json').read_text())['base_position_world_m'][2]
map_reports={};floor_samples={};map_files=[Path('/data0/rl/quadruped_nav/maps/d1_full_room_20261109_02/localization.pcd'),Path('/data0/rl/quadruped_nav/maps/d1_full_room_web_20261113_01/navigation.pcd')]
for file in map_files:
 points=np.asarray(o3d.io.read_point_cloud(str(file)).points);world=points.copy();world[:,:2]=points[:,:2]@R.T+translation;world[:,2]+=base_z
 keep=(world[:,0]>xlo+.3)&(world[:,0]<xhi-.3)&(world[:,1]>ylo+.3)&(world[:,1]<yhi-.3)&(abs(world[:,2]-floor_z)<.22)
 for name in ['box','pillar']:
  ob=room[name];lo=np.array(ob['center'][:2])-np.array(ob['size'][:2])/2-.2;hi=np.array(ob['center'][:2])+np.array(ob['size'][:2])/2+.2;keep &= ~np.all((world[:,:2]>=lo)&(world[:,:2]<=hi),axis=1)
 floor=world[keep];A=np.column_stack([floor[:,:2],np.ones(len(floor))]);use=np.ones(len(floor),bool)
 for _ in range(5):
  coef=np.linalg.lstsq(A[use],floor[use,2],rcond=None)[0];residual=floor[:,2]-A@coef;use=abs(residual)<=np.percentile(abs(residual),95)
 normal=np.r_[-coef[:2],1.];normal/=np.linalg.norm(normal);plane_res=(floor[:,2]-A@coef)/np.linalg.norm(np.r_[-coef[:2],1.])
 tile=np.floor(floor[:,:2]/.5).astype(int);unique,indices=np.unique(tile,axis=0,return_inverse=True);local_spans=[]
 for j in range(len(unique)):
  zz=plane_res[indices==j]
  if len(zz)>=10:local_spans.append(float(np.percentile(zz,95)-np.percentile(zz,5)))
 map_reports[file.name]=dict(path=str(file),sha256=hashlib.sha256(file.read_bytes()).hexdigest(),points=len(points),floor_points=len(floor),error_against_fixed_physical_floor_m=stats(floor[:,2]-floor_z),best_plane_residual_m=stats(plane_res),best_plane_z_ax_by_c=coef.tolist(),best_plane_tilt_deg=float(np.degrees(np.arccos(normal[2]))),local_half_m_tile_90percent_thickness_m=stats(local_spans))
 floor_samples[file.name]=floor
np.savez_compressed(a.output/'final_floor_samples.npz',**floor_samples)
# Verify the concrete bump that was visible in Web: one floor point crosses z=-0.30.
nav=np.asarray(o3d.io.read_point_cloud(str(map_files[1])).points);x,y=.15,.45;cell=nav[(abs(nav[:,0]-x)<.05)&(abs(nav[:,1]-y)<.05)&(nav[:,2]<.1)];upper=cell[(cell[:,2]>=-.3)&(cell[:,2]<-.2)]
example=dict(map_xy=[x,y],original_navigation_pcd_xyz=cell.tolist(),upper_voxel_point_count=len(upper),map_z_boundary_m=-.3,resolution_m=.1,min_points_per_voxel=1,raw_z_span_m=float(np.ptp(cell[:,2])),explanation='A tiny crossing of the -0.30m boundary creates an additional occupied cell; the voxel side length is not the physical bump height.')
gravity_log=np.loadtxt(run/'mapping/lio/generation_000/algorithm/Log/mat_pre.txt');reference=yaml.safe_load(Path('/data0/rl/quadruped_nav/maps/d1_full_room_20261109_02/manifest.yaml').read_text());map_from_lio=np.array(reference['map_from_camera_init']['rotation']);up=(-gravity_log[:,-3:])@map_from_lio.T;up/=np.linalg.norm(up,axis=1)[:,None];up_world=up.copy();up_world[:,:2]=up[:,:2]@R.T;tilt=np.degrees(np.arccos(np.clip(up[:,2],-1,1)));change=np.degrees(np.arccos(np.clip(up@up[0],-1,1)));coef=np.array(map_reports['navigation.pcd']['best_plane_z_ax_by_c']);normal_world=np.r_[-coef[:2],1.];normal_world/=np.linalg.norm(normal_world)
gravity_report=dict(samples=len(gravity_log),source=str(run/'mapping/lio/generation_000/algorithm/Log/mat_pre.txt'),logged_initial_gravity=gravity_log[0,-3:].tolist(),logged_final_gravity=gravity_log[-1,-3:].tolist(),initial_tilt_relative_to_fixed_map_deg=float(tilt[0]),final_tilt_relative_to_fixed_map_deg=float(tilt[-1]),max_tilt_relative_to_fixed_map_deg=float(tilt.max()),initial_to_final_direction_change_deg=float(change[-1]),final_estimated_vertical_to_navigation_floor_normal_angle_deg=float(np.degrees(np.arccos(np.clip(up_world[-1]@normal_world,-1,1)))),interpretation='The frozen initial map vertical and subsequently estimated gravity diverge. Final estimated up agrees closely with the fitted map floor normal. This supports a gravity/attitude-reference contribution; it does not uniquely identify a code defect or parameter cause.')
np.savez_compressed(a.output/'gravity_series.npz',time_s=gravity_log[:,0],up_map=up,tilt_deg=tilt,direction_change_deg=change)
report=dict(scope='Read-only offline inspection of the ORIGINAL mapping run, not a new simulation or replay; no source data or map edits',run=str(run),sensor_noise_enabled=scene['noise_enabled'],gravity_estimate=gravity_report,ray_box_boundary_tolerance_m=1e-5,boundary_classification_note='First diagnostic used exact AABBs: one pillar-corner return lay 3.9 micrometres outside its mathematical box and was labelled floor. A 10-micrometre geometric tolerance fixes that label; the original unfiltered diagnostic is retained in ground_diagnosis_20261116_01.',selected_scans=len(rows),all_scan_counts_by_motion={k:classes.count(k) for k in set(classes)},classification=dict(stationary_max_speed_m_s=.01,stationary_max_gyro_rad_s=.03,moving_median_speed_min_m_s=.05,moving_max_gyro_min_rad_s=.15),method='Analytical scene ray/box intersections identify floor returns independently of observed endpoint height. Actual per-ray timestamps and recorded 200Hz poses reconstruct each return. Truth is offline scoring only. Best plane fits are diagnostics, never applied to saved maps.',scan_summaries=summaries,maps=map_reports,bump_example=example,limitations=['Intermediate post-ICP registered scans and full post-update poses were not retained in the original run. This comparison localizes error to the LIO/registration/export chain but cannot uniquely blame deskew vs pose estimation vs map fusion.','The saved mat_pre log is pre-update and uses a stale Euler variable; it is not used as a post-update pose trajectory.','Ideal simulated raycasts do not model real Livox noise or self occlusion. Results do not certify hardware.'],maps_modified=False,simulation_started=False,wall_s=time.monotonic()-start)
(a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');(a.output/'scans.json').write_text(json.dumps(rows,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
