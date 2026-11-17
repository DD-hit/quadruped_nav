#!/usr/bin/env python3
"""Isolated full-recording estimator regression. Truth is evaluation only."""
import os
os.environ.update(ROS_DOMAIN_ID='89',ROS_LOCALHOST_ONLY='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
import argparse,json,signal,subprocess,time
from pathlib import Path
import numpy as np,yaml,open3d as o3d
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Imu,PointCloud2
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from livox_ros_driver2.msg import CustomMsg,CustomPoint
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=270);p.add_argument('--init-samples',type=int,default=10);p.add_argument('--si',action='store_true');p.add_argument('--center-acc',action='store_true');p.add_argument('--pose-imu',action='store_true');p.add_argument('--realistic-cov',action='store_true');p.add_argument('--rate',type=float,default=2);a=p.parse_args()
root=Path('/home/rl/workspace/quadruped_nav');source=Path('/data0/rl/quadruped_nav/runs/d1_full_room_survey_20261109_02');raw=source/'isaac_gait';a.output.mkdir(parents=True,exist_ok=False)
cfg=yaml.safe_load((source/'mapping/lio/generation_000/fastlio.yaml').read_text());params=cfg['/**']['ros__parameters'];params['output_directory']=str(a.output/'algorithm');params['mapping'].update(imu_init_samples=a.init_samples,imu_acceleration_in_si=a.si,pose_imu=a.pose_imu,center_acc=a.center_acc,realistic_cov=a.realistic_cov);params['publish']['dense_publish_en']=True
if a.realistic_cov:params['mapping'].update(acc_cov=.1,gyr_cov=.001,b_acc_cov=.0001,b_gyr_cov=.000001)
(a.output/'params.yaml').write_text(yaml.safe_dump(cfg));imu=dict(np.load(raw/'imu_200hz.npz'));lidar=dict(np.load(raw/'lidar_10hz.npz')); starts=np.searchsorted(lidar['scan'],np.arange(len(lidar['scan_stamp_s'])+1))
if a.pose_imu:
 gen0=json.loads((raw/'generation.json').read_text());br=Rotation.from_quat(imu['base_quat_wxyz'][:,[1,2,3,0]]);sr=br*Rotation.from_quat(gen0['base_to_imu_quat_xyzw']);sp=imu['base_position_world_m']+br.apply(np.broadcast_to(gen0['base_to_imu_translation_m'],imu['base_position_world_m'].shape));dt=.005
 acc=np.empty_like(sp);acc[1:-1]=(sp[2:]-2*sp[1:-1]+sp[:-2])/dt**2;acc[0]=acc[1];acc[-1]=acc[-2]
 wr=(sr[1:]*sr[:-1].inv()).as_rotvec()/dt;w=np.empty_like(sp);w[1:-1]=.5*(wr[:-1]+wr[1:]);w[0]=wr[0];w[-1]=wr[-1]
 imu['specific_force_mps2']=sr.inv().apply(acc+[0,0,9.80665]);imu['angular_velocity_radps']=sr.inv().apply(w)
if a.center_acc:
 gen0=json.loads((raw/'generation.json').read_text());sr=Rotation.from_quat(imu['base_quat_wxyz'][:,[1,2,3,0]])*Rotation.from_quat(gen0['base_to_imu_quat_xyzw']);wa=sr.apply(imu['specific_force_mps2']);wa[:-1]=.5*(wa[:-1]+wa[1:]);imu['specific_force_mps2']=sr.inv().apply(wa)
rclpy.init();n=Node('ground_regression');q=QoSProfile(depth=1000,reliability=ReliabilityPolicy.RELIABLE);cp=n.create_publisher(Clock,'/clock',q);ip=n.create_publisher(Imu,'/lio/imu',q);lp=n.create_publisher(CustomMsg,'/lidar/livox',q);od=[];grav=[];chunks=[];clouds=0
# Capture registered points without injecting any truth into LIO.
def cloud(m):
 global clouds
 fields={f.name:f.offset for f in m.fields};dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k] for k in ['x','y','z']],'itemsize':m.point_step});v=np.frombuffer(m.data,dtype=dtype);chunks.append(np.column_stack([v[k] for k in ['x','y','z']]));clouds+=1
 if len(chunks)>=100:
  pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate(chunks)));chunks[:]=[np.asarray(pc.voxel_down_sample(.025).points).copy()]
def odom(m):
 v=m.pose.pose;od.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,v.position.x,v.position.y,v.position.z,v.orientation.x,v.orientation.y,v.orientation.z,v.orientation.w])
def gravity(m):grav.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.vector.x,m.vector.y,m.vector.z])
n.create_subscription(PointCloud2,'/cloud_registered',cloud,q);n.create_subscription(Odometry,'/Odometry',odom,q);n.create_subscription(Vector3Stamped,'/gravity',gravity,q)
h=(a.output/'fastlio.log').open('w');binary=root/'ros2_ws/install/fast_lio/lib/fast_lio/fastlio_mapping';child=subprocess.Popen([str(binary),'--ros-args','--params-file',str(a.output/'params.yaml')],stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
try:
 end=time.monotonic()+20
 while ip.get_subscription_count()<1 or lp.get_subscription_count()<1:
  rclpy.spin_once(n,timeout_sec=.05)
  if time.monotonic()>end:raise RuntimeError('DDS discovery timeout')
 time.sleep(1);wall=time.monotonic();scan=0
 for i,t in enumerate(imu['time_s']):
  if t>a.seconds:break
  if child.poll() is not None:raise RuntimeError('estimator exited')
  msg=Imu();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(round(float(t)*1e9),10**9);msg.header.frame_id='lidar_imu_link';c=Clock();c.clock.sec=msg.header.stamp.sec;c.clock.nanosec=msg.header.stamp.nanosec;cp.publish(c);msg.orientation_covariance[0]=-1.
  msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=map(float,imu['specific_force_mps2'][i]);msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=map(float,imu['angular_velocity_radps'][i]);ip.publish(msg)
  while scan<len(starts)-1 and t>=lidar['scan_stamp_s'][scan]+.1:
   lo,hi=starts[scan:scan+2];m=CustomMsg();m.header.stamp.sec,m.header.stamp.nanosec=divmod(round(float(lidar['scan_stamp_s'][scan])*1e9),10**9);m.header.frame_id='lidar_link';m.timebase=round(float(lidar['scan_stamp_s'][scan])*1e9);m.point_num=int(hi-lo)
   m.points=[CustomPoint(x=float(x),y=float(y),z=float(z),offset_time=int(dt),reflectivity=1,tag=16,line=0) for (x,y,z),dt in zip(lidar['xyz_lidar_m'][lo:hi],lidar['offset_time_ns'][lo:hi])];lp.publish(m);scan+=1
  for _ in range(3):rclpy.spin_once(n,timeout_sec=0.)
  delay=wall+float(t)/a.rate-time.monotonic()
  if delay>0:time.sleep(delay)
  if i%4000==0:print('REPLAY',round(float(t),2),'ODOM',len(od),'CLOUD',clouds,flush=True)
 deadline=time.monotonic()+4
 while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
finally:
 os.killpg(child.pid,signal.SIGINT);child.wait(timeout=20);h.close();n.destroy_node();rclpy.shutdown()
np.savez_compressed(a.output/'states.npz',odometry=np.array(od),gravity=np.array(grav));pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate(chunks)));pc=pc.voxel_down_sample(.025);o3d.io.write_point_cloud(str(a.output/'camera_init.pcd'),pc)
gen=json.loads((raw/'generation.json').read_text());R=Rotation.from_quat(imu['base_quat_wxyz'][0,[1,2,3,0]])*Rotation.from_quat(gen['base_to_imu_quat_xyzw']);origin=imu['base_position_world_m'][0]+Rotation.from_quat(imu['base_quat_wxyz'][0,[1,2,3,0]]).apply(gen['base_to_imu_translation_m']);points=R.apply(np.asarray(pc.points))+origin
mask=(abs(points[:,0])<5.7)&(abs(points[:,1])<2.7)&(abs(points[:,2])<.2)
for x,y,hx,hy in [(-1.5,-.8,.4,.4),(1.5,1.8,.45,.5)]:mask &= ~((abs(points[:,0]-x)<hx)&(abs(points[:,1]-y)<hy))
f=points[mask];A=np.c_[f[:,:2],np.ones(len(f))];keep=np.ones(len(f),bool)
for _ in range(5):
 coef=np.linalg.lstsq(A[keep],f[keep,2],rcond=None)[0];err=f[:,2]-A@coef;keep=abs(err)<=np.percentile(abs(err),95)
g=np.array(grav);gv=R.apply(g[:,1:]);angle=np.degrees(np.arccos(np.clip(-gv[:,2]/np.linalg.norm(gv,axis=1),-1,1)))
result=dict(scans_sent=scan,clouds=clouds,odometry=len(od),floor_points=len(f),floor_tilt_deg=float(np.degrees(np.arctan(np.linalg.norm(coef[:2])))),floor_residual_p95_m=float(np.percentile(abs(err),95)),gravity_final_tilt_deg=float(angle[-1]),gravity_initial_final_change_deg=float(np.degrees(np.arccos(np.clip(g[0,1:]@g[-1,1:]/np.linalg.norm(g[0,1:])/np.linalg.norm(g[-1,1:]),-1,1)))),floor_plane=coef.tolist(),truth_used_as_estimator_input=False,seconds=a.seconds,init_samples=a.init_samples,si=a.si,pose_imu=a.pose_imu,center_acc=a.center_acc,realistic_cov=a.realistic_cov)
(a.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
