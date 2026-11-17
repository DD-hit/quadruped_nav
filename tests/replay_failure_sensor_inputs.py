#!/usr/bin/env python3
"""Replay saved sensor measurements to inspect LIO/local-map inputs, without Isaac or motion feedback."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--session',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
os.environ.update(ROS_DOMAIN_ID='42',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',FASTRTPS_DEFAULT_PROFILES_FILE=str(a.root/'ros2_ws/src/quadruped_bringup/config/dds/local_sim.xml'))
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import Imu,PointCloud2,PointField
from sensor_msgs_py.point_cloud2 import read_points
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import String
rclpy.init();n=Node('failure_sensor_replay',parameter_overrides=[Parameter('use_sim_time',value=True)])
children=[];handles=[];loc={};count=0;last_report=-1
def launch(name,args):
    h=(a.output/(name+'.log')).open('w');handles.append(h);proc=subprocess.Popen(args,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);children.append(proc)
def on_loc(m):
    global loc
    loc=json.loads(m.data)
latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/localization/status',on_loc,latched)
sample_log=(a.output/'poses.jsonl').open('w',buffering=1)
def on_pose(m):
    p=m.pose.pose.position;q=m.pose.pose.orientation
    sample_log.write(json.dumps(dict(t=m.header.stamp.sec+m.header.stamp.nanosec/1e9,position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w]))+'\n')
n.create_subscription(Odometry,'/navigation/body_pose',on_pose,qos_profile_sensor_data)
snapshot_times=[118.,121.4,123.8,124.2];saved={}
def on_cloud(name,m):
    t=m.header.stamp.sec+m.header.stamp.nanosec/1e9
    for target in snapshot_times:
        key=(name,target)
        if target<=t<target+.6 and key not in saved:
            data=read_points(m,field_names=('x','y','z'),skip_nans=True)
            points=np.column_stack([data[k] for k in ('x','y','z')]).astype(float) if isinstance(data,np.ndarray) and data.dtype.names else np.array(list(data),dtype=float)
            np.save(a.output/(name+'_'+str(target)+'.npy'),points);saved[key]=dict(stamp=t,points=len(points),frame=m.header.frame_id)
            (a.output/'snapshots.json').write_text(json.dumps({str(k):v for k,v in saved.items()},indent=2))
for name,topic in [('registered','/navigation/cloud'),('occupied','/navigation/grid_map/occupancy'),('inflated','/navigation/grid_map/occupancy_inflate')]:
    n.create_subscription(PointCloud2,topic,lambda m,name=name:on_cloud(name,m),qos_profile_sensor_data)
imu_pub=n.create_publisher(Imu,'/imu/data',QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT))
cloud_pub=n.create_publisher(PointCloud2,'/lidar/points',qos_profile_sensor_data)
clock_pub=n.create_publisher(Clock,'/clock',qos_profile_sensor_data)
initial_pub=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
imu_file=np.load(a.session/'simulation/imu_200hz.npz');imu={k:imu_file[k] for k in ['time_s','specific_force_mps2','angular_velocity_radps']}
lidar_file=np.load(a.session/'simulation/lidar_10hz.npz');lidar={k:lidar_file[k] for k in lidar_file.files}
scan=lidar['scan'];boundaries=np.r_[0,np.flatnonzero(np.diff(scan))+1,len(scan)]
dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('offset_time','<u4')])
fields=[PointField(name=name,offset=i*4,datatype=PointField.UINT32 if name=='offset_time' else PointField.FLOAT32,count=1) for i,name in enumerate(dtype.names)]
def stamp(t,obj):obj.sec,obj.nanosec=divmod(round(t*1e9),1_000_000_000)
try:
    launch('localization',['/usr/bin/python3',str(a.root/'scripts/run_localization.py'),'--map-id','d1_full_room_20261109_02','--with-lio','--base-config-file',str(a.root/'ros2_ws/src/quadruped_localization/config/base_odometry_dynamic_gait.yaml'),'--localization-config-file',str(a.session/'localization.yaml'),'--output-dir',str(a.output/'localization')])
    launch('navigation',['ros2','launch','quadruped_navigation','navigation.launch.py','use_sim_time:=true','params_file:='+str(a.session/'navigation.yaml')])
    start=time.monotonic()
    while (imu_pub.get_subscription_count()<1 or cloud_pub.get_subscription_count()<1) and time.monotonic()-start<30:rclpy.spin_once(n,timeout_sec=.05)
    start=time.monotonic();scan_index=0;initial_sent=False
    for i,t in enumerate(imu['time_s']):
        if t>124.3:break
        due=start+float(t)
        while time.monotonic()<due:rclpy.spin_once(n,timeout_sec=min(.002,max(0.,due-time.monotonic())))
        c=Clock();stamp(float(t),c.clock);clock_pub.publish(c)
        m=Imu();stamp(float(t),m.header.stamp);m.header.frame_id='lidar_imu_link';m.orientation.w=1.;m.orientation_covariance[0]=-1.
        m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,imu['specific_force_mps2'][i])
        m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=map(float,imu['angular_velocity_radps'][i]);imu_pub.publish(m)
        if not initial_sent and t>=7.005:
            seed=PoseWithCovarianceStamped();seed.header.frame_id='map';stamp(float(t),seed.header.stamp);seed.pose.pose.position.x=.35;seed.pose.pose.position.y=-.2;seed.pose.pose.orientation.z=np.sin(.06);seed.pose.pose.orientation.w=np.cos(.06);initial_pub.publish(seed);initial_sent=True
        while scan_index<len(boundaries)-1 and float(lidar['scan_stamp_s'][scan_index])+.1<=t+1e-7:
            begin,end=boundaries[scan_index:scan_index+2];data=np.zeros(end-begin,dtype=dtype)
            for j,name in enumerate(['x','y','z']):data[name]=lidar['xyz_lidar_m'][begin:end,j]
            data['intensity']=100.;data['offset_time']=lidar['offset_time_ns'][begin:end]
            cloud=PointCloud2();cloud.header.frame_id='lidar_link';stamp(float(lidar['scan_stamp_s'][scan_index]),cloud.header.stamp);cloud.height=1;cloud.width=len(data);cloud.fields=fields;cloud.point_step=dtype.itemsize;cloud.row_step=cloud.point_step*len(data);cloud.data=data.tobytes();cloud.is_dense=True;cloud_pub.publish(cloud);scan_index+=1
        rclpy.spin_once(n,timeout_sec=0.)
        if int(t)//10!=last_report:
            last_report=int(t)//10;print(json.dumps(dict(t=float(t),phase=loc.get('phase'),scans=scan_index,snapshots=len(saved))),flush=True)
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
    (a.output/'verification.json').write_text(json.dumps(dict(kind='measurement replay and map reconstruction; not a closed-loop navigation acceptance',Isaac_running=False,truth_input=False,snapshots=len(saved),localization=loc),indent=2)+'\n')
finally:
    for proc in reversed(children):
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL)
    sample_log.close();n.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    for h in handles:h.close()
