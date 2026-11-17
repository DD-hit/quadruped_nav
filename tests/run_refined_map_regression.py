#!/usr/bin/env python3
"""Record the real Isaac/LIO/localization/upstream navigation chain; no truth feedback."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback
import yaml
import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--project-root', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
root, out = a.project_root, a.output
out.mkdir(parents=True, exist_ok=False)
(out/'recorder_source.py').write_bytes(Path(__file__).read_bytes())
os.environ.update(ROS_DOMAIN_ID='42', ROS_LOCALHOST_ONLY='0',
                  RMW_IMPLEMENTATION='rmw_fastrtps_cpp', OMP_NUM_THREADS='1',
                  OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1',
                  QUADRUPED_SIM_CAPTURE_RGB='0',
                  FASTRTPS_DEFAULT_PROFILES_FILE=str(root/'ros2_ws/src/quadruped_bringup/config/dds/local_sim.xml'))
children, handles, checks = [], [], []
events = (out/'events.jsonl').open('w', buffering=1)
started = time.monotonic()
loc, body, command, states = {}, None, [0., 0., 0.], {}
cloud_samples=[];cloud_sample_times=[];last_progress=0
cloud_count = spline_count = 0
active = None

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from quadruped_interfaces.msg import NavigationState, Bspline
from quadruped_interfaces.srv import SaveNavigationMapPackage
from nav2_msgs.action import NavigateToPose, FollowWaypoints

rclpy.init()
n = Node('isaac_navigation_demo_recorder', parameter_overrides=[Parameter('use_sim_time', value=True)])
def now():
    return n.get_clock().now().nanoseconds/1e9
def save(kind, data):
    events.write(json.dumps(dict(t=now(), wall_s=time.monotonic()-started, kind=kind, data=data))+'\n')
def launch(name, args):
    h = (out/(name+'.log')).open('w'); handles.append(h)
    proc = subprocess.Popen(args, stdout=h, stderr=subprocess.STDOUT, start_new_session=True)
    children.append((name, proc))
    (out/'processes.json').write_text(json.dumps({k:v.pid for k,v in children}, indent=2))
    return proc
def wait(pred, timeout=120):
    global last_progress
    end = time.monotonic()+timeout
    while not pred():
        rclpy.spin_once(n, timeout_sec=.02)
        if time.monotonic()-last_progress>20:
            print('PROGRESS',json.dumps(dict(t=now(),pose=body,command=command,phase=loc.get('phase'),states=states)),flush=True);last_progress=time.monotonic()
        dead = [(name, proc.returncode) for name,proc in children if proc.poll() is not None]
        if dead:
            raise RuntimeError(f'Child exited: {dead}')
        if time.monotonic()>end:
            raise TimeoutError(json.dumps(dict(loc=loc, states=states, pose=body)))
def hold(seconds):
    end = now()+seconds
    wait(lambda: now()>=end, max(30,seconds*10))
def check(name, passed, **data):
    row = dict(test=name, passed=bool(passed), t=now(), **data)
    checks.append(row); save('check',row)
    (out/'checks.json').write_text(json.dumps(checks, indent=2)+'\n')
    print(json.dumps(row), flush=True)
    if not passed:
        raise AssertionError(name)
def getpose(msg):
    q=msg.orientation; p=msg.position
    return [p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
def on_loc(m):
    global loc
    loc=json.loads(m.data);save('localization_status',loc)
def on_body(m):
    global body
    body=getpose(m.pose.pose);save('body_pose',body)
    p=m.pose.pose;save('body_pose_stamped',dict(stamp_s=m.header.stamp.sec+m.header.stamp.nanosec/1e9,frame=m.header.frame_id,position=body[:3],quaternion_xyzw=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]))
def on_cmd(m):
    global command
    command=[m.linear.x,m.linear.y,m.angular.z];save('cmd_vel',command)
def on_cloud(m):
    global cloud_count
    cloud_count+=1
    if cloud_count%20==0:
        offsets={f.name:f.offset for f in m.fields};dt=np.dtype(dict(names=['x','y','z'],formats=['<f4']*3,offsets=[offsets[k] for k in ['x','y','z']],itemsize=m.point_step));v=np.frombuffer(m.data,dtype=dt);pts=np.column_stack([v[k] for k in ['x','y','z']])[::5]
        if m.header.frame_id!='map':raise RuntimeError('Cloud not in map frame')
        cloud_samples.append(pts);cloud_sample_times.append(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
def on_spline(m):
    global spline_count
    spline_count+=1
    save('spline',dict(id=m.traj_id,order=m.order,knots=list(m.knots),start_time_s=m.start_time.sec+m.start_time.nanosec/1e9,points=[[v.x,v.y,v.z] for v in m.pos_pts]))
def on_state(name,m):
    states[name]=dict(task_id=m.task_id,state=m.state);save(name+'_state',states[name])
latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/localization/status',on_loc,latched)
n.create_subscription(Odometry,'/navigation/body_pose',on_body,qos_profile_sensor_data)
n.create_subscription(Twist,'/cmd_vel',on_cmd,100)
n.create_subscription(PointCloud2,'/navigation/cloud',on_cloud,qos_profile_sensor_data)
n.create_subscription(Bspline,'/navigation/planning/bspline',on_spline,20)
n.create_subscription(NavPath,'/navigation/global_path',lambda m:save('global_path',[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in m.poses]),10)
for name in ('planner','controller'):
    n.create_subscription(NavigationState,'/navigation/planning/'+name+'_state',lambda m,name=name:on_state(name,m),latched)
initial_pub=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
def pose(values):
    m=PoseStamped();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg()
    m.pose.position.x=float(values[0]);m.pose.position.y=float(values[1])
    m.pose.orientation.z=math.sin(values[2]/2);m.pose.orientation.w=math.cos(values[2]/2)
    return m
def execute(kind, targets):
    global active
    name='navigate_to_pose' if kind==NavigateToPose else 'follow_waypoints'
    client=ActionClient(n,kind,'/'+name);wait(client.server_is_ready,30)
    goal=kind.Goal()
    if kind==NavigateToPose:goal.pose=pose(targets[0])
    else:goal.poses=[pose(v) for v in targets]
    save('request',dict(action=name,targets=targets))
    last_index=[None]
    def feedback(m):
        f=m.feedback
        if hasattr(f,'current_waypoint') and f.current_waypoint!=last_index[0]:
            last_index[0]=f.current_waypoint;save('waypoint_index',int(f.current_waypoint))
            print('WAYPOINT',f.current_waypoint,'SIM',now(),flush=True)
    f=client.send_goal_async(goal,feedback_callback=feedback);wait(f.done,30)
    active=f.result();check(name+'_accepted',active.accepted)
    result=active.get_result_async();wait(result.done,2400)
    response=result.result();save('result',dict(action=name,status=response.status,pose=body))
    check(name+'_succeeded',response.status==4 and not list(getattr(response.result,'missed_waypoints',[])),status=response.status,pose=body,
          missed_waypoints=list(getattr(response.result,'missed_waypoints',[])))
    active=None

try:
    config=yaml.safe_load((root/'ros2_ws/src/quadruped_navigation/config/navigation.yaml').read_text())
    # Match the existing simulation receiver's declared angular-speed bound.
    config['/navigation/trajectory_controller']['ros__parameters']['max_vyaw']=.3
    config['/navigation/trajectory_controller']['ros__parameters']['kp_pos']=2.0
    config['/navigation/navigation_api']['ros__parameters']['goal_timeout']=600.
    (out/'navigation.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    loc_config=yaml.safe_load((root/'ros2_ws/src/quadruped_localization/config/localization.yaml').read_text())
    loc_config['registration']['tracking_level_start']=1
    loc_config['runtime']['stationary_linear_m_s']=.12
    (out/'localization.yaml').write_text(yaml.safe_dump(loc_config,sort_keys=False))
    launch('localization',['/usr/bin/python3',str(root/'scripts/run_localization.py'),'--map-id','d1_full_room_web_20261117_01','--with-lio','--base-config-file',str(root/'ros2_ws/src/quadruped_localization/config/base_odometry_dynamic_gait.yaml'),'--localization-config-file',str(out/'localization.yaml'),'--output-dir',str(out/'localization')])
    launch('navigation',['ros2','launch','quadruped_navigation','navigation.launch.py','use_sim_time:=true','map_package:=/data0/rl/quadruped_nav/maps/d1_full_room_web_20261117_01','params_file:='+str(out/'navigation.yaml')])
    sim=launch('simulation',['/usr/bin/python3',str(root/'scripts/run_gait_cmd_vel.py'),'--standby-on-stop','--max-seconds','600','--gpu-index','1','--output-dir',str(out/'simulation')])
    wait(lambda:now()>5 and loc.get('phase')=='waiting_initial_pose',300)
    hold(2)
    check('waiting_for_explicit_initial_pose',not loc.get('initialized') and max(map(abs,command))<1e-8,phase=loc.get('phase'))
    requested=[.35,-.2,.12];save('initial_pose_request',requested)
    m=PoseWithCovarianceStamped();v=pose(requested);m.header=v.header;m.pose.pose=v.pose
    m.pose.covariance[0]=m.pose.covariance[7]=.25;m.pose.covariance[35]=.0685
    initial_pub.publish(m)
    wait(lambda:loc.get('initialized') and loc.get('phase')=='tracking' and body is not None and cloud_count>5,60)
    save('initial_pose_confirmed',dict(pose=body,quality=loc.get('quality')))
    check('initial_pose_corrected_by_real_registration',math.hypot(body[0]-requested[0],body[1]-requested[1])>.1,pose=body,quality=loc.get('quality'))
    hold(3)
    def reinitialize(label,offset):
        hold(3)
        requested=[body[0]+offset[0],body[1]+offset[1],body[3]+offset[2]]
        save('initial_pose_request',dict(case=label,pose=requested))
        value=pose(requested);m=PoseWithCovarianceStamped();m.header=value.header;m.pose.pose=value.pose;m.pose.covariance[0]=m.pose.covariance[7]=.25;m.pose.covariance[35]=.0685
        sent_ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec;initial_pub.publish(m)
        wait(lambda:loc.get('last_initial_pose_ns')==sent_ns and loc.get('initialized') and loc.get('phase')=='tracking',90)
        hold(1)
        check(label+'_tracking_after_explicit_seed',True,pose=body,requested=requested,quality=loc.get('quality'))
        save('initial_pose_confirmed',dict(case=label,pose=body,quality=loc.get('quality')))
    execute(NavigateToPose,[[.15,.45,.5]])
    reinitialize('second_location',[.25,-.2,.2])
    execute(FollowWaypoints,[[-.55,1.55,1.5],[-3.55,-1.85,-2.3],[-2.95,-.70,1.2]])
    reinitialize('third_location',[-.25,.2,-.2])
    execute(FollowWaypoints,[[-.65,-.65,0.],[4.05,-.55,0.],[0.,0.,0.]])
    hold(3)
    check('final_zero_command',max(map(abs,command))<1e-8,command=command)
    check('native_spline_controller_used',spline_count>2,count=spline_count)
    save_client=n.create_client(SaveNavigationMapPackage,'/navigation/map_package_manager/save_package')
    wait(save_client.service_is_ready,10)
    request=SaveNavigationMapPackage.Request();request.package_path=str(out/'navigation_map')
    f=save_client.call_async(request);wait(f.done,30)
    check('native_map_saved',f.result().success,message=f.result().message)
    (out/'verification.json').write_text(json.dumps(dict(result='PASS',scope='Isaac PhysX gait + raycast lidar/IMU + FAST-LIO + saved-map ICP + native upstream navigation; no hardware acceptance',truth_used_by_navigation=False,checks=checks),indent=2)+'\n')
except Exception:
    (out/'error.txt').write_text(traceback.format_exc());traceback.print_exc()
    (out/'verification.json').write_text(json.dumps(dict(result='FAIL',checks=checks),indent=2)+'\n')
    raise
finally:
    if active and active.accepted:
        try:
            f=active.cancel_goal_async()
            end=time.monotonic()+3
            while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
        except Exception:pass
    for name,proc in reversed(children):
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=25 if name=='simulation' else 15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL)
    if cloud_samples:np.savez_compressed(out/'cloud_samples.npz',points=np.concatenate(cloud_samples),counts=np.array([len(v) for v in cloud_samples]),stamp_s=np.array(cloud_sample_times))
    events.close();n.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    for h in handles:h.close()
