#!/usr/bin/env python3
"""One complete Isaac -> FAST-LIO -> explicit initialpose -> map-localizer trial."""
import argparse,os,sys,json,time,signal,subprocess,traceback,math
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--x',type=float,required=True);p.add_argument('--y',type=float,required=True);p.add_argument('--yaw-deg',type=float,required=True);p.add_argument('--seed-dx',type=float,default=.35);p.add_argument('--seed-dy',type=float,default=-.2);p.add_argument('--seed-yaw-error-deg',type=float,default=10);a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True)
os.environ.update(ROS_DOMAIN_ID='42',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',QUADRUPED_SIM_CAPTURE_RGB='0',FASTRTPS_DEFAULT_PROFILES_FILE=str(a.root/'ros2_ws/src/quadruped_bringup/config/dds/local_sim.xml'))
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from std_msgs.msg import String
rclpy.init();n=Node('initial_pose_coverage_recorder',parameter_overrides=[Parameter('use_sim_time',value=True)])
children=[];handles=[];loc={};poses=[];start=time.monotonic();events=(a.output/'events.jsonl').open('w',buffering=1)
def now():return n.get_clock().now().nanoseconds/1e9
def save(kind,data):events.write(json.dumps(dict(kind=kind,t=now(),wall_s=time.monotonic()-start,data=data))+'\n')
def on_loc(m):
 global loc
 loc=json.loads(m.data);save('status',loc)
def on_pose(m):
 q=m.pose.orientation;p=m.pose.position;v=dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,position=[p.x,p.y,p.z],quat_xyzw=[q.x,q.y,q.z,q.w]);poses.append(v);save('pose',v)
q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/localization/status',on_loc,q);n.create_subscription(PoseStamped,'/localization/pose',on_pose,100);pub=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
def launch(name,args,env=None):
 h=(a.output/(name+'.log')).open('w');handles.append(h);proc=subprocess.Popen(args,env=env,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);children.append((name,proc));return proc
def wait(pred,timeout):
 end=time.monotonic()+timeout
 while not pred():
  rclpy.spin_once(n,timeout_sec=.02)
  dead=[(name,p.returncode) for name,p in children if p.poll() is not None]
  if dead:raise RuntimeError('Children exited '+str(dead))
  if time.monotonic()>end:raise TimeoutError(json.dumps(dict(clock=now(),status=loc,poses=len(poses))))
result=dict(requested_spawn=[a.x,a.y,a.yaw_deg],seed_error=[a.seed_dx,a.seed_dy,a.seed_yaw_error_deg],initialpose_publish_count=0,truth_used_for_localization=False)
try:
 config=a.root/'data/runs/navigation_core_isaac_demo_20261112_01/localization.yaml'
 # Preserve the exact previously demonstrated configuration.
 if not config.is_file():config=Path('/data0/rl/quadruped_nav/runs/navigation_core_isaac_demo_20261112_01/localization.yaml')
 (a.output/'localization.yaml').write_bytes(config.read_bytes())
 launch('localization',['/usr/bin/python3',str(a.root/'scripts/run_localization.py'),'--map-id','d1_full_room_20261109_02','--with-lio','--base-config-file',str(a.root/'ros2_ws/src/quadruped_localization/config/base_odometry_dynamic_gait.yaml'),'--localization-config-file',str(config),'--output-dir',str(a.output/'localization')])
 env=dict(os.environ);env.pop('PYTHONPATH',None);env.update(ROS_DISTRO='humble',LD_LIBRARY_PATH='/home/rl/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:'+env.get('LD_LIBRARY_PATH',''))
 launch('simulation',['/home/rl/isaacsim/python.sh',str(a.root/'tests/gait_initial_pose_probe.py'),'--test-root',str(a.root),'--test-map-x',str(a.x),'--test-map-y',str(a.y),'--test-map-yaw-deg',str(a.yaw_deg),'--project-root',str(a.root),'--output-dir',str(a.output/'simulation'),'--model','loaded','--policy','--standby-on-stop','--gpu-index','1','--max-seconds','150'],env)
 wait(lambda:now()>5 and loc.get('phase')=='waiting_initial_pose',300)
 t=now();wait(lambda:now()>t+1,30)
 result['uninitialized_before_request']=not bool(loc.get('initialized'));save('before_initialpose',loc)
 seed=[a.x+a.seed_dx,a.y+a.seed_dy,math.radians(a.yaw_deg+a.seed_yaw_error_deg)]
 m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg();m.pose.pose.position.x=seed[0];m.pose.pose.position.y=seed[1];m.pose.pose.orientation.z=math.sin(seed[2]/2);m.pose.pose.orientation.w=math.cos(seed[2]/2);m.pose.covariance[0]=m.pose.covariance[7]=.25;m.pose.covariance[35]=.0685
 pub.publish(m);result['initialpose_publish_count']=1;result['seed']=seed;result['request_clock_s']=now();save('initialpose',seed)
 wait(lambda:loc.get('initialized') and loc.get('phase')=='tracking' and len(poses)>0,90)
 result['first_tracking_clock_s']=now();save('first_tracking',loc)
 t=now();wait(lambda:now()>t+6,100)
 result.update(result='TRACKING_OBSERVED',last_status=loc,pose_count=len(poses))
except Exception:
 result.update(result='FAIL',error=traceback.format_exc(),last_status=loc,pose_count=len(poses));traceback.print_exc()
finally:
 for name,proc in reversed(children):
  if proc.poll() is None:
   os.killpg(proc.pid,signal.SIGINT)
   try:proc.wait(timeout=35)
   except subprocess.TimeoutExpired:
    os.killpg(proc.pid,signal.SIGTERM)
    try:proc.wait(timeout=15)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5)
 result['wall_s']=time.monotonic()-start
 (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');(a.output/'poses.json').write_text(json.dumps(poses)+'\n');events.close();n.destroy_node();rclpy.shutdown()
 for h in handles:h.close()
 print(json.dumps(result),flush=True)
