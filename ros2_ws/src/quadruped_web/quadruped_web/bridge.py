# SPDX-License-Identifier: MIT
import copy,json,math,os,queue,threading,time,concurrent.futures,uuid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Twist
from nav_msgs.msg import Odometry,Path
from nav2_msgs.action import NavigateToPose,FollowWaypoints
from action_msgs.srv import CancelGoal
from sensor_msgs.msg import PointCloud2,Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from quadruped_interfaces.msg import NavigationState,Bspline
from .control import VelocityRouter

class Bridge(Node):
 def __init__(self,cfg,map_data,mode,event_file):
  super().__init__('quadruped_web_bridge_'+str(os.getpid()),parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=mode=='sim')])
  self.cfg=cfg;self.map_data=map_data;self.environment=mode;self.events=event_file;self.lock=threading.RLock();self.queue=queue.Queue();self.router=VelocityRouter(cfg);self.goal_handle=None;self.task={'phase':'idle'};self.loc={};self.seen={};self.pose=None;self.paths={};self.cloud=[];self.states={};self.raw_lio=None;self.odometry=None;self.observed_cmd=[0.,0.,0.];self.conflict=False;self.map_switching=False;self.last_conflict_check=0.;self.started=time.monotonic();self.last_record=0.;self.last_published=[0.,0.,0.];self.stopping=False;self.task_epoch=0;self.callback_error=None
  self.action_clients={name:ActionClient(self,kind,'/'+name) for name,kind in [('navigate_to_pose',NavigateToPose),('follow_waypoints',FollowWaypoints)]}
  self.cancel_clients=[self.create_client(CancelGoal,'/'+name+'/_action/cancel_goal') for name in self.action_clients]
  self.initial_pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10);self.velocity_pub=self.create_publisher(Twist,cfg['output_velocity_topic'],10)
  latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.create_subscription(String,'/localization/status',self.wrap(self.on_localization),latched)
  self.create_subscription(PoseStamped,'/localization/pose',self.wrap(self.on_pose),20)
  self.create_subscription(Odometry,'/odom',self.wrap(lambda m:self.on_odom('odom',m)),qos_profile_sensor_data)
  self.create_subscription(Odometry,'/lio/odometry',self.wrap(lambda m:self.on_odom('lio',m)),qos_profile_sensor_data)
  self.create_subscription(PointCloud2,'/lidar/points',self.wrap(lambda m:self.mark('lidar')),qos_profile_sensor_data)
  self.create_subscription(Imu,'/imu/data',self.wrap(lambda m:self.mark('imu')),qos_profile_sensor_data)
  self.create_subscription(PointCloud2,'/navigation/cloud',self.wrap(self.on_cloud),qos_profile_sensor_data)
  self.create_subscription(Twist,cfg['navigation_velocity_topic'],self.wrap(self.on_nav_velocity),20)
  self.create_subscription(Twist,cfg['output_velocity_topic'],self.wrap(lambda m:self.mark('velocity_echo')),20)
  self.create_subscription(Path,'/navigation/global_path',self.wrap(self.on_path),10)
  self.create_subscription(Bspline,'/navigation/planning/bspline',self.wrap(self.on_spline),10)
  for name in ['planner','controller']:
   self.create_subscription(NavigationState,'/navigation/planning/'+name+'_state',self.wrap(lambda m,name=name:self.on_state(name,m)),latched)
  self.wall_clock=Clock(clock_type=ClockType.STEADY_TIME);self.create_timer(.02,self.wrap(self.process_requests),clock=self.wall_clock);self.create_timer(.05,self.wrap(self.tick),clock=self.wall_clock)
 def wrap(self,fn):
  def callback(*args):
   with self.lock:
    try:return fn(*args)
    except Exception as error:
     self.callback_error=type(error).__name__+': '+str(error);self.router.stop('bridge callback error')
     try:self.publish_velocity([0.,0.,0.]);self.event('bridge_callback_error',self.callback_error)
     except Exception:pass
  return callback
 def mark(self,key):self.seen[key]=time.monotonic()
 def event(self,kind,data):
  self.events.write(json.dumps(dict(wall=time.time(),sim_s=self.get_clock().now().nanoseconds/1e9,kind=kind,data=data),allow_nan=False)+'\n');self.events.flush()
 def rpc(self,fn,timeout=8):
  result=concurrent.futures.Future();self.queue.put((fn,result));return result.result(timeout)
 def process_requests(self):
  for _ in range(15):
   try:fn,f=self.queue.get_nowait()
   except queue.Empty:break
   if f.cancelled():continue
   try:f.set_result(fn())
   except Exception as error:f.set_exception(error)
 def fresh(self,key,age=1.):return time.monotonic()-self.seen.get(key,-1e9)<age
 def on_localization(self,m):
  self.loc=json.loads(m.data);self.mark('localization')
 def on_pose(self,m):
  if m.header.frame_id!='map':return
  p=m.pose.position;q=m.pose.orientation
  self.pose=dict(x=p.x,y=p.y,z=p.z,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),quaternion=[q.x,q.y,q.z,q.w],stamp_s=m.header.stamp.sec+m.header.stamp.nanosec/1e9);self.mark('pose')
 def on_odom(self,key,m):
  self.mark(key)
  if key=='odom':self.odometry=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]
 def on_path(self,m):
  if m.header.frame_id=='map':self.paths['global']=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in m.poses]
 def on_spline(self,m):
  try:
   from scipy.interpolate import BSpline
   knots=np.asarray(m.knots);start=float(knots[m.order]);end=float(knots[-m.order-1]);spline=BSpline(knots,[[p.x,p.y,p.z] for p in m.pos_pts],m.order)
   self.paths['local']=spline(np.linspace(start,end,min(2000,max(2,int((end-start)/.05))))).tolist()
  except Exception as error:self.event('spline_display_error',str(error))
 def on_state(self,name,m):
  self.states[name]=dict(state=m.state,task_id=m.task_id);self.mark(name)
 def on_nav_velocity(self,m):
  v=[m.linear.x,m.linear.y,m.angular.z]
  if all(math.isfinite(x) for x in v) and math.hypot(*v[:2])<=.500001 and abs(v[2])<=.300001:self.router.nav=v;self.router.nav_stamp=time.monotonic()
  else:self.router.nav=[0.,0.,0.];self.router.nav_stamp=0.;self.event('invalid_navigation_velocity',v)
 def on_cloud(self,m):
  self.mark('cloud')
  if m.header.frame_id!='map' or time.monotonic()-self.seen.get('cloud_display',0)<.5:return
  self.mark('cloud_display');raw=point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True);points=np.column_stack([raw[k] for k in ('x','y','z')]).astype(float) if isinstance(raw,np.ndarray) and raw.dtype.names else np.asarray(list(raw),dtype=float)
  if points.size:self.cloud=points[::max(1,math.ceil(len(points)/5000))].tolist()
 def localization_ready(self):
  return not self.callback_error and self.fresh('pose',.8) and self.fresh('localization',1.) and self.loc.get('initialized') and self.loc.get('phase')=='tracking' and self.loc.get('map',{}).get('localization_sha256')==self.map_data['localization_sha256']
 def inputs_ready(self):return self.fresh('imu',1.) and self.fresh('lidar',1.)
 def publish_velocity(self,v):
  m=Twist();m.linear.x,m.linear.y,m.angular.z=map(float,v);self.velocity_pub.publish(m);self.last_published=v.copy()
 def tick(self):
  now=time.monotonic()
  if now-self.last_conflict_check>.5:
   self.last_conflict_check=now
   self.conflict=any(p.node_name!=self.get_name() or p.node_namespace!=self.get_namespace() for p in self.get_publishers_info_by_topic(self.cfg['output_velocity_topic']))
  health=not self.callback_error and self.inputs_ready() and not self.conflict and not self.map_switching and not self.stopping
  if self.router.mode=='navigation':health=health and self.localization_ready()
  v=self.router.tick(now,healthy=health);self.publish_velocity(v)
  if self.router.expired:self.router.expired=False;self.cancel_navigation('browser lease expired')
  if self.router.mode=='navigation' and not health and self.task.get('phase') in ('submitting','executing'):
   self.router.stop('navigation inputs lost');self.cancel_navigation('navigation inputs lost')
  if now-self.last_record>.1:
   self.last_record=now;self.event('control_sample',dict(mode=self.router.mode,output=v,reason=self.router.reason,pose=self.pose,task_phase=self.task.get('phase')))
 def active(self):return self.task.get('phase') in ('submitting','executing','canceling')
 def acquire(self,mode,owner):
  if self.map_switching:raise ValueError('Map is switching')
  if mode=='manual':
   if self.active() or any(x.get('task_id') for x in self.states.values()):raise ValueError('Cancel navigation and wait for stop before manual control')
  if mode!='idle' and (not self.inputs_ready() or self.conflict):raise ValueError('Sensor inputs unavailable or another velocity publisher is active')
  self.router.acquire(mode,owner);self.event('control_mode',dict(mode=mode,owner=owner));return dict(mode=mode)
 def cancel_navigation(self,reason):
  if self.goal_handle:
   self.goal_handle.cancel_goal_async()
  if self.active():self.task['phase']='canceling';self.task['message']=reason
  for c in self.cancel_clients:
   if c.service_is_ready():c.call_async(CancelGoal.Request())
 def stop(self,reason='user stop'):
  self.router.stop(reason);self.publish_velocity([0.,0.,0.]);self.cancel_navigation(reason);self.event('stop_requested',reason)
  return dict(requested=True,confirmed=False,message='Stop requested; awaiting controller acknowledgment')
 def initial(self,pose,owner):
  if self.active() or self.router.mode=='manual':raise ValueError('Stop current motion before initializing')
  if not self.inputs_ready():raise ValueError('Sensor inputs are not ready')
  self.validate_pose(pose,initial=True);m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();self.fill_pose(m.pose.pose,pose)
  if m.header.stamp.sec<=0 and m.header.stamp.nanosec<=0:raise ValueError('Clock is not ready')
  m.pose.covariance[0]=m.pose.covariance[7]=.25;m.pose.covariance[35]=math.radians(15)**2
  self.initial_pub.publish(m);self.event('initialpose',pose);return dict(sent=True,accepted_by_localizer=False)
 def validate_pose(self,p,initial=False):
  if not isinstance(p,dict) or not all(k in p for k in ('x','y','z','yaw')):raise ValueError('Expected x, y, z, yaw')
  if not all(isinstance(p[k],(int,float)) and not isinstance(p[k],bool) and math.isfinite(p[k]) and abs(p[k])<1e4 for k in ('x','y','z','yaw')):raise ValueError('Invalid pose')
  support=np.asarray(self.map_data['support_points']);z=p['z']-(self.map_data['robot']['base_height_above_support_m'] if initial else 0.)
  d=np.linalg.norm(support[:,:2]-[p['x'],p['y']],axis=1);valid=(d<=self.map_data['resolution']*1.1)&(abs(support[:,2]-z)<=self.map_data['resolution']*.8)
  if not valid.any():raise ValueError('Selected position is not on a traversable support surface')
 def fill_pose(self,target,p):
  target.position.x=float(p['x']);target.position.y=float(p['y']);target.position.z=float(p['z']);target.orientation.z=math.sin(p['yaw']/2);target.orientation.w=math.cos(p['yaw']/2)
 def submit(self,poses,kind,owner):
  if self.active():raise ValueError('A navigation task is active')
  if self.router.mode=='manual':raise ValueError('Release manual control before navigation')
  if not self.localization_ready() or not self.inputs_ready() or self.conflict:raise ValueError('Localization, sensor inputs or velocity authority not ready')
  if not isinstance(poses,list) or not 1<=len(poses)<=100:raise ValueError('Provide 1..100 waypoints')
  if kind not in self.action_clients or kind=='navigate_to_pose' and len(poses)!=1:raise ValueError('Invalid navigation kind')
  for p in poses:self.validate_pose(p)
  client=self.action_clients[kind]
  if not client.server_is_ready():raise ValueError('Navigation action server is not ready')
  self.router.acquire('navigation',owner);self.task_epoch+=1;epoch=self.task_epoch
  self.task=dict(id=uuid.uuid4().hex,phase='submitting',kind=kind,poses=poses,current_waypoint=0);self.event('navigation_requested',self.task)
  goal=NavigateToPose.Goal() if kind=='navigate_to_pose' else FollowWaypoints.Goal();messages=[]
  for p in poses:
   msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg();self.fill_pose(msg.pose,p);messages.append(msg)
  if kind=='navigate_to_pose':goal.pose=messages[0]
  else:goal.poses=messages
  def feedback(m):
   if epoch!=self.task_epoch:return
   f=m.feedback
   if hasattr(f,'current_waypoint'):self.task['current_waypoint']=int(f.current_waypoint)
   if hasattr(f,'distance_remaining'):self.task['distance_remaining_m']=float(f.distance_remaining)
  future=client.send_goal_async(goal,feedback_callback=self.wrap(feedback))
  def accepted(f):
   handle=f.result();self.goal_handle=handle
   if not handle.accepted:
    self.task.update(phase='rejected',message='Navigation action rejected request');self.router.stop('task rejected');self.event('navigation_rejected',self.task);return
   if epoch!=self.task_epoch or self.task['phase']=='canceling':handle.cancel_goal_async()
   else:self.task['phase']='executing'
   self.event('navigation_accepted',self.task)
   def completed(f):
    response=f.result()
    if epoch!=self.task_epoch:return
    self.task.update(phase={4:'succeeded',5:'canceled',6:'failed'}.get(response.status,'failed'),status=response.status,missed_waypoints=list(getattr(response.result,'missed_waypoints',[])))
    self.router.stop('task finished');self.goal_handle=None;self.event('navigation_result',self.task)
   handle.get_result_async().add_done_callback(self.wrap(completed))
  future.add_done_callback(self.wrap(accepted));return copy.deepcopy(self.task)
 def snapshot(self):
  with self.lock:
   now=time.monotonic();ages={key:round(now-t,3) for key,t in self.seen.items()};pose=self.pose if self.fresh('pose',1.) else None
   stopped=not any(abs(v)>1e-8 for v in self.last_published) and self.states.get('controller',{}).get('state')=='IDLE' and self.fresh('controller',2.) and not self.active()
   return copy.deepcopy(dict(server_time=time.time(),bridge_error=self.callback_error,clock_s=self.get_clock().now().nanoseconds/1e9,environment=self.environment,map_id=self.map_data['id'],map_revision=self.map_data['revision'],map_switching=self.map_switching,localization=self.loc,localization_ready=bool(self.localization_ready()),pose=pose,pose_stale=self.pose is not None and pose is None,ages_s=ages,inputs_ready=self.inputs_ready(),control=dict(mode=self.router.mode,owner=self.router.owner,command=self.last_published,reason=self.router.reason,conflicting_publisher=self.conflict,stop_confirmed=stopped),actual_velocity=self.odometry if self.fresh('odom') else None,task=self.task,states=self.states,paths=self.paths,cloud=self.cloud if self.fresh('cloud',1.) else [],capabilities=dict(camera=False,preset_actions=False,manual=True),known_limits=self.map_data['known_limits']))
