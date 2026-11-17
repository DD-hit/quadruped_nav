#!/usr/bin/env python3
"""ROS cmd_vel D1 gait with PD standby until the first motion command."""
import argparse,os
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--project-root',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--model',choices=['reference','loaded'],default='loaded');p.add_argument('--walk',action='store_true');p.add_argument('--speed',type=float,default=.2);p.add_argument('--yaw-rate',type=float,default=0.);p.add_argument('--teleop',action='store_true');p.add_argument('--linear-speed',type=float,default=.5);p.add_argument('--turn-speed',type=float,default=.3);p.add_argument('--max-seconds',type=float,default=300.);p.add_argument('--pd-only',action='store_true');p.add_argument('--policy',action='store_true');p.add_argument('--gui',action='store_true');p.add_argument('--gpu-index',type=int,default=0);a=p.parse_args();a.pd_only=not a.policy
if a.walk and not a.policy:p.error('--walk requires --policy')
if a.teleop and (not a.gui or not a.policy or a.walk):p.error('--teleop requires --gui --policy and cannot be combined with --walk')
if not (-1.<=a.speed<=1. and -1.<=a.yaw_rate<=1.):p.error('speed and yaw-rate must be within [-1, 1]')
if not (0.<a.linear_speed<=1. and 0.<a.turn_speed<=1. and .1<=a.max_seconds<=3600.):p.error('teleop speeds must be within (0, 1] and max-seconds within [0.1, 3600]')
debug_first_scan=os.environ.get('D1_SENSOR_DEBUG_FIRST_SCAN')=='1'
if not a.policy or a.teleop or a.gui or a.model!='loaded':p.error('ROS gait requires --policy --model loaded, headless')
out=a.output_dir;out.mkdir(parents=True,exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp(dict(headless=not a.gui,active_gpu=a.gpu_index,physics_gpu=a.gpu_index,multi_gpu=False,renderer='RaytracedLighting',width=960,height=720))
import json,math,time,signal,traceback,hashlib
import numpy as np,yaml,torch
from scipy.spatial.transform import Rotation
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.prims import RigidPrim
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,UsdLux
from runtime_gait_lidar import create_validation_room,GeometricLidar
from PIL import Image
import omni.replicator.core as rep
import omni.usd,omni.kit.commands
if a.teleop:
 import ctypes
 from omni.kit.viewport.utility import get_active_viewport
code=1;world=None;robot=None;rows=[];stopping=False

def stop(sig,frame):
 global stopping
 stopping=True
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
try:
 root=a.project_root;ref=root/'data/assets/d1_edu/gait_source';base=yaml.safe_load((ref/'policy/d1/base.yaml').read_text())['d1'];cfg=yaml.safe_load((ref/'policy/d1/robot_lab/config.yaml').read_text())['d1/robot_lab']
 dt=float(base['dt']);dec=int(base['decimation']);torch.set_num_threads(1);policy_file=ref/'policy/d1/robot_lab/policy.pt'
 assert hashlib.sha256(policy_file.read_bytes()).hexdigest()=='64919894098b80af2f18b6bf191c85ff0840e460ae02acf36ea4eb4ca4c4977f','Candidate policy hash mismatch'
 model=torch.jit.load(str(policy_file),map_location='cpu').eval()
 asset=Path('/data0/rl/quadruped_nav/assets/d1_edu/model_review/d1_edu.usd')
 if a.model=='reference':
  enable_extension('isaacsim.asset.importer.urdf');ok,config=omni.kit.commands.execute('URDFCreateImportConfig');assert ok
  config.merge_fixed_joints=False;config.import_inertia_tensor=True;config.fix_base=False;config.distance_scale=1.;config.default_drive_strength=0.;config.default_position_drive_damping=0.
  asset=out/'reference.usd';ok,prim=omni.kit.commands.execute('URDFParseAndImportFile',urdf_path=str(ref/'src/rl_sar_zoo/d1_description/urdf/d1_description.urdf'),import_config=config,dest_path=str(asset));assert ok,prim
  ss=Usd.Stage.Open(str(asset));ss.SetDefaultPrim(ss.GetPrimAtPath('/edu_urdf'));ss.GetRootLayer().Save();del ss
 omni.usd.get_context().new_stage()
 world=World(physics_dt=dt,rendering_dt=1/30,stage_units_in_meters=1.,backend='numpy')
 world.get_physics_context().set_enable_scene_query_support(True)
 stage=omni.usd.get_context().get_stage();UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
 stage.DefinePrim('/World/D1','Xform').GetReferences().AddReference(str(asset));stage.Load()
 if a.model=='loaded':
  # The URDF IMU is a coordinate frame, not an additional 1 kg body.
  imu=stage.GetPrimAtPath('/World/D1/lidar_imu_link')
  mass=UsdPhysics.MassAPI.Apply(imu);mass.CreateMassAttr(1e-6);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-9,1e-9,1e-9))
 mat=PhysicsMaterial('/World/GroundMaterial',static_friction=.4,dynamic_friction=.4,restitution=0.)
 world.scene.add_default_ground_plane(static_friction=.4,dynamic_friction=.4,restitution=0.)
 robot=world.scene.add(Robot(prim_path='/World/D1',name='gait_d1',position=np.array([0.,0.,.42])))
 feet=world.scene.add(RigidPrim(prim_paths_expr='/World/D1/.*_FOOT_LINK',name='feet',track_contact_forces=True))
 for prim in stage.Traverse():
  if prim.HasAPI(UsdPhysics.DriveAPI):
   drive=UsdPhysics.DriveAPI.Get(prim,'angular')
   if drive:
    drive.GetStiffnessAttr().Set(0.);drive.GetDampingAttr().Set(0.)
 lidar_cfg=yaml.safe_load((root/'ros2_ws/src/quadruped_sim/config/geometric_lidar.yaml').read_text())
 lidar_cfg['samples_per_scan']=6000
 # Leave clearance for the moving D1, while keeping the configured room geometry.
 for name in ('floor','ceiling'):
  lidar_cfg['room'][name]['size'][0]=12.4
 for name,coordinate in (('east',6.1),('west',-6.1)):
  lidar_cfg['room'][name]['center'][0]=coordinate
 lidar_cfg['room']['box']['center'][1]=1.8
 create_validation_room(stage,lidar_cfg)
 enable_extension('isaacsim.ros2.bridge')
 for _ in range(3):app.update()
 import rclpy
 from rclpy.node import Node
 from rclpy.qos import QoSProfile,ReliabilityPolicy
 from rclpy.signals import SignalHandlerOptions
 from sensor_msgs.msg import Imu
 from geometry_msgs.msg import Twist
 from rosgraph_msgs.msg import Clock
 light=UsdLux.DomeLight.Define(stage,'/World/Light');light.CreateIntensityAttr(1000.)
 if a.teleop:
  follow=UsdGeom.Camera.Define(stage,'/World/TeleopFollowCamera')
  follow.CreateFocalLengthAttr(24.)
  follow_op=UsdGeom.Xformable(follow.GetPrim()).AddTransformOp()
 else:
  camera=rep.create.camera(position=(1.5,1.5,1.),look_at=(.1,0,.3));product=rep.create.render_product(camera,(960,720));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([product])
 world.reset();print('DOFS',robot.dof_names,flush=True)
 if a.teleop:
  viewport=get_active_viewport()
  if viewport is None:raise RuntimeError('No active Isaac viewport; use the rl desktop DISPLAY=:2 with --gui --gpu-index 1')
  viewport.set_active_camera('/World/TeleopFollowCamera')
  # X11 key state is polled because Kit's appwindow keyboard does not deliver
  # events in this standalone SimulationApp loop on the server's Xorg desktop.
  xlib=ctypes.CDLL('libX11.so.6')
  xlib.XOpenDisplay.argtypes=[ctypes.c_char_p];xlib.XOpenDisplay.restype=ctypes.c_void_p
  xlib.XStringToKeysym.argtypes=[ctypes.c_char_p];xlib.XStringToKeysym.restype=ctypes.c_ulong
  xlib.XKeysymToKeycode.argtypes=[ctypes.c_void_p,ctypes.c_ulong];xlib.XKeysymToKeycode.restype=ctypes.c_ubyte
  xlib.XQueryKeymap.argtypes=[ctypes.c_void_p,ctypes.c_void_p];xlib.XQueryKeymap.restype=ctypes.c_int
  xlib.XGetInputFocus.argtypes=[ctypes.c_void_p,ctypes.POINTER(ctypes.c_ulong),ctypes.POINTER(ctypes.c_int)]
  xlib.XFetchName.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.POINTER(ctypes.c_char_p)];xlib.XFetchName.restype=ctypes.c_int
  xlib.XFree.argtypes=[ctypes.c_void_p];xlib.XCloseDisplay.argtypes=[ctypes.c_void_p]
  display=xlib.XOpenDisplay(os.environ.get('DISPLAY','').encode())
  if not display:raise RuntimeError('Cannot open X11 display for keyboard teleop')
  keycodes={name:xlib.XKeysymToKeycode(display,xlib.XStringToKeysym(name.encode())) for name in ('w','s','a','d','q','e','space','Escape')}
  if any(code==0 for code in keycodes.values()):raise RuntimeError('X11 keyboard layout lacks a teleop key')
  focus_window=0;focus_is_isaac=False
  def read_keys():
   global focus_window,focus_is_isaac
   window=ctypes.c_ulong();revert=ctypes.c_int()
   xlib.XGetInputFocus(display,ctypes.byref(window),ctypes.byref(revert))
   if window.value!=focus_window:
    focus_window=window.value;name=ctypes.c_char_p();focus_is_isaac=False
    if xlib.XFetchName(display,focus_window,ctypes.byref(name)) and name.value:
     focus_is_isaac=b'Isaac Sim Python' in name.value
     xlib.XFree(ctypes.cast(name,ctypes.c_void_p))
   if not focus_is_isaac:return set()
   state=ctypes.create_string_buffer(32);xlib.XQueryKeymap(display,state)
   return {name for name,code in keycodes.items() if state.raw[code//8]&(1<<(code%8))}
  def follow_base(position):
   eye=Gf.Vec3d(float(position[0])+1.4,float(position[1])+2.,float(position[2])+1.)
   target=Gf.Vec3d(float(position[0]),float(position[1]),float(position[2]))
   follow_op.Set(Gf.Matrix4d(1.).SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse())
  print('TELEOP_READY W/S forward/back A/D left/right Q/E turn; SPACE stop; ESC exit. Click the Isaac window to focus.',flush=True)
 names=list(robot.dof_names);mapping=np.array([names.index(n) for n in base['joint_names']]);assert len(names)==12
 controller=robot.get_articulation_controller();controller.set_gains(np.zeros(12),np.zeros(12));controller.switch_control_mode('effort')
 robot.set_solver_position_iteration_count(16);robot.set_solver_velocity_iteration_count(4)
 nominal=np.array(cfg['default_dof_pos']);initial=np.zeros(12);initial[mapping]=nominal;robot.set_joint_positions(initial);robot.set_joint_velocities(np.zeros(12));robot.set_linear_velocity(np.zeros(3));robot.set_angular_velocity(np.zeros(3))
 previous=np.zeros(12);target=nominal.copy();kp=np.array(cfg['rl_kp']);kd=np.array(cfg['rl_kd']);scale=np.array(cfg['action_scale']);limit=np.array(cfg['torque_limits']);obs=np.zeros(45);runtime=time.monotonic()
 masses=np.asarray(robot._articulation_view.get_body_masses()).reshape(-1)
 print('PHYSICS_MASSES',masses.tolist(),'TOTAL',float(masses.sum()),flush=True)
 (out/'ready.json').write_text(json.dumps(dict(model=a.model,dof_names=names,physics_masses_kg=masses.tolist(),total_mass_kg=float(masses.sum()),policy_mapping=mapping.tolist(),physics_hz=1/dt,policy_hz=1/dt/dec,hardware=False),indent=2))
 profile=yaml.safe_load((root/'ros2_ws/src/quadruped_bringup/config/robots/d1_edu.yaml').read_text())
 common=yaml.safe_load((root/'ros2_ws/src/quadruped_bringup/config/common.yaml').read_text())
 def spec_matrix(spec):
  matrix=np.eye(4);matrix[:3,:3]=Rotation.from_euler('xyz',spec['rpy_rad']).as_matrix();matrix[:3,3]=spec['translation_m'];return matrix
 expected=spec_matrix(profile['sensor']['base_to_lidar'])@spec_matrix(profile['sensor']['lidar_to_imu'])
 base_prim=stage.GetPrimAtPath('/World/D1/'+profile['model']['target_root_frame'])
 imu_prim=stage.GetPrimAtPath('/World/D1/'+profile['frame_aliases']['imu'])
 tb=np.array(UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
 ti=np.array(UsdGeom.Xformable(imu_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
 actual=np.linalg.inv(tb)@ti
 mount_error=float(np.max(np.abs(actual-expected)))
 if mount_error>1e-6:raise RuntimeError(f'IMU mount mismatch {mount_error}')
 r_bi=Rotation.from_matrix(actual[:3,:3]);lever=actual[:3,3]
 rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
 node=Node('d1_runtime_gait_sensors');node.epoch=0;node.obstacles=None
 qos=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT)
 received_cmd=np.zeros(3);last_cmd_wall=0.;accepted_cmds=0;rejected_cmds=0;watchdog_stops=0
 command_log=(out/'cmd_vel_events.jsonl').open('w',buffering=1)
 def on_cmd(message):
  global received_cmd,last_cmd_wall,accepted_cmds,rejected_cmds
  value=np.array([message.linear.x,message.linear.y,message.angular.z],dtype=float)
  if not np.isfinite(value).all() or np.max(np.abs(value))>1.:
   rejected_cmds+=1;command_log.write(json.dumps(dict(event='rejected',value=value.tolist()))+'\n');return
  received_cmd=value;last_cmd_wall=time.monotonic();accepted_cmds+=1
  if accepted_cmds==1 or accepted_cmds%20==0:command_log.write(json.dumps(dict(event='accepted',value=value.tolist(),wall_s=last_cmd_wall))+'\n')
 node.create_subscription(Twist,'/cmd_vel',on_cmd,10)
 imu_pub=node.create_publisher(Imu,'/imu/data',qos)
 clock_pub=node.create_publisher(Clock,'/clock',qos)
 lidar=GeometricLidar(stage,node,lidar_cfg,profile,common,out)
 imu_times=[];imu_force=[];imu_gyro=[];truth_pos=[];truth_quat=[];imu_log=(out/'imu_samples.jsonl').open('w',buffering=1)
 previous_sensor_velocity=None;sensor_started=False
 def record_imu(time_s,position,quaternion,velocity,angular,acceleration):
  rotation=Rotation.from_quat(np.r_[quaternion[1:],quaternion[0]])
  sensor_rotation=rotation*r_bi
  force=sensor_rotation.inv().apply(acceleration-np.array([0.,0.,-9.80665]))
  gyro=sensor_rotation.inv().apply(angular)
  sec,nsec=divmod(round(time_s*1e9),1_000_000_000)
  clock=Clock();clock.clock.sec=sec;clock.clock.nanosec=nsec;clock_pub.publish(clock)
  msg=Imu();msg.header.stamp.sec=sec;msg.header.stamp.nanosec=nsec;msg.header.frame_id='lidar_imu_link'
  msg.orientation.w=1.;msg.orientation_covariance[0]=-1.
  msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=map(float,force)
  msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=map(float,gyro)
  imu_pub.publish(msg)
  imu_times.append(time_s);imu_force.append(force);imu_gyro.append(gyro)
  truth_pos.append(np.asarray(position).copy());truth_quat.append(np.asarray(quaternion).copy())
  imu_log.write(json.dumps(dict(time_s=time_s,specific_force_mps2=force.tolist(),angular_velocity_radps=gyro.tolist(),base_position_world_m=np.asarray(position).tolist()))+'\n')
 # Run until the requested simulation duration; all motion comes from /cmd_vel.
 duration=a.max_seconds
 failure=None
 last_command=np.zeros(3)
 policy_activated=False
 policy_activation_t=None
 for step in range(int(duration/dt)):
  if stopping:failure='user_exit';break
  rclpy.spin_once(node,timeout_sec=0.)
  if a.teleop and not app.is_running():failure='window_closed';break
  t=step*dt;q=np.asarray(robot.get_joint_positions())[mapping];dq=np.asarray(robot.get_joint_velocities())[mapping];pos,quat=robot.get_world_pose();R=Rotation.from_quat(np.r_[quat[1:],quat[0]]);rpy=R.as_euler('xyz');angular=R.inv().apply(robot.get_angular_velocity());linear=R.inv().apply(robot.get_linear_velocity())
  if a.teleop:
   pressed=read_keys() if step%dec==0 else pressed
   if 'Escape' in pressed:failure='user_exit';break
   cmd=np.array([float('w' in pressed)-float('s' in pressed),float('a' in pressed)-float('d' in pressed),float('q' in pressed)-float('e' in pressed)])
   cmd[:2]*=a.linear_speed;cmd[2]*=a.turn_speed
   if 'space' in pressed or t<2.:cmd[:]=0.
   if step%dec==0 and not np.array_equal(cmd,last_command):
    print('TELEOP_COMMAND',json.dumps(dict(t=round(t,3),vx=float(cmd[0]),vy=float(cmd[1]),wz=float(cmd[2]))),flush=True);last_command=cmd.copy()
  else:
   fresh=last_cmd_wall>0 and time.monotonic()-last_cmd_wall<=.5
   cmd=received_cmd.copy() if fresh and t>=2. else np.zeros(3)
   if not fresh and last_cmd_wall>0 and np.any(received_cmd):
    received_cmd[:]=0.;watchdog_stops+=1
    command_log.write(json.dumps(dict(event='watchdog_stop',simulation_t=t))+'\n')
  if t>=2. and np.any(cmd) and not policy_activated:
   policy_activated=True;policy_activation_t=t
   command_log.write(json.dumps(dict(event='policy_activated',simulation_t=t))+'\n')
  use_pd=not policy_activated or a.pd_only
  phase='pd_stand' if use_pd else ('walk' if np.any(cmd) else 'policy_stand')
  if not use_pd and step%dec==0:
   obs=np.r_[angular*.25,R.inv().apply([0,0,-1]),cmd,(q-nominal),dq*.05,previous]
   with torch.inference_mode():act=model(torch.tensor(np.clip(obs,-100,100),dtype=torch.float32).reshape(1,45)).numpy().reshape(-1)
   if not np.isfinite(act).all():raise ValueError('nonfinite policy output')
   previous=np.clip(act,-3,3);target=nominal+scale*previous
  torque=np.clip((80. if use_pd else kp)*(target-q)-(3. if use_pd else kd)*dq,-limit,limit)
  efforts=np.zeros(12);efforts[mapping]=torque;robot.set_joint_efforts(efforts)
  if step>=400:
   before_pose=np.eye(4);before_pose[:3,:3]=R.as_matrix();before_pose[:3,3]=pos
   if not sensor_started:
    previous_sensor_velocity=robot.get_linear_velocity()+np.cross(robot.get_angular_velocity(),R.apply(lever))
    record_imu(0.,pos,quat,robot.get_linear_velocity(),robot.get_angular_velocity(),np.zeros(3))
    sensor_started=True
  world.step(render=False)
  if sensor_started:
   after_pos,after_quat=robot.get_world_pose()
   after_rotation=Rotation.from_quat(np.r_[after_quat[1:],after_quat[0]])
   after_pose=np.eye(4);after_pose[:3,:3]=after_rotation.as_matrix();after_pose[:3,3]=after_pos
   after_velocity=robot.get_linear_velocity();after_angular=robot.get_angular_velocity()
   if debug_first_scan and step==400:
    probe=lidar.query.raycast_closest(Gf.Vec3f(0.,0.,1.),Gf.Vec3f(0.,0.,-1.),10.)
    print('SCENE_QUERY_PROBE',repr(probe),'SCENE_QUERY_SUPPORT',world.get_physics_context().get_enable_scene_query_support(),flush=True)
   sensor_velocity=after_velocity+np.cross(after_angular,after_rotation.apply(lever))
   sensor_time=(step-400+1)*dt
   record_imu(sensor_time,after_pos,after_quat,after_velocity,after_angular,(sensor_velocity-previous_sensor_velocity)/dt)
   previous_sensor_velocity=sensor_velocity
   lidar.sample(sensor_time-dt,sensor_time,before_pose,after_pose)
  if step%10==0:
   if a.teleop:follow_base(pos)
   world.render()
  force=np.asarray(feet.get_net_contact_forces(dt=dt))
  if not a.teleop or step%10==0:rows.append(dict(t=t,phase=phase,pos=np.asarray(pos).tolist(),quat=np.asarray(quat).tolist(),rpy=rpy.tolist(),body_velocity=linear.tolist(),angular_velocity=angular.tolist(),q=q.tolist(),dq=dq.tolist(),target=target.tolist(),torque=torque.tolist(),foot_forces=force.tolist(),command=cmd.tolist()))
  if pos[2]<.15 or max(abs(rpy[:2]))>.8:failure='fallen';break
  if step%200==0:print(json.dumps(dict(t=t,phase=phase,pos=np.asarray(pos).tolist(),rpy=rpy.tolist(),contacts=np.linalg.norm(force,axis=1).tolist())),flush=True)
  if debug_first_scan and sensor_started and lidar.scan_number>=1:break
 if not a.teleop:
  image=np.asarray(rgb.get_data())
  if image.ndim==3 and image.shape[2]>=3 and image[:,:,:3].std()>2:Image.fromarray(image[:,:,:3].astype(np.uint8)).save(out/'simulation.png')
 (out/'states.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in rows))
 lidar.close();imu_log.close();command_log.close()
 np.savez_compressed(out/'imu_200hz.npz',time_s=np.asarray(imu_times),specific_force_mps2=np.asarray(imu_force),
  angular_velocity_radps=np.asarray(imu_gyro),base_position_world_m=np.asarray(truth_pos),base_quat_wxyz=np.asarray(truth_quat))
 generation=dict(result='GENERATED' if failure is None else 'PARTIAL' if failure=='user_exit' else 'FAIL',gait_run=str(out),
  source='live_isaac_physx_gait_and_raycast',imu_frame='lidar_imu_link',lidar_frame='lidar_link',
  imu_hz=200,scan_hz=lidar.hz,rays_per_scan=lidar.size,scans=lidar.scan_number,
  walking_window_s=[6.,14.],imu_samples=len(imu_times),
  base_to_imu_translation_m=lever.tolist(),base_to_imu_quat_xyzw=r_bi.as_quat().tolist(),
  imu_mount_profile_max_error=mount_error,manufacturer_scan_pattern=False,sensor_noise=False,
  ros_published=True,hardware=False)
 (out/'generation.json').write_text(json.dumps(generation,indent=2)+'\n')
 summary=dict(result='USER_STOPPED' if failure=='user_exit' else 'DYNAMICS_PROBE_COMPLETE' if failure is None else 'FAIL',failure=failure,model=a.model,walk=a.walk,teleop=a.teleop,steps=step+1,recorded_rows=len(rows),wall_s=time.monotonic()-runtime,physics_dt_s=dt,policy_hz=50.,base_teleported_after_initialization=False,hardware=False,final=rows[-1] if rows else None)
 summary.update(cmd_vel_accepted=accepted_cmds,cmd_vel_rejected=rejected_cmds,watchdog_stops=watchdog_stops,control_source='ros_cmd_vel',policy_activation_t=policy_activation_t)
 (out/'verification.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps({k:v for k,v in summary.items() if k!='final'}),flush=True);code=0 if failure in (None,'user_exit') else 1
except Exception:
 (out/'verification.json').write_text(json.dumps(dict(result='FAIL',failure='runtime_exception')))
 (out/'error.txt').write_text(traceback.format_exc());traceback.print_exc()
finally:
 if a.teleop and 'display' in locals() and display:
  try:xlib.XCloseDisplay(display)
  except Exception:pass
 if robot:
  try:robot.set_joint_efforts(np.zeros(12))
  except Exception:pass
 if world:
  try:world.stop()
  except Exception:pass
 if 'node' in locals() and node:
  try:node.destroy_node();rclpy.shutdown()
  except Exception:pass
 app.app.post_quit(code)
 app.close()
raise SystemExit(code)
