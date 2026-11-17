#!/usr/bin/env python3
"""Measure low average speed using only the existing D1 /cmd_vel policy."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

def main():
 p=argparse.ArgumentParser();p.add_argument('--project-root',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--period',type=float,default=3.);p.add_argument('--duty',type=float,default=1/3);a=p.parse_args()
 a.output_dir.mkdir(parents=True,exist_ok=False)
 os.environ['ROS_DOMAIN_ID']='42';os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
 rclpy.init();node=Node('d1_pulse_speed_verifier');clock=None;sent=0
 def on_clock(m):
  nonlocal clock
  clock=m.clock.sec+m.clock.nanosec/1e9
 def publish():
  nonlocal sent
  if clock is None:return
  m=Twist()
  if 6<=clock<16 and (clock-6)%a.period<a.period*a.duty:m.linear.x=.5
  pub.publish(m);sent+=1
 node.create_subscription(Clock,'/clock',on_clock,QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
 pub=node.create_publisher(Twist,'/cmd_vel',10);node.create_timer(.05,publish)
 isaac=Path('/home/rl/isaacsim');env=dict(os.environ);env.pop('PYTHONPATH',None);env['ROS_DISTRO']='humble';env['LD_LIBRARY_PATH']=str(isaac/'exts/isaacsim.ros2.bridge/humble/lib')+os.pathsep+env.get('LD_LIBRARY_PATH','')
 log=(a.output_dir/'isaac.log').open('w');gait=None;failure=None
 try:
  gait=subprocess.Popen([str(isaac/'python.sh'),str(a.project_root/'ros2_ws/src/quadruped_sim/scripts/gait_ros_cmd_ready.py'),
   '--project-root',str(a.project_root),'--output-dir',str(a.output_dir/'isaac_gait'),'--model','loaded','--policy','--gpu-index','1','--max-seconds','20'],
   cwd=a.project_root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  deadline=time.monotonic()+240
  while gait.poll() is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
  if gait.poll() is None:raise TimeoutError('Isaac timed out')
  if gait.returncode:raise RuntimeError('Isaac failed')
 except Exception as e:failure=repr(e)
 finally:
  if gait and gait.poll() is None:
   os.killpg(gait.pid,signal.SIGINT)
   try:gait.wait(timeout=30)
   except subprocess.TimeoutExpired:os.killpg(gait.pid,signal.SIGKILL);gait.wait()
  log.close();node.destroy_node();rclpy.shutdown()
 rows=[json.loads(s) for s in (a.output_dir/'isaac_gait/states.jsonl').read_text().splitlines()] if (a.output_dir/'isaac_gait/states.jsonl').exists() else []
 t=np.array([r['t'] for r in rows]);vx=np.array([r['body_velocity'][0] for r in rows]);x=np.array([r['pos'][0] for r in rows])
 select=(t>=6)&(t<16)
 fit=np.polyfit(t[select],x[select],1) if select.sum()>2 else [0,0]
 deviation=float(np.max(np.abs(x[select]-np.polyval(fit,t[select])))) if select.any() else None
 result=dict(result='PASS' if not failure and select.sum()>100 else 'FAIL',failure=failure,
  command_vx_mps=.5,duty_cycle=a.duty,period_s=a.period,mean_vx_mps=float(vx[select].mean()) if select.any() else None,
  peak_abs_vx_mps=float(np.abs(vx[select]).max()) if select.any() else None,
  peak_abs_tracking_deviation_m=deviation,
  displacement_m=float(np.interp(16,t,x)-np.interp(6,t,x)) if len(t) else None,
  sent=sent,isaac_exit_code=gait.returncode if gait else None,hardware=False)
 (a.output_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
 if result['result']!='PASS':raise SystemExit(1)
if __name__=='__main__':main()
