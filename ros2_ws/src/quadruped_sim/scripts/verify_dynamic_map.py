#!/usr/bin/env python3
"""Build a localization map from the current live D1 Isaac gait scene."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist


def stop(process):
    if process and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try: process.wait(timeout=50)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=10)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--project-root',type=Path,required=True)
    parser.add_argument('--map-id',required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    a=parser.parse_args()
    a.output_dir.mkdir(parents=True,exist_ok=False)
    map_dir=Path('/data0/rl/quadruped_nav/maps')/a.map_id
    if map_dir.exists():raise ValueError('Map id already exists')
    os.environ['ROS_DOMAIN_ID']='42';os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
    rclpy.init()
    node=Node('d1_dynamic_map_verifier')
    clock_s=None;sent=0;save_started=False;save_result=None
    def on_clock(msg):
        nonlocal clock_s
        clock_s=msg.clock.sec+msg.clock.nanosec/1e9
    def send_command():
        nonlocal sent
        if clock_s is not None and 6.<=clock_s<14.:
            m=Twist();m.linear.x=.5;pub.publish(m);sent+=1
    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Clock,'/clock',on_clock,qos)
    pub=node.create_publisher(Twist,'/cmd_vel',10)
    node.create_timer(.05,send_command)
    mapping_log=(a.output_dir/'mapping.log').open('w')
    gait_log=(a.output_dir/'isaac.log').open('w')
    mapping=gait=None;error=None
    try:
        mapping=subprocess.Popen(['/usr/bin/python3',str(a.project_root/'scripts/run_mapping.py'),
            '--map-id',a.map_id,'--output-dir',str(a.output_dir/'mapping'),
            '--base-config-file',str(a.project_root/'ros2_ws/src/quadruped_localization/config/base_odometry_dynamic_gait.yaml')],cwd=a.project_root,
            stdout=mapping_log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+40
        while not (a.output_dir/'mapping/lio/status.json').is_file() and time.monotonic()<deadline:
            if mapping.poll() is not None:raise RuntimeError('Mapping manager exited before LIO ready')
            rclpy.spin_once(node,timeout_sec=.1)
        if not (a.output_dir/'mapping/lio/status.json').is_file():raise TimeoutError('LIO startup timeout')
        isaac=Path('/home/rl/isaacsim')
        env=dict(os.environ);env.pop('PYTHONPATH',None)
        env['ROS_DISTRO']='humble'
        env['LD_LIBRARY_PATH']=str(isaac/'exts/isaacsim.ros2.bridge/humble/lib')+os.pathsep+env.get('LD_LIBRARY_PATH','')
        for _ in range(20):rclpy.spin_once(node,timeout_sec=.1)
        gait=subprocess.Popen([str(isaac/'python.sh'),
            str(a.project_root/'ros2_ws/src/quadruped_sim/scripts/gait_ros_cmd_ready.py'),
            '--project-root',str(a.project_root),'--output-dir',str(a.output_dir/'isaac_gait'),
            '--model','loaded','--policy','--gpu-index','1','--max-seconds','20'],
            cwd=a.project_root,env=env,stdout=gait_log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+240
        while gait.poll() is None and time.monotonic()<deadline:
            if mapping.poll() is not None:raise RuntimeError('Mapping manager exited during Isaac')
            rclpy.spin_once(node,timeout_sec=.05)
            if not save_started and clock_s is not None and clock_s>=16.:
                save_started=True
                save_result=subprocess.run(['/usr/bin/python3',str(a.project_root/'scripts/save_map.py')],
                    cwd=a.project_root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=70)
                (a.output_dir/'save_map.log').write_text(save_result.stdout)
        if gait.poll() is None:raise TimeoutError('Isaac simulation timeout')
        if gait.returncode!=0:raise RuntimeError('Isaac gait failed')
        if not save_started or not save_result or save_result.returncode!=0:raise RuntimeError('Map save failed')
    except Exception as exc:
        error=repr(exc)
    finally:
        stop(gait);stop(mapping)
        mapping_log.close();gait_log.close();node.destroy_node();rclpy.shutdown()
    status=json.loads((map_dir/'recording_status.json').read_text()) if (map_dir/'recording_status.json').exists() else {}
    result=dict(result='PASS' if not error and (map_dir/'manifest.yaml').exists() and status.get('phase')=='saved' else 'FAIL',
                failure=error,map_id=a.map_id,map_directory=str(map_dir),status=status,
                cmd_vel_sent=sent,isaac_exit_code=gait.returncode if gait else None,
                mapping_exit_code=mapping.returncode if mapping else None,hardware=False)
    (a.output_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    if result['result']!='PASS':raise SystemExit(1)


if __name__=='__main__':main()
