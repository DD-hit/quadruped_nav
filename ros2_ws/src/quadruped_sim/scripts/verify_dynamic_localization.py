#!/usr/bin/env python3
"""Exercise saved-map localization against live D1 gait and sensor streams."""
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
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String


def stop(process):
    if process and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try: process.wait(timeout=50)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--spawn-offset-x', type=float, default=0.)
    a = parser.parse_args()
    a.output_dir.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = '42'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    rclpy.init()
    node = Node('d1_dynamic_localization_verifier')
    clock_s = None
    commands = 0
    initial_pose_sent = False
    tf_samples = []
    status_events=[]
    def on_status(msg):
        try: s=json.loads(msg.data)
        except json.JSONDecodeError:return
        e=dict(t=clock_s,phase=s.get('phase'),reason=s.get('reason'),accepted=s.get('accepted_matches'),failed=s.get('failed_matches'))
        if not status_events or (e['phase'],e['reason'],e['accepted'],e['failed']) != tuple(status_events[-1].get(k) for k in ('phase','reason','accepted','failed')):
            status_events.append(e)
    def on_clock(msg):
        nonlocal clock_s
        clock_s = msg.clock.sec + msg.clock.nanosec / 1e9
    def on_tf(msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                tf_samples.append(dict(t=t.header.stamp.sec+t.header.stamp.nanosec/1e9,
                    xyz=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]))
    def command():
        nonlocal commands, initial_pose_sent
        if not initial_pose_sent and clock_s is not None and clock_s >= 4.0:
            pose=PoseWithCovarianceStamped()
            pose.header.frame_id='map'
            pose.header.stamp.sec=int(clock_s)
            pose.header.stamp.nanosec=int((clock_s-int(clock_s))*1e9)
            pose.pose.pose.position.x=a.spawn_offset_x
            pose.pose.pose.orientation.w=1.0
            for i in range(6):pose.pose.covariance[6*i+i]=0.01
            pose_pub.publish(pose)
            initial_pose_sent=True
        if clock_s is not None and 6 <= clock_s < 14:
            msg = Twist(); msg.linear.x = .5; pub.publish(msg); commands += 1
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Clock, '/clock', on_clock, qos)
    node.create_subscription(TFMessage, '/tf', on_tf, 100)
    node.create_subscription(String, '/localization/status', on_status, 20)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    node.create_timer(.05, command)
    loc_log = (a.output_dir/'localization.log').open('w')
    gait_log = (a.output_dir/'isaac.log').open('w')
    loc = gait = None
    failure = None
    try:
        loc = subprocess.Popen(['/usr/bin/python3', str(a.project_root/'scripts/run_localization.py'),
            '--map-id', a.map_id, '--with-lio', '--base-config-file',
            str(a.project_root/'ros2_ws/src/quadruped_localization/config/base_odometry_dynamic_gait.yaml'),
            '--localization-config-file',
            str(a.project_root/'ros2_ws/src/quadruped_localization/config/localization_dynamic_gait.yaml'),
            '--output-dir', str(a.output_dir/'localization')], cwd=a.project_root,
            stdout=loc_log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+45
        while not (a.output_dir/'localization/lio/status.json').exists() and time.monotonic()<deadline:
            if loc.poll() is not None: raise RuntimeError('Localization exited during startup')
            rclpy.spin_once(node,timeout_sec=.1)
        if not (a.output_dir/'localization/lio/status.json').exists(): raise TimeoutError('LIO startup timeout')
        for _ in range(20):rclpy.spin_once(node,timeout_sec=.1)
        isaac = Path('/home/rl/isaacsim')
        env = dict(os.environ); env.pop('PYTHONPATH',None); env['ROS_DISTRO']='humble'
        env['LD_LIBRARY_PATH']=str(isaac/'exts/isaacsim.ros2.bridge/humble/lib')+os.pathsep+env.get('LD_LIBRARY_PATH','')
        gait = subprocess.Popen([str(isaac/'python.sh'),
            str(a.project_root/'ros2_ws/src/quadruped_sim/scripts/gait_ros_cmd_ready.py'),
            '--project-root',str(a.project_root),'--output-dir',str(a.output_dir/'isaac_gait'),
            '--model','loaded','--policy','--gpu-index','1','--max-seconds','20',
            '--spawn-offset-x',str(a.spawn_offset_x)],
            cwd=a.project_root,env=env,stdout=gait_log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline = time.monotonic()+240
        while gait.poll() is None and time.monotonic()<deadline:
            if loc.poll() is not None:raise RuntimeError('Localization exited during gait')
            rclpy.spin_once(node,timeout_sec=.05)
        if gait.poll() is None:raise TimeoutError('Isaac timeout')
        if gait.returncode:raise RuntimeError('Isaac failed')
        for _ in range(20):rclpy.spin_once(node,timeout_sec=.1)
    except Exception as exc:
        failure=repr(exc)
    finally:
        stop(gait);stop(loc)
        loc_log.close();gait_log.close();node.destroy_node();rclpy.shutdown()
    status_path=a.output_dir/'localization/status.json'
    status=json.loads(status_path.read_text()) if status_path.exists() else {}
    result=dict(result='PASS' if not failure and len(tf_samples)>10 and status.get('accepted_matches',0)>0 else 'FAIL',
        failure=failure,map_id=a.map_id,status=status,map_odom_tf_count=len(tf_samples),
        map_odom_tf_first=tf_samples[0] if tf_samples else None,
        map_odom_tf_last=tf_samples[-1] if tf_samples else None,
        status_events=status_events,
        commands=commands,initial_pose_sent=initial_pose_sent,isaac_exit_code=gait.returncode if gait else None,
        localization_exit_code=loc.returncode if loc else None,hardware=False)
    (a.output_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    if result['result']!='PASS':raise SystemExit(1)


if __name__=='__main__':main()
