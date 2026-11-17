#!/usr/bin/env python3
"""Verify public base odometry and TF during live ROS-controlled D1 gait."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation

os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=35)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--yaw-rate', type=float, default=0.)
    args = parser.parse_args()
    if args.yaw_rate not in (-.3, 0., .3):
        parser.error('--yaw-rate must be -0.3, 0, or 0.3')
    command_end = 10. if args.yaw_rate else 14.
    args.output_dir.mkdir(parents=True, exist_ok=False)
    gait_dir = args.output_dir / 'isaac_gait'
    lio_dir = args.output_dir / 'lio_runtime'
    isaac_root = Path('/home/rl/isaacsim')
    bridge_libraries = isaac_root / 'exts/isaacsim.ros2.bridge/humble/lib'
    isaac_env = dict(os.environ)
    isaac_env.pop('PYTHONPATH', None)
    isaac_env.update(ROS_DISTRO='humble', ROS_DOMAIN_ID='42', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                     LD_LIBRARY_PATH=str(bridge_libraries) + os.pathsep + isaac_env.get('LD_LIBRARY_PATH', ''))
    lio_log = (args.output_dir / 'lio.log').open('w')
    gait_log = (args.output_dir / 'isaac.log').open('w')
    lio = gait = base_process = None
    estimates = []
    base_estimates = []
    base_status = {}
    tf_children = set()
    counts = {'clock': 0, 'imu': 0, 'cloud': 0}
    clock_s = None
    sent_commands = 0
    cloud_wall = {}
    delays = []
    frame_names = {'imu': set(), 'cloud': set()}
    status = {}
    adapter = {}
    failure = None
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('d1_live_gait_lio_verifier')
    sensor_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def on_clock(msg):
        nonlocal clock_s
        counts['clock'] += 1
        clock_s = msg.clock.sec + msg.clock.nanosec / 1e9

    def send_command():
        nonlocal sent_commands
        if clock_s is None or not 6. <= clock_s < command_end:
            return
        message = Twist()
        message.linear.x = .5
        message.angular.z = args.yaw_rate
        cmd_pub.publish(message)
        sent_commands += 1

    def on_imu(msg):
        counts['imu'] += 1
        frame_names['imu'].add(msg.header.frame_id)

    def on_cloud(msg):
        counts['cloud'] += 1
        frame_names['cloud'].add(msg.header.frame_id)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        cloud_wall[round(t, 1)] = time.monotonic()

    def on_odom(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        estimates.append(dict(t=t, wall_recv_s=time.monotonic(),
                              pos=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                              quat=[msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]))

    def on_status(msg):
        try: status.update(json.loads(msg.data))
        except json.JSONDecodeError: pass

    def on_adapter(msg):
        try: adapter.update(json.loads(msg.data))
        except json.JSONDecodeError: pass

    def on_base(msg):
        base_estimates.append(dict(t=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                                   frame=msg.header.frame_id, child=msg.child_frame_id,
                                   pos=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z],
                                   quat=[msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]))

    def on_base_status(msg):
        try: base_status.update(json.loads(msg.data))
        except json.JSONDecodeError: pass

    def on_tf(msg):
        tf_children.update((x.header.frame_id,x.child_frame_id) for x in msg.transforms)

    node.create_subscription(Clock, '/clock', on_clock, sensor_qos)
    node.create_subscription(Imu, '/imu/data', on_imu, sensor_qos)
    node.create_subscription(PointCloud2, '/lidar/points', on_cloud, sensor_qos)
    node.create_subscription(Odometry, '/lio/odometry', on_odom, 200)
    node.create_subscription(String, '/lio/runtime_status', on_status, 20)
    node.create_subscription(String, '/lio/input_status', on_adapter, 20)
    node.create_subscription(Odometry, '/odom', on_base, 200)
    node.create_subscription(String, '/lio/base_odometry_status', on_base_status, 20)
    node.create_subscription(TFMessage, '/tf', on_tf, 200)
    node.create_subscription(TFMessage, '/tf_static', on_tf, 20)
    node.create_timer(.05, send_command)
    try:
        lio = subprocess.Popen(['/usr/bin/python3', str(args.project_root / 'scripts/run_lio.py'),
                                '--robot', 'd1_edu', '--raw-only', '--output-dir', str(lio_dir)],
                               cwd=args.project_root, stdout=lio_log, stderr=subprocess.STDOUT,
                               start_new_session=True)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and not (lio_dir / 'status.json').is_file():
            if lio.poll() is not None: raise RuntimeError('FAST-LIO exited before Isaac started')
            rclpy.spin_once(node, timeout_sec=.1)
        if not (lio_dir / 'status.json').is_file(): raise RuntimeError('FAST-LIO startup timed out')
        # Let DDS subscriptions settle before the first Isaac sensor sample.
        for _ in range(20): rclpy.spin_once(node, timeout_sec=.1)
        base_log = (args.output_dir / 'base_odometry.log').open('w')
        base_process = subprocess.Popen(['/usr/bin/python3',
            str(args.project_root / 'ros2_ws/src/quadruped_localization/scripts/base_odometry.py'),
            '--project-root',str(args.project_root),'--config-file',
            str(args.project_root / 'ros2_ws/src/quadruped_localization/config/base_odometry_dynamic_gait.yaml'),
            '--status-file',str(args.output_dir / 'base_status.json')],
            cwd=args.project_root,stdout=base_log,stderr=subprocess.STDOUT,start_new_session=True)
        for _ in range(15):
            if base_process.poll() is not None: raise RuntimeError('Base odometry exited during startup')
            rclpy.spin_once(node,timeout_sec=.1)
        gait = subprocess.Popen([str(isaac_root / 'python.sh'),
            str(args.project_root / 'ros2_ws/src/quadruped_sim/scripts/gait_ros_cmd.py'),
            '--project-root', str(args.project_root), '--output-dir', str(gait_dir),
            '--model', 'loaded', '--policy', '--gpu-index', '1', '--max-seconds', '20'],
            cwd=args.project_root, env=isaac_env, stdout=gait_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        deadline = time.monotonic() + 240
        while gait.poll() is None and time.monotonic() < deadline:
            if lio.poll() is not None: raise RuntimeError('FAST-LIO exited during Isaac gait')
            if base_process.poll() is not None: raise RuntimeError('Base odometry exited during Isaac gait')
            rclpy.spin_once(node, timeout_sec=.05)
        if gait.poll() is None: raise RuntimeError('Isaac gait timed out')
        if gait.returncode != 0: raise RuntimeError(f'Isaac gait exit code {gait.returncode}')
        for _ in range(50): rclpy.spin_once(node, timeout_sec=.1)
    except Exception as exc:
        failure = repr(exc)
    finally:
        stop(gait)
        stop(base_process)
        stop(lio)
        lio_log.close()
        gait_log.close()
        if 'base_log' in locals(): base_log.close()
        node.destroy_node()
        rclpy.shutdown()

    for estimate in estimates:
        scan_start = round(np.floor(estimate['t'] * 10) / 10, 1)
        if scan_start in cloud_wall:
            delays.append(estimate['wall_recv_s'] - cloud_wall[scan_start])
    result = dict(result='FAIL', failure=failure, live_ros_counts=counts,cmd_vel_published=sent_commands,
                  base_odometry_messages=len(base_estimates),base_status=base_status,
                  tf_edges=sorted([list(x) for x in tf_children]),
                  base_exit_code=base_process.returncode if base_process else None,
                  command_vx_mps=.5,command_wz_radps=args.yaw_rate,command_sensor_window_s=[6.,command_end],
                  live_ros_frames={key: sorted(value) for key, value in frame_names.items()},
                  odometry_messages=len(estimates), adapter=adapter, runtime=status,
                  isaac_exit_code=gait.returncode if gait else None,
                  lio_exit_code=lio.returncode if lio else None,
                  odometry_after_cloud_delay_p95_s=float(np.percentile(delays, 95)) if delays else None,
                  odometry_after_cloud_delay_median_s=float(np.median(delays)) if delays else None,
                  odometry_latency_samples=len(delays), hardware=False, navigation_connected=False)
    (args.output_dir / 'odometry.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in estimates))
    (args.output_dir / 'base_odometry.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in base_estimates))
    if failure is None:
        try:
            generation = json.loads((gait_dir / 'generation.json').read_text())
            dynamics = json.loads((gait_dir / 'verification.json').read_text())
            states = [json.loads(line) for line in (gait_dir / 'states.jsonl').open()]
            state_t = np.asarray([row['t'] for row in states])
            state_linear = np.asarray([row['body_velocity'] for row in states])
            state_angular = np.asarray([row['angular_velocity'] for row in states])
            motion = (state_t >= 8.) & (state_t < command_end + 2.)
            settled = (state_t >= command_end + 3.) & (state_t < 19.9)
            actual_forward = float(np.mean(state_linear[motion, 0]))
            actual_yaw = float(np.mean(state_angular[motion, 2]))
            stop_yaw = float(np.mean(np.abs(state_angular[settled, 2])))
            result.update(actual_forward_mps=actual_forward,actual_yaw_rate_radps=actual_yaw,
                          settled_abs_yaw_rate_radps=stop_yaw)
            result.update(cmd_vel_accepted=dynamics['cmd_vel_accepted'],cmd_vel_rejected=dynamics['cmd_vel_rejected'],watchdog_stops=dynamics['watchdog_stops'])
            imu = np.load(gait_dir / 'imu_200hz.npz')
            true_t = imu['time_s']
            true_base = imu['base_position_world_m']
            true_r = Rotation.from_quat(imu['base_quat_wxyz'][:, [1, 2, 3, 0]])
            mount_r = Rotation.from_quat(generation['base_to_imu_quat_xyzw'])
            true_p = true_base + true_r.apply(generation['base_to_imu_translation_m'])
            true_r = true_r * mount_r
            est_t = np.asarray([x['t'] for x in estimates])
            est_p = np.asarray([x['pos'] for x in estimates])
            est_r = Rotation.from_quat(np.asarray([x['quat'] for x in estimates]))
            match = np.abs(true_t[:, None] - est_t[None, :]).argmin(axis=0)
            align_index = np.flatnonzero(est_t >= 5.)[0]
            align_r = true_r[match[align_index]] * est_r[align_index].inv()
            align_p = true_p[match[align_index]] - align_r.apply(est_p[align_index])
            walking = (est_t >= 6.) & (est_t < command_end)
            position_error = np.linalg.norm(align_r.apply(est_p[walking]) + align_p - true_p[match[walking]], axis=1)
            orientation_error = (true_r[match[walking]].inv() * align_r * est_r[walking]).magnitude()
            result.update(evaluated_odometry=int(walking.sum()),
                          walking_truth_displacement_m=float(np.linalg.norm(true_base[(true_t >= 6.) & (true_t < command_end)][-1] - true_base[(true_t >= 6.) & (true_t < command_end)][0])),
                          position_rmse_m=float(np.sqrt(np.mean(position_error ** 2))),
                          position_p95_m=float(np.percentile(position_error, 95)),
                          orientation_p95_deg=float(np.degrees(np.percentile(orientation_error, 95))))
            if (sent_commands >= 50 and dynamics['cmd_vel_accepted'] >= 50 and dynamics['watchdog_stops'] >= 1 and
                len(base_estimates)>=140 and base_status.get('aligned') and
                ('odom','base_link') in tf_children and ('odom','camera_init') in tf_children and
                all(x['frame']=='odom' and x['child']=='base_link' for x in base_estimates) and
                abs(actual_forward-.5)<.08 and abs(actual_yaw-args.yaw_rate)<.07 and stop_yaw<.04 and
                counts['cloud'] >= 170 and counts['imu'] >= 3500 and len(estimates) >= 160 and
                walking.sum() >= (35 if args.yaw_rate else 70) and adapter.get('clouds', 0) >= 170 and
                adapter.get('rejected_clouds', 1) == 0 and adapter.get('rejected_imus', 1) == 0 and
                status.get('initialized') and result['position_rmse_m'] < .1 and
                result['orientation_p95_deg'] < 5 and len(delays) >= 100 and
                result['odometry_after_cloud_delay_p95_s'] < 1.):
                result['result'] = 'PASS'
            else:
                result['failure'] = 'online delivery, localization, or latency acceptance failed'
        except Exception as exc:
            result['failure'] = repr(exc)
    (args.output_dir / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)
    if result['result'] != 'PASS': raise SystemExit(1)


if __name__ == '__main__':
    main()
