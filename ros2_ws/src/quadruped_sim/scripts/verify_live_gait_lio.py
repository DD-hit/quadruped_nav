#!/usr/bin/env python3
"""Run Isaac D1 gait and FAST-LIO concurrently on one ROS simulation clock."""
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
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


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
    args = parser.parse_args()
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
    lio = gait = None
    estimates = []
    counts = {'clock': 0, 'imu': 0, 'cloud': 0}
    cloud_wall = {}
    delays = []
    frame_names = {'imu': set(), 'cloud': set()}
    status = {}
    adapter = {}
    failure = None
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('d1_live_gait_lio_verifier')
    sensor_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)

    def on_clock(msg):
        counts['clock'] += 1

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

    node.create_subscription(Clock, '/clock', on_clock, sensor_qos)
    node.create_subscription(Imu, '/imu/data', on_imu, sensor_qos)
    node.create_subscription(PointCloud2, '/lidar/points', on_cloud, sensor_qos)
    node.create_subscription(Odometry, '/lio/odometry', on_odom, 200)
    node.create_subscription(String, '/lio/runtime_status', on_status, 20)
    node.create_subscription(String, '/lio/input_status', on_adapter, 20)
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
        gait = subprocess.Popen([str(isaac_root / 'python.sh'),
            str(args.project_root / 'ros2_ws/src/quadruped_sim/scripts/gait_runtime_sensors.py'),
            '--project-root', str(args.project_root), '--output-dir', str(gait_dir),
            '--model', 'loaded', '--walk', '--policy', '--speed', '0.5', '--gpu-index', '1'],
            cwd=args.project_root, env=isaac_env, stdout=gait_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        deadline = time.monotonic() + 240
        while gait.poll() is None and time.monotonic() < deadline:
            if lio.poll() is not None: raise RuntimeError('FAST-LIO exited during Isaac gait')
            rclpy.spin_once(node, timeout_sec=.05)
        if gait.poll() is None: raise RuntimeError('Isaac gait timed out')
        if gait.returncode != 0: raise RuntimeError(f'Isaac gait exit code {gait.returncode}')
        for _ in range(50): rclpy.spin_once(node, timeout_sec=.1)
    except Exception as exc:
        failure = repr(exc)
    finally:
        stop(gait)
        stop(lio)
        lio_log.close()
        gait_log.close()
        node.destroy_node()
        rclpy.shutdown()

    for estimate in estimates:
        scan_start = round(np.floor(estimate['t'] * 10) / 10, 1)
        if scan_start in cloud_wall:
            delays.append(estimate['wall_recv_s'] - cloud_wall[scan_start])
    result = dict(result='FAIL', failure=failure, live_ros_counts=counts,
                  live_ros_frames={key: sorted(value) for key, value in frame_names.items()},
                  odometry_messages=len(estimates), adapter=adapter, runtime=status,
                  isaac_exit_code=gait.returncode if gait else None,
                  lio_exit_code=lio.returncode if lio else None,
                  odometry_after_cloud_delay_p95_s=float(np.percentile(delays, 95)) if delays else None,
                  odometry_after_cloud_delay_median_s=float(np.median(delays)) if delays else None,
                  odometry_latency_samples=len(delays), hardware=False, navigation_connected=False)
    (args.output_dir / 'odometry.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in estimates))
    if failure is None:
        try:
            generation = json.loads((gait_dir / 'generation.json').read_text())
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
            walking = (est_t >= 6.) & (est_t < 14.)
            position_error = np.linalg.norm(align_r.apply(est_p[walking]) + align_p - true_p[match[walking]], axis=1)
            orientation_error = (true_r[match[walking]].inv() * align_r * est_r[walking]).magnitude()
            result.update(evaluated_odometry=int(walking.sum()),
                          walking_truth_displacement_m=float(np.linalg.norm(true_base[(true_t >= 6.) & (true_t < 14.)][-1] - true_base[(true_t >= 6.) & (true_t < 14.)][0])),
                          position_rmse_m=float(np.sqrt(np.mean(position_error ** 2))),
                          position_p95_m=float(np.percentile(position_error, 95)),
                          orientation_p95_deg=float(np.degrees(np.percentile(orientation_error, 95))))
            if (counts['cloud'] >= 170 and counts['imu'] >= 3500 and len(estimates) >= 160 and
                walking.sum() >= 70 and adapter.get('clouds', 0) >= 170 and
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
