#!/usr/bin/env python3
"""Replay dynamic Isaac gait sensors into the owned FAST-LIO runtime."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import String


POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                        ('intensity', '<f4'), ('time', '<f4'),
                        ('offset_time', '<u4'), ('ray_id', '<u4')])


def stamp(seconds):
    return divmod(round(float(seconds) * 1e9), 1_000_000_000)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--sensor-run', type=Path, required=True)
    parser.add_argument('--gait-run', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--playback-rate', type=float, default=.5)
    args = parser.parse_args()
    if not 0 < args.playback_rate <= 1:
        parser.error('playback-rate must be in (0, 1]')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    generation = json.loads((args.sensor_run / 'generation.json').read_text())
    if generation['gait_run'] != str(args.gait_run):
        raise ValueError('Sensor input does not match gait source run')
    imu = np.load(args.sensor_run / 'imu_200hz.npz')
    lidar = np.load(args.sensor_run / 'lidar_10hz.npz')
    if len(imu['base_position_world_m']) != len(imu['time_s']):
        raise ValueError('Missing generated ground-truth sample')
    scan_ids = lidar['scan']
    scan_starts = np.searchsorted(scan_ids, np.arange(len(lidar['scan_stamp_s'])), side='left')
    scan_ends = np.searchsorted(scan_ids, np.arange(len(lidar['scan_stamp_s'])), side='right')
    if np.min(scan_ends - scan_starts) < 2:
        raise ValueError('Empty dynamic scan')

    os.environ['ROS_DOMAIN_ID'] = '42'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    env = dict(os.environ)
    lio_dir = args.output_dir / 'lio_runtime'
    log_handle = (args.output_dir / 'lio_runtime.log').open('w')
    lio = subprocess.Popen(['/usr/bin/python3', str(args.project_root / 'scripts/run_lio.py'),
                            '--robot', 'd1_edu', '--raw-only', '--output-dir', str(lio_dir)],
                           cwd=str(args.project_root), env=env, stdout=log_handle,
                           stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('dynamic_gait_sensor_replay')
    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
    clock_pub = node.create_publisher(Clock, '/clock', qos)
    imu_pub = node.create_publisher(Imu, '/imu/data', qos)
    cloud_pub = node.create_publisher(PointCloud2, '/lidar/points', qos)
    estimates = []
    status = {}
    adapter_status = {}

    def on_odom(message):
        estimates.append(dict(t=message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                              pos=[message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z],
                              quat=[message.pose.pose.orientation.x, message.pose.pose.orientation.y,
                                    message.pose.pose.orientation.z, message.pose.pose.orientation.w]))

    def on_status(message):
        try:status.update(json.loads(message.data))
        except json.JSONDecodeError:pass

    def on_adapter(message):
        try:adapter_status.update(json.loads(message.data))
        except json.JSONDecodeError:pass

    node.create_subscription(Odometry, '/lio/odometry', on_odom, 100)
    node.create_subscription(String, '/lio/runtime_status', on_status, 20)
    node.create_subscription(String, '/lio/input_status', on_adapter, 20)
    cloud_fields = [PointField(name=name, offset=i * 4,
                   datatype=PointField.UINT32 if name in ('offset_time', 'ray_id') else PointField.FLOAT32,
                   count=1) for i, name in enumerate(POINT_DTYPE.names)]
    sent_scans = 0
    failure = None
    try:
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and not lio_dir.joinpath('status.json').is_file():
            if lio.poll() is not None:raise RuntimeError('LIO launcher exited before replay')
            rclpy.spin_once(node, timeout_sec=.1)
        if not lio_dir.joinpath('status.json').is_file():
            raise RuntimeError('LIO launcher did not become ready')
        time.sleep(2)
        start = time.monotonic()
        for i, value in enumerate(imu['time_s']):
            if lio.poll() is not None:raise RuntimeError('LIO launcher exited during replay')
            sec, nsec = stamp(value)
            clock = Clock();clock.clock.sec = sec;clock.clock.nanosec = nsec
            clock_pub.publish(clock)
            message = Imu();message.header.stamp.sec = sec;message.header.stamp.nanosec = nsec
            message.header.frame_id = 'lidar_imu_link'
            message.orientation.w = 1.;message.orientation_covariance[0] = -1.
            a = imu['specific_force_mps2'][i];w = imu['angular_velocity_radps'][i]
            message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = map(float, a)
            message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = map(float, w)
            imu_pub.publish(message)
            if i % 20 == 0 and i // 20 < len(scan_starts):
                scan = i // 20;begin,end = int(scan_starts[scan]), int(scan_ends[scan])
                points = np.empty(end - begin, dtype=POINT_DTYPE)
                points['x'], points['y'], points['z'] = lidar['xyz_lidar_m'][begin:end].T
                points['intensity'] = 1.
                points['offset_time'] = lidar['offset_time_ns'][begin:end]
                points['time'] = points['offset_time'].astype(np.float32) / 1e9
                points['ray_id'] = np.arange(begin, end, dtype=np.uint32)
                cloud = PointCloud2();cloud.header.stamp.sec = sec;cloud.header.stamp.nanosec = nsec
                cloud.header.frame_id = 'lidar_link';cloud.height = 1;cloud.width = len(points)
                cloud.fields = cloud_fields;cloud.is_bigendian = False;cloud.point_step = POINT_DTYPE.itemsize
                cloud.row_step = cloud.width * cloud.point_step;cloud.data = points.tobytes();cloud.is_dense = True
                cloud_pub.publish(cloud);sent_scans += 1
            rclpy.spin_once(node, timeout_sec=0.)
            target = start + float(value) / args.playback_rate
            if target > time.monotonic():time.sleep(target - time.monotonic())
        for _ in range(50):rclpy.spin_once(node, timeout_sec=.1)
    except Exception as error:
        failure = repr(error)
    finally:
        if lio.poll() is None:
            os.killpg(lio.pid, signal.SIGINT)
            try:lio.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(lio.pid, signal.SIGKILL);lio.wait(timeout=10)
        log_handle.close()
        node.destroy_node();rclpy.shutdown()

    (args.output_dir / 'odometry.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in estimates))
    if failure is None and (len(estimates) < 10 or sent_scans < 170 or adapter_status.get('clouds', 0) < 170 or adapter_status.get('imus', 0) < 3500):
        failure = 'insufficient connected LIO/adapter data'
    metrics = dict(result='PROBE_COMPLETE' if failure is None else 'FAIL', failure=failure,
                   sent_imu=len(imu['time_s']) if failure is None else None, sent_scans=sent_scans,
                   odometry_messages=len(estimates), adapter=adapter_status, runtime=status,
                   lio_exit_code=lio.returncode, hardware=False, navigation_connected=False)
    if estimates:
        imu_mount = Rotation.from_quat(generation['base_to_imu_quat_xyzw'])
        lever = np.asarray(generation['base_to_imu_translation_m'])
        true_t = imu['time_s']
        true_position = imu['base_position_world_m']
        true_rotation = Rotation.from_quat(imu['base_quat_wxyz'][:, [1,2,3,0]])
        true_imu_position = true_position + true_rotation.apply(lever)
        true_imu_rotation = true_rotation * imu_mount
        est_t = np.asarray([item['t'] for item in estimates])
        est_position = np.asarray([item['pos'] for item in estimates])
        est_rotation = Rotation.from_quat(np.asarray([item['quat'] for item in estimates]))
        match = np.abs(true_t[:, None] - est_t[None, :]).argmin(axis=0)
        walk_start, walk_end = generation['walking_window_s']
        first = np.flatnonzero(est_t >= max(1., walk_start - 1.))
        if len(first):
            i = int(first[0]);align = true_imu_rotation[match[i]] * est_rotation[i].inv()
            offset = true_imu_position[match[i]] - align.apply(est_position[i])
            aligned = align.apply(est_position) + offset
            evaluate = (est_t >= walk_start) & (est_t < walk_end)
            error = np.linalg.norm(aligned[evaluate] - true_imu_position[match[evaluate]], axis=1)
            orientation = (true_imu_rotation[match[evaluate]].inv() * align * est_rotation[evaluate]).magnitude()
            metrics.update(evaluated_odometry=int(np.count_nonzero(evaluate)),
                           position_rmse_m=float(np.sqrt(np.mean(error ** 2))) if len(error) else None,
                           position_p95_m=float(np.percentile(error, 95)) if len(error) else None,
                           orientation_p95_deg=float(np.degrees(np.percentile(orientation, 95))) if len(orientation) else None)
            if len(error) < 20 or metrics['position_rmse_m'] > 1.0 or metrics['orientation_p95_deg'] > 30.:
                metrics['result'] = 'FAIL'
                metrics['failure'] = 'FAST-LIO trajectory exceeds position/orientation acceptance limits'
            else:
                metrics['result'] = 'PASS'
    (args.output_dir / 'result.json').write_text(json.dumps(metrics, indent=2) + '\n')
    print(json.dumps(metrics, indent=2))
    if failure is not None:raise RuntimeError(failure)


if __name__ == '__main__':
    main()
