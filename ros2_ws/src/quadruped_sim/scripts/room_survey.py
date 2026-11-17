#!/usr/bin/env python3
"""Isaac-only known-room data collection. Control uses LIO odometry; truth only evaluates."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

import numpy as np
import yaml


def clearance_xy(points, room, radius):
    """Signed clearance of a yaw-invariant padded robot circle to room geometry."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    obstacles = [room[name] for name in ('box', 'pillar')]
    xlo = room['west']['center'][0] + room['west']['size'][0] / 2
    xhi = room['east']['center'][0] - room['east']['size'][0] / 2
    ylo = room['south']['center'][1] + room['south']['size'][1] / 2
    yhi = room['north']['center'][1] - room['north']['size'][1] / 2
    values = [points[:, 0] - xlo, xhi - points[:, 0],
              points[:, 1] - ylo, yhi - points[:, 1]]
    for box in obstacles:
        low = np.asarray(box['center'][:2]) - np.asarray(box['size'][:2]) / 2
        high = np.asarray(box['center'][:2]) + np.asarray(box['size'][:2]) / 2
        outside = np.linalg.norm(np.maximum(low - points, 0) + np.maximum(points - high, 0), axis=1)
        inside = np.all((points >= low) & (points <= high), axis=1)
        outside[inside] = -np.min(np.minimum(points[inside] - low, high - points[inside]), axis=1)
        values.append(outside)
    return np.min(values, axis=0) - radius


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--profile', type=Path)
    args = parser.parse_args(argv)
    root = args.project_root.resolve()
    sys.path.insert(0, str(root / 'ros2_ws/src/quadruped_localization/scripts'))
    from map_bundle import identifier, load_bundle, pointcloud_xyzi
    identifier(args.map_id)
    configs = root / 'ros2_ws/src/quadruped_bringup/config'
    host = yaml.safe_load((configs / 'hosts/server.yaml').read_text())
    cfg = yaml.safe_load((args.profile or root / 'ros2_ws/src/quadruped_sim/config/room_survey.yaml').read_text())
    room_config = yaml.safe_load((root / 'ros2_ws/src/quadruped_sim/config/geometric_lidar.yaml').read_text())
    room = room_config['room']
    geometry = yaml.safe_load((root / 'ros2_ws/src/d1_edu_description/config/footprint.yaml').read_text())
    if geometry['supported_motion'] != dict(mode='sim', backend='planar_kinematic', articulated_gait=False, arm_articulation=False):
        raise ValueError('Survey requires the verified fixed-pose planar robot profile')
    radius = float(geometry['yaw_invariant_cylinder']['radius_m'])
    waypoints = np.asarray(cfg['waypoints_xy_m'], dtype=float)
    if waypoints.ndim != 2 or waypoints.shape[1] != 2 or not np.isfinite(waypoints).all():
        raise ValueError('Invalid route')
    route = np.vstack(([0., 0.], waypoints))
    sampled = np.concatenate([np.linspace(a, b, max(2, math.ceil(np.linalg.norm(b-a) / .01)))
                              for a, b in zip(route, route[1:])])
    planned_clearance = float(clearance_xy(sampled, room, radius).min())
    if planned_clearance < cfg['additional_clearance_m']:
        raise ValueError(f'Route leaves only {planned_clearance:.3f} m beyond padded robot radius')
    if not (0 < cfg['speed_mps'] <= .2 and 0 < cfg['yaw_rate_radps'] <= .4):
        raise ValueError('Survey exceeds the verified motion profile')
    if (Path(host['data_root']) / 'maps' / args.map_id).exists():
        raise FileExistsError('Map id already exists')
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / 'scans').mkdir()
    (out / 'survey_profile.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    contract = dict(scope=cfg['scope'], map_id=args.map_id, control_pose='/odom',
                    truth_usage='offline_evaluation_only', initial_base_xy=[0., 0.],
                    padded_robot_radius_m=radius, planned_min_clearance_m=planned_clearance,
                    route_length_m=float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum()),
                    lidar_samples_per_scan=6000, scan_hz=room_config['scan_hz'],
                    near_range_m=room_config['near_range_m'], far_range_m=room_config['far_range_m'],
                    manufacturer_lidar_pattern=False, hardware_started=False)
    (out / 'contract.json').write_text(json.dumps(contract, indent=2) + '\n')
    os.environ['ROS_DOMAIN_ID'] = str(host['ros_domain_id'])
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ['OMP_NUM_THREADS'] = '2'
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    children, logs, checks = {}, [], []
    interrupted = False
    result = 'FAIL'
    node = None
    start_wall = time.monotonic()

    def on_signal(signum, frame):
        nonlocal interrupted
        interrupted = True

    def launch(name, command):
        handle = (out / (name + '.log')).open('w')
        logs.append(handle)
        p = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        children[name] = p
        return p

    def stop(name):
        p = children[name]
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=45)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                try:
                    p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait(timeout=5)
        return p.returncode

    def record(name, **details):
        row = dict(check=name, result='PASS', **details)
        checks.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)

    def stamp(header):
        return header.stamp.sec + header.stamp.nanosec / 1e9

    def pose(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return [stamp(message.header), p.x, p.y, p.z, yaw]

    class Survey(Node):
        def __init__(self):
            super().__init__('quadruped_room_survey')
            self.mapping = self.base = self.state = self.runtime = None
            self.odom = None
            self.odom_wall = 0
            self.truth, self.estimates, self.scan_index, self.arrivals = [], [], [], []
            self.last_scan_time = -100
            self.last_truth_time = -100
            self.current_target = None
            self.phase = 'starting'
            self.command_permitted = False
            self.cmd = self.create_publisher(Twist, '/cmd_vel', 1)
            self.create_subscription(String, '/mapping/status', lambda m: setattr(self, 'mapping', json.loads(m.data)), 100)
            self.create_subscription(String, '/lio/base_odometry_status', lambda m: setattr(self, 'base', json.loads(m.data)), 100)
            self.create_subscription(String, '/lio/runtime_status', lambda m: setattr(self, 'runtime', json.loads(m.data)), 100)
            self.create_subscription(String, '/sim/motion_state', lambda m: setattr(self, 'state', json.loads(m.data)), 100)
            self.create_subscription(Odometry, '/odom', self.on_odom, 200)
            # This callback stores truth for later evaluation. No control method reads it.
            self.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_truth, 1000)
            self.create_subscription(PointCloud2, '/lio/cloud_registered', self.on_cloud, 20)
            self.controls = {n: self.create_client(Trigger, t) for n, t in [('save', '/mapping/save'), ('pause', '/sim/pause')]}

        def on_odom(self, message):
            if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
                raise RuntimeError('Unexpected controller odometry frame')
            row = pose(message)
            if not np.isfinite(row).all():
                raise RuntimeError('Non-finite control pose')
            self.odom = row
            self.odom_wall = time.monotonic()
            self.estimates.append(row)

        def on_truth(self, message):
            row = pose(message)
            if row[0] - self.last_truth_time >= .049:
                self.truth.append(row)
                self.last_truth_time = row[0]

        def on_cloud(self, message):
            t = stamp(message.header)
            if not self.base or not self.base['aligned'] or t < self.base['anchor_ns']/1e9:
                return
            if t - self.last_scan_time < cfg['scan_replay_period_s'] - .001:
                return
            points = pointcloud_xyzi(message, 200000)
            R = np.asarray(self.base['odom_from_lio_rotation'])
            points[:, :3] = points[:, :3] @ R.T + np.asarray(self.base['odom_from_lio_translation'])
            name = f'{len(self.scan_index):05d}.npz'
            np.savez_compressed(out/'scans'/name, points=points, stamp_s=t)
            self.scan_index.append(dict(file=name, stamp_s=t, points=len(points)))
            self.last_scan_time = t

        def spin(self, duration=.02):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if interrupted:
                    raise InterruptedError('Survey interrupted')
                rclpy.spin_once(self, timeout_sec=min(.01, max(0, deadline-time.monotonic())))
            for name, p in children.items():
                if p.poll() is not None:
                    raise RuntimeError(f'{name} exited: {p.returncode}')
            if self.mapping and self.mapping['phase'] == 'invalidated':
                raise RuntimeError('Map invalidated: ' + self.mapping['reason'])
            if self.command_permitted and self.runtime and self.runtime['generation'] != 0:
                raise RuntimeError('Estimator generation changed during survey')

        def wait(self, predicate, timeout=90):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self.spin()
                if predicate():
                    return
            raise TimeoutError(dict(phase=self.phase, state=self.state, mapping=self.mapping))

        def command(self, vx=0., vy=0., wz=0.):
            if not self.command_permitted:
                raise RuntimeError('Simulator ownership not confirmed')
            m = Twist()
            m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
            self.cmd.publish(m)

        def report(self):
            value = dict(phase=self.phase, waypoint=self.current_target, waypoints=len(waypoints),
                         pose_odom=self.odom, motion_state=self.state, mapping=self.mapping,
                         arrivals=self.arrivals, replay_scans=len(self.scan_index),
                         elapsed_wall_s=time.monotonic()-start_wall)
            temp = out/'progress.pending'
            temp.write_text(json.dumps(value, indent=2)+'\n')
            temp.replace(out/'progress.json')

        def service(self, name):
            c = self.controls[name]
            if not c.wait_for_service(timeout_sec=10):
                raise RuntimeError('Service unavailable: ' + name)
            f = c.call_async(Trigger.Request())
            self.wait(f.done, 40)
            if not f.result().success:
                raise RuntimeError(f.result().message)
            return f.result()

        def hold(self, seconds, yaw_rate=0.):
            begin = self.state['sim_time_s']
            end_wall = time.monotonic() + max(120., seconds*30)
            last_report = 0.
            while self.state['sim_time_s'] < begin + seconds:
                if time.monotonic() > end_wall:
                    raise TimeoutError('Simulation did not advance while holding')
                self.command(wz=yaw_rate)
                self.spin(.035)
                if time.monotonic()-last_report > 2:
                    self.report()
                    last_report = time.monotonic()

        def drive_to(self, target, index):
            self.current_target = index
            begin = self.state['sim_time_s']
            deadline = time.monotonic()+1200
            last_report = 0.
            while True:
                if time.monotonic() > deadline or self.state['sim_time_s']-begin > 100:
                    raise TimeoutError('Waypoint not reached')
                if time.monotonic()-self.odom_wall > 8 or not self.base['aligned']:
                    self.command()
                    raise RuntimeError('Control odometry unavailable')
                xy, yaw = np.asarray(self.odom[1:3]), self.odom[4]
                delta = target - xy
                distance = float(np.linalg.norm(delta))
                if float(clearance_xy([xy], room, radius)[0]) < .04:
                    self.command()
                    raise RuntimeError('Estimated pose approaches the known-room clearance boundary')
                if distance < cfg['position_tolerance_m']:
                    self.command()
                    self.arrivals.append(dict(index=index, target=target.tolist(), odom=self.odom,
                                              sim_time_s=self.state['sim_time_s']))
                    print('WAYPOINT', index+1, '/', len(waypoints), json.dumps(self.arrivals[-1]), flush=True)
                    self.hold(cfg['settle_at_waypoint_s'])
                    self.report()
                    return
                velocity = delta/max(distance, 1e-9) * min(cfg['speed_mps'], .8*distance)
                c, s = math.cos(yaw), math.sin(yaw)
                self.command(c*velocity[0]+s*velocity[1], -s*velocity[0]+c*velocity[1], cfg['yaw_rate_radps'])
                self.spin(.035)
                if time.monotonic()-last_report > 2:
                    self.report()
                    last_report = time.monotonic()

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        node = Survey()
        node.spin(.8)
        if node.get_subscriptions_info_by_topic('/cmd_vel') or node.get_publishers_info_by_topic('/mapping/status'):
            raise RuntimeError('Existing motion/mapping session detected; refusing survey')
        record('planned_route_clears_complete_padded_robot', **contract)
        launch('mapping', ['/usr/bin/python3', str(root/'scripts/run_mapping.py'), '--map-id', args.map_id,
                           '--output-dir', str(out/'mapping')])
        launch('simulation', ['/usr/bin/python3', str(root/'scripts/run_motion.py'), '--lidar', '--imu',
                              '--lidar-samples-per-scan', '6000', '--output-dir', str(out/'backend')])
        node.wait(lambda: node.mapping and node.mapping['scans'] >= 10 and node.odom is not None, 240)
        ready = json.loads((out/'backend/ready.json').read_text())
        subscribers = node.get_subscriptions_info_by_topic('/cmd_vel')
        publishers = node.get_publishers_info_by_topic('/cmd_vel')
        if (len(subscribers) != 1 or subscribers[0].node_name != 'quadruped_kinematic_sim'
                or len(publishers) != 1 or publishers[0].node_name != node.get_name()
                or ready['robot'] != 'd1_edu' or not ready['lidar_enabled'] or not ready['imu_enabled']):
            raise RuntimeError('Velocity topic is not exclusive to this Isaac simulation')
        node.command_permitted = True
        record('commands_bound_to_owned_simulation_only', subscriber=subscribers[0].node_name)
        node.phase = 'surveying'
        for index, target in enumerate(waypoints):
            node.drive_to(target, index)
        node.phase = 'final_scan'
        node.hold(cfg['final_scan_s'], cfg['yaw_rate_radps'])
        node.hold(1.)
        node.phase = 'saving'
        node.service('pause')
        node.spin(.6)
        saved = node.service('save')
        manifest, points = load_bundle(Path(saved.message))
        if len(node.arrivals) != len(waypoints) or manifest['truth_used']:
            raise RuntimeError('Incomplete survey or invalid map provenance')
        record('all_waypoints_reached_and_sensor_map_saved', waypoints=len(node.arrivals),
               scans=manifest['source']['scans'], points=len(points), map_directory=saved.message)
        node.phase = 'complete'
        node.report()
        result = 'PASS'
    except Exception:
        error = traceback.format_exc()
        print(error, flush=True)
        (out/'error.txt').write_text(error)
    finally:
        if node is not None and node.command_permitted and rclpy.ok():
            node.command()
        codes = {name: stop(name) for name in reversed(children)}
        if node is not None:
            np.savez_compressed(out/'trajectory.npz', truth=np.asarray(node.truth), odometry=np.asarray(node.estimates))
            (out/'scan_index.json').write_text(json.dumps(node.scan_index, indent=2)+'\n')
            (out/'arrivals.json').write_text(json.dumps(node.arrivals, indent=2)+'\n')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for handle in logs:
            handle.close()
        if any(c != 0 for c in codes.values()):
            result = 'FAIL'
        report = dict(result=result, checks=checks, return_codes=codes,
                      map_id=args.map_id, output_directory=str(out), elapsed_wall_s=time.monotonic()-start_wall)
        (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        print('ROOM_SURVEY', json.dumps(report), flush=True)
    return 0 if result == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
