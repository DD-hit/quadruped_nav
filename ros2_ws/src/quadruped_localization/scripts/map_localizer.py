#!/usr/bin/env python3
"""Owned, synchronized saved-map localization; sole new TF is map -> odom."""
import argparse
from collections import OrderedDict, deque
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import fcntl
import hashlib
import json
from pathlib import Path
import os
import time

import numpy as np
import yaml
import rclpy
from rclpy.clock import Clock as ROSClock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from base_geometry import quaternion_matrix, matrix_quaternion
from lio_config import rotation
from map_bundle import pointcloud_xyzi
from map_registration import (MapRegistration, RegistrationConfig, rigid_transform,
    inverse_transform, base_pose_to_registration_guess, registration_to_map_odom)
from localization_map_source import load_localization_source
from localization_config import validate_profile


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def pose_matrix(pose):
    T = np.eye(4)
    q = pose.orientation
    T[:3, :3] = quaternion_matrix([q.x, q.y, q.z, q.w])
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return rigid_transform(T)


def fill_pose(pose, T):
    pose.position.x, pose.position.y, pose.position.z = map(float, T[:3, 3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, matrix_quaternion(T[:3, :3]))


def tf_signature(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    return (ns(transform.header.stamp), transform.header.frame_id, transform.child_frame_id,
            t.x, t.y, t.z, q.x, q.y, q.z, q.w)


class MapLocalizer(Node):
    def __init__(self, common, robot, profile, map_info, points_map, status_file=None):
        super().__init__('quadruped_map_localizer')
        validate_profile(profile)
        self.common, self.robot = common, robot
        self.cfg = profile['runtime']
        self.frames = common['frames']
        self.registration = MapRegistration(points_map, RegistrationConfig.from_dict(profile['registration']))
        self.map_info, self.status_file = map_info, status_file
        self.clock_ns = None
        self.last_clock_wall = time.monotonic()
        self.generation = None
        self.generation_floor = 0
        self.alignment_id = None
        self.anchor = None
        self.base_status = self.runtime_status = None
        self.base_status_wall = self.runtime_status_wall = 0.
        self.epoch = 0
        self.pending = OrderedDict()
        self.history = deque(maxlen=self.cfg['max_pending_pairs'])
        self.initial = None
        self.last_stationary_measurement = None
        self.accepted = None
        self.accepted_ns = None
        self.last_pair_ns = self.last_pub_ns = self.last_submit_ns = None
        self.phase, self.reason = 'waiting_lio', 'waiting_for_aligned_inputs'
        self.rejected_inputs = self.discarded_results = self.published = 0
        self.accepted_matches = self.failed_matches = 0
        self.quality = None
        self.initialization_correction = None
        self.last_initial_pose_ns = None
        self.future = None
        self.icp_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='map_icp')
        self.fatal_error = None
        self.sent_tf_signatures = deque(maxlen=200)
        mount = robot['sensor']['base_to_lidar']
        self.base_from_sensor = np.eye(4)
        self.base_from_sensor[:3, :3] = rotation(mount['rpy_rad'])
        self.base_from_sensor[:3, 3] = mount['translation_m']
        interfaces = common['interfaces']
        self.pose_pub = self.create_publisher(PoseStamped, interfaces['localization_pose']['topic'], 20)
        self.status_pub = self.create_publisher(String, interfaces['localization_status']['topic'],
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = TransformBroadcaster(self)
        self.create_subscription(PointCloud2, self.cfg['scan_topic'], lambda m: self.pair('scan', m), 50)
        self.create_subscription(Odometry, interfaces['odometry']['topic'], lambda m: self.pair('odom', m), 50)
        self.create_subscription(String, self.cfg['base_status_topic'], self.on_base_status, 20)
        self.create_subscription(String, self.cfg['runtime_status_topic'], self.on_runtime_status, 20)
        self.create_subscription(PoseWithCovarianceStamped, interfaces['initial_pose']['topic'], self.on_initial_pose, 10)
        self.create_subscription(Clock, '/clock', self.on_clock,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(TFMessage, '/tf_static', self.on_tf,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05, self.poll, clock=ROSClock(clock_type=ClockType.STEADY_TIME))
        self.report()

    def reset(self, reason):
        self.epoch += 1
        self.pending.clear()
        self.history.clear()
        self.initial = self.accepted = self.accepted_ns = None
        self.last_pair_ns = self.last_pub_ns = self.last_submit_ns = None
        self.anchor = None
        self.alignment_id = None
        self.base_status = self.runtime_status = None
        self.base_status_wall = self.runtime_status_wall = 0.
        self.last_initial_pose_ns = None
        self.quality = None
        self.phase, self.reason = 'waiting_lio', reason
        self.initialization_correction = None
        self.report()

    def on_clock(self, message):
        stamp = ns(message.clock)
        if self.clock_ns is not None and stamp < self.clock_ns - 1000:
            self.generation_floor = max(self.generation_floor, (self.generation or 0) + 1)
            self.reset('clock_rollback_requires_new_initial_pose')
            self.generation = None
        if stamp != self.clock_ns:
            self.last_clock_wall = time.monotonic()
        self.clock_ns = stamp

    def on_runtime_status(self, message):
        try:
            status = json.loads(message.data)
            generation = int(status['generation'])
            if generation < self.generation_floor:
                raise ValueError('Runtime status belongs to the previous epoch')
            if generation < 0 or not status['base_odometry_enabled']:
                raise ValueError('Public base odometry is required')
            if self.generation is not None and generation != self.generation:
                self.reset('estimator_generation_changed_requires_new_initial_pose')
            self.generation = generation
            self.runtime_status, self.runtime_status_wall = status, time.monotonic()
        except (ValueError, KeyError, TypeError) as error:
            self.reject_input(str(error))

    def on_base_status(self, message):
        try:
            status = json.loads(message.data)
            if status['generation'] < self.generation_floor or (self.generation is not None and status['generation'] < self.generation):
                raise ValueError('Base alignment belongs to the previous epoch')
            if status['aligned']:
                if status['anchor_ns'] > self.now_ns() + int(self.cfg['future_tolerance_s'] * 1e9):
                    raise ValueError('Base anchor is from a future epoch')
                if status['frames'] != {'parent': self.frames['odom'], 'child': self.robot['frame_aliases']['base']}:
                    raise ValueError('Wrong base alignment frames')
                anchor = np.eye(4)
                anchor[:3, :3] = status['odom_from_lio_rotation']
                anchor[:3, 3] = status['odom_from_lio_translation']
                anchor = rigid_transform(anchor)
                alignment_id = (status['generation'], status['local_reset_count'], status['anchor_ns'])
                if self.alignment_id is not None and (alignment_id != self.alignment_id or not np.allclose(anchor, self.anchor, atol=1e-9, rtol=0)):
                    self.reset('base_alignment_changed_requires_new_initial_pose')
                self.anchor, self.alignment_id = anchor, alignment_id
            elif self.anchor is not None:
                self.reset('base_alignment_lost_requires_new_initial_pose')
            self.base_status, self.base_status_wall = status, time.monotonic()
        except (ValueError, KeyError, TypeError) as error:
            self.reject_input(str(error))

    def ready(self):
        wall = time.monotonic()
        if self.get_parameter('use_sim_time').value:
            fresh = (self.base_status is not None and self.runtime_status is not None
                     and self.now_ns() - self.base_status.get('last_input_ns', 0) <= int(self.cfg['status_max_age_s'] * 1e9)
                     and self.now_ns() - int((self.runtime_status.get('last_odometry_s') or 0) * 1e9) <= int(self.cfg['status_max_age_s'] * 1e9))
        else:
            fresh = wall - min(self.base_status_wall, self.runtime_status_wall) <= self.cfg['status_timeout_wall_s']
        return (self.anchor is not None and self.base_status is not None and self.runtime_status is not None
                and self.base_status.get('aligned', False)
                and self.base_status['generation'] == self.generation
                and self.runtime_status.get('initialized', False)
                and fresh)

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def reject_input(self, reason):
        self.rejected_inputs += 1
        self.reason = reason

    def pair(self, kind, message):
        try:
            stamp = ns(message.header.stamp)
            now = self.now_ns()
            if stamp <= 0 or stamp > now + int(self.cfg['future_tolerance_s'] * 1e9):
                raise ValueError('Input timestamp is zero or from a future epoch')
            if now - stamp > int(self.cfg['input_max_age_s'] * 1e9):
                raise ValueError('Input is stale')
            if self.last_pair_ns is not None and stamp <= self.last_pair_ns:
                raise ValueError('Duplicate or out-of-order input')
            row = self.pending.setdefault(stamp, {})
            if kind in row:
                raise ValueError('Duplicate input')
            row[kind] = message
            while len(self.pending) > self.cfg['max_pending_pairs']:
                self.pending.popitem(last=False)
            if len(row) == 2:
                self.pending.pop(stamp)
                self.process_pair(row['scan'], row['odom'])
        except (ValueError, KeyError, TypeError) as error:
            self.reject_input(str(error))

    def process_pair(self, scan, odometry):
        if self.fatal_error:
            return
        if not self.ready():
            self.phase, self.reason = 'waiting_lio', 'waiting_for_aligned_inputs'
            return
        if (scan.header.frame_id != self.cfg['scan_frame'] or odometry.header.frame_id != self.frames['odom']
                or odometry.child_frame_id != self.robot['frame_aliases']['base']):
            raise ValueError('Unexpected input frame')
        stamp = ns(scan.header.stamp)
        points = pointcloud_xyzi(scan, self.registration.config.max_scan_points)[:, :3]
        odom_from_base = pose_matrix(odometry.pose.pose)
        linear, angular = odometry.twist.twist.linear, odometry.twist.twist.angular
        velocity = np.array([linear.x, linear.y, linear.z, angular.x, angular.y, angular.z])
        if not np.isfinite(velocity).all():
            raise ValueError('Nonfinite odometry velocity')
        self.last_pair_ns = stamp
        self.history.append((stamp, odom_from_base, velocity))
        # Output follows fresh paired odometry, never a wall timer or stale pose.
        if self.accepted is not None and stamp - self.accepted_ns <= int(self.cfg['correction_max_age_s'] * 1e9):
            self.publish_pose(odometry, odom_from_base)
        elif self.accepted is not None:
            self.accepted = None
            self.epoch += 1
            self.phase, self.reason = 'lost', 'correction_expired_requires_new_initial_pose'
        if self.future is not None or self.fatal_error:
            return
        if self.last_submit_ns is not None and stamp - self.last_submit_ns < int(self.cfg['registration_interval_s'] * 1e9):
            return
        if self.accepted is None:
            if self.initial is None:
                if self.phase not in ('lost', 'initialization_failed'):
                    self.phase, self.reason = 'waiting_initial_pose', 'set_mechanical_base_pose_in_map'
                return
            if stamp - self.initial['stamp'] > int(self.cfg['initial_pose_max_age_s'] * 1e9):
                self.initial = None
                self.phase, self.reason = 'waiting_initial_pose', 'initial_pose_expired'
                return
            initial_sample = min(self.history, key=lambda item: abs(item[0] - self.initial['stamp']))
            if abs(initial_sample[0] - self.initial['stamp']) > int(self.cfg['initial_pair_tolerance_s'] * 1e9):
                self.reason = 'waiting_for_initial_pose_time_pair'
                return
            recent = [s for s in self.history if stamp - s[0] <= int(self.cfg['stationary_window_s'] * 1.2e9)]
            if len(recent) < 2 or stamp - recent[0][0] < int(self.cfg['stationary_window_s'] * 1e9):
                self.reason = 'waiting_for_stationary_initialization_window'
                return
            positions = np.array([s[1][:3, 3] for s in recent] + [initial_sample[1][:3, 3]])
            velocities = np.array([s[2] for s in recent] + [initial_sample[2]])
            rotations = [s[1][:3, :3] for s in recent] + [initial_sample[1][:3, :3]]
            angles = [np.arccos(np.clip((np.trace(R @ rotations[-1].T)-1)/2, -1, 1)) for R in rotations]
            self.last_stationary_measurement = dict(
                linear_m_s=float(np.max(np.linalg.norm(velocities[:, :3], axis=1))),
                angular_rad_s=float(np.max(np.linalg.norm(velocities[:, 3:], axis=1))),
                span_m=float(np.max(np.linalg.norm(positions - positions[-1], axis=1))),
                angle_rad=float(max(angles)))
            if (np.max(np.linalg.norm(velocities[:, :3], axis=1)) > self.cfg['stationary_linear_m_s']
                    or np.max(np.linalg.norm(velocities[:, 3:], axis=1)) > self.cfg['stationary_angular_rad_s']
                    or np.max(np.linalg.norm(positions - positions[-1], axis=1)) > self.cfg['stationary_span_m']
                    or max(angles) > self.cfg['stationary_angle_rad']):
                self.initial = None
                self.phase, self.reason = 'waiting_initial_pose', 'initialization_requires_stationary_robot'
                return
            guess = base_pose_to_registration_guess(self.initial['pose'], initial_sample[1], self.anchor)
            mode, previous = 'initialize', None
            self.phase, self.reason = 'initializing', 'icp_running'
        else:
            guess, previous, mode = self.accepted.copy(), self.accepted.copy(), 'tracking'
        sensor_in_scan = inverse_transform(self.anchor) @ odom_from_base @ self.base_from_sensor
        self.last_submit_ns = stamp
        self.job = dict(epoch=self.epoch, stamp=stamp, anchor=self.anchor.copy(),
                        initial_serial=None if self.initial is None else self.initial['serial'], mode=mode)
        self.future = self.icp_worker.submit(self.registration.register, points, guess, sensor_in_scan,
            mode=mode, previous_map_from_scan=previous)

    def on_initial_pose(self, message):
        try:
            now, stamp = self.now_ns(), ns(message.header.stamp)
            if message.header.frame_id != self.frames['map']:
                raise ValueError('Initial pose must be a mechanical-base pose in map')
            if stamp <= 0 or stamp > now + int(self.cfg['future_tolerance_s'] * 1e9) or now - stamp > int(self.cfg['initial_pose_max_age_s'] * 1e9):
                raise ValueError('Initial pose timestamp is stale or invalid')
            pose = pose_matrix(message.pose.pose)
            covariance = np.array(message.pose.covariance).reshape(6, 6)
            if (not np.isfinite(covariance).all() or not np.allclose(covariance, covariance.T, atol=1e-6)
                    or np.linalg.eigvalsh(covariance).min() < -1e-7):
                raise ValueError('Invalid initial pose covariance')
            self.epoch += 1  # invalidate a running match immediately
            self.accepted = self.accepted_ns = None
            self.last_submit_ns = None
            self.initial = dict(stamp=stamp, pose=pose, serial=self.epoch)
            self.initialization_correction = None
            self.last_initial_pose_ns = stamp
            self.phase, self.reason = 'waiting_initial_pose', 'initial_pose_cached_waiting_for_paired_stationary_inputs'
            self.report()
        except (ValueError, TypeError) as error:
            self.reject_input(str(error))

    def publish_pose(self, odometry, odom_from_base):
        stamp = ns(odometry.header.stamp)
        if self.last_pub_ns is not None and stamp <= self.last_pub_ns:
            return
        map_from_odom = registration_to_map_odom(self.accepted, self.anchor)
        result = PoseStamped()
        result.header = deepcopy(odometry.header)
        result.header.frame_id = self.frames['map']
        fill_pose(result.pose, map_from_odom @ odom_from_base)
        transform = TransformStamped()
        transform.header = deepcopy(result.header)
        transform.child_frame_id = self.frames['odom']
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, map_from_odom[:3, 3])
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, matrix_quaternion(map_from_odom[:3, :3]))
        self.pose_pub.publish(result)
        self.sent_tf_signatures.append(tf_signature(transform))
        self.tf.sendTransform(transform)
        self.last_pub_ns = stamp
        self.published += 1

    def on_tf(self, message):
        for transform in message.transforms:
            if transform.child_frame_id != self.frames['odom']:
                continue
            # This installed Humble rclpy passes no publisher MessageInfo.
            # Startup graph checks + the domain/child-frame process lock prevent
            # ordinary duplicate launch. During runtime, recognize exact echoes
            # of emitted messages and stop on any other transform owning odom.
            # An external publisher replaying an identical message is not
            # distinguishable on this older callback API.
            if tf_signature(transform) not in self.sent_tf_signatures:
                self.fatal_error = 'Another TF publisher owns odom'
                self.accepted = None
                self.initial = None
                self.epoch += 1
                self.phase, self.reason = 'conflict', self.fatal_error

    def poll(self):
        if self.future is not None and self.future.done():
            future, job = self.future, self.job
            self.future = None
            try:
                result = future.result()
                discarded = bool(self.fatal_error) or job['epoch'] != self.epoch or not self.ready() or self.now_ns() - job['stamp'] > int(self.cfg['result_max_age_s'] * 1e9)
                if discarded:
                    self.discarded_results += 1
                elif result.accepted:
                    self.accepted, self.accepted_ns = result.map_from_scan.copy(), job['stamp']
                    self.accepted_matches += 1
                    self.phase, self.reason = 'tracking', 'match_accepted'
                    if job['mode'] == 'initialize':
                        self.initialization_correction = dict(shift_m=result.shift_m,
                            rotation_rad=result.rotation_rad, scan_ns=job['stamp'])
                    self.initial = None
                else:
                    self.failed_matches += 1
                    if job['mode'] == 'initialize':
                        self.initial = None
                        self.phase = 'initialization_failed'
                    else:
                        self.phase = 'degraded'
                    self.reason = result.reason
                self.quality = dict(accepted=result.accepted, reason=result.reason, fitness=result.fitness,
                                    rmse_m=result.rmse_m, correspondences=result.correspondences, scan_ns=job['stamp'], discarded=discarded,
                                    registration_mode=job['mode'],levels_used=[s.level for s in result.stages])
            except Exception as error:
                if job['epoch'] != self.epoch or self.fatal_error:
                    self.discarded_results += 1
                else:
                    self.failed_matches += 1
                    self.initial = None
                    self.phase, self.reason = ('initialization_failed' if self.accepted is None else 'degraded'), 'icp_exception: ' + str(error)
        if self.accepted is not None and (not self.ready() or self.now_ns() - self.accepted_ns > int(self.cfg['correction_max_age_s'] * 1e9)):
            self.accepted = None
            self.phase, self.reason = 'lost', 'localization_inputs_or_correction_expired'
            self.epoch += 1  # do not revive a lost state from a pending job
        self.report()

    def report(self):
        state = dict(phase=self.phase, reason=self.reason, map=self.map_info,
            registration_backend='fast_lio_localization2', initialization_mode='manual_initial_pose',
            initial_pose_reference='mechanical_base',initialization_correction=self.initialization_correction,
            epoch=self.epoch, generation=self.generation, initialized=self.accepted is not None,
            published=self.published, accepted_matches=self.accepted_matches, failed_matches=self.failed_matches,
            discarded_results=self.discarded_results, rejected_inputs=self.rejected_inputs,
            pending_pairs=len(self.pending), last_pair_ns=self.last_pair_ns,
            last_initial_pose_ns=self.last_initial_pose_ns,
            last_published_ns=self.last_pub_ns, correction_scan_ns=self.accepted_ns,
            matching=self.future is not None, quality=self.quality,
            simulation_clock_paused=bool(self.get_parameter('use_sim_time').value and self.clock_ns is not None and time.monotonic()-self.last_clock_wall > .5),
            last_stationary_measurement=self.last_stationary_measurement,
            tf_parent=self.frames['map'], tf_child=self.frames['odom'], pose_reference='mechanical_base',
            truth_used=False, fatal_error=self.fatal_error)
        self.status_pub.publish(String(data=json.dumps(state)))
        if self.status_file is not None:
            temporary = self.status_file.with_suffix('.pending')
            temporary.write_text(json.dumps(state, indent=2) + '\n')
            temporary.replace(self.status_file)

    def close(self):
        self.epoch += 1
        self.icp_worker.shutdown(wait=True, cancel_futures=True)
        self.destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--map-directory', type=Path, required=True)
    parser.add_argument('--robot', default='d1_edu')
    parser.add_argument('--status-file', type=Path)
    parser.add_argument('--config-file', type=Path)
    args, ros_args = parser.parse_known_args(argv)
    root = args.project_root.resolve()
    configs = root / 'ros2_ws/src/quadruped_bringup/config'
    from map_bundle import identifier
    identifier(args.robot)
    common = yaml.safe_load((configs / 'common.yaml').read_text())
    robot = yaml.safe_load((configs / 'robots' / (args.robot + '.yaml')).read_text())
    profile = yaml.safe_load((args.config_file or root / 'ros2_ws/src/quadruped_localization/config/localization.yaml').read_text())
    map_info, points = load_localization_source(args.map_directory)
    lock_name = hashlib.sha256((os.environ.get('ROS_DOMAIN_ID', '0') + ':' + common['frames']['odom']).encode()).hexdigest()[:16]
    lock = open('/tmp/quadruped_localization_' + str(os.getuid()) + '_' + lock_name + '.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('Another localization process already holds this ROS domain/odom authority')
    rclpy.init(args=ros_args)
    node = guard = None
    try:
        guard = Node('quadruped_localization_startup_guard')
        occupied = set()
        def observe(message):
            occupied.update(t.child_frame_id for t in message.transforms if t.child_frame_id == common['frames']['odom'])
        guard.create_subscription(TFMessage, '/tf', observe, 100)
        guard.create_subscription(TFMessage, '/tf_static', observe,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = time.monotonic() + .8
        while time.monotonic() < deadline:
            rclpy.spin_once(guard, timeout_sec=.05)
        collisions = guard.get_publishers_info_by_topic(common['interfaces']['localization_pose']['topic'])
        guard.destroy_node(); guard = None
        if occupied or collisions:
            raise RuntimeError('Another map/odom or localization-pose authority is active')
        node = MapLocalizer(common, robot, profile, map_info, points, args.status_file)
        print('LOCALIZATION_READY', map_info['map_id'], flush=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            if node.fatal_error:
                raise RuntimeError(node.fatal_error)
    except KeyboardInterrupt:
        pass
    finally:
        if guard is not None:
            guard.destroy_node()
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    main()
