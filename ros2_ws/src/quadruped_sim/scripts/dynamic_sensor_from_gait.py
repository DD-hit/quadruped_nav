#!/usr/bin/env python3
"""Generate ideal Mid-360S lidar and built-in IMU from an Isaac gait trajectory.

This is an offline sensor probe. Ground truth is used only to generate simulated
measurements and later evaluate them, never as an estimator input.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml


def transform(spec):
    return Rotation.from_euler('xyz', spec['rpy_rad']), np.asarray(spec['translation_m'], dtype=np.float64)


def raycast_boxes(origin, direction, boxes, minimum, maximum):
    nearest = np.full(len(origin), np.inf)
    box_index = np.full(len(origin), -1, dtype=np.int16)
    with np.errstate(divide='ignore', invalid='ignore'):
        for number, (_, low, high) in enumerate(boxes):
            ta = (low - origin) / direction
            tb = (high - origin) / direction
            entry = np.max(np.minimum(ta, tb), axis=1)
            exit_ = np.min(np.maximum(ta, tb), axis=1)
            hit = (exit_ >= np.maximum(entry, 0.)) & (entry >= minimum) & (entry <= maximum) & (entry < nearest)
            nearest[hit] = entry[hit]
            box_index[hit] = number
    return nearest, box_index


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--gait-run', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--samples-per-scan', type=int, default=1200)
    parser.add_argument('--initial-static-seconds', type=float, default=0.)
    parser.add_argument('--source-start-s', type=float, default=2.)
    args = parser.parse_args()
    if args.samples_per_scan <= 0 or args.samples_per_scan > 10000:
        parser.error('samples-per-scan must be in [1, 10000]')
    if args.initial_static_seconds < 0 or args.initial_static_seconds > 10:
        parser.error('initial-static-seconds must be in [0, 10]')
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=False)
    gait = json.loads((args.gait_run / 'acceptance.json').read_text())
    if gait['result'] != 'PASS':
        raise ValueError('Input gait run must have a formal PASS acceptance')
    rows = [json.loads(line) for line in (args.gait_run / 'states.jsonl').open()]
    original_time = np.asarray([row['t'] for row in rows], dtype=np.float64)
    dt = np.diff(original_time)
    if len(rows) != 3600 or not np.allclose(dt, .005, atol=1e-9):
        raise ValueError('Expected continuous 200 Hz Isaac physics trajectory')
    profile = yaml.safe_load((args.project_root / 'ros2_ws/src/quadruped_bringup/config/robots/d1_edu.yaml').read_text())
    lidar_cfg = yaml.safe_load((args.project_root / 'ros2_ws/src/quadruped_sim/config/geometric_lidar.yaml').read_text())
    imu_cfg = yaml.safe_load((args.project_root / 'ros2_ws/src/quadruped_sim/config/ideal_imu.yaml').read_text())
    r_bl, p_bl = transform(profile['sensor']['base_to_lidar'])
    r_li, p_li = transform(profile['sensor']['lidar_to_imu'])
    r_bi = r_bl * r_li
    p_bi = p_bl + r_bl.apply(p_li)

    position = np.asarray([row['pos'] for row in rows], dtype=np.float64)
    quaternion = np.asarray([row['quat'] for row in rows], dtype=np.float64)
    body_velocity = np.asarray([row['body_velocity'] for row in rows], dtype=np.float64)
    body_angular = np.asarray([row['angular_velocity'] for row in rows], dtype=np.float64)
    prefix_count = round(args.initial_static_seconds / .005)
    if prefix_count:
        start_index = round(args.source_start_s / .005)
        if start_index < 400 or start_index >= len(rows) - 100:
            raise ValueError('Static prefix must join a settled source pose')
        position = np.concatenate([np.repeat(position[start_index:start_index+1], prefix_count, axis=0), position[start_index:]])
        quaternion = np.concatenate([np.repeat(quaternion[start_index:start_index+1], prefix_count, axis=0), quaternion[start_index:]])
        body_velocity = np.concatenate([np.zeros((prefix_count, 3)), body_velocity[start_index:]])
        body_angular = np.concatenate([np.zeros((prefix_count, 3)), body_angular[start_index:]])
        source_time = np.concatenate([np.full(prefix_count, original_time[start_index]), original_time[start_index:]])
        t = np.arange(len(position)) * .005
    else:
        source_time = original_time.copy()
        t = original_time.copy()
    base_rotation = Rotation.from_quat(quaternion[:, [1, 2, 3, 0]])
    world_velocity = base_rotation.apply(body_velocity)
    world_angular = base_rotation.apply(body_angular)
    acceleration = np.empty_like(world_velocity)
    alpha = np.empty_like(world_angular)
    acceleration[1:] = np.diff(world_velocity, axis=0) / .005
    alpha[1:] = np.diff(world_angular, axis=0) / .005
    acceleration[0], alpha[0] = acceleration[1], alpha[1]
    lever_world = base_rotation.apply(p_bi)
    sensor_acceleration = acceleration + np.cross(alpha, lever_world) + np.cross(world_angular, np.cross(world_angular, lever_world))
    sensor_rotation = base_rotation * r_bi
    gravity = np.array([0., 0., -float(imu_cfg['gravity_mps2'])])
    specific_force = sensor_rotation.inv().apply(sensor_acceleration - gravity)
    gyro = sensor_rotation.inv().apply(world_angular)
    sensor_position = position + lever_world
    sensor_velocity = world_velocity + np.cross(world_angular, lever_world)
    finite_acceleration = np.diff(sensor_velocity, axis=0) / .005
    lever_consistency = np.linalg.norm(finite_acceleration - sensor_acceleration[1:], axis=1)
    np.savez_compressed(output / 'imu_200hz.npz', time_s=t, specific_force_mps2=specific_force,
                        angular_velocity_radps=gyro, sensor_position_world_m=sensor_position,
                        base_position_world_m=position, base_quat_wxyz=quaternion,
                        source_gait_time_s=source_time)

    boxes = []
    for name, spec in lidar_cfg['room'].items():
        center = np.asarray(spec['center'], dtype=np.float64)
        half = np.asarray(spec['size'], dtype=np.float64) / 2
        boxes.append((name, center - half, center + half))
    scan_hz = int(lidar_cfg['scan_hz'])
    size = args.samples_per_scan
    rate = scan_hz * size
    scan_count = int(np.floor((t[-1] - 1 / scan_hz) * scan_hz)) + 1
    slerp = Slerp(t, base_rotation)
    low, high = np.sin(np.radians(lidar_cfg['elevation_deg']))
    records = []
    scan_stats = []
    for scan in range(scan_count):
        index = scan * size + np.arange(size)
        ray_time = index / rate
        rotation = slerp(ray_time)
        base_point = np.column_stack([np.interp(ray_time, t, position[:, dim]) for dim in range(3)])
        lidar_origin = base_point + rotation.apply(p_bl)
        azimuth = 2 * np.pi * (((index + 1) * ((np.sqrt(5) - 1) / 2)) % 1)
        sine = low + (high - low) * (((index + 1) * (np.sqrt(2) - 1)) % 1)
        cosine = np.sqrt(np.maximum(0., 1 - sine * sine))
        local_direction = np.column_stack([cosine * np.cos(azimuth), cosine * np.sin(azimuth), sine])
        world_direction = (rotation * r_bl).apply(local_direction)
        distance, hit_box = raycast_boxes(lidar_origin, world_direction, boxes,
                                          float(lidar_cfg['near_range_m']), float(lidar_cfg['far_range_m']))
        valid = hit_box >= 0
        point_local = local_direction[valid] * distance[valid, None]
        point_world = lidar_origin[valid] + world_direction[valid] * distance[valid, None]
        scan_start_rotation = slerp([scan / scan_hz])[0] * r_bl
        scan_start_position = np.array([np.interp(scan / scan_hz, t, position[:, dim]) for dim in range(3)]) + slerp([scan / scan_hz])[0].apply(p_bl)
        scan_start_projected = scan_start_position + scan_start_rotation.apply(point_local)
        distortion = np.linalg.norm(point_world - scan_start_projected, axis=1)
        records.append((np.full(np.count_nonzero(valid), scan, dtype=np.uint16),
                        np.rint((ray_time[valid] - scan / scan_hz) * 1e9).astype(np.uint32),
                        point_local.astype(np.float32), hit_box[valid].astype(np.uint8)))
        scan_stats.append(dict(scan=scan, stamp_s=scan / scan_hz, rays=size, points=int(np.count_nonzero(valid)),
                               motion_projection_p95_m=float(np.percentile(distortion, 95)) if len(distortion) else 0.))
    np.savez_compressed(output / 'lidar_10hz.npz',
                        scan=np.concatenate([v[0] for v in records]),
                        offset_time_ns=np.concatenate([v[1] for v in records]),
                        xyz_lidar_m=np.concatenate([v[2] for v in records]),
                        hit_box=np.concatenate([v[3] for v in records]),
                        scan_stamp_s=np.arange(scan_count) / scan_hz)

    walking = (source_time >= 4.) & (source_time < 12.) & (t >= args.initial_static_seconds)
    standing = (t >= 1.) & (t < args.initial_static_seconds) if prefix_count else (t >= 2.) & (t < 4.)
    summary = dict(result='GENERATED', gait_run=str(args.gait_run), imu_frame='lidar_imu_link', lidar_frame='lidar_link',
                   imu_hz=200, scan_hz=scan_hz, rays_per_scan=size, scans=scan_count,
                   initial_static_seconds=args.initial_static_seconds, source_start_s=args.source_start_s if prefix_count else 0.,
                   walking_window_s=[float(np.min(t[walking])), float(np.max(t[walking]) + .005)],
                   imu_samples=len(t), emitted_points=int(sum(v['points'] for v in scan_stats)),
                   base_to_imu_translation_m=p_bi.tolist(), base_to_imu_quat_xyzw=r_bi.as_quat().tolist(),
                   base_to_lidar_translation_m=p_bl.tolist(), base_to_lidar_quat_xyzw=r_bl.as_quat().tolist(),
                   walking_body_roll_pitch_max_rad=float(np.max(np.abs(base_rotation.as_euler('xyz')[walking, :2]))),
                   walking_gyro_xy_p95_radps=float(np.percentile(np.linalg.norm(gyro[walking, :2], axis=1), 95)),
                   standing_gyro_xy_p95_radps=float(np.percentile(np.linalg.norm(gyro[standing, :2], axis=1), 95)),
                   walking_specific_force_norm_p95_mps2=float(np.percentile(np.linalg.norm(specific_force[walking], axis=1), 95)),
                   imu_lever_acceleration_consistency_p95_mps2=float(np.percentile(lever_consistency, 95)),
                   scan_motion_projection_p95_m=float(np.percentile([v['motion_projection_p95_m'] for v in scan_stats if
                       float(np.min(t[walking])) <= v['stamp_s'] < float(np.max(t[walking]))], 95)),
                   point_counts=[v['points'] for v in scan_stats],
                   sensor_noise=False, manufacturer_scan_pattern=False, robot_self_returns=False,
                   ros_published=False, fastlio_run=False, hardware=False)
    (output / 'scan_stats.jsonl').write_text(''.join(json.dumps(v) + '\n' for v in scan_stats))
    (output / 'generation.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'point_counts'}, indent=2))


if __name__ == '__main__':
    main()
