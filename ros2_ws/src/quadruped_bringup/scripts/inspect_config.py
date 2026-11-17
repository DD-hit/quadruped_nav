#!/usr/bin/env python3
"""Resolve project profiles for inspection only; never start a backend."""
import argparse
import json
import math
from pathlib import Path
import re
import sys

import yaml


def read_profile(path):
    with path.open(encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict) or profile.get('schema_version') != 1:
        raise ValueError(f'Unsupported or missing schema_version: {path}')
    return profile


def identifier(value):
    if not re.fullmatch(r'[a-z][a-z0-9_]*', value):
        raise ValueError(f'Invalid profile identifier: {value}')
    return value


def resolve(config_dir, robot, mode, host):
    robot, mode, host = map(identifier, (robot, mode, host))
    common = read_profile(config_dir / 'common.yaml')
    robot_config = read_profile(config_dir / 'robots' / f'{robot}.yaml')
    mode_config = read_profile(config_dir / 'modes' / f'{mode}.yaml')
    host_config = read_profile(config_dir / 'hosts' / f'{host}.yaml')
    if robot_config['robot_id'] != robot or mode_config['mode'] != mode or host_config['host_id'] != host:
        raise ValueError('Profile identity does not match selected filename')
    if mode_config['use_sim_time'] is not (mode == 'sim'):
        raise ValueError('Clock configuration conflicts with selected mode')
    if common['stage'] not in ('skeleton', 'model_review', 'kinematic_motion', 'lidar_geometry', 'imu_simulation', 'lio_input_adapter', 'fastlio_odometry', 'base_odometry_tf', 'localization_map_io', 'navigation_map_tools', 'saved_map_localization_ros', 'robot_geometry_verified', 'map_state_queries_verified', 'simulation_velocity_tracking', 'continuous_replanning_verified', 'upstream_navigation_core') or mode_config['implementation_status'] not in ('pending', 'kinematic_motion_verified'):
        raise ValueError('Unsupported project stage for configuration inspection')
    if mode_config['hardware_enabled'] is not False:
        raise ValueError('Current profiles must not enable hardware')
    for transform in ('base_to_lidar', 'lidar_to_imu'):
        for field in ('translation_m', 'rpy_rad'):
            values = robot_config['sensor'][transform][field]
            if len(values) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
                raise ValueError(f'Invalid transform vector: {transform}.{field}')
    topics = [v['topic'] for v in common['interfaces'].values()]
    if len(topics) != len(set(topics)) or not all(t.startswith('/') for t in topics):
        raise ValueError('Interface topics must be absolute and distinct')
    backend = mode_config['platform_backend']
    if backend == 'robot_adapter':
        backend = robot_config['adapter_package']
    return dict(stage=common['stage'], execution='configuration_inspection_only',
                selected=dict(robot=robot, mode=mode, host=host, backend=backend),
                common=common, robot=robot_config, mode=mode_config, host=host_config)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='d1_edu')
    parser.add_argument('--mode', choices=('sim', 'real'), default='sim')
    parser.add_argument('--host', default='server')
    parser.add_argument('--config-dir', type=Path, default=Path(__file__).resolve().parents[1] / 'config')
    args = parser.parse_args()
    try:
        result = resolve(args.config_dir, args.robot, args.mode, args.host)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        print(f'Configuration error: {error}', file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
