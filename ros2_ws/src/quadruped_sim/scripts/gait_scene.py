"""Shared, explicit room geometry for the D1 dynamic simulation and survey."""
from copy import deepcopy
from pathlib import Path
import yaml


def load_gait_lidar_config(root):
    directory = Path(root) / 'ros2_ws/src/quadruped_sim/config'
    config = yaml.safe_load((directory / 'geometric_lidar.yaml').read_text())
    overrides = yaml.safe_load((directory / 'gait_world.yaml').read_text())
    config = deepcopy(config)
    config['samples_per_scan'] = overrides['lidar_samples_per_scan']
    for name, value in overrides['room_overrides'].items():
        config['room'][name].update(value)
    room = config['room']
    xmin = room['west']['center'][0] - room['west']['size'][0] / 2
    xmax = room['east']['center'][0] + room['east']['size'][0] / 2
    for name in ('north', 'south', 'floor', 'ceiling'):
        low = room[name]['center'][0] - room[name]['size'][0] / 2
        high = room[name]['center'][0] + room[name]['size'][0] / 2
        if low > xmin + 1e-6 or high < xmax - 1e-6:
            raise ValueError(f'{name} does not span the full dynamic room')
    return config
