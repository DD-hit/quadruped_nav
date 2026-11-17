#!/usr/bin/env python3
"""Launch the kinematic backend in Isaac's isolated ROS 2 Python runtime."""
import argparse
import os
from pathlib import Path
import re
import yaml

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--robot', default='d1_edu')
parser.add_argument('--host', default='server')
parser.add_argument('--gui', action='store_true')
parser.add_argument('--lidar', action='store_true', help='Enable the idealized timed geometric lidar')
parser.add_argument('--imu', action='store_true', help='Enable the ideal 200 Hz built-in IMU')
parser.add_argument('--lidar-samples-per-scan', type=int, help='Override geometric scan density, e.g. 6000 for LIO')
parser.add_argument('--output-dir', type=Path)
parser.add_argument('--duration-wall', type=float, default=0.0)
args = parser.parse_args()
if not all(re.fullmatch(r'[a-z][a-z0-9_]*', v) for v in (args.robot, args.host)):
    parser.error('Invalid profile identifier')
if args.lidar_samples_per_scan is not None and (not args.lidar or args.lidar_samples_per_scan < 2):
    parser.error('--lidar-samples-per-scan requires --lidar and a value of at least 2')
root = Path(__file__).resolve().parents[1]
profile_dir = root/'ros2_ws/src/quadruped_bringup/config'
host = yaml.safe_load((profile_dir/'hosts'/f'{args.host}.yaml').read_text())
isaac_root = Path(host['isaac_root'])
bridge_libraries = isaac_root/'exts/isaacsim.ros2.bridge'/host['ros_distro']/'lib'
command = [str(isaac_root/'python.sh'), str(root/'ros2_ws/src/quadruped_sim/scripts/kinematic_sim.py'),
           '--project-root', str(root), '--robot', args.robot, '--host', args.host,
           '--duration-wall', str(args.duration_wall)]
if args.gui:
    command.append('--gui')
if args.lidar:
    command.append('--lidar')
if args.imu:
    command.append('--imu')
if args.lidar_samples_per_scan is not None:
    command += ['--lidar-samples-per-scan', str(args.lidar_samples_per_scan)]
if args.output_dir:
    command += ['--output-dir', str(args.output_dir)]
environment = dict(os.environ)
environment.pop('PYTHONPATH', None)
environment['ROS_DISTRO'] = host['ros_distro']
environment['ROS_DOMAIN_ID'] = str(host['ros_domain_id'])
environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
environment['LD_LIBRARY_PATH'] = str(bridge_libraries)+os.pathsep+environment.get('LD_LIBRARY_PATH', '')
os.execve(command[0], command, environment)
