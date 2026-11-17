#!/usr/bin/env python3
"""Start the simulation-only D1 ROS /cmd_vel controller in Isaac Sim."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import sys

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller-calibration-only', action='store_true')
    parser.add_argument('--standby-on-stop', action='store_true', help='Return to stable PD stand after zero command')
    parser.add_argument('--command-timeout-sim-s', type=float, default=.1)
    parser.add_argument('--max-seconds', type=float, default=60., help='Simulation duration')
    parser.add_argument('--spawn-offset-x',type=float,default=0.,help='Simulation-only initial placement offset')
    parser.add_argument('--gpu-index', type=int, default=1)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if not 1. <= args.max_seconds <= 3600.:
        parser.error('--max-seconds must be between 1 and 3600')
    root = Path(__file__).resolve().parents[1]
    host = yaml.safe_load((root / 'ros2_ws/src/quadruped_bringup/config/hosts/server.yaml').read_text())
    isaac = Path(host['isaac_root'])
    output = args.output_dir or Path(host['data_root']) / 'runs' / ('gait_cmd_vel_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    command = [str(isaac / 'python.sh'), str(root / 'ros2_ws/src/quadruped_sim/scripts/gait_ros_cmd_ready.py'),
               '--project-root', str(root), '--output-dir', str(output), '--model', 'loaded', '--policy',
               '--gpu-index', str(args.gpu_index), '--max-seconds', str(args.max_seconds)]
    command += ['--spawn-offset-x',str(args.spawn_offset_x)]
    if args.standby_on_stop:command += ['--standby-on-stop','--command-timeout-sim-s',str(args.command_timeout_sim_s)]
    if args.controller_calibration_only:command += ['--controller-calibration-only']
    env = dict(os.environ)
    env.pop('PYTHONPATH', None)
    env['ROS_DISTRO'] = host['ros_distro']
    env['ROS_DOMAIN_ID'] = str(host['ros_domain_id'])
    env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    libraries = isaac / 'exts/isaacsim.ros2.bridge' / host['ros_distro'] / 'lib'
    env['LD_LIBRARY_PATH'] = str(libraries) + os.pathsep + env.get('LD_LIBRARY_PATH', '')
    print(f'Starting Isaac D1 ROS /cmd_vel. Run directory: {output}', flush=True)
    os.execve(command[0], command, env)


if __name__ == '__main__':
    main()
