#!/usr/bin/env python3
"""Start the isolated Isaac interpreter using a selected host profile."""
import argparse
import os
from pathlib import Path
import re
import yaml

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--robot', default='d1_edu')
parser.add_argument('--host', default='server')
parser.add_argument('--gui', action='store_true')
parser.add_argument('--reuse-import', action='store_true')
args = parser.parse_args()
if not all(re.fullmatch(r'[a-z][a-z0-9_]*', v) for v in (args.robot, args.host)):
    parser.error('Invalid profile identifier')
root = Path(__file__).resolve().parents[1]
host_file = root/'ros2_ws/src/quadruped_bringup/config/hosts'/f'{args.host}.yaml'
host = yaml.safe_load(host_file.read_text())
runtime = Path(host['isaac_root'])/'python.sh'
script = root/'ros2_ws/src/quadruped_sim/scripts/model_review.py'
command = [str(runtime), str(script), '--project-root', str(root),
           '--robot', args.robot, '--host', args.host]
if args.gui:
    command.append('--gui')
if args.reuse_import:
    command.append('--reuse-import')
environment = dict(os.environ)
environment.pop('PYTHONPATH', None)
os.execve(str(runtime), command, environment)
