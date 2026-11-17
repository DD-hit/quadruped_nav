#!/usr/bin/env python3
"""Check a native Livox ROS 2 recording offline, without starting any node."""
import argparse,json
from pathlib import Path
import sys,yaml
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'ros2_ws/src/quadruped_localization/scripts'))
from livox_bag_check import check
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--bag',required=True,type=Path)
parser.add_argument('--profile',type=Path,default=root/'ros2_ws/src/quadruped_localization/config/livox_recording.yaml')
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
try:report=check(args.bag,yaml.safe_load(args.profile.read_text()))
except Exception as error:report=dict(result='FAIL',error=repr(error),real_hardware_validation_complete=False)
args.output.parent.mkdir(parents=True,exist_ok=True)
with args.output.open('x') as stream:stream.write(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
raise SystemExit(0 if report['result']=='PASS' else 1)
