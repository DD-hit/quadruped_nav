#!/usr/bin/env python3
"""Run only the sensor adapter, not FAST-LIO. Requires the system ROS overlay."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import sys
from zoneinfo import ZoneInfo
import yaml
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'ros2_ws/src/quadruped_localization/scripts'))
from lio_config import prepare
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--robot',default='d1_edu')
parser.add_argument('--output-dir',type=Path)
parser.add_argument('--config-only',action='store_true')
args=parser.parse_args()
host=yaml.safe_load((root/'ros2_ws/src/quadruped_bringup/config/hosts/server.yaml').read_text())
os.environ['ROS_DOMAIN_ID']=str(host['ros_domain_id'])
os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
output=args.output_dir or Path(host['data_root'])/'runs'/('lio_inputs_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
if not args.config_only:
    try:import rclpy,livox_ros_driver2.msg
    except ImportError:
        parser.error('Source /opt/ros/humble/setup.bash and ros2_ws/install/setup.bash in system Python first')
adapter,fastlio=prepare(root,args.robot,output)
print('Generated input configs:',output,flush=True)
if not args.config_only:
    command=['/usr/bin/python3',str(root/'ros2_ws/src/quadruped_localization/scripts/lio_input_adapter.py'),
        '--config',str(adapter),'--ros-args','-p','use_sim_time:=true']
    os.execv(command[0],command)
