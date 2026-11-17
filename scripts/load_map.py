#!/usr/bin/env python3
"""Load a committed PCD map bundle for display, without starting localization."""
import argparse,os
from pathlib import Path
import sys,yaml
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'ros2_ws/src/quadruped_localization/scripts'))
from map_bundle import identifier
parser=argparse.ArgumentParser(description=__doc__);group=parser.add_mutually_exclusive_group(required=True)
group.add_argument('--map-id');group.add_argument('--map-directory',type=Path)
args=parser.parse_args()
host=yaml.safe_load((root/'ros2_ws/src/quadruped_bringup/config/hosts/server.yaml').read_text())
os.environ['ROS_DOMAIN_ID']=str(host['ros_domain_id']);os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
directory=args.map_directory if args.map_directory else Path(host['data_root'])/'maps'/identifier(args.map_id)
os.execv('/usr/bin/python3',['/usr/bin/python3',str(root/'ros2_ws/src/quadruped_localization/scripts/map_loader.py'),
    '--project-root',str(root),'--map-directory',str(directory)])
