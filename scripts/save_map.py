#!/usr/bin/env python3
"""Commit the currently recording localization map via the mapping service."""
import argparse,json,os
from pathlib import Path
import time,yaml
root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__);parser.parse_args()
host=yaml.safe_load((root/'ros2_ws/src/quadruped_bringup/config/hosts/server.yaml').read_text());os.environ['ROS_DOMAIN_ID']=str(host['ros_domain_id'])
config=yaml.safe_load((root/'ros2_ws/src/quadruped_localization/config/map_export.yaml').read_text())
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
rclpy.init(args=[]);node=Node('quadruped_save_map_request');code=1
try:
    client=node.create_client(Trigger,config['save_service'])
    if not client.wait_for_service(timeout_sec=10):raise RuntimeError('Map recorder save service unavailable')
    future=client.call_async(Trigger.Request());deadline=time.monotonic()+60
    while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    if not future.done():raise TimeoutError('Map save timed out; inspect recorder status before retrying')
    result=future.result();print(json.dumps(dict(success=result.success,message=result.message)),flush=True);code=0 if result.success else 2
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
raise SystemExit(code)
