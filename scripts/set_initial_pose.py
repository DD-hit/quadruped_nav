#!/usr/bin/env python3
"""Queue a timestamped mechanical-base pose in map; this does not prove a match."""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import yaml

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--xyz', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'))
parser.add_argument('--rpy', type=float, nargs=3, default=(0., 0., 0.), metavar=('ROLL', 'PITCH', 'YAW'), help='Radians')
parser.add_argument('--mode', choices=('sim', 'real'), default='sim')
parser.add_argument('--domain-id', type=int)
args = parser.parse_args()
configs = root / 'ros2_ws/src/quadruped_bringup/config'
host = yaml.safe_load((configs / 'hosts/server.yaml').read_text())
common = yaml.safe_load((configs / 'common.yaml').read_text())
domain = args.domain_id if args.domain_id is not None else host['ros_domain_id']
if not 0 <= domain <= 232:
    parser.error('ROS domain must lie in 0..232')
os.environ['ROS_DOMAIN_ID'] = str(domain)
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
sys.path.insert(0, str(root / 'ros2_ws/src/quadruped_localization/scripts'))
from lio_config import rotation
from base_geometry import matrix_quaternion
from map_registration import rigid_transform
import numpy as np
T = np.eye(4)
T[:3, :3], T[:3, 3] = rotation(args.rpy), args.xyz
rigid_transform(T)
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
rclpy.init(args=[])
node = Node('quadruped_initial_pose_tool', parameter_overrides=[Parameter('use_sim_time', value=args.mode == 'sim')])
try:
    publisher = node.create_publisher(PoseWithCovarianceStamped, common['interfaces']['initial_pose']['topic'], 10)
    acknowledgments = []
    def receive(message):
        try:
            acknowledgments.append(json.loads(message.data))
        except json.JSONDecodeError:
            pass
    node.create_subscription(String, common['interfaces']['localization_status']['topic'], receive,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic() + 5.
    while (publisher.get_subscription_count() == 0 or node.get_clock().now().nanoseconds <= 0) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    if publisher.get_subscription_count() == 0 or node.get_clock().now().nanoseconds <= 0:
        raise RuntimeError('No localization subscriber or valid clock')
    message = PoseWithCovarianceStamped()
    message.header.frame_id = common['frames']['map']
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = args.xyz
    message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w = map(float, matrix_quaternion(T[:3, :3]))
    stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
    publisher.publish(message)
    deadline = time.monotonic() + 3.
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        if any(x.get('last_initial_pose_ns') == stamp for x in acknowledgments):
            print(json.dumps(dict(initial_pose_queued=True, localization_converged=False,
                                  stamp_ns=stamp, reference='mechanical_base_in_map')))
            break
    else:
        raise RuntimeError('Initial pose was not acknowledged by the localization node')
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
