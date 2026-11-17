#!/usr/bin/env python3
"""Subscribe to Isaac's live D1 gait sensor topics for a short smoke test."""
import json
import os
import time

os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from rosgraph_msgs.msg import Clock


def main():
    rclpy.init()
    node = Node('d1_live_sensor_topic_check')
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    counts = {'clock': 0, 'imu': 0, 'cloud': 0}
    last = {}

    def clock(message):
        counts['clock'] += 1
        last['clock_ns'] = message.clock.sec * 10**9 + message.clock.nanosec

    def imu(message):
        counts['imu'] += 1
        last['imu_frame'] = message.header.frame_id
        last['imu_stamp_ns'] = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

    def cloud(message):
        counts['cloud'] += 1
        last['cloud_frame'] = message.header.frame_id
        last['cloud_stamp_ns'] = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        last['cloud_points'] = message.width

    node.create_subscription(Clock, '/clock', clock, qos)
    node.create_subscription(Imu, '/imu/data', imu, qos)
    node.create_subscription(PointCloud2, '/lidar/points', cloud, qos)
    deadline = time.monotonic() + 80
    while time.monotonic() < deadline and not (counts['imu'] >= 20 and counts['cloud'] >= 1):
        rclpy.spin_once(node, timeout_sec=.1)
    result = {'result': 'PASS' if counts['imu'] >= 20 and counts['cloud'] >= 1 and
              last.get('imu_frame') == 'lidar_imu_link' and last.get('cloud_frame') == 'lidar_link' and
              last.get('cloud_points', 0) > 1000 else 'FAIL', 'counts': counts, 'last': last}
    print(json.dumps(result), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    if result['result'] != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
