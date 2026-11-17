#!/usr/bin/env python3
"""Small CLI for standard initial-pose topics and Nav2 actions."""
import argparse, math, time
from pathlib import Path
import yaml
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--use-sim-time', action='store_true')
p.add_argument('--frame', default='map')
sub = p.add_subparsers(dest='command', required=True)
for name in ['initial', 'goal']:
    sub.add_parser(name).add_argument('pose', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
w = sub.add_parser('waypoints')
w.add_argument('file', type=Path)
w.add_argument('--through', action='store_true')
sub.add_parser('cancel')
a = p.parse_args()
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints, NavigateThroughPoses
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
n = Node('navigation_cli', parameter_overrides=[Parameter('use_sim_time', value=a.use_sim_time)])
handle = None

def wait(pred, timeout=30):
    end = time.monotonic() + timeout
    while not pred():
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.monotonic() > end:
            raise TimeoutError('ROS interface did not respond')

def pose(values):
    if len(values) != 3 or not all((math.isfinite(v) for v in values)):
        raise ValueError('Expected finite x, y, yaw')
    m = PoseStamped()
    m.header.frame_id = a.frame
    m.header.stamp = n.get_clock().now().to_msg()
    m.pose.position.x = float(values[0])
    m.pose.position.y = float(values[1])
    m.pose.orientation.z = math.sin(values[2] / 2)
    m.pose.orientation.w = math.cos(values[2] / 2)
    return m
try:
    if a.command == 'cancel':
        count = 0
        for name in ['navigate_to_pose', 'follow_waypoints', 'navigate_through_poses']:
            client = n.create_client(CancelGoal, '/' + name + '/_action/cancel_goal')
            if client.wait_for_service(timeout_sec=2.0):
                f = client.call_async(CancelGoal.Request())
                wait(f.done, 5)
                count += len(f.result().goals_canceling)
        print(f'Cancellation requested for {count} active task(s).')
    elif a.command == 'initial':
        pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        wait(lambda: pub.get_subscription_count() > 0 and n.get_clock().now().nanoseconds > 0)
        value = pose(a.pose)
        m = PoseWithCovarianceStamped()
        m.header = value.header
        m.pose.pose = value.pose
        m.pose.covariance[0] = m.pose.covariance[7] = 0.25
        m.pose.covariance[35] = 0.0685
        pub.publish(m)
        until = time.monotonic() + 0.3
        wait(lambda: time.monotonic() >= until, 2)
        print('Initial pose published. Localization confirmation belongs to the external localization node.')
    else:
        kind = NavigateToPose if a.command == 'goal' else NavigateThroughPoses if a.through else FollowWaypoints
        name = 'navigate_to_pose' if kind == NavigateToPose else 'navigate_through_poses' if kind == NavigateThroughPoses else 'follow_waypoints'
        client = ActionClient(n, kind, '/' + name)
        wait(client.server_is_ready)
        if a.use_sim_time:
            wait(lambda: n.get_clock().now().nanoseconds > 0)
        goal = kind.Goal()
        if kind == NavigateToPose:
            goal.pose = pose(a.pose)
        else:
            values = yaml.safe_load(a.file.read_text())
            values = values['waypoints'] if isinstance(values, dict) else values
            goal.poses = [pose(v) for v in values]
        f = client.send_goal_async(goal)
        wait(f.done)
        handle = f.result()
        if not handle.accepted:
            raise RuntimeError('Goal rejected; inspect navigation_api logs')
        f = handle.get_result_async()
        wait(f.done, 3600)
        result = f.result()
        print(f'Action status: {result.status} (4=succeeded, 5=canceled, 6=aborted)')
        if hasattr(result.result, 'missed_waypoints'):
            print('Missed waypoints:', list(result.result.missed_waypoints))
        if result.status != 4:
            raise SystemExit(1)
except KeyboardInterrupt:
    if handle is not None and handle.accepted:
        f = handle.cancel_goal_async()
        wait(f.done, 5)
finally:
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
