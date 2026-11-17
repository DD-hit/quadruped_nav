#!/usr/bin/env python3
"""ROS integration using ideal external odometry/cloud, not an Isaac or hardware acceptance."""
import argparse, json, math, os, subprocess, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, FollowWaypoints, NavigateThroughPoses
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from quadruped_interfaces.msg import Bspline, NavigationState
from quadruped_interfaces.srv import SaveNavigationMapPackage, LoadNavigationMapPackage
p = argparse.ArgumentParser()
p.add_argument('--project-root', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--avoidance-only', action='store_true')
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=False)
root = a.project_root
checks = []
children = []
handles = []

def check(name, condition, **values):
    row = dict(test=name, passed=bool(condition), **values)
    checks.append(row)
    (a.output / 'checks.json').write_text(json.dumps(checks, indent=2) + '\n')
    print(json.dumps(row), flush=True)
    if not condition:
        raise AssertionError(name)
g = np.arange(-2.95, 3.0, 0.1)
xx, yy = np.meshgrid(g, g)
floor = np.c_[xx.ravel(), yy.ravel(), np.zeros(xx.size)]
points = [floor]
for x0, y0, sx, sy in [(3.1, 0, 0.1, 6.4), (-3.1, 0, 0.1, 6.4), (0, 3.1, 6.4, 0.1), (0, -3.1, 6.4, 0.1), (1.5, -0.2 if a.avoidance_only else 1.5, 0.4, 0.4)]:
    for z in np.arange(0.1, 1.51, 0.1):
        for t in np.arange(-0.5, 0.51, 0.1):
            points.extend([np.array([[x0 - sx / 2, y0 + t * sy, z], [x0 + sx / 2, y0 + t * sy, z], [x0 + t * sx, y0 - sy / 2, z], [x0 + t * sx, y0 + sy / 2, z]])])
points = np.concatenate(points).astype(np.float32)
pcd = a.output / 'input.pcd'
with pcd.open('w') as f:
    f.write(f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n')
    np.savetxt(f, points, fmt='%.4f')

def launch(name, args):
    h = (a.output / (name + '.log')).open('w')
    handles.append(h)
    proc = subprocess.Popen(args, stdout=h, stderr=subprocess.STDOUT, start_new_session=True)
    children.append(proc)
    return proc
launch('navigation', ['ros2', 'launch', 'quadruped_navigation', 'navigation.launch.py', 'odom_topic:=/test/odom', 'cloud_topic:=/test/cloud'])
launch('map', ['ros2', 'run', 'quadruped_navigation', 'pcd_to_octomap_node', '--ros-args', '-p', f'pcd_file:={pcd}', '-p', 'resolution:=0.1', '-p', 'min_points_per_voxel:=1', '-p', 'min_cluster_voxels:=1'])
rclpy.init()
n = Node('navigation_core_integration')
odom_pub = n.create_publisher(Odometry, '/test/odom', qos_profile_sensor_data)
cloud_pub = n.create_publisher(PointCloud2, '/test/cloud', qos_profile_sensor_data)
late_pub = n.create_publisher(Bspline, '/navigation/planning/bspline', 10)
tf = TransformBroadcaster(n)
static = StaticTransformBroadcaster(n)
angle = 0.35
rot = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
translation = np.array([0.3, -0.2])
state = np.array([0.0, 0.0, 0.0])
command = np.zeros(3)
last = time.monotonic()
cloud_last = 0.0
publish_inputs = True
odom_out = None
sensor_out = None
cloud_out = None
last_spline = None
command_rows = []
arrivals = []
spline_count = 0
history = []
static_transforms = []
for parent, child, xyz, yaw in [('map', 'test_odom', [*translation, 0.0], angle), ('base_link', 'lidar_link', [0.1, 0.0, 0.2], 0.0)]:
    m = TransformStamped()
    m.header.frame_id = parent
    m.child_frame_id = child
    m.header.stamp = n.get_clock().now().to_msg()
    m.transform.translation.x = float(xyz[0])
    m.transform.translation.y = float(xyz[1])
    m.transform.translation.z = float(xyz[2])
    m.transform.rotation.z = math.sin(yaw / 2)
    m.transform.rotation.w = math.cos(yaw / 2)
    static_transforms.append(m)
static.sendTransform(static_transforms)
input_points = points.copy()
input_points[:, :2] = (points[:, :2] - translation) @ rot

def tick():
    global last, cloud_last
    now = time.monotonic()
    dt = min(now - last, 0.05)
    last = now
    c, s = (math.cos(state[2]), math.sin(state[2]))
    state[:2] += np.array([c * command[0] - s * command[1], s * command[0] + c * command[1]]) * dt
    state[2] += command[2] * dt
    history.append(map_pose())
    if not publish_inputs:
        return
    stamp = n.get_clock().now().to_msg()
    qz, qw = (math.sin(state[2] / 2), math.cos(state[2] / 2))
    m = TransformStamped()
    m.header = Header(frame_id='test_odom', stamp=stamp)
    m.child_frame_id = 'base_link'
    m.transform.translation.x = float(state[0])
    m.transform.translation.y = float(state[1])
    m.transform.translation.z = 0.3
    m.transform.rotation.z = qz
    m.transform.rotation.w = qw
    tf.sendTransform(m)
    od = Odometry(header=m.header, child_frame_id='base_link')
    od.pose.pose.position.x = float(state[0])
    od.pose.pose.position.y = float(state[1])
    od.pose.pose.position.z = 0.3
    od.pose.pose.orientation.z = qz
    od.pose.pose.orientation.w = qw
    od.twist.twist.linear.x = float(command[0])
    od.twist.twist.linear.y = float(command[1])
    od.twist.twist.angular.z = float(command[2])
    odom_pub.publish(od)
    if now - cloud_last > 0.1:
        cloud_pub.publish(create_cloud_xyz32(m.header, input_points.tolist()))
        cloud_last = now
n.create_timer(0.02, tick)

def cmd(m):
    global command
    command = np.array([m.linear.x, m.linear.y, m.angular.z])
    command_rows.append([time.monotonic(), *command.tolist()])

def odom(m):
    global odom_out
    odom_out = m

def sensor(m):
    global sensor_out
    sensor_out = m

def cloud(m):
    global cloud_out
    cloud_out = m

def spline(m):
    global last_spline, spline_count
    last_spline = m
    spline_count += 1

def status(m):
    if m.state == 'SUCCEEDED' and (not arrivals or arrivals[-1]['task'] != m.task_id):
        arrivals.append(dict(task=m.task_id, pose=map_pose()))
n.create_subscription(Twist, '/cmd_vel', cmd, 100)
n.create_subscription(Odometry, '/navigation/body_pose', odom, qos_profile_sensor_data)
n.create_subscription(Odometry, '/navigation/sensor_pose', sensor, qos_profile_sensor_data)
n.create_subscription(PointCloud2, '/navigation/cloud', cloud, qos_profile_sensor_data)
n.create_subscription(Bspline, '/navigation/planning/bspline', spline, 10)
n.create_subscription(NavigationState, '/navigation/planning/controller_state', status, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

def map_pose():
    return [*(rot @ state[:2] + translation).tolist(), float(state[2] + angle)]

def wait(pred, seconds=90):
    until = time.monotonic() + seconds
    while not pred():
        rclpy.spin_once(n, timeout_sec=0.01)
        if any((v.poll() is not None for v in children)):
            raise RuntimeError('A core process exited')
        if time.monotonic() > until:
            raise TimeoutError(dict(pose=map_pose(), command=command.tolist(), splines=spline_count))

def pause(seconds):
    until = time.monotonic() + seconds
    wait(lambda: time.monotonic() >= until, seconds + 3.0)

def pose(x, y, yaw):
    p = PoseStamped(header=Header(frame_id='map', stamp=n.get_clock().now().to_msg()))
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = 0.3
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p
clients = {k: ActionClient(n, k, name) for k, name in [(NavigateToPose, '/navigate_to_pose'), (FollowWaypoints, '/follow_waypoints'), (NavigateThroughPoses, '/navigate_through_poses')]}

def send(kind, targets):
    wait(clients[kind].server_is_ready, 20)
    g = kind.Goal()
    if kind == NavigateToPose:
        g.pose = targets[0]
    else:
        g.poses = targets
    f = clients[kind].send_goal_async(g)
    wait(f.done, 10)
    h = f.result()
    check('action_accepted', h.accepted, kind=kind.__name__)
    return (h, h.get_result_async())
try:
    if a.avoidance_only:
        wait(lambda: odom_out is not None and sensor_out is not None and (cloud_out is not None), 30)
        pause(3.0)
        h, f = send(NavigateToPose, [pose(2.3, -0.2, 0.0)])
        wait(f.done, 90)
        check('blocked_straight_route_reached', f.result().status == 4, pose=map_pose())
        track = np.array(history)
        yaw = track[:, 2]
        box = np.array([1.5, -0.2])
        minimum = float('inf')
        for direction in [-1.0, 1.0]:
            centers = track[:, :2] + direction * 0.18 * np.c_[np.cos(yaw), np.sin(yaw)]
            distance = np.linalg.norm(np.maximum(np.abs(centers - box) - 0.2, 0.0), axis=1) - 0.25
            minimum = min(minimum, float(distance.min()))
        check('native_double_cylinder_clearance', minimum > 0.0, minimum_gap_m=minimum)
        check('path_goes_around_box', float(np.max(np.abs(track[:, 1] + 0.2))) > 0.45, maximum_lateral_detour_m=float(np.max(np.abs(track[:, 1] + 0.2))))
        check('native_replanning_runs', spline_count > 1, trajectories=spline_count)
    else:
        wait(lambda: odom_out is not None and sensor_out is not None and (cloud_out is not None), 30)
        check('odom_coordinate_transform', abs(odom_out.pose.pose.position.x - 0.3) < 0.001 and abs(odom_out.pose.pose.position.y + 0.2) < 0.001)
        expected = translation + rot @ np.array([0.1, 0.0])
        check('lidar_origin_not_base_origin', np.linalg.norm([sensor_out.pose.pose.position.x - expected[0], sensor_out.pose.pose.position.y - expected[1]]) < 0.001 and abs(sensor_out.pose.pose.position.z - 0.5) < 0.001)
        output = np.asarray(list(read_points(cloud_out, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
        check('cloud_coordinate_transform', abs(float(output['x'][0]) - float(points[0, 0])) < 0.0001)
        pause(3.0)
        publishers = n.get_publishers_info_by_topic('/cmd_vel')
        check('one_native_velocity_publisher', len(publishers) == 1 and publishers[0].node_name == 'trajectory_controller', publishers=[v.node_name for v in publishers])
        h, f = send(NavigateToPose, [pose(1.0, -0.2, 0.4)])
        wait(f.done, 90)
        check('single_goal_and_heading', f.result().status == 4 and np.linalg.norm(np.array(map_pose()[:2]) - [1.0, -0.2]) < 0.17 and (abs(math.remainder(map_pose()[2] - 0.4, 2 * math.pi)) < 0.17), pose=map_pose())
        h, f = send(NavigateToPose, [pose(-1.0, -0.2, 0.0)])
        wait(lambda: np.linalg.norm(command) > 0.04, 20)
        cancel = h.cancel_goal_async()
        wait(cancel.done, 5)
        wait(f.done, 8)
        check('cancel_action', f.result().status == 5)
        saved = last_spline
        pause(0.2)
        check('cancel_zero_velocity', np.linalg.norm(command) < 1e-06)
        if saved is not None:
            late_pub.publish(saved)
        pause(0.4)
        check('late_trajectory_does_not_restart', np.linalg.norm(command) < 1e-06)
        goals = [pose(0.3, 0.6, -0.3), pose(-0.5, 0.4, 0.0), pose(0.3, -0.2, 0.2)]
        before = len(arrivals)
        h, f = send(FollowWaypoints, goals)
        wait(f.done, 150)
        check('waypoints_all_reached', f.result().status == 4 and (not f.result().result.missed_waypoints), arrivals=arrivals[before:])
        recent = arrivals[before:]
        errors = [math.hypot(row['pose'][0] - goal.pose.position.x, row['pose'][1] - goal.pose.position.y) for row, goal in zip(recent, goals)]
        check('waypoint_actual_goal_errors', len(errors) == 3 and max(errors) < 0.17, position_errors_m=errors)
        h, f = send(NavigateThroughPoses, [pose(0.6, -0.1, 0.0)])
        wait(f.done, 50)
        check('navigate_through_poses', f.result().status == 4)
        h, f = send(NavigateToPose, [pose(1.8, -0.2, 0.0)])
        wait(lambda: np.linalg.norm(command) > 0.04, 20)
        publish_inputs = False
        wait(f.done, 8)
        check('missing_inputs_abort_and_stop', f.result().status == 6 and np.linalg.norm(command) < 1e-06)
        publish_inputs = True
        pause(0.4)
        check('no_resume_without_new_goal', np.linalg.norm(command) < 1e-06)
        initial_pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        h, f = send(NavigateToPose, [pose(1.5, -0.2, 0.0)])
        wait(lambda: np.linalg.norm(command) > 0.04, 20)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.header.stamp = n.get_clock().now().to_msg()
        initial.pose.pose.orientation.w = 1.0
        initial_pub.publish(initial)
        wait(f.done, 8)
        check('initial_pose_change_stops_active_task', f.result().status == 6 and np.linalg.norm(command) < 1e-06)
        save = n.create_client(SaveNavigationMapPackage, '/navigation/map_package_manager/save_package')
        wait(lambda: save.service_is_ready(), 10)
        request = SaveNavigationMapPackage.Request()
        request.package_path = str(a.output / 'saved_map')
        future = save.call_async(request)
        wait(future.done, 40)
        check('upstream_map_save', future.result().success, message=future.result().message)
        load = n.create_client(LoadNavigationMapPackage, '/navigation/map_package_manager/load_package')
        wait(lambda: load.service_is_ready(), 10)
        request = LoadNavigationMapPackage.Request()
        request.package_path = str(a.output / 'saved_map')
        future = load.call_async(request)
        wait(future.done, 20)
        check('upstream_map_load', future.result().success, message=future.result().message)
        check('native_trajectories_observed', spline_count > 1, count=spline_count)
    (a.output / 'verification.json').write_text(json.dumps(dict(result='PASS', scope='ideal external ROS inputs; no Isaac or hardware validation', avoidance_only=a.avoidance_only, checks=checks, arrivals=arrivals), indent=2) + '\n')
    print('NAVIGATION_CORE_PASS', flush=True)
finally:
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    import signal
    for proc in children:
        if proc.poll() is None:
            if proc == children[0]:
                os.kill(proc.pid, signal.SIGINT)
            else:
                os.killpg(proc.pid, signal.SIGINT)
    for proc in children:
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)
    for h in handles:
        h.close()
    (a.output / 'process_exits.json').write_text(json.dumps([v.returncode for v in children]))
