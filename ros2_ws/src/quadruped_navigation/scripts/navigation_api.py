#!/usr/bin/env python3
"""Nav2 task interface. All planning and velocity control stay in the upstream nodes."""
import signal
from rclpy.signals import SignalHandlerOptions
import math
import threading
import time
from copy import deepcopy

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan
from nav2_msgs.action import FollowWaypoints, NavigateThroughPoses, NavigateToPose
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from quadruped_interfaces.msg import NavigationState


class NavigationAPI(Node):
    def __init__(self):
        super().__init__('navigation_api')
        self.frame = self.declare_parameter('planning_frame', 'map').value
        self.input_timeout = self.declare_parameter('input_timeout', 2.0).value
        self.plan_timeout = self.declare_parameter('plan_timeout', 30.0).value
        self.goal_timeout = self.declare_parameter('goal_timeout', 180.0).value
        self.obstacle_wait_timeout = self.declare_parameter('obstacle_wait_timeout_s', 20.0).value
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.15).value
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.busy = False
        self.cancel_event = threading.Event()
        self.closing = False
        self.odom = None
        self.odom_wall = self.cloud_wall = 0.0
        self.states = {}
        self.emergency_events = {}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.task_pub = self.create_publisher(String, 'planning/task', latched)
        self.goal_pub = self.create_publisher(PoseStamped, 'planning/goal', 10)
        self.path_pub = self.create_publisher(Path, 'initial_path', 10)
        self.create_subscription(Odometry, 'body_pose', self.on_odom, qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(PointCloud2, 'cloud', self.on_cloud, qos_profile_sensor_data, callback_group=self.group)
        for name in ('planner', 'controller'):
            self.create_subscription(NavigationState, f'planning/{name}_state',
                lambda msg, name=name: self.on_backend_state(name,msg), latched, callback_group=self.group)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial, 10, callback_group=self.group)
        self.plan_client = self.create_client(GetPlan, 'compute_path', callback_group=self.group)
        self.rviz_client = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=self.group)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_rviz_goal, 10, callback_group=self.group)
        self.servers = []
        for name, kind in [('navigate_to_pose', NavigateToPose), ('follow_waypoints', FollowWaypoints),
                           ('navigate_through_poses', NavigateThroughPoses)]:
            self.servers.append(ActionServer(self, kind, '/'+name,
                execute_callback=lambda handle, kind=kind: self.execute(handle, kind),
                goal_callback=self.accept, cancel_callback=self.cancel, callback_group=self.group))
        self.task_pub.publish(String(data=''))

    def on_backend_state(self,name,msg):
        self.states[name]=msg
        # Preserve short emergency transitions even when execute() polls later.
        if name=='planner' and msg.task_id and msg.state=='EMERGENCY_STOP':
            self.emergency_events.setdefault(msg.task_id,self.get_clock().now().nanoseconds/1e9)

    def on_odom(self, msg):
        if msg.header.frame_id == self.frame:
            self.odom = msg
            self.odom_wall = time.monotonic()

    def on_cloud(self, msg):
        if msg.header.frame_id == self.frame:
            self.cloud_wall = time.monotonic()

    def on_initial(self, _msg):
        # Localization owns this input and map->odom; stop navigation while it changes.
        if self.busy:
            self.cancel_event.set()
            self.task_pub.publish(String(data=''))

    def on_rviz_goal(self, msg):
        if self.rviz_client.server_is_ready():
            goal = NavigateToPose.Goal(); goal.pose = msg
            self.rviz_client.send_goal_async(goal)

    def health(self):
        now = time.monotonic()
        if self.odom is None or now-self.odom_wall > self.input_timeout:
            return 'localized odometry unavailable'
        if now-self.cloud_wall > self.input_timeout:
            return 'registered point cloud unavailable'
        return None

    def validate(self, pose):
        p, q = pose.pose.position, pose.pose.orientation
        if pose.header.frame_id != self.frame:
            raise ValueError(f'Goal frame must be {self.frame}')
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Goal contains non-finite values')
        if abs(q.x)>1e-5 or abs(q.y)>1e-5 or abs(q.z*q.z+q.w*q.w-1)>1e-3:
            raise ValueError('Goal must contain a normalized planar yaw quaternion')

    def accept(self, request):
        try:
            poses = [request.pose] if hasattr(request, 'pose') else list(request.poses)
            if not poses or len(poses)>1000:
                raise ValueError('Provide 1..1000 goals')
            if getattr(request, 'behavior_tree', ''):
                raise ValueError('Custom Nav2 behavior trees are not part of this interface')
            for pose in poses:
                self.validate(pose)
            if self.health():
                raise ValueError(self.health())
            with self.lock:
                if self.busy:
                    raise ValueError('A task is active; cancel it before submitting another')
                self.busy = True
                self.cancel_event.clear()
            return GoalResponse.ACCEPT
        except ValueError as error:
            self.get_logger().warn(str(error))
            return GoalResponse.REJECT

    def cancel(self, _handle):
        self.cancel_event.set()
        self.task_pub.publish(String(data=''))
        return CancelResponse.ACCEPT

    def check(self, handle):
        if self.closing or self.cancel_event.is_set() or handle.is_cancel_requested:
            raise InterruptedError('Task canceled or initial pose changed')
        if self.health():
            raise RuntimeError(self.health())

    def wait(self, predicate, timeout, handle=None):
        deadline = time.monotonic()+timeout
        while not predicate():
            if handle is not None:
                self.check(handle)
            if not rclpy.ok() or time.monotonic()>deadline:
                raise TimeoutError('Navigation backend did not respond')
            time.sleep(0.01)

    def acknowledged(self, task):
        return all(name in self.states and self.states[name].task_id == task for name in ('planner', 'controller'))

    def stop(self):
        self.task_pub.publish(String(data=''))
        try:
            self.wait(lambda: self.acknowledged('') and self.states['controller'].state=='IDLE', 3.0)
            return True
        except TimeoutError:
            self.get_logger().error('Backend stop acknowledgment timed out')
            return False

    def feedback(self, handle, kind, index, total, started, target):
        feedback = kind.Feedback()
        if kind == FollowWaypoints:
            feedback.current_waypoint = index
        else:
            feedback.current_pose = PoseStamped(header=deepcopy(self.odom.header), pose=deepcopy(self.odom.pose.pose))
            p = self.odom.pose.pose.position
            feedback.distance_remaining = math.hypot(p.x-target.pose.position.x, p.y-target.pose.position.y)
            elapsed = max(0., (self.get_clock().now().nanoseconds-started)/1e9)
            feedback.navigation_time.sec = int(elapsed)
            feedback.navigation_time.nanosec = int((elapsed%1)*1e9)
            if hasattr(feedback, 'number_of_poses_remaining'):
                feedback.number_of_poses_remaining = total-index
        handle.publish_feedback(feedback)

    def reference_path(self, handle, target):
        if not self.plan_client.wait_for_service(timeout_sec=2.):
            raise RuntimeError('Global planner unavailable')
        request = GetPlan.Request()
        request.start = PoseStamped(header=deepcopy(self.odom.header), pose=deepcopy(self.odom.pose.pose))
        request.goal = deepcopy(target)
        future = self.plan_client.call_async(request)
        self.wait(future.done, self.plan_timeout, handle)
        path = future.result().plan
        if not path.poses:
            raise RuntimeError('Global planner returned no route')
        endpoint = path.poses[-1].pose.position
        if math.hypot(endpoint.x-target.pose.position.x, endpoint.y-target.pose.position.y)>self.goal_tolerance:
            raise RuntimeError('Snapped endpoint is outside the requested goal tolerance')
        # Upstream global paths follow the support surface. SCAN gets
        # a body-height path preserving the terrain's relative height.
        height_offset = self.odom.pose.pose.position.z-path.poses[0].pose.position.z
        stamp = self.get_clock().now().to_msg()
        path.header.stamp = stamp; path.header.frame_id = self.frame
        for pose in path.poses:
            pose.header = deepcopy(path.header)
            pose.pose.position.z += height_offset
        return path

    def execute(self, handle, kind):
        result = kind.Result()
        poses = [handle.request.pose] if kind == NavigateToPose else list(handle.request.poses)
        started = self.get_clock().now().nanoseconds
        index = 0
        try:
            for index, target in enumerate(poses):
                self.check(handle)
                task = bytes(handle.goal_id.uuid).hex()+f':{index}'
                self.task_pub.publish(String(data=task))
                self.wait(lambda: self.acknowledged(task), 5.0, handle)
                path = self.reference_path(handle, target)
                stamp = path.header.stamp
                goal = deepcopy(target); goal.header.stamp = stamp
                self.check(handle)
                self.goal_pub.publish(goal)
                self.path_pub.publish(path)
                deadline = time.monotonic()+self.goal_timeout
                last_feedback = 0.
                obstacle_wait_started = None
                obstacle_wait_xy = None
                last_retry_s = -float('inf')
                while True:
                    self.check(handle)
                    controller = self.states.get('controller')
                    planner = self.states.get('planner')
                    if controller and controller.task_id == task:
                        if controller.state == 'SUCCEEDED': break
                        if controller.state == 'ODOM_STALE': raise RuntimeError('Controller odometry stale')
                    clock_s = self.get_clock().now().nanoseconds/1e9
                    if planner and planner.task_id == task:
                        if (planner.state == 'EMERGENCY_STOP' or task in self.emergency_events) and obstacle_wait_started is None:
                            obstacle_wait_started = self.emergency_events.get(task,clock_s)
                            p=self.odom.pose.pose.position
                            obstacle_wait_xy=(p.x,p.y)
                            self.get_logger().warn('Obstacle stop: keeping task active while SCAN waits for a safe trajectory')
                        if obstacle_wait_started is not None:
                            p=self.odom.pose.pose.position
                            progressed=obstacle_wait_xy is not None and math.hypot(p.x-obstacle_wait_xy[0],p.y-obstacle_wait_xy[1])>=0.1
                            if planner.state == 'EXEC_TRAJ' and progressed:
                                self.emergency_events.pop(task,None)
                                self.get_logger().info('Safe local trajectory recovered; continuing current waypoint')
                                obstacle_wait_started = None
                            elif clock_s-obstacle_wait_started > self.obstacle_wait_timeout:
                                raise TimeoutError('Obstacle did not clear within the recovery timeout')
                            elif planner.state == 'WAIT_TARGET' and clock_s-last_retry_s >= 1.0:
                                # SCAN exhausted its native retries. Request a fresh
                                # route from the measured pose; its collision checks
                                # still decide whether movement can resume.
                                last_retry_s = clock_s
                                path = self.reference_path(handle, target)
                                goal = deepcopy(target); goal.header.stamp = path.header.stamp
                                self.check(handle)
                                self.goal_pub.publish(goal); self.path_pub.publish(path)
                    if time.monotonic()>deadline: raise TimeoutError('Goal timeout')
                    if time.monotonic()-last_feedback>.2:
                        self.feedback(handle, kind, index, len(poses), started, target)
                        last_feedback = time.monotonic()
                    time.sleep(.02)
            if not self.stop(): raise RuntimeError('Stop not acknowledged')
            handle.succeed()
        except InterruptedError as error:
            self.get_logger().info(str(error))
            stopped = self.stop()
            if handle.is_cancel_requested and stopped: handle.canceled()
            else: handle.abort()
        except Exception as error:
            self.get_logger().error(str(error))
            self.stop()
            if kind == FollowWaypoints: result.missed_waypoints = [index]
            handle.abort()
        finally:
            with self.lock: self.busy = False
        return result


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NavigationAPI()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    def stop(_signal, _frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.closing = True
        node.task_pub.publish(String(data=''))
        executor.shutdown(timeout_sec=5.)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
