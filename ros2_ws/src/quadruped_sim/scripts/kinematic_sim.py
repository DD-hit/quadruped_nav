#!/usr/bin/env python3
"""Receive body-frame Twist and move an Isaac model with planar kinematics."""
import argparse
from pathlib import Path
import re

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--project-root', required=True, type=Path)
parser.add_argument('--robot', default='d1_edu')
parser.add_argument('--host', default='server')
parser.add_argument('--gui', action='store_true')
parser.add_argument('--lidar', action='store_true')
parser.add_argument('--imu', action='store_true')
parser.add_argument('--lidar-samples-per-scan', type=int)
parser.add_argument('--output-dir', type=Path)
parser.add_argument('--duration-wall', type=float, default=0.0)
args = parser.parse_args()
if not all(re.fullmatch(r'[a-z][a-z0-9_]*', v) for v in (args.robot, args.host)):
    parser.error('Invalid profile identifier')

if args.lidar_samples_per_scan is not None and (not args.lidar or args.lidar_samples_per_scan < 2):
    parser.error('--lidar-samples-per-scan requires --lidar and a value of at least 2')

from isaacsim import SimulationApp
app = SimulationApp({'headless': not args.gui, 'active_gpu': 0, 'physics_gpu': 0,
                     'multi_gpu': False, 'renderer': 'RaytracedLighting',
                     'width': 1280, 'height': 960})

import json
import math
import os
import signal
import time
import traceback
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
import yaml
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, Usd, UsdGeom
from planar_motion import PlanarMotion

node = None
world = None
trace = None
ros_started = False
shutdown_requested = False


def request_shutdown(signum, frame):
    global shutdown_requested
    shutdown_requested = True


try:
    root = args.project_root.resolve()
    config_root = root/'ros2_ws/src/quadruped_bringup/config'
    robot = yaml.safe_load((config_root/'robots'/f'{args.robot}.yaml').read_text())
    host = yaml.safe_load((config_root/'hosts'/f'{args.host}.yaml').read_text())
    common = yaml.safe_load((config_root/'common.yaml').read_text())
    config = yaml.safe_load((root/'ros2_ws/src/quadruped_sim/config/kinematic_motion.yaml').read_text())
    scene = Path(host['data_root'])/'assets'/args.robot/'model_review/model_review.usda'
    if not scene.is_file():
        raise FileNotFoundError(f'Run model preview first: {scene}')
    output = args.output_dir or Path(host['data_root'])/'runs'/('motion_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True, exist_ok=True)
    (output/'ready.json').unlink(missing_ok=True)
    # Author transient overrides in an anonymous layer, leaving preview assets intact.
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    stage.GetRootLayer().subLayerPaths.append(str(scene))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.Load()
    imu_config = None
    if args.imu:
        from ideal_imu import IdealImu
        imu_config = yaml.safe_load((root/'ros2_ws/src/quadruped_sim/config'/robot['sensor']['simulation_imu_profile']).read_text())
        for key in ('sample_hz', 'render_hz', 'gravity_mps2'):
            if not math.isfinite(float(imu_config[key])) or float(imu_config[key]) <= 0:
                raise ValueError(f'Invalid IMU setting {key}')
        if imu_config['sample_hz'] < imu_config['render_hz']:
            raise ValueError('IMU sample rate must be at least the render rate')
    lidar_config = None
    if args.lidar:
        from geometric_lidar import create_validation_room, GeometricLidar
        lidar_profile = robot['sensor']['simulation_lidar_profile']
        lidar_config = yaml.safe_load((root/'ros2_ws/src/quadruped_sim/config'/lidar_profile).read_text())
        if args.lidar_samples_per_scan is not None:
            lidar_config['samples_per_scan'] = args.lidar_samples_per_scan
        create_validation_room(stage,lidar_config)
    # A visual grid makes translation visible even when the GUI camera follows.
    grid = UsdGeom.BasisCurves.Define(stage, '/World/MotionGrid')
    grid.CreateTypeAttr('linear')
    points = []
    for index in range(-8, 9):
        coordinate = index*0.25
        points += [Gf.Vec3f(coordinate,-2,0.002), Gf.Vec3f(coordinate,2,0.002),
                   Gf.Vec3f(-2,coordinate,0.002), Gf.Vec3f(2,coordinate,0.002)]
    grid.CreateCurveVertexCountsAttr([2]*(len(points)//2))
    grid.CreatePointsAttr(points)
    grid.CreateWidthsAttr([0.002])
    grid.SetWidthsInterpolation('constant')
    grid.CreateDisplayColorAttr([(0.35,0.42,0.5)])
    body = stage.GetPrimAtPath('/World/Robot')
    base = stage.GetPrimAtPath('/World/Robot/'+robot['model']['target_root_frame'])
    assert body.IsValid() and base.IsValid()
    initial = np.array(UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
    initial_height = float(initial[2, 3])
    (output/'run_config.yaml').write_text(yaml.safe_dump(
        dict(schema_version=1, robot=robot, host=host, interfaces=common['interfaces'], motion=config,
             lidar=lidar_config, imu=imu_config),
        sort_keys=False, allow_unicode=True))
    body_op = UsdGeom.Xformable(body).MakeMatrixXform()
    frame_op = UsdGeom.Xform.Define(stage, '/World/Frames').MakeMatrixXform()
    enable_extension('isaacsim.ros2.bridge')
    for _ in range(3):
        app.update()
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    # Isaac's default SIGINT handler unloads plugins immediately. Defer that
    # until the loop has stopped commands and persisted its final state.
    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    ros_started = True
    frequency = float(imu_config['sample_hz'] if args.imu else config['simulation_step_hz'])
    if not math.isfinite(frequency) or frequency <= 0:
        raise ValueError('simulation_step_hz must be positive')
    render_hz = float(imu_config['render_hz']) if args.imu else frequency
    world = World(physics_dt=1.0/frequency, rendering_dt=1.0/render_hz, stage_units_in_meters=1.0)
    if args.imu:
        world.get_physics_context().set_gravity(-float(imu_config['gravity_mps2']))
    if args.lidar:
        world.get_physics_context().set_enable_scene_query_support(True)
    world.reset()
    motion = PlanarMotion(config)

    def apply_pose():
        matrix = Gf.Matrix4d(1)
        matrix.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(motion.yaw)))
        matrix.SetTranslateOnly(Gf.Vec3d(motion.x, motion.y, initial_height))
        body_op.Set(matrix)
        frame_matrix = Gf.Matrix4d(matrix)
        frame_matrix.SetTranslateOnly(Gf.Vec3d(motion.x, motion.y, 0))
        frame_op.Set(frame_matrix)

    class MotionNode(Node):
        def __init__(self):
            super().__init__('quadruped_kinematic_sim')
            self.epoch = 0
            self.resetting = False
            self.time_origin = world.current_time
            self.last_clock = 0.0
            self.lidar = None
            self.imu = None
            self.clock = self.create_publisher(Clock, common['interfaces']['clock']['topic'],
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.truth = self.create_publisher(Odometry, common['interfaces']['ground_truth']['topic'], 10)
            self.state = self.create_publisher(String, config['state_topic'], 10)
            self.command_subscription = None
            self.renew_command_subscription()
            self.create_service(Trigger, config['services']['pause'], self.on_pause)
            self.create_service(Trigger, config['services']['resume'], self.on_resume)
            self.create_service(Trigger, config['services']['reset'], self.on_reset)
            self.obstacles = None
            if args.lidar:
                from runtime_obstacles import RuntimeObstacles
                obstacle_config = yaml.safe_load((root/'ros2_ws/src/quadruped_sim/config/runtime_obstacles.yaml').read_text())
                self.obstacles = RuntimeObstacles(stage, self, obstacle_config, output)

        def renew_command_subscription(self):
            if self.command_subscription is not None:
                self.destroy_subscription(self.command_subscription)
            self.command_subscription = self.create_subscription(
                Twist, common['interfaces']['cmd_vel']['topic'], self.on_command, 1)

        def on_command(self, message):
            all_values = (message.linear.x, message.linear.y, message.linear.z,
                          message.angular.x, message.angular.y, message.angular.z)
            if not all(math.isfinite(v) for v in all_values) or any(abs(v) > 1e-12 for v in all_values[2:5]):
                motion.rejections += 1
                motion.clear_command()
                self.get_logger().warn('Rejected non-finite or non-planar cmd_vel; stopped.')
                return
            motion.receive((message.linear.x, message.linear.y, message.angular.z), time.monotonic())

        def advance(self, dt):
            if not self.resetting:
                motion.advance(float(dt), time.monotonic())
                apply_pose()

        def on_pause(self, request, response):
            motion.set_paused(True)
            world.pause()
            if self.imu is not None:
                self.imu.reset_derivative()
            self.publish()
            response.success, response.message = True, 'Paused; old commands cleared.'
            return response

        def on_resume(self, request, response):
            motion.set_paused(False)
            self.renew_command_subscription()
            world.play()
            if self.imu is not None:
                self.imu.reset_derivative()
            self.publish()
            response.success, response.message = True, 'Resumed; a fresh command is required.'
            return response

        def reset(self, paused=None):
            if paused is None:
                paused = motion.paused
            self.resetting = True
            motion.reset(paused=paused)
            world.reset()
            self.time_origin = world.current_time
            if paused:
                world.pause()
            apply_pose()
            self.renew_command_subscription()
            self.epoch += 1
            if self.obstacles is not None:
                self.obstacles.clear(None, Trigger.Response())
            if self.lidar is not None:
                self.lidar.reset()
            if self.imu is not None:
                self.imu.reset()
            self.resetting = False
            self.publish()

        def on_reset(self, request, response):
            self.reset()
            response.success, response.message = True, f'Reset to initial pose/time; epoch={self.epoch}.'
            return response

        def publish(self):
            sim_time = max(0.0, float(world.current_time-self.time_origin))
            nanoseconds = round(sim_time*1e9)
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(nanoseconds, 1_000_000_000)
            self.clock.publish(clock)
            transform = np.array(UsdGeom.Xformable(base).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
            x, y, z = (float(v) for v in transform[:3, 3])
            yaw = math.atan2(transform[1, 0], transform[0, 0])
            odometry = Odometry()
            odometry.header.stamp = clock.clock
            odometry.header.frame_id = config['ground_truth_frames']['parent']
            odometry.child_frame_id = config['ground_truth_frames']['child']
            odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z = x, y, z
            odometry.pose.pose.orientation.z = math.sin(yaw/2)
            odometry.pose.pose.orientation.w = math.cos(yaw/2)
            odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.angular.z = motion.velocity
            self.truth.publish(odometry)
            state = dict(epoch=self.epoch, sim_time_s=sim_time, paused=motion.paused,
                         pose=[x, y, yaw], velocity=list(motion.velocity), target=list(motion.target),
                         stop_reason=motion.stop_reason, rejected_commands=motion.rejections,
                         physics_time_s=float(world.current_time), backend='planar_kinematic')
            self.state.publish(String(data=json.dumps(state)))
            trace.write(json.dumps(state)+'\n')
            self.last_clock = sim_time

    trace = (output/'states.jsonl').open('w', buffering=1)
    node = MotionNode()
    apply_pose()
    if args.lidar:
        node.lidar = GeometricLidar(stage,node,lidar_config,robot,common,output)
    if args.imu:
        node.imu = IdealImu(stage,node,imu_config,robot,common,output)
    world.add_physics_callback('quadruped_planar_motion', node.advance)
    node.publish()
    if args.gui:
        set_camera_view(eye=np.array([1.5, -1.8, initial_height+1.1]),
                        target=np.array([0.0, 0.0, initial_height+0.04]), camera_prim_path='/OmniverseKit_Persp')
    readiness = dict(robot=args.robot, ros_domain_id=int(os.environ['ROS_DOMAIN_ID']),
                     rclpy_path=rclpy.__file__, output_dir=str(output), sensors_enabled=args.lidar or args.imu,
                     lidar_enabled=args.lidar, imu_enabled=args.imu, physics_step_hz=frequency, render_hz=render_hz,
                     hardware_enabled=False, ground_truth_tf_published=False)
    (output/'ready.json').write_text(json.dumps(readiness, indent=2)+'\n')
    print('KINEMATIC_READY', json.dumps(readiness), flush=True)
    next_render_time = 0.0
    started = time.monotonic()
    while app.is_running() and rclpy.ok() and not shutdown_requested:
        frame_start = time.monotonic()
        if args.duration_wall > 0 and frame_start-started >= args.duration_wall:
            break
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.0)
        if world.is_stopped():
            node.reset(paused=True)
        elif world.is_playing() and motion.paused:
            motion.set_paused(False)
            if node.imu is not None:
                node.imu.reset_derivative()
        elif not world.is_playing() and not motion.paused:
            motion.set_paused(True)
            if node.imu is not None:
                node.imu.reset_derivative()
        if motion.paused:
            app.update()
        else:
            previous_time = max(0.0,world.current_time-node.time_origin)
            previous_pose = np.array(UsdGeom.Xformable(base).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
            world.step(render=not args.imu)
            current_time = max(0.0,world.current_time-node.time_origin)
            current_pose = np.array(UsdGeom.Xformable(base).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
            if node.imu is not None:
                node.imu.sample(current_time,current_time-previous_time,current_pose,motion.velocity)
            if node.lidar is not None:
                node.lidar.sample(previous_time,current_time,previous_pose,current_pose)
            if args.imu and (current_time >= next_render_time-1e-9 or current_time < next_render_time-1/render_hz-1e-9):
                world.render()
                next_render_time = current_time+1/render_hz
        node.publish()
        if args.gui and config['follow_camera']:
            set_camera_view(eye=np.array([motion.x+1.5, motion.y-1.8, initial_height+1.1]),
                            target=np.array([motion.x, motion.y, initial_height+0.04]), camera_prim_path='/OmniverseKit_Persp')
        time.sleep(max(0.0, 1.0/frequency-(time.monotonic()-frame_start)))
except KeyboardInterrupt:
    pass
except Exception:
    traceback.print_exc()
    if 'output' in locals():
        (output/'error.txt').write_text(traceback.format_exc())
    app.app.post_quit(1)
    raise
finally:
    if node is not None:
        try:
            motion.clear_command()
            node.publish()
            (output/'final_state.json').write_text(json.dumps(dict(pose=[motion.x, motion.y, motion.yaw],
                velocity=list(motion.velocity), sim_time_s=node.last_clock, epoch=node.epoch), indent=2)+'\n')
        finally:
            if node.lidar is not None:
                node.lidar.close()
            if node.imu is not None:
                node.imu.close()
            node.destroy_node()
    if trace is not None:
        trace.close()
    if ros_started and rclpy.ok():
        rclpy.shutdown()
    if world is not None:
        world.stop()
    app.close()
