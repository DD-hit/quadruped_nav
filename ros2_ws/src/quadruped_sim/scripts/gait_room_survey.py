"""Known-room collection through /cmd_vel; no runtime truth in the controller."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

import numpy as np
import yaml

from gait_scene import load_gait_lidar_config
from room_survey import clearance_xy


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    root = args.project_root.resolve()
    sys.path.insert(0, str(root/'ros2_ws/src/quadruped_localization/scripts'))
    from map_bundle import identifier
    identifier(args.map_id)
    host = yaml.safe_load((root/'ros2_ws/src/quadruped_bringup/config/hosts/server.yaml').read_text())
    cfg = yaml.safe_load((root/'ros2_ws/src/quadruped_sim/config/gait_room_survey.yaml').read_text())
    lidar_cfg = load_gait_lidar_config(root)
    room = lidar_cfg['room']
    radius = cfg['collection_radius_m']
    footprint = yaml.safe_load((root/'ros2_ws/src/d1_edu_description/config/footprint.yaml').read_text())
    if radius < footprint['joint_limit_envelope']['radius_xy_from_base_m'] + .05:
        raise ValueError('Collection radius does not contain the recorded joint envelope')
    waypoints = np.asarray(cfg['waypoints_xy_m'], dtype=float)
    route = np.vstack(([0., 0.], waypoints))
    samples = np.concatenate([np.linspace(a,b,max(2,math.ceil(np.linalg.norm(b-a)/.01)))
                              for a,b in zip(route,route[1:])])
    planned_clearance = float(clearance_xy(samples, room, radius).min())
    if planned_clearance < .12:
        raise ValueError(f'Insufficient known-room route separation: {planned_clearance}')
    map_dir = Path(host['data_root'])/'maps'/args.map_id
    if map_dir.exists():
        raise FileExistsError(map_dir)
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out/'scene_config.yaml').write_text(yaml.safe_dump(lidar_cfg, sort_keys=False))
    (out/'survey_config.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    settled = json.loads((root/'ros2_ws/src/quadruped_sim/config/d1_standby_initial_state.json').read_text())
    q = settled['base_quat_wxyz']
    yaw0 = math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2))
    rotation = np.array([[math.cos(yaw0),-math.sin(yaw0)], [math.sin(yaw0),math.cos(yaw0)]])
    translation = np.asarray(settled['base_position_world_m'][:2])
    contract = dict(scope=cfg['scope'], control_pose='/odom', control_source='/cmd_vel',
        runtime_truth_subscribed=False, truth_usage='offline_physics_log_evaluation_only',
        world_from_odom_initial_placement=dict(rotation=rotation.tolist(),translation=translation.tolist()),
        planned_clearance_m=planned_clearance, collection_radius_m=radius,
        collection_radius_basis=cfg['collection_radius_basis'], waypoints=waypoints.tolist(),
        hardware=False, world_geometry_changed='close north/south wall gaps after room extension',
        near_range_m=lidar_cfg['near_range_m'], far_range_m=lidar_cfg['far_range_m'])
    (out/'contract.json').write_text(json.dumps(contract,indent=2)+'\n')
    os.environ['ROS_DOMAIN_ID'] = str(host['ros_domain_id'])
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    interrupted = False
    children, logs = {}, []
    error = None
    codes = {}
    started = time.monotonic()
    saved = False

    def handle_signal(signum, frame):
        nonlocal interrupted
        interrupted = True

    def launch(name, command, env=None):
        handle = (out/(name+'.log')).open('w')
        logs.append(handle)
        process = subprocess.Popen(command,cwd=root,env=env,stdout=handle,
                                   stderr=subprocess.STDOUT,start_new_session=True)
        children[name] = process
        return process

    def cleanup(name):
        process = children[name]
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait(timeout=10)
        return process.returncode

    class Survey(Node):
        def __init__(self):
            super().__init__('quadruped_gait_room_survey')
            self.odom = None
            self.odom_wall = self.clock_wall = 0.
            self.sim = None
            self.mapping = self.base = self.runtime = None
            self.permitted = False
            self.desired = [0.,0.,0.]
            self.phase = 'startup'
            self.current = -1
            self.arrivals = []
            self.trace = []
            self.last_report = 0.
            self.last_trace_s = -1.
            self.command_count = 0
            qos = QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT)
            self.pub = self.create_publisher(Twist,'/cmd_vel',1)
            self.create_subscription(Odometry,'/odom',self.on_odom,100)
            self.create_subscription(Clock,'/clock',self.on_clock,qos)
            for topic, attribute in [('/mapping/status','mapping'),('/lio/base_odometry_status','base'),('/lio/runtime_status','runtime')]:
                self.create_subscription(String,topic,lambda m,a=attribute:setattr(self,a,json.loads(m.data)),50)
            self.create_timer(.05,self.publish)
            self.save_client = self.create_client(Trigger,'/mapping/save')

        def on_clock(self, msg):
            value = msg.clock.sec+msg.clock.nanosec/1e9
            if self.sim is not None and value < self.sim - 1e-5:
                raise RuntimeError('Clock rollback during survey')
            self.sim = value
            self.clock_wall = time.monotonic()

        def on_odom(self, msg):
            if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
                raise ValueError('Unexpected control odometry frame')
            p,q = msg.pose.pose.position,msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            self.odom = np.array([stamp,p.x,p.y,p.z,yaw])
            self.odom_wall = time.monotonic()
            if not np.isfinite(self.odom).all():
                raise ValueError('Nonfinite odometry')
            if stamp - self.last_trace_s >= .049:
                self.trace.append(self.odom.tolist())
                self.last_trace_s = stamp

        def publish(self):
            if not self.permitted:
                return
            command = self.desired
            if time.monotonic()-self.odom_wall > 2. or time.monotonic()-self.clock_wall > 2.:
                command = [0.,0.,0.]
            msg = Twist()
            msg.linear.x,msg.linear.y,msg.angular.z = map(float,command)
            self.pub.publish(msg)
            self.command_count += 1

        def set_command(self, vx=0.,vy=0.,wz=0.):
            self.desired = [float(vx),float(vy),float(wz)]
            self.publish()

        def world_pose(self):
            return rotation @ self.odom[1:3]+translation, wrap(self.odom[4]+yaw0)

        def spin(self):
            if interrupted:
                raise InterruptedError('Survey interrupted')
            rclpy.spin_once(self,timeout_sec=.015)
            for name, process in children.items():
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited: {process.returncode}')
            if self.mapping and self.mapping.get('phase') == 'invalidated':
                raise RuntimeError('Map recorder invalidated: '+str(self.mapping))
            if self.permitted:
                if time.monotonic()-self.odom_wall > 2. or time.monotonic()-self.clock_wall > 2.:
                    self.set_command()
                    raise RuntimeError('Control pose or clock stale')
                if not self.base or not self.base.get('aligned') or (self.runtime and self.runtime.get('generation') != 0):
                    self.set_command()
                    raise RuntimeError('Estimator alignment/reset during collection')
                xy,_ = self.world_pose()
                if clearance_xy([xy],room,radius)[0] < .015:
                    self.set_command()
                    raise RuntimeError('Estimated pose reaches collection separation boundary')
            if time.monotonic()-self.last_report > 5.:
                self.report()

        def wait(self, predicate, timeout=240):
            deadline = time.monotonic()+timeout
            while not predicate():
                self.spin()
                if time.monotonic() > deadline:
                    raise TimeoutError('Timed out in '+self.phase)

        def report(self):
            value = dict(phase=self.phase,waypoint_index=self.current,waypoints=len(waypoints),
                arrivals=len(self.arrivals),sim_time_s=self.sim,odom=self.odom.tolist() if self.odom is not None else None,
                world_estimate=self.world_pose()[0].tolist() if self.odom is not None else None,
                desired=self.desired,mapping=self.mapping,elapsed_wall_s=time.monotonic()-started)
            tmp=out/'progress.pending'
            tmp.write_text(json.dumps(value,indent=2)+'\n');tmp.replace(out/'progress.json')
            self.last_report=time.monotonic()

        def hold(self, duration):
            self.set_command()
            start = self.sim
            self.wait(lambda:self.sim >= start+duration,timeout=120)

        def align(self, target):
            self.phase = 'aligning'
            start = self.sim
            while abs(wrap(target-self.world_pose()[1])) > cfg['heading_tolerance_rad']:
                error = wrap(target-self.world_pose()[1])
                self.set_command(wz=math.copysign(cfg['turn_speed_radps'],error))
                self.spin()
                if self.sim-start > 45:
                    raise TimeoutError('Heading alignment did not converge')
            self.hold(.6)

        def full_scan(self):
            self.phase = 'rotating_scan'
            start = self.sim
            previous = self.world_pose()[1]
            angle = 0.
            while angle < 2*math.pi-.05:
                self.set_command(wz=cfg['turn_speed_radps'])
                self.spin()
                yaw = self.world_pose()[1]
                angle += wrap(yaw-previous)
                previous = yaw
                if self.sim-start > 60:
                    raise TimeoutError('Rotating scan did not complete')
            self.hold(cfg['settle_s'])

        def drive(self, target, index):
            self.current = index
            delta = target-self.world_pose()[0]
            if np.linalg.norm(delta) > .25:
                self.align(math.atan2(delta[1],delta[0]))
            self.phase = 'traversing'
            start = self.sim
            while True:
                xy,yaw = self.world_pose()
                delta = target-xy
                distance = float(np.linalg.norm(delta))
                if distance <= cfg['arrival_tolerance_m']+cfg['braking_lead_m']:
                    break
                heading = math.atan2(delta[1],delta[0])
                error = wrap(heading-yaw)
                if abs(error) > .4:
                    self.align(heading)
                    self.phase = 'traversing'
                    continue
                speed = cfg['speed_mps']
                predicted = xy+.5*speed*np.array([math.cos(yaw),math.sin(yaw)])
                if clearance_xy([predicted],room,radius)[0] < .02:
                    raise RuntimeError('Predicted collection separation boundary')
                self.set_command(vx=speed,wz=np.clip(1.4*error,-.3,.3))
                self.spin()
                if self.sim-start > 70:
                    raise TimeoutError('Waypoint did not converge')
            self.hold(cfg['settle_s'])
            xy,yaw = self.world_pose()
            row=dict(index=index,target=target.tolist(),estimate=xy.tolist(),sim_time_s=self.sim,
                     error_m=float(np.linalg.norm(target-xy)),yaw=yaw)
            if row['error_m'] > .4:
                raise RuntimeError('Waypoint stopping error exceeded .4 m: '+str(row))
            self.arrivals.append(row)
            (out/'arrivals.json').write_text(json.dumps(self.arrivals,indent=2)+'\n')
            print('WAYPOINT',json.dumps(row),flush=True)
            self.report()

    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,handle_signal);signal.signal(signal.SIGTERM,handle_signal)
    node = Survey()
    try:
        until = time.monotonic()+1.
        while time.monotonic()<until:
            node.spin()
        if node.get_subscriptions_info_by_topic('/cmd_vel') or node.get_publishers_info_by_topic('/mapping/status'):
            raise RuntimeError('Another robot/mapping session owns this ROS domain')
        launch('mapping',['/usr/bin/python3',str(root/'scripts/run_mapping.py'),'--map-id',args.map_id,
            '--output-dir',str(out/'mapping'),'--base-config-file',
            str(root/'ros2_ws/src/quadruped_localization/config/base_odometry_dynamic_gait.yaml')])
        node.wait(lambda:(out/'mapping/lio/status.json').exists(),timeout=60)
        launch('isaac',['/usr/bin/python3',str(root/'scripts/run_gait_cmd_vel.py'),
            '--max-seconds',str(cfg['max_sim_seconds']),'--output-dir',str(out/'isaac_gait')])
        node.wait(lambda:node.mapping and node.mapping.get('scans',0)>=25 and node.odom is not None
                  and node.base and node.base.get('aligned'),timeout=240)
        consumers = node.get_subscriptions_info_by_topic('/cmd_vel')
        producers = node.get_publishers_info_by_topic('/cmd_vel')
        if len(consumers)!=1 or consumers[0].node_name!='d1_runtime_gait_sensors' or len(producers)!=1 or producers[0].node_name!=node.get_name():
            raise RuntimeError('Velocity ownership does not match the owned Isaac session')
        node.permitted=True
        node.hold(1.)
        node.full_scan()
        for index,target in enumerate(waypoints):
            node.drive(target,index)
            if index in cfg['full_scan_after_waypoint_indices']:
                node.full_scan()
        node.full_scan()
        node.phase='final_settling';node.hold(2.)
        node.phase='saving'
        if not node.save_client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Map save service unavailable')
        future=node.save_client.call_async(Trigger.Request())
        node.wait(future.done,timeout=90)
        if not future.result().success:
            raise RuntimeError('Map save failed: '+future.result().message)
        saved=True
        node.phase='complete';node.report()
    except Exception:
        error=traceback.format_exc()
        (out/'error.txt').write_text(error)
        print(error,flush=True)
    finally:
        if node.permitted:
            node.set_command()
            # Deliver the final zero command before terminating only owned children.
            deadline=time.monotonic()+.3
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
        for name in reversed(children):
            codes[name]=cleanup(name)
        np.savez_compressed(out/'trajectory.npz',odometry=np.asarray(node.trace))
        (out/'arrivals.json').write_text(json.dumps(node.arrivals,indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for handle in logs:handle.close()
        result=dict(result='COLLECTED' if saved and not error and all(v==0 for v in codes.values()) else 'FAIL',
            failure=error,map_id=args.map_id,map_directory=str(map_dir),arrivals=len(node.arrivals),
            waypoints=len(waypoints),process_exits=codes,coverage_accepted=False,
            actual_physics_geometry_accepted=False,commands=node.command_count,hardware=False)
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
    return 0 if result['result']=='COLLECTED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
