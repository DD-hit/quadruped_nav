#!/usr/bin/env python3
"""Convert gravity-stamped FAST-LIO IMU state into a level base odometry frame."""
import argparse
from collections import OrderedDict,deque
from copy import deepcopy
import json,math
from pathlib import Path
import re
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import TransformStamped,Vector3Stamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage
from base_geometry import mount,rotation,quaternion_matrix,matrix_quaternion,base_in_reference,level_anchor,convert


def ns(stamp):return stamp.sec*1_000_000_000+stamp.nanosec
def vector(v):return np.array([v.x,v.y,v.z])
def assign_vector(target,v):target.x,target.y,target.z=map(float,v)
def assign_rotation(target,R):target.x,target.y,target.z,target.w=map(float,matrix_quaternion(R))


class BaseOdometry(Node):
    def __init__(self,config,robot,common,generation=0,status_file=None):
        super().__init__('quadruped_base_odometry')
        config=deepcopy(config)
        config['output']['odometry_topic']=common['interfaces']['odometry']['topic']
        if config['input']['covariance_convention']!='ros_fixed_axes_reference_frame':raise ValueError('Unsupported source covariance convention')
        if config['initialization']['origin']!='initial_base_position' or config['initialization']['heading']!='initial_base_forward_projected_on_gravity_plane':raise ValueError('Unsupported initialization convention')
        self.config=config;self.robot=robot;self.frames=common['frames'];self.generation=generation
        self.status_file=status_file;self.clock_ns=None;self.resets=0;self.published=0;self.rejected=0
        self.RBI,self.rBI=mount(robot);self.ROC=self.tOC=None;self.last_ns=None
        self.window=deque();self.pending=OrderedDict();self.reason='waiting_for_paired_lio_state_and_gravity'
        output=config['output'];input_=config['input']
        self.odom_pub=self.create_publisher(Odometry,output['odometry_topic'],20)
        self.status_pub=self.create_publisher(String,output['status_topic'],10)
        self.tf=TransformBroadcaster(self)
        self.static_tf=StaticTransformBroadcaster(self) if output['publish_sensor_tf'] else None
        self.create_subscription(Odometry,input_['odometry_topic'],self.on_odometry,100)
        self.create_subscription(Vector3Stamped,input_['gravity_topic'],self.on_gravity,100)
        self.create_subscription(Clock,'/clock',self.on_clock,QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(.25,self.report)
        if self.static_tf:
            aliases=robot['frame_aliases'];sensor=robot['sensor']
            transforms=[]
            for parent,child,key in ((aliases['base'],aliases['lidar'],'base_to_lidar'),
                                     (aliases['lidar'],aliases['imu'],'lidar_to_imu')):
                transform=TransformStamped();transform.header.frame_id=parent;transform.child_frame_id=child
                transform.header.stamp=self.get_clock().now().to_msg()
                assign_vector(transform.transform.translation,sensor[key]['translation_m'])
                assign_rotation(transform.transform.rotation,rotation(sensor[key]['rpy_rad']))
                transforms.append(transform)
            self.static_tf.sendTransform(transforms)

    def clear(self,reason):
        self.ROC=self.tOC=None;self.window.clear();self.pending.clear();self.last_ns=None
        self.reason=reason;self.resets+=1;self.report()

    def on_clock(self,message):
        current=ns(message.clock)
        if self.clock_ns is not None and current<self.clock_ns-1000:self.clear('clock_rollback_waiting_for_fresh_estimator')
        self.clock_ns=current

    def on_odometry(self,message):self.pair('odometry',message)
    def on_gravity(self,message):self.pair('gravity',message)

    def pair(self,kind,message):
        stamp=ns(message.header.stamp)
        if self.clock_ns is not None and stamp>self.clock_ns+200_000_000:
            self.rejected+=1;return  # queued sample from the epoch before reset
        if self.last_ns is not None and stamp<=self.last_ns:
            self.rejected+=1;return
        row=self.pending.setdefault(stamp,{})
        row[kind]=message
        if len(self.pending)>200:self.pending.popitem(last=False)
        if len(row)==2:
            self.pending.pop(stamp,None)
            try:self.process(row['odometry'],row['gravity'])
            except ValueError as error:
                self.rejected+=1;self.reason=str(error);self.report()

    def process(self,source,gravity):
        input_=self.config['input'];init=self.config['initialization']
        if source.header.frame_id!=input_['reference_frame'] or source.child_frame_id!=input_['state_frame'] or gravity.header.frame_id!=input_['reference_frame']:
            raise ValueError('Unexpected LIO reference/state frame')
        stamp=ns(source.header.stamp);pose=source.pose.pose;twist=source.twist.twist
        pCI=vector(pose.position);q=pose.orientation
        RCI=quaternion_matrix([q.x,q.y,q.z,q.w]);vI=vector(twist.linear);wI=vector(twist.angular)
        Cpose=np.array(source.pose.covariance).reshape(6,6);Ctwist=np.array(source.twist.covariance).reshape(6,6)
        g=vector(gravity.vector)
        if not np.isfinite(np.r_[pCI,vI,wI,g,Cpose.ravel(),Ctwist.ravel()]).all():raise ValueError('Non-finite filter state')
        if not 7<np.linalg.norm(g)<12.5:raise ValueError('Invalid estimated gravity')
        for covariance in (Cpose,Ctwist):
            if np.max(abs(covariance-covariance.T))>1e-6 or np.linalg.eigvalsh(covariance).min()<-1e-7:
                raise ValueError('Invalid filter covariance')
        pCB,RCB=base_in_reference(pCI,RCI,self.RBI,self.rBI)
        if self.last_ns is not None and (stamp-self.last_ns)/1e9>init['max_input_gap_s'] and self.ROC is None:self.window.clear()
        self.last_ns=stamp
        if self.ROC is None:
            # Use velocity at the BASE origin for the stationary gate.
            temporary=convert(pCI,RCI,vI,wI,Cpose,Ctwist,self.RBI,self.rBI,np.eye(3),np.zeros(3))
            stationary=(np.linalg.norm(temporary['linear_velocity'])<=init['max_linear_speed_m_s'] and np.linalg.norm(wI)<=init['max_angular_speed_rad_s'])
            if not stationary:
                self.window.clear();self.reason='waiting_for_stationary_initialization';self.report();return
            self.window.append((stamp,pCB,g))
            while len(self.window)>1 and (stamp-self.window[1][0])/1e9>=init['stationary_duration_s']:self.window.popleft()
            positions=np.array([item[1] for item in self.window]);directions=np.array([item[2]/np.linalg.norm(item[2]) for item in self.window])
            if np.max(np.linalg.norm(positions-positions[-1],axis=1))>init['max_position_span_m'] or np.min(directions@directions[-1])<math.cos(math.radians(init['max_gravity_angle_deg'])):
                self.window.clear();self.window.append((stamp,pCB,g));self.reason='waiting_for_stable_pose_and_gravity';self.report();return
            if (stamp-self.window[0][0])/1e9<init['stationary_duration_s']:
                self.reason='stationary_warmup';self.report();return
            self.ROC,self.tOC=level_anchor(pCB,RCB,g)
            self.reason='aligned';self.anchor_ns=stamp;self.gravity_at_anchor=g.tolist()
        state=convert(pCI,RCI,vI,wI,Cpose,Ctwist,self.RBI,self.rBI,self.ROC,self.tOC)
        result=Odometry();result.header=deepcopy(source.header);result.header.frame_id=self.frames['odom'];result.child_frame_id=self.robot['frame_aliases']['base']
        assign_vector(result.pose.pose.position,state['position']);assign_rotation(result.pose.pose.orientation,state['rotation'])
        assign_vector(result.twist.twist.linear,state['linear_velocity']);assign_vector(result.twist.twist.angular,state['angular_velocity'])
        result.pose.covariance=state['pose_covariance'].ravel().tolist();result.twist.covariance=state['twist_covariance'].ravel().tolist()
        self.odom_pub.publish(result)
        transforms=[]
        for child,p,R in ((result.child_frame_id,state['position'],state['rotation']),
                          (input_['reference_frame'],self.tOC,self.ROC)):
            if child==input_['reference_frame'] and not self.config['output']['publish_reference_tf']:continue
            transform=TransformStamped();transform.header=deepcopy(result.header);transform.child_frame_id=child
            assign_vector(transform.transform.translation,p);assign_rotation(transform.transform.rotation,R)
            transforms.append(transform)
        self.tf.sendTransform(transforms);self.published+=1;self.report()

    def report(self):
        state=dict(generation=self.generation,local_reset_count=self.resets,aligned=self.ROC is not None,
            reason=self.reason,published=self.published,rejected=self.rejected,last_input_ns=self.last_ns,
            odometry_topic=self.config['output']['odometry_topic'],frames=dict(parent=self.frames['odom'],child=self.robot['frame_aliases']['base']),
            truth_used=False,reference_alignment_frozen=self.ROC is not None,
            sensor_tf_published=self.config['output']['publish_sensor_tf'],sdk_alignment_status=self.robot['base_reference']['sdk_alignment_status'])
        if self.window and self.last_ns is not None:state['stationary_window_s']=(self.last_ns-self.window[0][0])/1e9
        if self.ROC is not None:
            state.update(anchor_ns=self.anchor_ns,odom_from_lio_rotation=self.ROC.tolist(),odom_from_lio_translation=self.tOC.tolist(),gravity_at_anchor=self.gravity_at_anchor)
        self.status_pub.publish(String(data=json.dumps(state)))
        if self.status_file:self.status_file.write_text(json.dumps(state,indent=2)+'\n')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--project-root',required=True,type=Path)
    parser.add_argument('--robot',default='d1_edu');parser.add_argument('--generation',type=int,default=0)
    parser.add_argument('--status-file',type=Path)
    parser.add_argument('--config-file',type=Path,help='Explicit alternate initialization profile; default remains base_odometry.yaml')
    args,ros_args=parser.parse_known_args(argv)
    if not re.fullmatch(r'[a-z][a-z0-9_]*',args.robot):parser.error('Invalid robot profile identifier')
    configs=args.project_root/'ros2_ws/src/quadruped_bringup/config'
    robot=yaml.safe_load((configs/'robots'/f'{args.robot}.yaml').read_text());common=yaml.safe_load((configs/'common.yaml').read_text())
    config_path=args.config_file or args.project_root/'ros2_ws/src/quadruped_localization/config/base_odometry.yaml'
    config=yaml.safe_load(config_path.read_text())
    rclpy.init(args=ros_args);node=None
    try:
        # Refuse a second public odometry authority before creating publishers.
        guard=Node('base_odometry_startup_guard')
        occupied=set()
        reserved={robot['frame_aliases']['base'],config['input']['reference_frame']}
        if config['output']['publish_sensor_tf']:reserved.update((robot['frame_aliases']['lidar'],robot['frame_aliases']['imu']))
        def observe(message):
            occupied.update(t.child_frame_id for t in message.transforms if t.child_frame_id in reserved)
        guard.create_subscription(TFMessage,'/tf',observe,100)
        guard.create_subscription(TFMessage,'/tf_static',observe,QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        import time
        until=time.monotonic()+.6
        while time.monotonic()<until:rclpy.spin_once(guard,timeout_sec=.05)
        clashes=guard.get_publishers_info_by_topic(common['interfaces']['odometry']['topic'])
        guard.destroy_node()
        if clashes:raise RuntimeError('Another public odometry publisher is active')
        if occupied:raise RuntimeError('Another TF authority already owns: '+', '.join(sorted(occupied)))
        node=BaseOdometry(config,robot,common,args.generation,args.status_file);rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
