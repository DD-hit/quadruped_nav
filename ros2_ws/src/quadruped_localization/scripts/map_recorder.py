#!/usr/bin/env python3
"""Record one LIO epoch into an immutable, gravity-aligned localization map."""
import argparse
from collections import OrderedDict,deque
from copy import deepcopy
from datetime import datetime,timezone
import json
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from map_alignment import final_alignment
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from base_geometry import mount,quaternion_matrix
from map_bundle import identifier,VoxelMap,pointcloud_xyzi,validate_rotation,commit_bundle


def ns(stamp):return stamp.sec*1_000_000_000+stamp.nanosec


class MapRecorder(Node):
    def __init__(self,root,map_id,robot_id,lio_directory,directory=None):
        super().__init__('quadruped_map_recorder')
        self.root=Path(root);configs=self.root/'ros2_ws/src/quadruped_bringup/config'
        self.common=yaml.safe_load((configs/'common.yaml').read_text());self.robot=yaml.safe_load((configs/'robots'/f'{robot_id}.yaml').read_text())
        self.config=yaml.safe_load((self.root/'ros2_ws/src/quadruped_localization/config/map_export.yaml').read_text())
        host=yaml.safe_load((configs/'hosts/server.yaml').read_text())
        if self.common['frames']['map']!='map':raise ValueError('This map format requires the canonical map frame')
        self.map_id=identifier(map_id);self.robot_id=robot_id;self.lio_directory=Path(lio_directory).resolve()
        self.directory=Path(directory) if directory is not None else Path(host['data_root'])/'maps'/self.map_id
        self.directory.mkdir(parents=True,exist_ok=False)
        self.gravity_samples=deque(maxlen=2000)
        self.create_subscription(Vector3Stamped,self.config.get('gravity_topic','/lio/gravity'),self.on_gravity,100)
        self.phase='waiting';self.reason='waiting_for_aligned_lio';self.runtime=None;self.base=None;self.clock_ns=None
        self.token=None;self.alignment=None;self.static={};self.pending=OrderedDict();self.last_ns=None;self.first_ns=None
        self.voxels=VoxelMap(self.config['voxel_size_m'],self.config['max_voxels']);self.scans=0;self.rejected=0
        self.pub=self.create_publisher(String,self.config['status_topic'],10)
        self.create_subscription(String,self.config['runtime_status_topic'],self.on_runtime,20)
        self.create_subscription(String,self.config['base_status_topic'],self.on_base,100)
        self.create_subscription(PointCloud2,self.config['registered_cloud_topic'],lambda m:self.pair('cloud',m),20)
        self.create_subscription(Odometry,self.common['interfaces']['odometry']['topic'],lambda m:self.pair('odometry',m),100)
        self.create_subscription(Clock,'/clock',self.on_clock,QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(TFMessage,'/tf_static',self.on_static,QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger,self.config['save_service'],self.save_callback)
        self.create_timer(.25,self.report)
        self.report()

    def on_gravity(self,message):
        if message.header.frame_id!=self.config['registered_cloud_frame']:return
        g=message.vector
        self.gravity_samples.append([ns(message.header.stamp)*1e-9,g.x,g.y,g.z])

    def invalidate(self,reason):
        if self.phase=='saved':return
        self.phase='invalidated';self.reason=reason;self.pending.clear();self.report()

    def on_runtime(self,message):
        value=json.loads(message.data)
        if value.get('robot')!=self.robot_id or Path(value.get('output_directory','')).resolve()!=self.lio_directory:return
        self.runtime=value
        if self.token is not None and (value['generation'],value['estimator_pid'])!=self.token:self.invalidate('estimator_generation_changed_start_new_map_session')

    def on_base(self,message):
        value=json.loads(message.data);self.base=value
        if self.token is not None and (value['generation']!=self.token[0] or not value['aligned']):self.invalidate('base_alignment_changed_start_new_map_session')

    def on_clock(self,message):
        current=ns(message.clock)
        if self.clock_ns is not None and current<self.clock_ns-1000:self.invalidate('clock_rollback_start_new_map_session')
        self.clock_ns=current

    def on_static(self,message):
        self.static.update({t.child_frame_id:t for t in message.transforms})

    def mount_matches(self):
        aliases=self.robot['frame_aliases']
        if not all(name in self.static for name in (aliases['lidar'],aliases['imu'])):return False
        a=self.static[aliases['lidar']];b=self.static[aliases['imu']]
        if a.header.frame_id!=aliases['base'] or b.header.frame_id!=aliases['lidar']:return False
        def R(q):return quaternion_matrix([q.x,q.y,q.z,q.w])
        def p(v):return np.array([v.x,v.y,v.z])
        RBI,rBI=mount(self.robot)
        return np.allclose(R(a.transform.rotation)@R(b.transform.rotation),RBI,atol=1e-7,rtol=0) and np.allclose(p(a.transform.translation)+R(a.transform.rotation)@p(b.transform.translation),rBI,atol=1e-7,rtol=0)

    def ready(self):
        return bool(self.runtime and self.base and self.base['aligned'] and self.runtime['initialized'] and
            self.base['generation']==self.runtime['generation'] and self.mount_matches())

    def pair(self,kind,message):
        if self.phase in ('invalidated','saved'):return
        if not self.ready():self.pending.clear();return
        stamp=ns(message.header.stamp)
        if stamp<self.base['anchor_ns'] or (self.clock_ns is not None and stamp>self.clock_ns+200_000_000):self.rejected+=1;return
        if self.last_ns is not None and stamp<=self.last_ns:self.rejected+=1;return
        row=self.pending.setdefault(stamp,{});row[kind]=message
        if len(self.pending)>self.config['max_pending_pairs']:self.pending.popitem(last=False);self.rejected+=1
        if len(row)!=2:return
        self.pending.pop(stamp,None)
        try:self.accumulate(row['cloud'],row['odometry'])
        except ValueError as error:self.invalidate(str(error))

    def accumulate(self,cloud,odometry):
        if cloud.header.frame_id!=self.config['registered_cloud_frame'] or odometry.header.frame_id!=self.common['frames']['odom'] or odometry.child_frame_id!=self.robot['frame_aliases']['base']:
            raise ValueError('Unexpected registered cloud or base odometry frame')
        R=validate_rotation(self.base['odom_from_lio_rotation']);t=np.array(self.base['odom_from_lio_translation'],dtype=float)
        if t.shape!=(3,) or not np.isfinite(t).all():raise ValueError('Invalid base alignment translation')
        token=(self.runtime['generation'],self.runtime['estimator_pid'])
        if self.token is None:
            self.token=token;self.alignment=dict(rotation=R.tolist(),translation_m=t.tolist())
            self.anchor=deepcopy(self.base);self.runtime_snapshot=deepcopy(self.runtime)
        elif token!=self.token or not np.allclose(R,self.alignment['rotation'],atol=1e-12,rtol=0) or not np.allclose(t,self.alignment['translation_m'],atol=1e-12,rtol=0):
            raise ValueError('Reference alignment changed during mapping')
        points=pointcloud_xyzi(cloud,self.config['max_cloud_points'])
        points[:,:3]=points[:,:3]@R.T+t
        self.voxels.add(points);self.scans+=1;self.last_ns=ns(cloud.header.stamp)
        if self.first_ns is None:self.first_ns=self.last_ns
        self.phase='recording';self.reason='collecting_registered_scans';self.report()

    def save_callback(self,request,response):
        if self.phase!='recording':response.success=False;response.message='Map is not recording: '+self.phase;return response
        duration=(self.last_ns-self.first_ns)/1e9
        if self.scans<self.config['minimum_scans'] or duration<self.config['minimum_duration_s'] or len(self.voxels.cells)<self.config['minimum_voxels']:
            response.success=False;response.message='Insufficient scan coverage/duration/points';return response
        try:
            alignment,dR,dt,alignment_report=final_alignment(self.alignment,list(self.gravity_samples),self.last_ns,
                self.config.get('gravity_finalize_window_s',1.),self.config.get('gravity_finalize_max_spread_deg',.1))
        except ValueError as error:
            response.success=False;response.message=str(error);return response
        try:
            generation=self.lio_directory/f'generation_{self.token[0]:03d}'
            actual=yaml.safe_load((generation/'fastlio.yaml').read_text())
            manifest=dict(map_id=self.map_id,frame_id='map',map_from_camera_init=alignment,export_alignment=alignment_report,
                origin=dict(position='base_position_at_alignment',vertical='opposite_estimated_gravity',heading='initial_base_forward_horizontal_projection',ground_plane_z='not_assumed_zero'),
                source=dict(kind='lio_registered_sensor_estimate',topic=self.config['registered_cloud_topic'],frame_id=self.config['registered_cloud_frame'],runtime_directory=str(self.lio_directory),generation=self.token[0],estimator_pid=self.token[1],first_scan_ns=self.first_ns,last_scan_ns=self.last_ns,scans=self.scans,input_points=self.voxels.input_points),
                voxel_size_m=self.voxels.resolution,robot_id=self.robot_id,base_alignment=self.anchor,
                upstream_commit='38b12d8d83d10ad1fe1e6b354a34fbf717ae42f4',created_utc=datetime.now(timezone.utc).isoformat(),
                truth_used=False,real_hardware_validation_complete=False,navigation_map_present=False)
            snapshots={'robot_profile.yaml':self.robot,'common.yaml':self.common,'map_export.yaml':self.config,'fastlio.yaml':actual,'runtime_status.json':self.runtime_snapshot}
            points=self.voxels.points().copy()
            points[:,:3]=points[:,:3]@dR.T+dt
            commit_bundle(self.directory,points,manifest,snapshots)
            self.phase='saved';self.reason='immutable_map_committed';self.pending.clear();self.report()
            response.success=True;response.message=str(self.directory)
        except Exception as error:
            self.invalidate('save_failed: '+str(error));response.success=False;response.message=self.reason
        return response

    def report(self):
        value=dict(map_id=self.map_id,phase=self.phase,reason=self.reason,scans=self.scans,voxels=len(self.voxels.cells),
            input_points=self.voxels.input_points,rejected=self.rejected,first_scan_ns=self.first_ns,last_scan_ns=self.last_ns,
            map_directory=str(self.directory),source_generation=self.token[0] if self.token else None,truth_used=False,
            navigation_map_present=False,real_hardware_validation_complete=False)
        if self.context.ok():self.pub.publish(String(data=json.dumps(value)))
        (self.directory/'recording_status.json').write_text(json.dumps(value,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--project-root',required=True,type=Path)
    parser.add_argument('--map-id',required=True);parser.add_argument('--robot',default='d1_edu');parser.add_argument('--lio-directory',required=True,type=Path)
    args,ros_args=parser.parse_known_args();identifier(args.map_id)
    if not args.robot or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789_' for c in args.robot):parser.error('Invalid robot identifier')
    rclpy.init(args=ros_args);node=None
    try:
        node=MapRecorder(args.project_root,args.map_id,args.robot,args.lio_directory);rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node is not None:node.report();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
