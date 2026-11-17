#!/usr/bin/env python3
"""Adapt explicit timed PointCloud2 to Livox CustomMsg; relay SI IMU reliably."""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Imu,PointField
from std_msgs.msg import String
from livox_ros_driver2.msg import CustomMsg,CustomPoint


def read_points(cloud,frame,period_ns):
    if cloud.header.frame_id!=frame:
        raise ValueError('unexpected lidar frame; adapter never transforms coordinates')
    if cloud.height!=1 or cloud.width<2 or cloud.width>1_000_000:
        raise ValueError('expected a nonempty unorganized scan with at least two points')
    if cloud.point_step<=0 or cloud.row_step!=cloud.width*cloud.point_step or len(cloud.data)!=cloud.row_step:
        raise ValueError('invalid PointCloud2 size/stride')
    fields={f.name:f for f in cloud.fields}
    if len(fields)!=len(cloud.fields):raise ValueError('duplicate PointCloud2 field')
    names=('x','y','z','intensity','offset_time')
    formats=[];offsets=[]
    endian='>' if cloud.is_bigendian else '<'
    for name in names:
        field=fields.get(name)
        expected=PointField.UINT32 if name=='offset_time' else PointField.FLOAT32
        if field is None or field.datatype!=expected or field.count!=1 or field.offset<0 or field.offset+4>cloud.point_step:
            raise ValueError('missing/invalid '+name+' field; offset_time must be uint32 nanoseconds')
        formats.append(endian+('u4' if name=='offset_time' else 'f4'));offsets.append(field.offset)
    if any(abs(a-b)<4 for i,a in enumerate(offsets) for b in offsets[i+1:]):
        raise ValueError('overlapping PointCloud2 fields')
    dtype=np.dtype(dict(names=names,formats=formats,offsets=offsets,itemsize=cloud.point_step))
    data=np.frombuffer(bytes(cloud.data),dtype=dtype,count=cloud.width)
    xyz=np.column_stack([data[name] for name in ('x','y','z')])
    if not np.isfinite(xyz).all() or not np.isfinite(data['intensity']).all():
        raise ValueError('non-finite points/intensities')
    if np.any(data['intensity']<0) or np.any(data['intensity']>255):
        raise ValueError('intensity outside uint8 reflectivity range')
    offsets=data['offset_time'].astype(np.int64)
    if np.any(np.diff(offsets)<=0) or offsets[-1]>=period_ns:
        raise ValueError('point offsets must increase strictly within the configured scan period')
    if cloud.header.stamp.sec<0:raise ValueError('negative scan time')
    return data


def convert_cloud(cloud,frame,period_ns):
    data=read_points(cloud,frame,period_ns)
    result=CustomMsg();result.header=cloud.header
    # Header remains the scan origin even if early rays were occluded/discarded.
    result.timebase=cloud.header.stamp.sec*1_000_000_000+cloud.header.stamp.nanosec
    result.point_num=len(data);result.lidar_id=0
    for row in data:
        p=CustomPoint();p.x=float(row['x']);p.y=float(row['y']);p.z=float(row['z'])
        p.offset_time=int(row['offset_time']);p.reflectivity=int(round(float(row['intensity'])))
        # A virtual line and accepted single-return tag, not physical emitter IDs.
        p.line=0;p.tag=0x10;result.points.append(p)
    return result


class InputAdapter(Node):
    def __init__(self,config):
        super().__init__('quadruped_lio_input_adapter')
        self.config=config;self.clouds=0;self.imus=0;self.rejected_clouds=0;self.rejected_imus=0
        self.rollbacks=0;self.last_stamps={};self.last_error=None
        inp=config['input'];out=config['output']
        if inp['lidar_topic']==out['lidar_topic'] or inp['imu_topic']==out['imu_topic']:
            raise ValueError('input and output topics must differ')
        self.cloud_pub=self.create_publisher(CustomMsg,out['lidar_topic'],QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE))
        self.imu_pub=self.create_publisher(Imu,out['imu_topic'],QoSProfile(depth=200,reliability=ReliabilityPolicy.RELIABLE))
        self.status=self.create_publisher(String,config['status_topic'],10)
        self.create_subscription(PointCloud2,inp['lidar_topic'],self.on_cloud,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Imu,inp['imu_topic'],self.on_imu,QoSProfile(depth=500,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.report()

    def observe_stamp(self,key,stamp):
        now=stamp.sec*1_000_000_000+stamp.nanosec
        if key in self.last_stamps and now<self.last_stamps[key]:
            self.rollbacks+=1
            self.get_logger().warn('Input simulation time rolled back; downstream LIO must be restarted.')
        self.last_stamps[key]=now

    def on_cloud(self,cloud):
        try:
            message=convert_cloud(cloud,self.config['input']['lidar_frame'],round(1e9/self.config['scan_hz']))
        except (ValueError,OverflowError) as error:
            self.rejected_clouds+=1;self.last_error=str(error);self.get_logger().warn(self.last_error);self.report();return
        self.observe_stamp('lidar',cloud.header.stamp)
        self.cloud_pub.publish(message);self.clouds+=1;self.report()

    def on_imu(self,message):
        values=[getattr(v,c) for v in (message.linear_acceleration,message.angular_velocity) for c in ('x','y','z')]
        if message.header.frame_id!=self.config['input']['imu_frame'] or message.header.stamp.sec<0 or not all(math.isfinite(v) for v in values):
            self.rejected_imus+=1;self.last_error='invalid IMU frame/time/measurement';self.report();return
        self.observe_stamp('imu',message.header.stamp)
        # No scale, rotation, gravity subtraction, orientation injection or retiming.
        self.imu_pub.publish(message);self.imus+=1

    def report(self):
        self.status.publish(String(data=json.dumps(dict(clouds=self.clouds,imus=self.imus,
            rejected_clouds=self.rejected_clouds,rejected_imus=self.rejected_imus,
            time_rollbacks=self.rollbacks,last_error=self.last_error,
            synthetic_line=0,synthetic_tag=16,output_reliability='reliable',
            imu_scale=1.0,coordinates_transformed=False,lio_started=False))))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True,type=Path)
    args,ros_args=parser.parse_known_args()
    config=yaml.safe_load(args.config.read_text())
    if config.get('schema_version')!=1 or not math.isfinite(config['scan_hz']) or config['scan_hz']<=0:
        parser.error('invalid adapter configuration')
    rclpy.init(args=ros_args);node=None
    try:
        node=InputAdapter(config);rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
