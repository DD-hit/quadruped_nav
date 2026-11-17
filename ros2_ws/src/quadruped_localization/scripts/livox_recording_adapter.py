#!/usr/bin/env python3
"""Normalize native Livox recordings without synthesizing point metadata."""
import argparse,json
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg
from std_msgs.msg import String
from livox_recording import normalize_cloud,normalize_imu

class Adapter(Node):
    def __init__(self,profile):
        super().__init__('livox_recording_adapter');self.profile=profile
        self.clouds=self.imus=self.rejections=0;self.last_error=None
        output=profile['output'];topics=profile['replay_topics']
        if len(set([topics['lidar'],topics['imu'],output['lidar_topic'],output['imu_topic'],output['lio_imu_topic']]))!=5:
            raise ValueError('Raw and normalized topics must be distinct')
        reliable=QoSProfile(depth=200,reliability=ReliabilityPolicy.RELIABLE)
        best=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cloud_pub=self.create_publisher(CustomMsg,output['lidar_topic'],reliable)
        self.imu_pub=self.create_publisher(Imu,output['imu_topic'],reliable)
        self.lio_pub=self.create_publisher(Imu,output['lio_imu_topic'],reliable)
        self.status_pub=self.create_publisher(String,profile['status_topic'],10)
        self.create_subscription(CustomMsg,topics['lidar'],self.on_cloud,best)
        self.create_subscription(Imu,topics['imu'],self.on_imu,best)

    def report(self):
        self.status_pub.publish(String(data=json.dumps(dict(clouds=self.clouds,imus=self.imus,
            rejections=self.rejections,last_error=self.last_error,
            acceleration_input_unit=self.profile['input']['acceleration_unit'],acceleration_output_unit='m/s^2',
            native_point_metadata_preserved=True,hardware_validation_complete=False))))

    def on_cloud(self,message):
        try:result=normalize_cloud(message,self.profile)
        except ValueError as error:
            self.rejections+=1;self.last_error=str(error);self.report();return
        self.cloud_pub.publish(result);self.clouds+=1;self.report()

    def on_imu(self,message):
        try:result=normalize_imu(message,self.profile)
        except ValueError as error:
            self.rejections+=1;self.last_error=str(error);self.report();return
        self.imu_pub.publish(result);self.lio_pub.publish(result);self.imus+=1


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--profile',required=True,type=Path)
    args,ros_args=parser.parse_known_args();profile=yaml.safe_load(args.profile.read_text())
    rclpy.init(args=ros_args);node=None
    try:node=Adapter(profile);rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
