#!/usr/bin/env python3
"""Validate and publish one saved localization map, without localization or TF."""
import argparse,json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import String
import yaml
from map_bundle import load_bundle


def cloud_message(points,frame):
    cloud=PointCloud2();cloud.header.frame_id=frame;cloud.height=1;cloud.width=len(points)
    cloud.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z','intensity'))]
    cloud.point_step=16;cloud.row_step=16*len(points);cloud.is_bigendian=False;cloud.is_dense=True
    cloud.data=points.astype('<f4',copy=False).tobytes();return cloud


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--project-root',required=True,type=Path);parser.add_argument('--map-directory',required=True,type=Path)
    args,ros_args=parser.parse_known_args()
    manifest,points=load_bundle(args.map_directory)
    config=yaml.safe_load((args.project_root/'ros2_ws/src/quadruped_localization/config/map_export.yaml').read_text())
    rclpy.init(args=ros_args);node=Node('quadruped_saved_map_loader')
    try:
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher=node.create_publisher(PointCloud2,config['loaded_cloud_topic'],qos)
        status=node.create_publisher(String,config['loaded_status_topic'],qos)
        publisher.publish(cloud_message(points,manifest['frame_id']))
        status.publish(String(data=json.dumps(dict(map_id=manifest['map_id'],map_directory=str(args.map_directory.resolve()),point_count=len(points),frame_id=manifest['frame_id'],tf_published=False,localization_started=False))))
        print('MAP_LOADED',manifest['map_id'],len(points),flush=True);rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
