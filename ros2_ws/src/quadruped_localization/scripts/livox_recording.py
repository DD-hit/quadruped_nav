"""Native Livox recording contracts and explicit g-to-SI normalization."""
from copy import deepcopy
import math
import numpy as np

G=9.80665


def stamp_ns(stamp):
    if stamp.sec<0 or not 0<=stamp.nanosec<1_000_000_000:raise ValueError('Invalid sensor timestamp')
    return stamp.sec*1_000_000_000+stamp.nanosec


def acceleration_scale(profile):
    unit=profile['input']['acceleration_unit']
    if unit=='g':return G
    if unit=='m_s2':return 1.0
    raise ValueError('Acceleration unit must be explicitly g or m_s2')


def inspect_cloud(message,profile):
    if profile['input']['cloud_coordinates']!='native_lidar':raise ValueError('Expected native lidar coordinates; pre-transformed clouds need a separate profile')
    if message.header.frame_id!=profile['input']['lidar_frame']:raise ValueError('Unexpected raw lidar frame')
    stamp=stamp_ns(message.header.stamp)
    if message.timebase!=stamp:raise ValueError('CustomMsg timebase differs from header time')
    if message.point_num!=len(message.points) or message.point_num<2:raise ValueError('Invalid CustomMsg point count')
    previous=-1;accepted=0
    for p in message.points:
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):raise ValueError('Non-finite lidar point')
        if p.offset_time<previous:raise ValueError('Point timestamps are not nondecreasing')
        previous=p.offset_time
        if p.line<profile['input']['scan_lines'] and p.tag&0x30 in (0,0x10):accepted+=1
    if previous>profile['input']['max_scan_span_s']*1e9:raise ValueError('Scan span exceeds configured bound')
    if accepted<2:raise ValueError('FAST-LIO would reject essentially all channel/tag fields')
    return dict(stamp_ns=stamp,end_ns=stamp+previous,points=len(message.points),accepted_tag_line_points=accepted)


def inspect_imu(message,profile):
    if message.header.frame_id!=profile['input']['imu_frame']:raise ValueError('Unexpected raw IMU frame')
    stamp=stamp_ns(message.header.stamp)
    if profile['input']['angular_velocity_unit']!='rad_s':raise ValueError('Only rad/s gyro input is supported')
    a=np.array([message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z],dtype=float)
    w=np.array([message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z],dtype=float)
    if not np.isfinite(a).all() or not np.isfinite(w).all():raise ValueError('Non-finite IMU measurement')
    for covariance in (message.linear_acceleration_covariance,message.angular_velocity_covariance):
        if not np.isfinite(covariance).all() or covariance[0]==-1:raise ValueError('Missing or invalid acceleration/gyro covariance availability')
    return stamp,a*acceleration_scale(profile),w


def normalize_cloud(message,profile):
    from livox_ros_driver2.msg import CustomMsg
    inspect_cloud(message,profile)
    result=CustomMsg();result.header=deepcopy(message.header);result.header.frame_id=profile['output']['lidar_frame']
    result.timebase=message.timebase;result.point_num=message.point_num;result.lidar_id=message.lidar_id
    result.rsvd=message.rsvd;result.points=message.points
    return result


def normalize_imu(message,profile):
    from sensor_msgs.msg import Imu
    _,a,w=inspect_imu(message,profile);scale=acceleration_scale(profile)
    result=Imu();result.header=deepcopy(message.header);result.header.frame_id=profile['output']['imu_frame']
    result.angular_velocity=deepcopy(message.angular_velocity)
    result.angular_velocity_covariance=list(message.angular_velocity_covariance)
    result.linear_acceleration.x,result.linear_acceleration.y,result.linear_acceleration.z=map(float,a)
    result.linear_acceleration_covariance=[float(v*scale*scale) for v in message.linear_acceleration_covariance]
    # The native six-axis IMU does not supply an absolute attitude estimate.
    result.orientation.w=1.0;result.orientation_covariance[0]=-1.0
    return result
