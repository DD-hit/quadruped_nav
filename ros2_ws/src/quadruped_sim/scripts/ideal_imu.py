"""Ideal rigid-body IMU for transform-driven planar simulation (SI units)."""
import json
import math
import numpy as np


def rotation_rpy(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def profile_transform(spec):
    result = np.eye(4)
    result[:3, :3] = rotation_rpy(spec['rpy_rad'])
    result[:3, 3] = spec['translation_m']
    return result


def measurement(velocity, previous_velocity, dt, world_rotation, mount, gravity):
    """Return specific force and gyro in IMU axes, including lever-arm terms.

    Velocity components are expressed in the moving base frame. The backward
    difference is the average commanded acceleration over the physics interval;
    discontinuous watchdog/invalid-command stops consequently produce impulses.
    """
    vx, vy, wz = velocity
    delta = (np.asarray(velocity)-previous_velocity)/dt
    v, omega = np.array([vx, vy, 0.0]), np.array([0.0, 0.0, wz])
    dv, alpha = np.array([delta[0], delta[1], 0.0]), np.array([0.0, 0.0, delta[2]])
    r = mount[:3, 3]
    acceleration = dv+np.cross(omega, v)+np.cross(alpha, r)+np.cross(omega, np.cross(omega, r))
    rotation = mount[:3, :3]
    return rotation.T@(acceleration-world_rotation.T@gravity), rotation.T@omega


class IdealImu:
    def __init__(self, stage, node, config, profile, common, output):
        from pxr import Usd, UsdGeom
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String
        self.Imu, self.String = Imu, String
        self.node, self.config = node, config
        self.frame = common['frames']['imu']
        if profile['frame_aliases']['imu'] != self.frame:
            raise ValueError('IMU frame differs between robot and common profiles')
        base = stage.GetPrimAtPath('/World/Robot/'+profile['model']['target_root_frame'])
        sensor = stage.GetPrimAtPath('/World/Robot/'+self.frame)
        if not base.IsValid() or not sensor.IsValid():
            raise ValueError('Missing base/IMU USD frame')
        tb = np.array(UsdGeom.Xformable(base).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        ti = np.array(UsdGeom.Xformable(sensor).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        self.mount = np.linalg.inv(tb)@ti
        expected = profile_transform(profile['sensor']['base_to_lidar'])@profile_transform(profile['sensor']['lidar_to_imu'])
        self.mount_error = float(np.max(np.abs(self.mount-expected)))
        if self.mount_error > 1e-6:
            raise ValueError(f'IMU USD/profile mount mismatch: {self.mount_error}')
        self.gravity = np.array([0.0, 0.0, -float(config['gravity_mps2'])])
        self.rate = float(config['sample_hz'])
        self.publisher = node.create_publisher(Imu, common['interfaces']['imu']['topic'],
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.status_pub = node.create_publisher(String, config['status_topic'], 10)
        self.log = (output/'imu_samples.jsonl').open('w', buffering=1)
        self.reset()

    def reset_derivative(self):
        self.previous_velocity = np.zeros(3)

    def reset(self):
        self.reset_derivative()
        self.last_time = None
        self.index = 0

    def sample(self, timestamp, dt, pose, velocity):
        if not math.isfinite(dt) or abs(dt-1/self.rate) > 1e-7:
            raise RuntimeError(f'IMU requires one real physics step per sample, dt={dt}')
        if self.last_time is not None and abs(timestamp-self.last_time-dt) > 1e-7:
            raise RuntimeError('IMU simulation time skipped or repeated')
        force, gyro = measurement(velocity, self.previous_velocity, dt, pose[:3, :3], self.mount, self.gravity)
        message = self.Imu()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(round(timestamp*1e9), 1_000_000_000)
        message.header.frame_id = self.frame
        # The built-in six-axis IMU does not measure absolute orientation.
        message.orientation.w = 1.0
        message.orientation_covariance[0] = -1.0
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = map(float, force)
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = map(float, gyro)
        self.publisher.publish(message)
        self.log.write(json.dumps(dict(epoch=self.node.epoch, index=self.index, time_s=timestamp,
            dt_s=dt, specific_force_mps2=force.tolist(), angular_velocity_radps=gyro.tolist()))+'\n')
        if self.index % round(self.rate) == 0:
            self.status_pub.publish(self.String(data=json.dumps(dict(epoch=self.node.epoch,
                index=self.index, sim_time_s=timestamp, frame=self.frame, sample_hz=self.rate,
                mount_matrix=self.mount.tolist(), mount_profile_max_error=self.mount_error,
                gravity_world_mps2=self.gravity.tolist(), model=self.config['model'],
                orientation_available=False, noise_enabled=False, bias_enabled=False,
                timing='one_sample_per_actual_physics_step'))))
        self.previous_velocity = np.asarray(velocity).copy()
        self.last_time = timestamp
        self.index += 1

    def close(self):
        self.log.close()
