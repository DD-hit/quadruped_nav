"""Planar kinematics for the simulator, independent of ROS and Isaac."""
import math


class PlanarMotion:
    def __init__(self, config):
        self.limits = tuple(float(config['max_speed'][key]) for key in ('vx_mps', 'vy_mps', 'wz_radps'))
        self.acceleration = tuple(float(config['max_acceleration'][key]) for key in ('ax_mps2', 'ay_mps2', 'yaw_radps2'))
        self.timeout = float(config['command_timeout_wall_s'])
        if not all(math.isfinite(v) and v > 0 for v in self.limits+self.acceleration+(self.timeout,)):
            raise ValueError('Motion limits and timeout must be finite and positive')
        self.rejections = 0
        self.reset()

    def reset(self, paused=False):
        self.x = self.y = self.yaw = 0.0
        self.paused = paused
        self.clear_command()

    def clear_command(self):
        self.target = self.velocity = (0.0, 0.0, 0.0)
        self.last_received = None
        self.stop_reason = 'paused' if self.paused else 'awaiting_fresh_command'

    def set_paused(self, paused):
        self.paused = paused
        self.clear_command()

    def receive(self, command, now):
        if self.paused:
            return False
        if len(command) != 3 or not all(math.isfinite(v) for v in command):
            self.rejections += 1
            self.clear_command()
            return False
        self.target = tuple(max(-limit, min(limit, float(v))) for v, limit in zip(command, self.limits))
        self.last_received = now
        return True

    def advance(self, dt, now):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Simulation step must be finite and positive')
        if self.paused or self.last_received is None:
            self.velocity = (0.0, 0.0, 0.0)
            self.stop_reason = 'paused' if self.paused else 'awaiting_fresh_command'
            return
        if now-self.last_received >= self.timeout:
            self.velocity = self.target = (0.0, 0.0, 0.0)
            self.stop_reason = 'command_timeout'
            return
        self.stop_reason = None
        self.velocity = tuple(v+max(-a*dt, min(a*dt, target-v))
                              for v, target, a in zip(self.velocity, self.target, self.acceleration))
        vx, vy, wz = self.velocity
        end = self.yaw+wz*dt
        if abs(wz) < 1e-9:
            self.x += (math.cos(self.yaw)*vx-math.sin(self.yaw)*vy)*dt
            self.y += (math.sin(self.yaw)*vx+math.cos(self.yaw)*vy)*dt
        else:
            self.x += (vx*(math.sin(end)-math.sin(self.yaw))+vy*(math.cos(end)-math.cos(self.yaw)))/wz
            self.y += (vx*(math.cos(self.yaw)-math.cos(end))+vy*(math.sin(end)-math.sin(self.yaw)))/wz
        self.yaw = math.atan2(math.sin(end), math.cos(end))
