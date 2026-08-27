#!/usr/bin/env python3
"""Obstacle avoidance: /cmd_vel_joy + /scan + /imu/data_raw -> /cmd_vel.

Two modes (switched by the joystick hotkey via /robowheels/mode):
  manual: joystick command passes through unchanged (clamped only)
  avoid:  joystick command is shaped by the sensors:
          - lidar forward sector scales linear velocity down to 0 as an
            obstacle approaches (stop_dist .. slow_dist ramp)
          - lidar side sectors add a repulsive steering bias away from
            the closer side
          - driving backward checks the rear sector the same way
          - IMU tilt (roll/pitch from accel) beyond tilt_limit -> stop
          - IMU gyro z far beyond the commanded turn (unexpected spin,
            e.g. wheel slip after hitting something) -> stop
          - stale /scan, /imu or /cmd_vel_joy -> stop (timeouts)

The motor driver's own 0.5 s cmd_vel watchdog remains as the final
backstop if this node dies.
"""
import faulthandler
import math
import signal
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String


class ObstacleAvoid(Node):

    def __init__(self):
        super().__init__('obstacle_avoid')
        p = self.declare_parameter

        # sectors / gains
        self.fwd_angle = p('forward_angle', 0.52).value
        self.stop_dist = p('stop_dist', 0.15).value
        self.slow_dist = p('slow_dist', 0.50).value
        self.side_min = p('side_angle_min', 0.52).value
        self.side_max = p('side_angle_max', 1.40).value
        self.gain = p('repulsion_gain', 0.35).value
        self.max_bias = p('max_steer_bias', 1.5).value
        self.rear_angle = p('rear_angle', 1.05).value
        self.rear_stop = p('rear_stop_dist', 0.15).value
        self.tilt_limit = p('tilt_limit', 0.35).value
        self.spin_limit = p('spin_limit', 3.0).value
        self.max_lin = p('max_linear', 0.5).value
        self.max_ang = p('max_angular', 2.0).value
        self.t_scan = p('scan_timeout', 0.5).value
        self.t_imu = p('imu_timeout', 0.5).value
        self.t_joy = p('joy_timeout', 0.5).value

        self.mode = 'manual'
        self._last_joy = Twist()
        self._t_joy = 0.0
        self._scan = None
        self._t_scan = 0.0
        self._imu = None
        self._t_imu = 0.0
        self._warn_until = 0.0
        # gravity-direction calibration: mounting-agnostic tilt estimate,
        # average the first ~1 s of accelerometer samples (car must be
        # level at startup)
        self._g_acc = [0.0, 0.0, 0.0]
        self._g_n = 0
        self._g_hat = None

        self.create_subscription(
            Twist, p('joy_cmd_topic', '/cmd_vel_joy').value,
            self._joy_cb, 10)
        self.create_subscription(
            LaserScan, p('scan_topic', '/scan').value, self._scan_cb, 10)
        self.create_subscription(
            Imu, p('imu_topic', '/imu/data_raw').value, self._imu_cb, 10)
        self.create_subscription(
            String, p('mode_topic', '/robowheels/mode').value,
            self._mode_cb, 1)
        self._cmd_pub = self.create_publisher(
            Twist, p('cmd_out_topic', '/cmd_vel').value, 10)
        self.create_timer(1.0 / p('rate', 20.0).value, self._tick)
        self.get_logger().info(
            'obstacle_avoid ready: mode=manual (passthrough), '
            f'avoid: stop {self.stop_dist}m / slow {self.slow_dist}m, '
            f'tilt {self.tilt_limit:.2f}rad')

    # ---- subscriptions ----

    def _joy_cb(self, msg):
        self._last_joy = msg
        self._t_joy = time.time()

    def _scan_cb(self, msg):
        self._scan = msg
        self._t_scan = time.time()

    def _imu_cb(self, msg):
        self._imu = msg
        self._t_imu = time.time()
        if self._g_hat is None:
            self._g_acc[0] += msg.linear_acceleration.x
            self._g_acc[1] += msg.linear_acceleration.y
            self._g_acc[2] += msg.linear_acceleration.z
            self._g_n += 1
            if self._g_n >= 50:
                m = math.sqrt(sum(c * c for c in self._g_acc))
                if m > 1.0:
                    self._g_hat = [c / m for c in self._g_acc]
                    self.get_logger().info(
                        'gravity axis calibrated: '
                        f'({self._g_hat[0]:.2f}, {self._g_hat[1]:.2f}, '
                        f'{self._g_hat[2]:.2f})')
                else:
                    self._g_acc = [0.0, 0.0, 0.0]
                    self._g_n = 0

    def _mode_cb(self, msg):
        if msg.data != self.mode:
            self.get_logger().info(f'mode -> {msg.data}')
            self.mode = msg.data

    # ---- control ----

    def _tick(self):
        now = time.time()
        cmd = Twist()

        # clamp joystick command
        v = max(min(self._last_joy.linear.x, self.max_lin), -self.max_lin)
        w = max(min(self._last_joy.angular.z, self.max_ang), -self.max_ang)

        if self.mode == 'manual':
            if now - self._t_joy <= self.t_joy:
                cmd.linear.x = v
                cmd.angular.z = w
            else:
                self._warn('manual: no /cmd_vel_joy - stopping')
        else:
            cmd.linear.x, cmd.angular.z, reason = self._avoid(v, w, now)
            if reason:
                self._warn(f'avoid: {reason}')
        self._cmd_pub.publish(cmd)

    def _avoid(self, v, w, now):
        """Shape (v, w) with lidar + IMU. Returns (v_out, w_out, reason)."""
        if now - self._t_joy > self.t_joy:
            return 0.0, 0.0, 'no /cmd_vel_joy - stopping'
        if self._scan is None or now - self._t_scan > self.t_scan:
            return 0.0, 0.0, 'no fresh /scan - stopping'
        if self._imu is None or now - self._t_imu > self.t_imu:
            return 0.0, 0.0, 'no fresh /imu - stopping'

        # tilt: angle between measured accel and calibrated gravity axis
        ax = self._imu.linear_acceleration.x
        ay = self._imu.linear_acceleration.y
        az = self._imu.linear_acceleration.z
        if self._g_hat is None:
            return 0.0, 0.0, 'gravity axis not calibrated yet - stop'
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        if mag < 1.0:
            mag = 1.0
        c = (ax * self._g_hat[0] + ay * self._g_hat[1]
             + az * self._g_hat[2]) / mag
        tilt = math.acos(max(-1.0, min(1.0, c)))
        if tilt > self.tilt_limit:
            return 0.0, 0.0, f'tilt {math.degrees(tilt):.0f}deg - stop'

        d_f = self._min_dist(self._scan, 0.0, self.fwd_angle)
        d_r = self._min_dist(self._scan, -self.side_max, -self.side_min)
        d_l = self._min_dist(self._scan, self.side_min, self.side_max)

        # forward / rear speed scaling
        if v > 0.0:
            v_out = v * self._ramp(d_f, self.stop_dist, self.slow_dist)
        elif v < 0.0:
            d_b = self._min_dist(self._scan, math.pi - self.rear_angle,
                                 math.pi)
            v_out = v * self._ramp(d_b, self.rear_stop, self.slow_dist)
        else:
            v_out = 0.0

        # repulsive steering: turn away from the closer side
        w_bias = self.gain * (1.0 / max(d_l, 0.05) - 1.0 / max(d_r, 0.05))
        w_bias = max(min(w_bias, self.max_bias), -self.max_bias)
        w_out = w + w_bias

        # unexpected spin guard (gyro z far beyond commanded turn)
        gz = self._imu.angular_velocity.z
        if abs(gz) > self.spin_limit and abs(w) < 0.3:
            return 0.0, 0.0, f'spin gyro_z={gz:.2f} - stop'

        return v_out, w_out, None

    @staticmethod
    def _ramp(d, d_stop, d_slow):
        if d <= d_stop:
            return 0.0
        if d >= d_slow:
            return 1.0
        return (d - d_stop) / (d_slow - d_stop)

    def _min_dist(self, scan, a0, a1):
        """Minimum valid range over angle window [a0, a1] (rad)."""
        best = math.inf
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min:
                continue
            a = scan.angle_min + i * scan.angle_increment
            if a0 <= a <= a1:
                if r < best:
                    best = r
        return best

    def _warn(self, text):
        now = time.time()
        if now >= self._warn_until:
            self.get_logger().warn(text)
            self._warn_until = now + 1.0


def main(args=None):
    faulthandler.enable()
    faulthandler.register(signal.SIGUSR1)
    rclpy.init(args=args)
    node = ObstacleAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        _shutdown_quiet()


def _shutdown_quiet():
    """rclpy.shutdown() without the cosmetic double-shutdown RCLError."""
    try:
        rclpy.shutdown()
    except rclpy._rclpy_pybind11.RCLError:
        pass


if __name__ == '__main__':
    main()
