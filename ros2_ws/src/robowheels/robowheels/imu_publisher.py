#!/usr/bin/env python3
"""MPU6050 IMU publisher: i2c-0 @ 0x68 -> /imu/data_raw (sensor_msgs/Imu).

Sensor setup (all values verified 2026-08-26/27, see HARDWARE.md):
  - PWR_MGMT_1 = 0x01 : wake, PLL on X gyro
  - CONFIG      = 0x02 : DLPF 98 Hz (accel + gyro), sample rate 1 kHz
  - GYRO_CONFIG = 0x00 : +-250 deg/s, 131 LSB/(deg/s)
  - ACCEL_CONFIG= 0x00 : +-2 g, 16384 LSB/g

Orientation covariance is set unknown (-1): MPU6050 has no magnetometer,
so heading drifts; consumers (obstacle_avoid) use raw gyro + tilt only.
"""
import faulthandler
import math
import signal
import time

import smbus2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        p = self.declare_parameter
        bus_no = p('i2c_bus', 0).value
        addr = p('i2c_address', 0x68).value
        self.rate = p('rate', 50.0).value
        frame_id = p('frame_id', 'imu_link').value

        self._bus_no = bus_no
        self._addr = addr
        self._frame_id = frame_id
        self.dlpf = p('dlpf_config', 0x02).value
        self.gyro_cfg = p('gyro_range', 0x00).value
        self.accel_cfg = p('accel_range', 0x00).value
        self._bus = smbus2.SMBus(bus_no)
        self._init_sensor()

        self._pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self._errs = 0
        self._err_log_until = 0.0
        self.create_timer(1.0 / self.rate, self._tick)

    def _init_sensor(self):
        """Wake + configure the MPU6050. Called at startup and again
        after a bus error (i2c is flaky with dupont wires)."""
        self._bus.write_byte_data(self._addr, 0x6B, 0x01)  # PWR_MGMT_1
        self._bus.write_byte_data(self._addr, 0x19, 0x00)  # SMPLRT_DIV
        self._bus.write_byte_data(self._addr, 0x1A, self.dlpf)
        self._bus.write_byte_data(self._addr, 0x1B, self.gyro_cfg)
        self._bus.write_byte_data(self._addr, 0x1C, self.accel_cfg)
        who = self._bus.read_byte_data(self._addr, 0x75)
        self.get_logger().info(
            f'MPU6050 @ i2c-{self._bus_no} 0x{self._addr:02x} '
            f'WHO_AM_I=0x{who:02x}, {self.rate} Hz')

    def _tick(self):
        # one 14-byte block: accel xyz (6) + temp (2) + gyro xyz (6)
        try:
            blk = self._bus.read_i2c_block_data(self._addr, 0x3B, 14)
        except OSError:
            self._errs += 1
            # drop this sample; re-init the sensor after the first error
            if self._errs == 1:
                self.get_logger().error(
                    'i2c read failed - re-initializing sensor '
                    '(check MPU6050 wiring)')
                try:
                    self._bus.close()
                except OSError:
                    pass
                self._bus = smbus2.SMBus(self._bus_no)
                try:
                    self._init_sensor()
                except OSError:
                    pass
            if time.time() >= self._err_log_until:
                self.get_logger().error(
                    f'i2c read failing ({self._errs} errors so far)')
                self._err_log_until = time.time() + 5.0
            return
        ax, ay, az = _twos(blk, 0), _twos(blk, 2), _twos(blk, 4)
        gx, gy, gz = _twos(blk, 8), _twos(blk, 10), _twos(blk, 12)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        # orientation unknown (no magnetometer)
        msg.orientation_covariance[0] = -1.0
        # +-250 deg/s, 131 LSB/(deg/s) -> rad/s
        msg.angular_velocity.x = gx / 131.0 * math.pi / 180.0
        msg.angular_velocity.y = gy / 131.0 * math.pi / 180.0
        msg.angular_velocity.z = gz / 131.0 * math.pi / 180.0
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        # +-2 g, 16384 LSB/g -> m/s^2
        msg.linear_acceleration.x = ax / 16384.0 * 9.80665
        msg.linear_acceleration.y = ay / 16384.0 * 9.80665
        msg.linear_acceleration.z = az / 16384.0 * 9.80665
        msg.linear_acceleration_covariance[0] = 0.05
        msg.linear_acceleration_covariance[4] = 0.05
        msg.linear_acceleration_covariance[8] = 0.05
        self._pub.publish(msg)
        self._errs = 0


def _twos(blk, i):
    """Two's-complement big-endian 16-bit value from byte block."""
    v = (blk[i] << 8) | blk[i + 1]
    return v - 65536 if v >= 32768 else v


def main(args=None):
    faulthandler.enable()
    faulthandler.register(signal.SIGUSR1)
    rclpy.init(args=args)
    node = ImuPublisher()
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
