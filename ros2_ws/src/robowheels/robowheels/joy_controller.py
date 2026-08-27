#!/usr/bin/env python3
"""Joystick controller: /joy -> /cmd_vel_joy + mode hotkey.

Xbox 360 controller defaults (all indices configurable in control.yaml):
  - drive:  left stick Y = linear, left stick X = angular (arcade style)
  - deadman: RB held = drive; released = immediate stop publish
  - mode:    A press toggles "manual" <-> "avoid", published latched on
             /robowheels/mode (std_msgs/String)

Safety: if /joy stops arriving (controller unplugged / joy node dies),
the watchdog publishes zero speed.
"""
import faulthandler
import signal
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class JoyController(Node):

    def __init__(self):
        super().__init__('joy_controller')
        p = self.declare_parameter

        self.axis_lin = p('axis_linear', 1).value
        self.axis_ang = p('axis_angular', 0).value
        self.inv_lin = -1.0 if p('invert_linear', True).value else 1.0
        self.inv_ang = -1.0 if p('invert_angular', False).value else 1.0
        self.scale_lin = p('scale_linear', 0.5).value
        self.scale_ang = p('scale_angular', 2.0).value
        self.deadzone = p('deadzone', 0.05).value
        self.enable_btn = p('enable_button', 5).value
        self.mode_btn = p('mode_button', 0).value
        self.timeout = p('joy_timeout', 0.5).value

        self.mode = p('default_mode', 'manual').value
        self._prev_mode_btn = 0
        self._last_joy = time.time()
        self._warned = False

        self._cmd_pub = self.create_publisher(
            Twist, p('cmd_out_topic', '/cmd_vel_joy').value, 10)
        # latched: late subscribers (obstacle_avoid) get current mode
        mode_qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self._mode_pub = self.create_publisher(
            String, p('mode_topic', '/robowheels/mode').value, mode_qos)
        self.create_subscription(
            Joy, p('joy_topic', '/joy').value, self._joy_cb, 10)
        self.create_timer(0.1, self._watchdog_cb)

        self._publish_mode()
        self.get_logger().info(
            f'joy_controller ready: deadman=button {self.enable_btn}, '
            f'mode=button {self.mode_btn}, mode={self.mode}')

    def _joy_cb(self, msg):
        self._last_joy = time.time()
        self._warned = False

        # mode hotkey: edge-triggered on press
        pressed = 0
        if self.mode_btn < len(msg.buttons):
            pressed = msg.buttons[self.mode_btn]
        if pressed and not self._prev_mode_btn:
            self.mode = 'avoid' if self.mode == 'manual' else 'manual'
            self._publish_mode()
            self.get_logger().info(f'mode switched to: {self.mode}')
        self._prev_mode_btn = pressed

        cmd = Twist()
        enabled = (self.enable_btn < len(msg.buttons)
                   and msg.buttons[self.enable_btn] == 1)
        if enabled:
            lin = self._axis(msg, self.axis_lin) * self.inv_lin * self.scale_lin
            ang = self._axis(msg, self.axis_ang) * self.inv_ang * self.scale_ang
            cmd.linear.x = lin
            cmd.angular.z = ang
        self._cmd_pub.publish(cmd)

    def _axis(self, msg, idx):
        if idx >= len(msg.axes):
            return 0.0
        v = msg.axes[idx]
        return 0.0 if abs(v) < self.deadzone else v

    def _watchdog_cb(self):
        if time.time() - self._last_joy > self.timeout:
            if not self._warned:
                self.get_logger().warn(
                    'no /joy messages - publishing stop (controller lost?)')
                self._warned = True
            self._cmd_pub.publish(Twist())

    def _publish_mode(self):
        self._mode_pub.publish(String(data=self.mode))


def main(args=None):
    faulthandler.enable()
    faulthandler.register(signal.SIGUSR1)
    rclpy.init(args=args)
    node = JoyController()
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
