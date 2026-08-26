#!/usr/bin/env python3
"""RoboWheels differential-drive base controller (step 4).

Subscribes cmd_vel (Twist) and drives the two AT8236 channels with
software PWM via OPi.GPIO (SUNXI mode).

Kinematics (per project spec):
    v_left  = v - omega * wheel_separation / 2
    v_right = v + omega * wheel_separation / 2

Safety:
  - all motor pins OUTPUT LOW at startup (coast)
  - cmd_vel older than cmd_vel_timeout -> stop
  - on shutdown: pins explicitly set LOW, then left that way
    (GPIO.cleanup() is NOT called: it would switch pins to INPUT and
    floating IN pins on the AT8236 could drive the motors)

Motor truth table (verified, HARDWARE.md):
    forward  = IN2 PWM, IN1 LOW
    backward = IN1 PWM, IN2 LOW
"""
import math
import time
import threading

import OPi.GPIO as GPIO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')
        p = self.declare_parameter

        # geometry
        self.wheel_radius = p('wheel_radius', 0.0318).value
        self.wheel_sep = p('wheel_separation', 0.18).value
        self.ticks_per_rev = p('encoder_ticks_per_revolution', 334).value
        self.gear_ratio = p('gear_ratio', 1.0).value

        # limits
        self.max_pwm = p('max_pwm', 40).value
        self.max_lin = p('max_linear_velocity', 0.5).value
        self.max_ang = p('max_angular_velocity', 2.0).value
        self.deadband = p('linear_deadband', 0.01).value

        # control
        self.pwm_freq = p('pwm_frequency', 200).value
        self.timeout = p('cmd_vel_timeout', 0.5).value
        cmd_topic = p('cmd_vel_topic', '/cmd_vel').value

        # pins (verified, from HARDWARE.md)
        self.pin_l_fwd = p('left_motor_fwd', 'PH6').value
        self.pin_l_rev = p('left_motor_rev', 'PH4').value
        self.pin_r_fwd = p('right_motor_fwd', 'PH5').value
        self.pin_r_rev = p('right_motor_rev', 'PL10').value
        self.all_pins = (self.pin_l_fwd, self.pin_l_rev,
                         self.pin_r_fwd, self.pin_r_rev)

        # target wheel speeds (m/s), set from cmd_vel, read by PWM thread
        self.v_left = 0.0
        self.v_right = 0.0
        self._lock = threading.Lock()
        self._last_cmd = time.time()

        # GPIO: startup safe state FIRST
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.SUNXI)
        for pin in self.all_pins:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        self.get_logger().info('motor pins OUTPUT LOW (coast)')

        self._running = True
        self._pwm_thread = threading.Thread(target=self._pwm_loop,
                                            daemon=True)
        self._pwm_thread.start()

        self._watchdog = self.create_timer(0.1, self._watchdog_cb)
        self._cmd_sub = self.create_subscription(
            Twist, cmd_topic, self._cmd_cb, 10)
        self._wheel_pub = self.create_publisher(
            Float32MultiArray, '/wheel_speeds', 10)

        self.get_logger().info(
            f'motor_driver ready: max {self.max_lin} m/s, '
            f'{self.max_ang} rad/s, pwm {self.pwm_freq} Hz, '
            f'max duty {self.max_pwm}%')

    # ---- cmd_vel ----

    def _cmd_cb(self, msg):
        v = max(min(msg.linear.x, self.max_lin), -self.max_lin)
        w = max(min(msg.angular.z, self.max_ang), -self.max_ang)
        vl = v - w * self.wheel_sep / 2.0
        vr = v + w * self.wheel_sep / 2.0
        with self._lock:
            self.v_left = vl
            self.v_right = vr
            self._last_cmd = time.time()

    def _watchdog_cb(self):
        if time.time() - self._last_cmd > self.timeout:
            with self._lock:
                self.v_left = 0.0
                self.v_right = 0.0
        with self._lock:
            msg = Float32MultiArray(data=[self.v_left, self.v_right])
        self._wheel_pub.publish(msg)

    # ---- PWM thread ----

    def _v_to_duty(self, v):
        """Linear map wheel speed (m/s) -> duty percent (+/-)."""
        if abs(v) < self.deadband:
            return 0.0
        return max(min(v / self.max_lin * self.max_pwm,
                       self.max_pwm), -self.max_pwm)

    def _pwm_loop(self):
        """Tight loop: 200 Hz soft PWM on up to 4 pins, phase-based."""
        period = 1.0 / self.pwm_freq
        # [on_period, pin, duty] per wheel
        left = [0.0, self.pin_l_fwd, 0.0]
        right = [0.0, self.pin_r_fwd, 0.0]
        t0 = time.time()
        while self._running:
            now = time.time() - t0
            phase = now % period

            with self._lock:
                dl = self._v_to_duty(self.v_left)
                dr = self._v_to_duty(self.v_right)

            # left wheel
            if dl >= 0:
                l_pin, l_on = self.pin_l_fwd, period * dl / 100.0
            else:
                l_pin, l_on = self.pin_l_rev, period * -dl / 100.0
            # right wheel
            if dr >= 0:
                r_pin, r_on = self.pin_r_fwd, period * dr / 100.0
            else:
                r_pin, r_on = self.pin_r_rev, period * -dr / 100.0

            # set levels (only touch pins whose state changed)
            for pin, on in ((l_pin, l_on), (r_pin, r_on)):
                GPIO.output(pin, GPIO.HIGH if phase < on else GPIO.LOW)
            for pin in self.all_pins:
                if pin != l_pin and pin != r_pin:
                    GPIO.output(pin, GPIO.LOW)

    # ---- shutdown ----

    def shutdown(self):
        self._running = False
        if self._pwm_thread.is_alive():
            self._pwm_thread.join(timeout=1.0)
        # explicit safe state; do NOT GPIO.cleanup() (see module docstring)
        for pin in self.all_pins:
            GPIO.output(pin, GPIO.LOW)
        self.get_logger().info('shutdown: motor pins LOW')


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
