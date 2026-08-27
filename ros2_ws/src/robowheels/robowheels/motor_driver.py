#!/usr/bin/env python3
"""RoboWheels differential-drive base controller (steps 4-5).

Subscribes cmd_vel (Twist) and drives the two AT8236 channels with
software PWM, samples the 4 encoder pins with 4x quadrature decoding,
integrates wheel odometry and publishes /odom plus odom->base_link TF.

Kinematics (per project spec):
    v_left  = v - omega * wheel_separation / 2
    v_right = v + omega * wheel_separation / 2

IO: register-level PIO access via fast_gpio.FastGPIO (/dev/mem mmap,
~3 us per pin op, register model copied from wiringOP). OPi.GPIO 0.5.2
is sysfs-based (~100-200 us per op) which made the phase-based PWM
loop alias (measured 11% vs 28% duty for the same 24% command and the
car did not move). With FastGPIO the hot loop runs at tens of kHz, so
200 Hz PWM duty is accurate and encoder edges are not missed.

Motor truth table (verified, HARDWARE.md):
    forward  = IN2 PWM, IN1 LOW   (left: PH6, right: PH5)
    backward = IN1 PWM, IN2 LOW   (left: PH4, right: PL10)
Encoders (verified, HARDWARE.md):
    left wheel  = PL2/PL3, right wheel = PD24/PD23 (cross-wired labels)
    forward counts positive; ticks/m (4x): left 1679, right 1656.

Safety:
  - all motor pins are driven LOW before their function is switched
    to output (see FastGPIO.setup) and stay OUTPUT LOW at shutdown
    (pins are never floated and never unexported, like wiringOP)
  - cmd_vel older than cmd_vel_timeout -> stop
"""
import faulthandler
import math
import signal
import threading
import time

from robowheels.fast_gpio import FastGPIO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


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

        # encoders (verified, from HARDWARE.md)
        self.enc_l_a = p('left_encoder_a', 'PL2').value
        self.enc_l_b = p('left_encoder_b', 'PL3').value
        self.enc_r_a = p('right_encoder_a', 'PD24').value
        self.enc_r_b = p('right_encoder_b', 'PD23').value
        self.tpm_left = p('left_ticks_per_meter', 1679.0).value
        self.tpm_right = p('right_ticks_per_meter', 1656.0).value
        self.enc_r_sign = -1.0 if p('right_encoder_inverted', False).value \
            else 1.0

        # odom
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_link').value
        self.odom_rate = p('odom_rate', 50.0).value

        # target wheel speeds (m/s), set from cmd_vel, read by hot loop
        self.v_left = 0.0
        self.v_right = 0.0
        self._lock = threading.Lock()
        self._last_cmd = time.time()

        # encoder counts since last odom tick + state, owned by hot loop
        self._left_count = 0
        self._right_count = 0
        self._enc_lock = threading.Lock()

        # GPIO: startup safe state FIRST (setup drives pins LOW before
        # switching them to output function, see FastGPIO.setup)
        self._gpio = FastGPIO()
        for pin in self.all_pins:
            self._gpio.setup(pin, 'out')
        for pin in (self.enc_l_a, self.enc_l_b, self.enc_r_a, self.enc_r_b):
            self._gpio.setup(pin, 'in')
        self.get_logger().info('motor pins OUTPUT LOW (coast)')

        # precomputed Pin objects for the hot loop
        self._l_fwd = self._gpio.pin(self.pin_l_fwd)
        self._l_rev = self._gpio.pin(self.pin_l_rev)
        self._r_fwd = self._gpio.pin(self.pin_r_fwd)
        self._r_rev = self._gpio.pin(self.pin_r_rev)
        self._la = self._gpio.pin(self.enc_l_a)
        self._lb = self._gpio.pin(self.enc_l_b)
        self._ra = self._gpio.pin(self.enc_r_a)
        self._rb = self._gpio.pin(self.enc_r_b)

        self._running = True
        self._hot_thread = threading.Thread(target=self._hot_loop,
                                            daemon=True)
        self._hot_thread.start()

        self._watchdog = self.create_timer(0.1, self._watchdog_cb)
        self._cmd_sub = self.create_subscription(
            Twist, cmd_topic, self._cmd_cb, 10)
        self._wheel_pub = self.create_publisher(
            Float32MultiArray, '/wheel_speeds', 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_timer = self.create_timer(1.0 / self.odom_rate,
                                             self._odom_cb)

        # odom pose state
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        self._last_odom = None

        self.get_logger().info(
            f'motor_driver ready: max {self.max_lin} m/s, '
            f'{self.max_ang} rad/s, pwm {self.pwm_freq} Hz, '
            f'max duty {self.max_pwm}%, odom {self.odom_rate} Hz')

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

    # ---- hot loop: software PWM + 4x quadrature ----

    def _v_to_duty(self, v):
        """Linear map wheel speed (m/s) -> duty percent (+/-)."""
        if abs(v) < self.deadband:
            return 0.0
        return max(min(v / self.max_lin * self.max_pwm,
                       self.max_pwm), -self.max_pwm)

    def _hot_loop(self):
        """Tight loop: 200 Hz phase-based soft PWM + 4x quadrature.

        Runs at tens of kHz thanks to register-level IO, so phase-based
        duty is accurate and quadrature transitions are not missed
        (max edge rate ~3.3 kHz when hand-pushing at 2 m/s).
        """
        period = 1.0 / self.pwm_freq
        l_fwd, l_rev = self._l_fwd, self._l_rev
        r_fwd, r_rev = self._r_fwd, self._r_rev
        la, lb, ra, rb = self._la, self._lb, self._ra, self._rb

        # 4x decode state: A<<1|B and previous B per wheel
        l_state = r_state = None
        l_prev_b = r_prev_b = 0
        l_cnt = r_cnt = 0
        t0 = time.time()
        log_t = 1.0  # elapsed-s boundary (now = time.time() - t0)
        iters = 0
        perf_t0 = time.perf_counter()
        last_perf = perf_t0
        max_gap = 0.0
        while self._running:
            now = time.time() - t0
            phase = now % period

            with self._lock:
                dl = self._v_to_duty(self.v_left)
                dr = self._v_to_duty(self.v_right)

            # left wheel: fwd pin PWM if dl>0, rev pin PWM if dl<0
            l_on = period * abs(dl) / 100.0
            l_fwd.out(1 if dl > 0 and phase < l_on else 0)
            l_rev.out(1 if dl < 0 and phase < l_on else 0)
            # right wheel
            r_on = period * abs(dr) / 100.0
            r_fwd.out(1 if dr > 0 and phase < r_on else 0)
            r_rev.out(1 if dr < 0 and phase < r_on else 0)

            # encoders: 4x decode, delta = (A_new^B_old) - (B_new^A_old)
            l_a, l_b = la.inp(), lb.inp()
            l_new = (l_a << 1) | l_b
            if l_state is not None and l_new != l_state:
                l_cnt += (l_a ^ l_prev_b) - (l_b ^ (l_state >> 1))
            l_state, l_prev_b = l_new, l_b

            r_a, r_b = ra.inp(), rb.inp()
            r_new = (r_a << 1) | r_b
            if r_state is not None and r_new != r_state:
                r_cnt += (r_a ^ r_prev_b) - (r_b ^ (r_state >> 1))
            r_state, r_prev_b = r_new, r_b

            # hand counts to the odom callback (rare: only on edges)
            if l_cnt or r_cnt:
                with self._enc_lock:
                    self._left_count += l_cnt
                    self._right_count += self.enc_r_sign * r_cnt
                l_cnt = r_cnt = 0

            iters += 1
            t_perf = time.perf_counter()
            if t_perf - last_perf > max_gap:
                max_gap = t_perf - last_perf
            last_perf = t_perf
            if now >= log_t:
                dtp = time.perf_counter() - perf_t0
                self.get_logger().info(
                    f'hot loop: {iters} iters in {dtp:.2f}s = '
                    f'{iters/dtp:.0f} Hz, max stall {max_gap*1000:.1f} ms')
                iters = 0
                max_gap = 0.0
                perf_t0 = time.perf_counter()
                log_t += 1.0

    # ---- odometry ----

    def _odom_cb(self):
        now = self.get_clock().now()
        with self._enc_lock:
            dl = self._left_count
            dr = self._right_count
            self._left_count = 0
            self._right_count = 0

        if self._last_odom is None:
            self._last_odom = now
            return
        dt = (now - self._last_odom).nanoseconds / 1e9
        self._last_odom = now
        if dt <= 0.0:
            return

        vl = dl / self.tpm_left / dt
        vr = dr / self.tpm_right / dt
        v = (vr + vl) / 2.0
        w = (vr - vl) / self.wheel_sep

        self._x += v * math.cos(self._th) * dt
        self._y += v * math.sin(self._th) * dt
        self._th += w * dt

        qz = math.sin(self._th / 2.0)
        qw = math.cos(self._th / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # covariances: measured-ish values, tune later
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.1
        self._odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)

    # ---- shutdown ----

    def shutdown(self):
        self._running = False
        if self._hot_thread.is_alive():
            self._hot_thread.join(timeout=1.0)
        # explicit safe state: pins stay OUTPUT and are driven LOW
        # (never unexported / never floated)
        for pin in (self._l_fwd, self._l_rev, self._r_fwd, self._r_rev):
            pin.out(0)
        self._gpio.close()
        self.get_logger().info('shutdown: motor pins LOW, io closed')


def main(args=None):
    faulthandler.enable()
    faulthandler.register(signal.SIGUSR1)
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
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
