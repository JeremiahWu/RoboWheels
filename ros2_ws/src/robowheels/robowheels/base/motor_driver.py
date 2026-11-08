#!/usr/bin/env python3
"""RoboWheels differential-drive base controller for RDK X5.

Subscribes cmd_vel (Twist), drives the two AT8236 channels and reads the
two quadrature encoders, publishes odometry + TF + wheel speeds.

Hardware (RDK X5, BOARD numbering, verified 2026-09-10):
    motors    AIN1=27  AIN2=29  BIN1=31  BIN2=33   (AT8236)
    encoders  E1A=11   E1B=13   E2A=15   E2B=16   (quadrature)

Motor backend:
    Default: hardware PWM via sysfs; stop disables both inputs at zero duty.
    GPIO software PWM is available only when the pin mux is configured for GPIO.
    Encoders default to queued GPIO v2 events; legacy sysfs polling is optional.

Vehicle forward (verified on X5 2026-11-08):
    right/A = IN2 PWM, IN1 LOW
    left/B  = IN1 PWM, IN2 LOW
    Reverse uses the other input.
    Both LOW = coast.  There is no STBY pin.

Safety:
  - all motor pins OUTPUT LOW at startup (coast)
  - cmd_vel older than cmd_vel_timeout -> stop
  - on shutdown pins are explicitly set LOW and left that way;
    GPIO.cleanup() is NOT called (it would switch the pins back to
    floating inputs and the AT8236 could drive the motors)

The channel<->wheel and encoder<->wheel mappings are configurable
(YAML).  Verified on X5: A=right, B=left; E1=left/B, E2=right/A.
Distance calibration remains unverified on X5.
"""
import math
import os
import threading
import time

import Hobot.GPIO as GPIO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Int64MultiArray, Float64MultiArray
from robowheels.base.hardware_pwm import MotorPWM
from robowheels.base.encoder_events import EncoderEvents
from robowheels.base.differential_drive import DifferentialDrive
from tf2_ros import TransformBroadcaster

# quadrature state machine, 4x decode.
# states are (A << 1) | B; forward order is 0 -> 1 -> 3 -> 2 -> 0
_QUAD_TRANS = {
    (0, 1): 1, (1, 3): 1, (3, 2): 1, (2, 0): 1,
    (0, 2): -1, (2, 3): -1, (3, 1): -1, (1, 0): -1,
}


def _sysfs_gpio_num(pin):
    """Global sysfs gpio number for a BOARD pin (from Hobot.GPIO data)."""
    return int(GPIO.all_pin_data['BOARD'][int(pin)].gpio_id)


class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')
        p = self.declare_parameter

        # ---- geometry / calibration ----
        self.wheel_sep = p('wheel_separation', 0.15631239107433037).value
        self.ticks_l = p('ticks_per_meter_left', 0.0).value
        self.ticks_r = p('ticks_per_meter_right', 0.0).value
        self.sign_l = p('encoder_sign_left', 1).value
        self.sign_r = p('encoder_sign_right', 1).value

        # ---- limits / control ----
        self.max_pwm = p('max_pwm', 20).value
        self.motor_backend = p('motor_backend', 'hardware').value
        self.encoder_backend = p('encoder_backend', 'events').value
        if self.encoder_backend not in ('events', 'polling'):
            raise ValueError('encoder_backend must be events or polling')
        if self.motor_backend not in ('hardware', 'gpio'):
            raise ValueError('motor_backend must be hardware or gpio')
        self.wheel_radius = float(p('wheel_radius', 0.0332).value)
        self.model = DifferentialDrive(self.wheel_radius, self.wheel_sep)
        n_left = float(p('encoder_ticks_per_revolution_left', 0.0).value)
        n_right = float(p('encoder_ticks_per_revolution_right', 0.0).value)
        if n_left > 0 and n_right > 0:
            self.ticks_l = self.model.ticks_per_meter(n_left)
            self.ticks_r = self.model.ticks_per_meter(n_right)
        if self.ticks_l <= 0 or self.ticks_r <= 0:
            raise ValueError('Positive measured encoder resolution is required')
        self.calibration_status = p('encoder_calibration_status', 'unverified').value
        self.max_lin = p('max_linear_velocity', 0.3).value
        self.max_wheel_speed = float(p('max_wheel_linear_velocity', 0.3).value)
        self.max_wheel_accel = float(p('max_wheel_acceleration', 0.5).value)
        self._limited_targets = {'left': 0.0, 'right': 0.0}
        self.max_ang = p('max_angular_velocity', 2.0).value
        self.deadband = p('linear_deadband', 0.01).value
        self.pwm_freq = p('pwm_frequency', 200).value
        self.timeout = p('cmd_vel_timeout', 0.5).value
        cmd_topic = p('cmd_vel_topic', '/cmd_vel').value
        self.use_pid = p('use_pid', False).value
        self.kp = p('pid_kp', 20.0).value
        self.ki = p('pid_ki', 0.0).value

        # ---- frames / topics ----
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.publish_tf = p('publish_tf', True).value

        # ---- pins (BOARD numbering) ----
        self.motor_pins = {
            'ain1': int(p('motor_pin_ain1', 27).value),
            'ain2': int(p('motor_pin_ain2', 29).value),
            'bin1': int(p('motor_pin_bin1', 31).value),
            'bin2': int(p('motor_pin_bin2', 33).value),
        }
        self.enc_pins = {
            'e1a': int(p('encoder_pin_e1a', 11).value),
            'e1b': int(p('encoder_pin_e1b', 13).value),
            'e2a': int(p('encoder_pin_e2a', 15).value),
            'e2b': int(p('encoder_pin_e2b', 16).value),
        }

        # channel A/B -> wheel ('left'/'right') and which IN gets PWM for fwd
        ch_a_wheel = p('channel_a_wheel', 'right').value
        ch_b_wheel = p('channel_b_wheel', 'left').value
        fwd_a = int(p('channel_a_forward_input', 2).value)
        fwd_b = int(p('channel_b_forward_input', 2).value)
        enc1_wheel = p('encoder1_wheel', 'left').value
        enc2_wheel = p('encoder2_wheel', 'right').value
        assert ch_a_wheel != ch_b_wheel, 'channel wheels must differ'
        assert enc1_wheel != enc2_wheel, 'encoder wheels must differ'

        # resolve wheels -> pins
        self.wheel_pins = {}
        self.wheel_pins[ch_a_wheel] = {
            'in1': self.motor_pins['ain1'],
            'in2': self.motor_pins['ain2'],
            'fwd_in': fwd_a,
        }
        self.wheel_pins[ch_b_wheel] = {
            'in1': self.motor_pins['bin1'],
            'in2': self.motor_pins['bin2'],
            'fwd_in': fwd_b,
        }
        # encoders -> wheels
        self.wheel_enc = {enc1_wheel: ('e1a', 'e1b'),
                          enc2_wheel: ('e2a', 'e2b')}
        self.wheel_ticks = {'left': self.ticks_l, 'right': self.ticks_r}
        self.wheel_sign = {'left': self.sign_l, 'right': self.sign_r}
        self.joint_names = {'left': 'left_wheel_joint',
                            'right': 'right_wheel_joint'}

        # ---- state ----
        self._lock = threading.Lock()
        self._v_target = {'left': 0.0, 'right': 0.0}   # from cmd_vel
        self._duty = {'left': 0.0, 'right': 0.0}       # signed duty %
        self._counts = {'left': 0, 'right': 0}         # cumulative ticks
        self._invalid = {'left': 0, 'right': 0}
        self._last_cmd = time.monotonic()
        self._integ = {'left': 0.0, 'right': 0.0}      # PI integral
        self._running = True

        # pose (odom)
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        self._last_odom = time.time()
        self._prev_counts = {'left': 0, 'right': 0}

        # ---- GPIO: motors OUTPUT LOW first (safe state) ----
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        self._hardware_pwm = None
        if self.motor_backend == 'hardware':
            self._hardware_pwm = MotorPWM(self.motor_pins.values(), self.pwm_freq)
        else:
            for pin in self.motor_pins.values():
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        self._encoder_events = None
        if self.encoder_backend == 'events':
            if self.motor_backend != 'hardware':
                raise ValueError('Event encoders require hardware motor PWM')
            self._encoder_events = EncoderEvents(list(self.enc_pins.values()))
        else:
            for pin in self.enc_pins.values():
                try:
                    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                except (AttributeError, ValueError, RuntimeError):
                    GPIO.setup(pin, GPIO.IN)
        self.get_logger().info('motor pins OUTPUT LOW (coast); encoders IN')

        # ---- fast sysfs fds ----
        self._out_fds = {}
        if self.motor_backend == 'gpio':
            for pin in self.motor_pins.values():
                path = '/sys/class/gpio/gpio%d/value' % _sysfs_gpio_num(pin)
                self._out_fds[pin] = os.open(path, os.O_WRONLY)
        # encoder fds keyed by pin NAME (e1a/e1b/e2a/e2b)
        self._in_fds = {}
        if self.encoder_backend == 'polling':
            for name, pin in self.enc_pins.items():
                path = '/sys/class/gpio/gpio%d/value' % _sysfs_gpio_num(pin)
                self._in_fds[name] = os.open(path, os.O_RDONLY)

        # pin levels cache (for change-detection writes)
        self._levels = {pin: 0 for pin in self.motor_pins.values()}
        # encoder pin states: pin -> last level
        self._enc_state = {}
        for wheel, (pa, pb) in self.wheel_enc.items():
            self._enc_state[wheel] = {
                'last': self._read_pair(pa, pb) if self._encoder_events is None else 0,
                'pins': (pa, pb),
            }

        # ---- ROS ----
        self._watchdog = self.create_timer(0.1, self._watchdog_cb)
        self._odom_timer = self.create_timer(0.05, self._odom_cb)
        self._cmd_sub = self.create_subscription(
            Twist, cmd_topic, self._cmd_cb, 10)
        self._target_pub = self.create_publisher(Float32MultiArray, '/wheel_speed_targets', 10)
        self._wheel_pub = self.create_publisher(
            Float32MultiArray, '/wheel_speeds', 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._raw_enc_pub = self.create_publisher(Int64MultiArray, '/rdk_debug/encoder_counts', 10)
        self._raw_rate_pub = self.create_publisher(Float64MultiArray, '/rdk_debug/encoder_rates', 10)
        self._invalid_pub = self.create_publisher(Int64MultiArray, '/rdk_debug/encoder_invalid', 10)
        self._drops_pub = self.create_publisher(Int64MultiArray, '/rdk_debug/encoder_event_drops', 10)
        self._event_drops = 0
        self._encoder_order = (enc1_wheel, enc2_wheel)
        self._raw_previous = [0, 0]
        self._tf_pub = TransformBroadcaster(self)

        # ---- control loop thread ----
        self._ctrl_thread = threading.Thread(target=self._ctrl_loop,
                                             daemon=True)
        self._ctrl_thread.start()

        self.get_logger().info(
            f'motor_driver ready: pins AIN1/2={self.motor_pins["ain1"]}/'
            f'{self.motor_pins["ain2"]} BIN1/2={self.motor_pins["bin1"]}/'
            f'{self.motor_pins["bin2"]}, enc E1A/B={self.enc_pins["e1a"]}/'
            f'{self.enc_pins["e1b"]} E2A/B={self.enc_pins["e2a"]}/'
            f'{self.enc_pins["e2b"]}, max {self.max_lin} m/s '
            f'{self.max_ang} rad/s, {self.motor_backend} PWM {self.pwm_freq} Hz '
            f'max {self.max_pwm}%')

    # ---- helpers ----

    def _read_pair(self, pa, pb):
        a = os.pread(self._in_fds[pa], 8, 0)[0] == 0x31
        b = os.pread(self._in_fds[pb], 8, 0)[0] == 0x31
        return (a << 1) | b

    def _write_pin(self, pin, level):
        if self._levels[pin] == level:
            return
        os.pwrite(self._out_fds[pin], b'1' if level else b'0', 0)
        self._levels[pin] = level

    def _v_to_duty(self, v):
        if abs(v) < self.deadband:
            return 0.0
        return max(min(v / self.max_lin * self.max_pwm,
                       self.max_pwm), -self.max_pwm)

    # ---- cmd_vel ----

    def _cmd_cb(self, msg):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            with self._lock:
                self._v_target = {'left': 0.0, 'right': 0.0}
                self._duty = {'left': 0.0, 'right': 0.0}
            return
        v = max(min(msg.linear.x, self.max_lin), -self.max_lin)
        w = max(min(msg.angular.z, self.max_ang), -self.max_ang)
        with self._lock:
            left,right=self.model.inverse(v,w,self.max_wheel_speed)
            self._v_target['left'] = left
            self._v_target['right'] = right
            self._last_cmd = time.monotonic()

    def _watchdog_cb(self):
        with self._lock:
            if time.monotonic() - self._last_cmd > self.timeout:
                self._v_target['left'] = 0.0
                self._v_target['right'] = 0.0

    # ---- odometry (50 Hz) ----

    def _odom_cb(self):
        now = time.time()
        with self._lock:
            dt = now - self._last_odom
            self._last_odom = now
            counts = dict(self._counts)
            invalid = dict(self._invalid)
            targets = dict(self._v_target)
        dt = max(dt, 1e-6)

        # wheel speeds from cumulative counts (delta over dt)
        meas = {}
        for wheel in ('left', 'right'):
            d = counts[wheel] - self._prev_counts[wheel]
            self._prev_counts[wheel] = counts[wheel]
            meas[wheel] = d / dt / self.wheel_ticks[wheel] * self.wheel_sign[wheel]

        # Wheel acceleration limiter, preserving the relative target change.
        # Explicit zero commands stop immediately; safety stop is not ramped.
        if all(abs(targets[w]) < 1e-12 for w in ('left','right')):
            self._limited_targets = {'left': 0.0, 'right': 0.0}
        else:
            change = max(abs(targets[w]-self._limited_targets[w]) for w in ('left','right'))
            fraction = min(1.0, self.max_wheel_accel*dt/max(change,1e-12))
            for w in ('left','right'):
                self._limited_targets[w] += fraction*(targets[w]-self._limited_targets[w])
        targets = dict(self._limited_targets)
        self._target_pub.publish(Float32MultiArray(data=[targets['left'],targets['right']]))
        # optional PI feedback on wheel speed
        for wheel in ('left', 'right'):
            duty = self._v_to_duty(targets[wheel])
            if abs(targets[wheel]) < self.deadband:
                self._integ[wheel] = 0.0
                duty = 0.0
            elif self.use_pid:
                err = targets[wheel] - meas[wheel]
                self._integ[wheel] = max(min(
                    self._integ[wheel] + err * dt * self.ki, 25.0), -25.0)
                duty = max(min(duty + self.kp * err + self._integ[wheel],
                               self.max_pwm), -self.max_pwm)
            with self._lock:
                self._duty[wheel] = duty

        # Exact planar differential-drive integration, matching simulation geometry.
        vl = meas['left']; vr = meas['right']
        v,w = self.model.forward(vl,vr)
        self._x,self._y,self._th = self.model.integrate(
            (self._x,self._y,self._th),vl*dt,vr*dt)

        # publish
        now = self.get_clock().now()
        # wheel_speeds [left, right] m/s
        self._wheel_pub.publish(
            Float32MultiArray(data=[vl, vr]))

        # odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        q = self._yaw_to_quat(self._th)
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self._odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation = q
            self._tf_pub.sendTransform(t)

        # joint states
        js = JointState()
        js.header.stamp = now.to_msg()
        for wheel in ('left', 'right'):
            js.name.append(self.joint_names[wheel])
            pos = counts[wheel] / self.wheel_ticks[wheel] * self.wheel_sign[wheel]
            js.position.append(pos / self.wheel_radius)  # radians
            js.velocity.append(meas[wheel] / self.wheel_radius)  # rad/s
        self._joint_pub.publish(js)
        # Preserve the read-only monitor's raw electrical sign and E1/E2 order.
        raw = [-counts[wheel] for wheel in self._encoder_order]
        self._raw_enc_pub.publish(Int64MultiArray(data=raw))
        self._raw_rate_pub.publish(Float64MultiArray(data=[(raw[i]-self._raw_previous[i])/dt for i in (0,1)]))
        self._invalid_pub.publish(Int64MultiArray(data=[invalid[wheel] for wheel in self._encoder_order]))
        self._drops_pub.publish(Int64MultiArray(data=[self._event_drops]))
        self._raw_previous = raw

    @staticmethod
    def _yaw_to_quat(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    # ---- control loop thread: software PWM + encoder sampling ----

    def _ctrl_loop(self):
        period = 1.0 / self.pwm_freq
        t0 = time.time()
        try:
            while self._running:
                now = time.time() - t0
                phase = now % period

                with self._lock:
                    duty = dict(self._duty)
                    if time.monotonic() - self._last_cmd > self.timeout:
                        duty = {'left': 0.0, 'right': 0.0}

                # drive wheels
                for wheel in ('left', 'right'):
                    cfg = self.wheel_pins[wheel]
                    in1, in2 = cfg['in1'], cfg['in2']
                    d = duty[wheel]
                    if self._hardware_pwm is not None:
                        self._hardware_pwm.drive(in1, in2, d, cfg['fwd_in'])
                        continue
                    if d > 0:
                        pwm_pin = in2 if cfg['fwd_in'] == 2 else in1
                    elif d < 0:
                        pwm_pin = in1 if cfg['fwd_in'] == 2 else in2
                        d = -d
                    else:
                        self._write_pin(in1, 0)
                        self._write_pin(in2, 0)
                        continue
                    on = period * d / 100.0
                    level = 1 if phase < on else 0
                    self._write_pin(pwm_pin, level)
                    self._write_pin(in1 if pwm_pin == in2 else in2, 0)

                if self._encoder_events is not None:
                    counts,invalid,dropped=self._encoder_events.read()
                    with self._lock:
                        for i,wheel in enumerate(self._encoder_order):
                            self._counts[wheel]=counts[i]
                            self._invalid[wheel]=invalid[i]
                        self._event_drops=dropped
                    continue
                # sample encoders (legacy polling)
                for wheel, st in self._enc_state.items():
                    pa, pb = st['pins']
                    state = self._read_pair(pa, pb)
                    last = st['last']
                    if state != last:
                        step = _QUAD_TRANS.get((last, state))
                        if step:
                            with self._lock:
                                self._counts[wheel] += step
                        else:
                            with self._lock:
                                self._invalid[wheel] += 1
                        st['last'] = state
        except Exception as exc:
            self.get_logger().error('Motor control loop stopped: %s' % exc)
            self._running = False
        finally:
            if self._hardware_pwm is not None:
                self._hardware_pwm.stop()

    # ---- shutdown ----

    def shutdown(self):
        self._running = False
        if self._ctrl_thread.is_alive():
            self._ctrl_thread.join(timeout=1.0)
        if self._hardware_pwm is not None:
            self._hardware_pwm.stop()
        if self._encoder_events is not None:
            self._encoder_events.close()
        # explicit safe state; do NOT GPIO.cleanup() (see module docstring)
        for pin in self._out_fds:
            try:
                os.pwrite(self._out_fds[pin], b'0', 0)
            except OSError:
                pass
        for fd in list(self._out_fds.values()) + list(self._in_fds.values()):
            try:
                os.close(fd)
            except OSError:
                pass
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
