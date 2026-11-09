#!/usr/bin/env python3
"""AT8236 motor test for RDK X5 (replaces the Orange Pi 3 LTS version).

Wiring (BOARD numbering, RDK X5 40PIN):
    channel A:  AIN1=27, AIN2=29
    channel B:  BIN1=31, BIN2=33
    forward  = IN2 PWM, IN1 LOW
    backward = IN1 PWM, IN2 LOW

Usage:
  motor_test.py [A|B|AB] [fwd|rev] [duty%] [seconds]
  default: AB, fwd, 20%, 2s   (duty clamped to 0..60 for safety)

Startup: all 4 pins OUTPUT LOW (coast, safe).
Exit:    all 4 pins left OUTPUT LOW (GPIO.cleanup NOT called - floating
         IN pins on the AT8236 could drive the motors).

First run on the X5: drive one channel at a time (A fwd / B fwd) and
note WHICH WHEEL turns - on the OPi build A=right B=left, on the new
wiring this has not been verified yet.  Record it in HARDWARE.md and
fix config/motor.yaml channel_a_wheel / channel_b_wheel accordingly.
"""
import os
import sys
import time

import Hobot.GPIO as GPIO

# channel A pins (AIN1=27, AIN2=29), channel B pins (BIN1=31, BIN2=33)
CH = {
    'A': {'in1': 27, 'in2': 29},
    'B': {'in1': 31, 'in2': 33},
}
ALL_PINS = (27, 29, 31, 33)
PWM_FREQ_HZ = 200


def _sysfs_gpio_num(pin):
    return int(GPIO.all_pin_data['BOARD'][int(pin)].gpio_id)


def soft_pwm(fds, duty_percent, seconds):
    """Bit-bang PWM on given output fds: HIGH for on-time, LOW for off."""
    period = 1.0 / PWM_FREQ_HZ
    on_time = period * duty_percent / 100.0
    end = time.time() + seconds
    while time.time() < end:
        for fd in fds:
            os.pwrite(fd, b'1', 0)
        time.sleep(on_time)
        for fd in fds:
            os.pwrite(fd, b'0', 0)
        time.sleep(period - on_time)
    for fd in fds:
        os.pwrite(fd, b'0', 0)


def main():
    channels = sys.argv[1].upper() if len(sys.argv) > 1 else "AB"
    direction = sys.argv[2].lower() if len(sys.argv) > 2 else "fwd"
    duty = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
    secs = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
    duty = min(max(duty, 0.0), 60.0)  # low-speed test clamp

    if any(c not in 'AB' for c in channels) or direction not in ("fwd", "rev"):
        print("usage: motor_test.py [A|B|AB] [fwd|rev] [duty%] [seconds]")
        sys.exit(1)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    # startup safe state: all motor pins OUTPUT LOW
    for pin in ALL_PINS:
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    print(">> all motor pins OUTPUT LOW (coast)", flush=True)

    # fast fds
    fds = {pin: os.open('/sys/class/gpio/gpio%d/value' % _sysfs_gpio_num(pin),
                        os.O_WRONLY) for pin in ALL_PINS}

    # pick PWM pins: fwd -> IN2, rev -> IN1
    pwm_fds = []
    for c in channels:
        if direction == "fwd":
            pwm_fds.append(fds[CH[c]['in2']])
        else:
            pwm_fds.append(fds[CH[c]['in1']])

    label = {'A': 'channel A', 'B': 'channel B', 'AB': 'channels A+B'}[channels]
    pins_used = ' '.join('%s=%s' % (k, CH[c][k])
                         for c in channels for k in ('in1', 'in2'))
    print(f">> {label} {direction} PWM {duty:.0f}% for {secs:.0f}s "
          f"({pins_used}) ...", flush=True)
    soft_pwm(pwm_fds, duty, secs)
    print(">> stopped.", flush=True)

    # final safe state, no cleanup
    for fd in fds.values():
        try:
            os.pwrite(fd, b'0', 0)
        except OSError:
            pass
        os.close(fd)
    print(">> all outputs LOW, done.", flush=True)


if __name__ == "__main__":
    main()
