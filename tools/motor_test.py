#!/usr/bin/env python3
"""AT8236 motor test (phase 2, verified 2026-08-26)

Verified physical mapping (see HARDWARE.md):
  A channel (PL10/PH5) -> RIGHT wheel
  B channel (PH4/PH6)  -> LEFT  wheel
  forward  = IN2 PWM, IN1 LOW   (left: PH6, right: PH5)
  backward = IN1 PWM, IN2 LOW   (left: PH4, right: PL10)

Usage:
  motor_test.py [L|R|LR] [fwd|rev] [duty%] [seconds]
  default: LR, fwd, 20%, 2s   (duty clamped to 0..60 for safety)

Startup: all 4 pins OUTPUT LOW (coast, safe).
Exit:    all 4 pins left OUTPUT LOW.
"""
import sys
import time

import OPi.GPIO as GPIO

# A channel = RIGHT wheel
R_FWD, R_REV = "PH5", "PL10"      # fwd=IN2(PH5) PWM; rev=IN1(PL10) PWM
# B channel = LEFT wheel
L_FWD, L_REV = "PH6", "PH4"       # fwd=IN2(PH6) PWM; rev=IN1(PH4) PWM

ALL_PINS = (R_FWD, R_REV, L_FWD, L_REV)
PWM_FREQ_HZ = 200


def soft_pwm(pins, duty_percent, seconds):
    """Bit-bang PWM on one or more pins: HIGH for on-time, LOW for off-time."""
    period = 1.0 / PWM_FREQ_HZ
    on_time = period * duty_percent / 100.0
    end = time.time() + seconds
    while time.time() < end:
        for p in pins:
            GPIO.output(p, GPIO.HIGH)
        time.sleep(on_time)
        for p in pins:
            GPIO.output(p, GPIO.LOW)
        time.sleep(period - on_time)
    for p in pins:
        GPIO.output(p, GPIO.LOW)


def wheels_pins(wheels, direction):
    pins = []
    for w in wheels:
        if w == "L":
            pins.append(L_FWD if direction == "fwd" else L_REV)
        elif w == "R":
            pins.append(R_FWD if direction == "fwd" else R_REV)
    return pins


def main():
    wheels = sys.argv[1].upper() if len(sys.argv) > 1 else "LR"
    direction = sys.argv[2].lower() if len(sys.argv) > 2 else "fwd"
    duty = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
    secs = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
    duty = min(max(duty, 0.0), 60.0)  # low-speed test clamp

    if wheels not in ("L", "R", "LR") or direction not in ("fwd", "rev"):
        print("usage: motor_test.py [L|R|LR] [fwd|rev] [duty%] [seconds]")
        sys.exit(1)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.SUNXI)

    # startup safe state: all motor pins OUTPUT LOW
    for pin in ALL_PINS:
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    print(">> all motor pins OUTPUT LOW (coast)", flush=True)

    pins = wheels_pins(wheels, direction)
    label = {"L": "left", "R": "right", "LR": "both"}[wheels]
    print(f">> {label} wheel(s) {direction} PWM {duty:.0f}% for {secs:.0f}s "
          f"(pin={','.join(pins)}) ...", flush=True)
    soft_pwm(pins, duty, secs)
    print(">> stopped.", flush=True)

    # final safe state
    for pin in ALL_PINS:
        GPIO.output(pin, GPIO.LOW)
    print(">> all outputs LOW, done.", flush=True)


if __name__ == "__main__":
    main()
