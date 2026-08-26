#!/usr/bin/env python3
"""Encoder readout test (phase 3)

Pins (from wiring table, do not change):
  left  encoder: LA=PD24 (wPi5), LB=PD23 (wPi7)
  right encoder: RA=PL2  (wPi3), RB=PL3  (wPi4)

Modes:
  watch  - poll all 4 pins, print level changes (verify which encoder
           pair follows which wheel; rotate wheels by hand)
  count  - count rising edges on each pin until Enter is pressed.
           Use it to measure ticks per revolution: rotate one wheel
           EXACTLY one full turn, then press Enter.
           ticks_per_rev = edges / 4  (quadrature 4x: both channels,
           both edges counted by the final driver)
"""
import sys
import time

import OPi.GPIO as GPIO

LA, LB = "PD24", "PD23"
RA, RB = "PL2", "PL3"
PINS = {"LA": LA, "LB": LB, "RA": RA, "RB": RB}

# Verified motor mapping (see HARDWARE.md):
#   left wheel  = B channel, fwd = PH6 PWM (IN2), rev = PH4 PWM (IN1)
#   right wheel = A channel, fwd = PH5 PWM (IN2), rev = PL10 PWM (IN1)
MOTOR = {"L": {"fwd": "PH6", "rev": "PH4"},
         "R": {"fwd": "PH5", "rev": "PL10"}}
MOTOR_ALL = ("PH6", "PH4", "PH5", "PL10")

POLL_S = 0.001  # 1 kHz polling, plenty for hand rotation


def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.SUNXI)
    for p in PINS.values():
        try:
            GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        except (AttributeError, ValueError):
            GPIO.setup(p, GPIO.IN)
    print(">> encoders setup done", flush=True)
    for name, p in PINS.items():
        print(f">> {name}({p}) idle = {GPIO.input(p)}", flush=True)


def watch(seconds):
    print(f">> watching {seconds}s - rotate wheels by hand; "
          "any level change is printed", flush=True)
    for i in (3, 2, 1):
        print(f">> start in {i}s ...", flush=True)
        time.sleep(1.0)
    state = {n: GPIO.input(p) for n, p in PINS.items()}
    print(f">> GO initial: {state}", flush=True)
    t0 = time.time()
    while time.time() - t0 < seconds:
        for n, p in PINS.items():
            v = GPIO.input(p)
            if v != state[n]:
                print(f">> t={time.time()-t0:.2f}s  {n} -> {v}", flush=True)
                state[n] = v
        time.sleep(POLL_S)
    print(">> watch done", flush=True)


def count(windows=3, window=8.0, turns=1):
    """Measure ticks per wheel revolution.

    Each window: user rotates ONE wheel EXACTLY `turns` full turns.
    ticks_per_rev (4x quadrature) = 4 x rising edges of one channel
    (A and B rising edges should be equal; 4x = both edges of both
    channels = 4 per encoder cycle).
    """
    print(f">> {windows} counting windows of {window:.0f}s each.", flush=True)
    print(f">> Rotate ONE wheel EXACTLY {turns} full turn(s) inside EACH "
          "window.", flush=True)
    for w in range(windows):
        for i in (3, 2, 1):
            print(f">> window {w+1}/{windows}: start in {i}s ...", flush=True)
            time.sleep(1.0)
        counts = {n: 0 for n in PINS}
        state = {n: GPIO.input(p) for n, p in PINS.items()}
        print(f">> GO window {w+1}: counting...", flush=True)
        t0 = time.time()
        while time.time() - t0 < window:
            for n, p in PINS.items():
                v = GPIO.input(p)
                if v == 1 and state[n] == 0:
                    counts[n] += 1
                state[n] = v
            time.sleep(POLL_S)
        tpr_l = 4 * counts["LA"] / turns
        tpr_r = 4 * counts["RA"] / turns
        print(f">> window {w+1} rising edges: {counts}", flush=True)
        print(f">>   ticks/rev 4x: LA pair = {tpr_l:.1f}, "
              f"RA pair = {tpr_r:.1f}", flush=True)


def spin(wheel, duty, seconds):
    """Drive ONE wheel at low PWM while sampling encoder edges.

    Tight single loop: 200Hz soft PWM + max-rate edge sampling.
    Prints cumulative rising edges per pin every second.
    User: put a tape mark on the wheel and count exact revolutions
    during the run; ticks/rev = 4*edges / reported_revs.
    Also shows the count SIGN for the driven direction (odom sign).
    """
    duty = min(max(duty, 5.0), 40.0)
    pwm_pin = MOTOR[wheel]["fwd"]
    print(f">> spin {wheel} wheel: {pwm_pin} PWM {duty:.0f}% for "
          f"{seconds:.0f}s", flush=True)
    for p in MOTOR_ALL:
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
    for i in (3, 2, 1):
        print(f">> start in {i}s ...", flush=True)
        time.sleep(1.0)
    period = 1.0 / 200.0
    on_time = period * duty / 100.0
    counts = {n: 0 for n in PINS}
    state = {n: GPIO.input(p) for n, p in PINS.items()}
    t0 = time.time()
    last_print = 0.0
    print(">> GO", flush=True)
    while True:
        now = time.time()
        elapsed = now - t0
        if elapsed >= seconds:
            break
        # soft PWM phase
        phase = elapsed % period
        GPIO.output(pwm_pin, GPIO.HIGH if phase < on_time else GPIO.LOW)
        # sample encoders as fast as possible
        for n, p in PINS.items():
            v = GPIO.input(p)
            if v == 1 and state[n] == 0:
                counts[n] += 1
            state[n] = v
        if int(elapsed) != last_print:
            last_print = int(elapsed)
            print(f">> t={last_print}s edges: {counts}", flush=True)
    GPIO.output(pwm_pin, GPIO.LOW)
    for p in MOTOR_ALL:
        GPIO.output(p, GPIO.LOW)
    print(f">> done. total rising edges: {counts}", flush=True)
    print(f">> report wheel revolutions, then: "
          f"ticks/rev(4x) = 4*edges/revs", flush=True)


def drive(duty, seconds):
    """Drive BOTH wheels forward (straight line) while counting edges.

    User measures distance traveled with a ruler afterwards:
      ticks_per_meter(4x) = 4 * edges / distance_m
      ticks_per_rev(4x)   = ticks_per_meter * wheel_circumference_m
    """
    duty = min(max(duty, 5.0), 40.0)
    pwm_pins = (MOTOR["L"]["fwd"], MOTOR["R"]["fwd"])
    print(f">> drive both wheels fwd PWM {duty:.0f}% for {seconds:.0f}s",
          flush=True)
    for p in MOTOR_ALL:
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
    for i in (3, 2, 1):
        print(f">> start in {i}s ...", flush=True)
        time.sleep(1.0)
    period = 1.0 / 200.0
    on_time = period * duty / 100.0
    counts = {n: 0 for n in PINS}
    state = {n: GPIO.input(p) for n, p in PINS.items()}
    t0 = time.time()
    print(">> GO", flush=True)
    while True:
        elapsed = time.time() - t0
        if elapsed >= seconds:
            break
        phase = elapsed % period
        level = GPIO.HIGH if phase < on_time else GPIO.LOW
        for p in pwm_pins:
            GPIO.output(p, level)
        for n, p in PINS.items():
            v = GPIO.input(p)
            if v == 1 and state[n] == 0:
                counts[n] += 1
            state[n] = v
    for p in MOTOR_ALL:
        GPIO.output(p, GPIO.LOW)
    print(f">> done. total rising edges: {counts}", flush=True)
    print(">> measure distance traveled (m), then:", flush=True)
    print(">>   left ticks/m(4x)  = 4*(RA+RB)/2... use: "
          "4*RA/d or 4*RB/d", flush=True)
    print(">>   right ticks/m(4x) = 4*LA/d or 4*LB/d", flush=True)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "watch"
    setup()
    if mode == "watch":
        watch(30.0)
    elif mode == "count":
        turns = int(sys.argv[2]) if len(sys.argv) > 2 else 1
        count(turns=turns)
    elif mode == "spin":
        wheel = sys.argv[2].upper() if len(sys.argv) > 2 else "L"
        duty = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
        seconds = float(sys.argv[4]) if len(sys.argv) > 4 else 25.0
        spin(wheel, duty, seconds)
    elif mode == "drive":
        duty = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
        seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
        drive(duty, seconds)
    else:
        print("usage: encoder_test.py [watch|count [turns]|"
              "spin [L|R] [duty%] [seconds]|drive [duty%] [seconds]]")
    GPIO.cleanup()
    print(">> cleanup done", flush=True)


if __name__ == "__main__":
    main()
