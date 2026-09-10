#!/usr/bin/env python3
"""Encoder readout test for RDK X5 (replaces the Orange Pi 3 LTS version).

Pins (BOARD numbering, RDK X5 40PIN, do not change):
  E1A=11, E1B=13   (encoder 1)
  E2A=15, E2B=16   (encoder 2)
Which encoder pair belongs to which wheel has NOT been verified on the
X5 wiring yet - `watch` mode tells you (rotate one wheel by hand).

Modes:
  watch  - poll all 4 pins, print level changes (verify which encoder
           pair follows which wheel; rotate wheels by hand)
  count  - count rising edges on each pin for a 30 s window.
           Rotate one wheel EXACTLY N full turns inside the window.
           ticks_per_rev = edges * 4  (4x quadrature, both edges of
           both channels counted by the driver)
  spin   - drive ONE motor channel (A or B) at low PWM while sampling
           encoder edges; shows which encoder counts and the count SIGN
           for the driven direction (odom sign calibration)
  drive  - drive BOTH channels forward (straight line) while counting;
           measure the distance traveled, then
             ticks_per_meter(4x) = 4 * edges / distance_m
"""
import os
import sys
import time

import Hobot.GPIO as GPIO

ENC = {'E1A': 11, 'E1B': 13, 'E2A': 15, 'E2B': 16}
CH = {'A': {'in1': 27, 'in2': 29}, 'B': {'in1': 31, 'in2': 33}}
ALL_PINS = tuple(ENC.values()) + (27, 29, 31, 33)
PWM_FREQ_HZ = 200
POLL_S = 0.001  # 1 kHz floor; the real loop spins much faster


def _sysfs_gpio_num(pin):
    return int(GPIO.all_pin_data['BOARD'][int(pin)].gpio_id)


def _open(pin, writable):
    return os.open('/sys/class/gpio/gpio%d/value' % _sysfs_gpio_num(pin),
                   os.O_WRONLY if writable else os.O_RDONLY)


def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    for p in ENC.values():
        try:
            GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        except (AttributeError, ValueError, RuntimeError):
            GPIO.setup(p, GPIO.IN)
    print(">> encoders setup done", flush=True)
    in_fds = {n: _open(p, False) for n, p in ENC.items()}
    print(">> idle levels:", {n: os.pread(fd, 8, 0)[0] for n, fd in in_fds.items()},
          flush=True)
    return in_fds


def read_level(fd):
    return os.pread(fd, 8, 0)[0] == 0x31


def watch(seconds, in_fds):
    print(f">> watching {seconds}s - rotate wheels by hand; "
          "any level change is printed", flush=True)
    for i in (3, 2, 1):
        print(f">> start in {i}s ...", flush=True)
        time.sleep(1.0)
    state = {n: read_level(fd) for n, fd in in_fds.items()}
    print(f">> GO initial: {state}", flush=True)
    t0 = time.time()
    while time.time() - t0 < seconds:
        for n, fd in in_fds.items():
            v = read_level(fd)
            if v != state[n]:
                print(f">> t={time.time()-t0:.2f}s  {n} -> {v}", flush=True)
                state[n] = v
        time.sleep(POLL_S)
    print(">> watch done", flush=True)


def count(turns, in_fds):
    """Rising edges per pin while the user rotates ONE wheel N turns."""
    print(f">> rotate ONE wheel EXACTLY {turns} full turn(s), "
          "then press Enter", flush=True)
    input(">> press Enter to start ...")
    counts = {n: 0 for n in ENC}
    state = {n: read_level(fd) for n, fd in in_fds.items()}
    print(">> counting for 30 s ...", flush=True)
    t0 = time.time()
    while time.time() - t0 < 30.0:
        for n, fd in in_fds.items():
            v = read_level(fd)
            if v and not state[n]:
                counts[n] += 1
            state[n] = v
        time.sleep(POLL_S)
    print(f">> rising edges: {counts}", flush=True)
    for pair in (('E1A', 'E1B'), ('E2A', 'E2B')):
        tpr = 4 * counts[pair[0]] / turns
        print(f">>   {pair[0]}+{pair[1]} ticks/rev (4x) = {tpr:.1f}",
              flush=True)


def _setup_motors():
    for p in (27, 29, 31, 33):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
    return {p: _open(p, True) for p in (27, 29, 31, 33)}


def spin(channel, duty, seconds, in_fds):
    """Drive ONE channel at low PWM while sampling encoder edges."""
    duty = min(max(duty, 5.0), 40.0)
    out_fds = _setup_motors()
    pwm_pin = CH[channel]['in2']  # forward = IN2 PWM
    print(f">> spin channel {channel} ({pwm_pin}=IN2) PWM {duty:.0f}% "
          f"for {seconds:.0f}s", flush=True)
    for i in (3, 2, 1):
        print(f">> start in {i}s ...", flush=True)
        time.sleep(1.0)
    period = 1.0 / PWM_FREQ_HZ
    on_time = period * duty / 100.0
    counts = {n: 0 for n in ENC}
    state = {n: read_level(fd) for n, fd in in_fds.items()}
    t0 = time.time()
    last_print = 0.0
    print(">> GO", flush=True)
    while True:
        elapsed = time.time() - t0
        if elapsed >= seconds:
            break
        phase = elapsed % period
        os.pwrite(out_fds[pwm_pin], b'1' if phase < on_time else b'0', 0)
        for n, fd in in_fds.items():
            v = read_level(fd)
            if v and not state[n]:
                counts[n] += 1
            state[n] = v
        if int(elapsed) != last_print:
            last_print = int(elapsed)
            print(f">> t={last_print}s rising edges: {counts}", flush=True)
    os.pwrite(out_fds[pwm_pin], b'0', 0)
    for fd in out_fds.values():
        os.pwrite(fd, b'0', 0)
        os.close(fd)
    print(f">> done. total rising edges: {counts}", flush=True)
    print(">>   ticks/rev(4x) = 4*edges/revs; sign = odom sign for fwd",
          flush=True)


def drive(duty, seconds, in_fds):
    """Drive BOTH channels forward (straight line) while counting edges."""
    duty = min(max(duty, 5.0), 40.0)
    out_fds = _setup_motors()
    pwm_pins = (CH['A']['in2'], CH['B']['in2'])
    print(f">> drive both channels fwd PWM {duty:.0f}% for {seconds:.0f}s",
          flush=True)
    for i in (3, 2, 1):
        print(f">> start in {i}s ...", flush=True)
        time.sleep(1.0)
    period = 1.0 / PWM_FREQ_HZ
    on_time = period * duty / 100.0
    counts = {n: 0 for n in ENC}
    state = {n: read_level(fd) for n, fd in in_fds.items()}
    t0 = time.time()
    print(">> GO", flush=True)
    while True:
        elapsed = time.time() - t0
        if elapsed >= seconds:
            break
        phase = elapsed % period
        level = b'1' if phase < on_time else b'0'
        for p in pwm_pins:
            os.pwrite(out_fds[p], level, 0)
        for n, fd in in_fds.items():
            v = read_level(fd)
            if v and not state[n]:
                counts[n] += 1
            state[n] = v
    for fd in out_fds.values():
        os.pwrite(fd, b'0', 0)
        os.close(fd)
    print(f">> done. total rising edges: {counts}", flush=True)
    print(">> measure distance traveled (m), then:", flush=True)
    print(">>   ticks/m(4x) = 4 * edges / distance  (per encoder pair)",
          flush=True)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "watch"
    in_fds = setup()
    if mode == "watch":
        watch(30.0, in_fds)
    elif mode == "count":
        turns = int(sys.argv[2]) if len(sys.argv) > 2 else 1
        count(turns, in_fds)
    elif mode == "spin":
        channel = sys.argv[2].upper() if len(sys.argv) > 2 else "A"
        duty = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
        seconds = float(sys.argv[4]) if len(sys.argv) > 4 else 25.0
        spin(channel, duty, seconds, in_fds)
    elif mode == "drive":
        duty = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
        seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
        drive(duty, seconds, in_fds)
    else:
        print("usage: encoder_test.py [watch|count [turns]|"
              "spin [A|B] [duty%] [seconds]|drive [duty%] [seconds]]")
    for fd in in_fds.values():
        os.close(fd)
    print(">> done", flush=True)


if __name__ == "__main__":
    main()
