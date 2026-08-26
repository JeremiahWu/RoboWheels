#!/usr/bin/env python3
"""Fast GPIO pin duty sampler (read-only, does NOT change pin mode).

Usage: pin_sampler.py <PIN_NAME> [seconds]
Reads the pin data register in a tight loop and reports observed
duty cycle + transition count. Use while motor_driver is running to
verify software PWM output, e.g.:
    pin_sampler.py PH6 1
"""
import sys
import time

import OPi.GPIO as GPIO


def main():
    pin = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.SUNXI)
    t0 = time.time()
    high = low = 0
    transitions = 0
    last = None
    while time.time() - t0 < seconds:
        v = GPIO.input(pin)
        if v:
            high += 1
        else:
            low += 1
        if last is not None and v != last:
            transitions += 1
        last = v
    total = high + low
    print(f"{pin}: {high}/{total} HIGH = {100.0*high/total:.1f}% duty, "
          f"{transitions} transitions in {seconds:.1f}s")


if __name__ == "__main__":
    main()
