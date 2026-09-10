#!/usr/bin/env python3
"""Fast GPIO pin duty sampler for RDK X5 (read-only, does NOT change mode).

Reads the pin value file through a kept-open fd in a tight loop and
reports observed duty cycle + transition count.  Use while motor_driver
is running to verify software PWM output, e.g.:
    pin_sampler.py 29 1

Usage: pin_sampler.py <BOARD_PIN> [seconds]
"""
import os
import sys
import time

import Hobot.GPIO as GPIO


def main():
    pin = int(sys.argv[1])
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    gpio_num = int(GPIO.all_pin_data['BOARD'][pin].gpio_id)
    fd = os.open('/sys/class/gpio/gpio%d/value' % gpio_num, os.O_RDONLY)
    t0 = time.time()
    high = low = 0
    transitions = 0
    last = None
    while time.time() - t0 < seconds:
        v = os.pread(fd, 8, 0)[0] == 0x31
        if v:
            high += 1
        else:
            low += 1
        if last is not None and v != last:
            transitions += 1
        last = v
    os.close(fd)
    total = high + low
    print(f"pin {pin} (gpio{gpio_num}): {high}/{total} HIGH = "
          f"{100.0*high/total:.1f}% duty, {transitions} transitions "
          f"in {seconds:.1f}s")


if __name__ == "__main__":
    main()
