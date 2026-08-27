#!/usr/bin/env python3
"""Fast GPIO pin duty sampler (READ-ONLY, does NOT change pin mode).

Samples a pin's PIO DAT register directly through /dev/mem
(robowheels.fast_gpio, same register model as the motor driver), so it
works whether or not the pin is exported in sysfs and never touches
the pin's direction. Falls back to the sysfs value file if /dev/mem
is unavailable.

Usage: pin_sampler.py <PIN_NAME> [seconds]
    pin_sampler.py PH6 1
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..',
    'ros2_ws', 'src', 'robowheels'))


def sample_register(pin, seconds):
    from robowheels.fast_gpio import FastGPIO
    gpio = FastGPIO()
    try:
        p = gpio.pin(pin)
        t0 = time.time()
        high = low = 0
        transitions = 0
        last = None
        while time.time() - t0 < seconds:
            v = p.inp()
            if v:
                high += 1
            else:
                low += 1
            if last is not None and v != last:
                transitions += 1
            last = v
    finally:
        gpio.close()
    return high, low, transitions


def sample_sysfs(pin, seconds):
    from OPi.constants import SUNXI
    from OPi.pin_mappings import get_gpio_pin
    gpio_num = get_gpio_pin(SUNXI, pin)
    value_path = f"/sys/class/gpio/gpio{gpio_num}/value"
    if not os.path.exists(value_path):
        return None
    fd = os.open(value_path, os.O_RDONLY)
    t0 = time.time()
    high = low = 0
    transitions = 0
    last = None
    while time.time() - t0 < seconds:
        os.lseek(fd, 0, os.SEEK_SET)
        ch = os.read(fd, 1)
        v = 1 if ch == b"1" else 0
        if v:
            high += 1
        else:
            low += 1
        if last is not None and v != last:
            transitions += 1
        last = v
    os.close(fd)
    return high, low, transitions


def main():
    pin = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    res = None
    source = None
    try:
        res = sample_register(pin, seconds)
        source = 'register'
    except (ImportError, PermissionError, OSError) as e:
        print(f"{pin}: register read unavailable ({e}), trying sysfs")
        res = sample_sysfs(pin, seconds)
        source = 'sysfs'
        if res is None:
            print(f"{pin}: not exported in sysfs either - no value file")
            sys.exit(1)

    high, low, transitions = res
    total = high + low
    if total == 0:
        print(f"{pin}: no samples")
        return
    print(f"{pin} [{source}]: {high}/{total} HIGH = "
          f"{100.0*high/total:.1f}% duty, "
          f"{transitions} transitions in {seconds:.1f}s "
          f"({total/seconds:.0f} samples/s)")


if __name__ == "__main__":
    main()
