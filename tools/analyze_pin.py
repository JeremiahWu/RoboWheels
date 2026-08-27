#!/usr/bin/env python3
"""PWM signal shape analyzer (READ-ONLY, register-level).

Samples a pin's PIO DAT bit at full speed and records every transition
timestamp, then reports duty and the duration distribution of the
high/low segments. Reveals the true PWM frequency and duty pattern.

Usage: analyze_pin.py <PIN_NAME> [seconds]
"""
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..',
    'ros2_ws', 'src', 'robowheels'))


def main():
    from robowheels.fast_gpio import FastGPIO

    pin = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    gpio = FastGPIO()
    try:
        p = gpio.pin(pin)
        t0 = time.perf_counter()
        t_end = t0 + seconds
        highs = 0
        samples = 0
        trans = []  # (t, new_value)
        last = None
        while time.perf_counter() < t_end:
            v = p.inp()
            samples += 1
            if v:
                highs += 1
            if last is not None and v != last:
                trans.append((time.perf_counter(), v))
            last = v
    finally:
        gpio.close()

    total = samples
    duty = 100.0 * highs / total if total else 0.0
    print(f"{pin}: {highs}/{total} = {duty:.1f}% duty, "
          f"{len(trans)} transitions in {seconds:.1f}s "
          f"({total/seconds:.0f} samples/s)")

    if len(trans) < 2:
        print("  not enough transitions for shape analysis")
        return
    # segment durations: time between consecutive transitions
    segs = [trans[i+1][0] - trans[i][0] for i in range(len(trans) - 1)]
    high_segs = [trans[i+1][0] - trans[i][0] for i in range(len(trans) - 1)
                 if trans[i+1][1] == 1]
    low_segs = [trans[i+1][0] - trans[i][0] for i in range(len(trans) - 1)
                if trans[i+1][1] == 0]
    if high_segs:
        print(f"  HIGH segs: n={len(high_segs)} med={statistics.median(high_segs)*1000:.2f}ms "
              f"min={min(high_segs)*1000:.2f}ms max={max(high_segs)*1000:.2f}ms")
    if low_segs:
        print(f"  LOW  segs: n={len(low_segs)} med={statistics.median(low_segs)*1000:.2f}ms "
              f"min={min(low_segs)*1000:.2f}ms max={max(low_segs)*1000:.2f}ms")
    if segs:
        print(f"  implied freq: {1/statistics.median(segs):.1f} Hz "
              f"(period {statistics.median(segs)*1000:.2f}ms)")


if __name__ == "__main__":
    main()
