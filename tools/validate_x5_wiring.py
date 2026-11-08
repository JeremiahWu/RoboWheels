#!/usr/bin/env python3
"""Short, wheels-off-ground X5 motor/encoder test. Explicit --run-motors required.
Hardware PWM is the default; duty is limited to 20%.
Leaves BOARD 27/29/31/33 OUTPUT LOW; never calls GPIO.cleanup().
"""
import argparse
import json
import os
import signal
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-motors', action='store_true')
    p.add_argument('--backend', choices=['gpio', 'hardware'], default='hardware')
    p.add_argument('--duty', type=float, default=10)
    p.add_argument('--frequency', type=int, default=200)
    p.add_argument('--seconds', type=float, default=0.5)
    p.add_argument('--channel', choices=['A', 'B', 'AB'], default='AB')
    p.add_argument('--direction', choices=['fwd', 'rev', 'both'], default='both')
    a = p.parse_args()
    if not a.run_motors:
        p.error('Raise wheels off ground, then explicitly pass --run-motors')
    if not 0 < a.duty <= 20 or not 0 < a.seconds <= 2:
        p.error('duty must be (0,20], seconds (0,2]')
    import Hobot.GPIO as G
    G.setwarnings(False); G.setmode(G.BOARD)
    motors = (27, 29, 31, 33)
    enc = (11, 13, 15, 16)
    from pathlib import Path
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"ros2_ws/src/robowheels"))
    from robowheels.hardware_pwm import MotorPWM
    outputs = {}; inputs = {}; hardware = None
    def halt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, halt)
    signal.signal(signal.SIGINT, halt)
    def path(pin):
        return '/sys/class/gpio/gpio%d/value' % int(G.all_pin_data['BOARD'][pin].gpio_id)
    def low():
        for fd in outputs.values(): os.pwrite(fd, b'0', 0)
        if hardware is not None: hardware.stop()
    def levels():
        return [int(os.pread(inputs[pin], 1, 0)) for pin in enc]
    # Gray-code decoder; sign is a raw electrical sign, not verified wheel forward.
    lut = (0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0)
    try:
        for pin in motors:
            if a.backend == 'gpio':
                G.setup(pin, G.OUT, initial=G.LOW)
                outputs[pin] = os.open(path(pin), os.O_WRONLY)
        if a.backend == 'hardware':
            hardware = MotorPWM(motors, frequency=a.frequency)
        for pin in enc:
            G.setup(pin, G.IN)
            inputs[pin] = os.open(path(pin), os.O_RDONLY)
        print(json.dumps({'idle_levels': levels()}), flush=True)
        for channel, pins in [('A',(27,29)),('B',(31,33))]:
            if channel not in a.channel: continue
            for direction, pin in [('fwd',pins[1]),('rev',pins[0])]:
                if a.direction != 'both' and direction != a.direction: continue
                low(); time.sleep(2)
                prev=levels(); edges=[0]*4; counts=[0,0]; invalid=[0,0]
                if a.backend == 'hardware': hardware.set(pin, a.duty)
                start=time.monotonic()
                while time.monotonic()-start < a.seconds:
                    elapsed=time.monotonic()-start
                    if a.backend == 'gpio': os.pwrite(outputs[pin], b'1' if elapsed % .005 < .005*a.duty/100 else b'0', 0)
                    now=levels()
                    for i in range(4): edges[i] += now[i] != prev[i]
                    for i in range(2):
                        old=prev[i*2]*2+prev[i*2+1]; new=now[i*2]*2+now[i*2+1]
                        counts[i] += lut[old*4+new]
                        invalid[i] += (old ^ new) == 3
                    prev=now
                low()
                print(json.dumps({'backend':a.backend,'channel':channel,'direction':direction,'duty':a.duty,'frequency':a.frequency,'seconds':a.seconds,'edges_by_pin':dict(zip(enc,edges)),'raw_quadrature_E1_E2':counts,'invalid_transitions':invalid}),flush=True)
    finally:
        low()
        for fd in list(outputs.values())+list(inputs.values()): os.close(fd)
        print('All motor outputs stopped (hardware PWM disabled)',flush=True)

if __name__ == '__main__': main()
