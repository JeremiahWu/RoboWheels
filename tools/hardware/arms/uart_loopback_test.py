#!/usr/bin/env python3
"""UART loopback self-test for RDK X5 40PIN pins 8/10 (UART1 /dev/ttyS1).

Purpose: prove the board-side UART works, independent of the bus-servo
controller wiring.  Short pins 8 (TX) and 10 (RX) with a jumper wire,
then run:

    python3 tools/hardware/arms/uart_loopback_test.py [baud]

Success = every byte sent is received back.  If this passes but the
servo controller does not answer (see servo_bus_test.py), the problem
is the external wiring (cross TX<->RX, common GND, power) or the
controller itself - not the RDK X5 pins.

Note: on the X5 the 40PIN UART1 is configured with SER_RS485_ENABLED
in the DT (verified 2026-09-10 via TIOCGRS485).  The loopback still
works, but if full-duplex peripherals misbehave, removing the rs485
property from the uart1 DT node is the fix (see HARDWARE.md).
"""
import sys
import time

import serial


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyS1'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    ser = serial.Serial(port, baud, timeout=0.5)
    print(f'{port} @{baud} opened; send/receive test ...')
    ok = total = 0
    for n in range(20):
        payload = bytes([0xAA, 0x55, (n * 7 + 1) & 0xFF, n & 0xFF])
        ser.reset_input_buffer()
        ser.write(payload)
        echo = ser.read(len(payload))
        total += 1
        if echo == payload:
            ok += 1
        else:
            print(f'  mismatch #{n}: sent {payload.hex()} got '
                  f'{echo.hex() if echo else "TIMEOUT"}')
    ser.close()
    print(f'loopback result: {ok}/{total} ok -> '
          f'{"PASS - pins 8/10 UART fine" if ok == total else "FAIL"}')
    return 0 if ok == total else 1


if __name__ == '__main__':
    sys.exit(main())
