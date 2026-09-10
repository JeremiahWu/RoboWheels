#!/usr/bin/env python3
"""Bus-servo controller communication test (40PIN pins 8/10, UART1).

Background (verified on the RDK X5, 2026-09-10):
  - 40PIN pin 8 = UART1 TX, pin 10 = UART1 RX, /dev/ttyS1, 3.3V TTL
  - the port opens and transmits fine; the DT enables RS485 mode on
    UART1 (TIOCGRS485 flag 0x2) which gates the RX while transmitting -
    full-duplex peripherals may need that removed (see HARDWARE.md)
  - first run uart_loopback_test.py with a jumper on 8<->10 to prove
    the board side, then use this script against the controller

This script probes the controller with the two common bus-servo
protocols (LewanSoul LOBOT 0x55 0x55 and Feetech SCS 0xFF 0xFF) over a
range of baudrates, and reports any structurally valid reply frame.
Read-only commands (read position / read temperature) - no motion.

Usage:
  python3 tools/servo_bus_test.py [port] [id1 id2 ...]
  e.g.  python3 tools/servo_bus_test.py /dev/ttyS1 1 2
"""
import sys
import time

import serial

BAUDS = (9600, 38400, 57600, 115200, 230400, 460800, 921600, 1000000)


def chk(payload):
    return (~sum(payload)) & 0xFF


def lobot_read_cmd(sid):
    """LewanSoul bus servo: read position (cmd 0x14)."""
    return bytes([0x55, 0x55, sid, 3, 0x14]) + bytes([chk([sid, 3, 0x14])])


def scs_read_cmd(sid):
    """Feetech SCS bus servo: read position (cmd 0x1C)."""
    return bytes([0xFF, 0xFF, sid, 4, 0x1C]) + bytes([chk([sid, 4, 0x1C])])


def valid_frame(data):
    if len(data) < 6:
        return None
    hdr = data[:2]
    if hdr == b'\x55\x55' or hdr == b'\xff\xff':
        sid, ln = data[2], data[3]
        if len(data) >= ln + 3 and chk(data[2:4] + data[4:ln + 2]) \
                == data[ln + 2]:
            return 'LOBOT' if hdr == b'\x55\x55' else 'SCS'
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyS1'
    ids = [int(a) for a in sys.argv[2:]] or [1]
    print(f'scanning {port} for bus servos (IDs {ids}), '
          f'bauds {BAUDS} ...')
    found = []
    for baud in BAUDS:
        try:
            ser = serial.Serial(port, baud, timeout=0.2)
        except Exception as e:  # noqa: BLE001
            print(f'  {baud}: open failed: {e}')
            continue
        for proto, mk in (('LOBOT', lobot_read_cmd), ('SCS', scs_read_cmd)):
            for sid in ids:
                ser.reset_input_buffer()
                ser.write(mk(sid))
                data = b''
                t0 = time.time()
                while time.time() - t0 < 0.3:
                    b = ser.read(1)
                    if not b:
                        break
                    data += b
                    if valid_frame(data):
                        break
                v = valid_frame(data)
                if v:
                    print(f'  {baud} {v} ID{sid}: FOUND reply '
                          f'{data.hex()}')
                    found.append((baud, v, sid))
                elif data:
                    print(f'  {baud} {proto} ID{sid}: garbage/echo '
                          f'{data.hex()}')
        ser.close()
    if found:
        print(f'RESULT: servo(s) respond at {found}')
    else:
        print('RESULT: no valid reply at any baud/protocol.')
        print('  -> check: TX(8)<->controller RX, RX(10)<->controller TX')
        print('            (crossed!), common GND, controller powered')
        print('  -> or run uart_loopback_test.py to verify the board side')
        print('  -> or switch to a USB-TTL adapter (/dev/ttyUSB0)')
        sys.exit(1)


if __name__ == '__main__':
    main()
