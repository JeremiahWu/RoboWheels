#!/usr/bin/env python3
"""Read-only Desheng v4.01 discovery (YB-SD15M); never writes registers."""
import argparse
import time
import serial


from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "ros2_ws/src/robowheels"))
from robowheels.arms.desheng_protocol import query


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--last-id', type=int, default=20, choices=range(1, 251))
    args = parser.parse_args()
    found = []
    with serial.Serial(args.port, args.baud, timeout=.08) as port:
        time.sleep(.1)
        for sid in range(1, args.last_id + 1):
            frame = query(port, sid, 1)
            if frame is None:
                continue
            found.append(sid)
            print(f'ID {sid}: status=0x{frame[4]:02x} PING={frame.hex()}', flush=True)
            position = query(port, sid, 2, bytes([0x38, 2]))
            if position is not None and len(position) == 8:
                print(f'ID {sid}: position={int.from_bytes(position[5:7], "big")} status=0x{position[4]:02x}', flush=True)
    print(f'Found IDs: {found}', flush=True)


if __name__ == '__main__':
    main()
