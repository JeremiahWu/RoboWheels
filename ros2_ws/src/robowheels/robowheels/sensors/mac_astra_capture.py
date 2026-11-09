#!/usr/bin/env python3
"""Capture Mac Astra RGB as framed BGR bytes for host_image_receiver.py."""
import argparse
import struct
import sys
import time
import cv2

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--index', type=int, default=2)
    a = p.parse_args()
    c = cv2.VideoCapture(a.index, cv2.CAP_AVFOUNDATION)
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not c.isOpened():
        raise SystemExit('Cannot open Astra RGB camera')
    try:
        while True:
            ok, frame = c.read()
            if not ok:
                raise RuntimeError('Camera frame capture failed')
            frame = cv2.resize(frame, (640, 480))
            data = frame.tobytes()
            sys.stdout.buffer.write(struct.pack('!IIIQ', 640, 480, len(data), time.time_ns()) + data)
            sys.stdout.buffer.flush()
            time.sleep(.1)
    except BrokenPipeError:
        pass
    finally:
        c.release()


if __name__ == "__main__":
    main()
