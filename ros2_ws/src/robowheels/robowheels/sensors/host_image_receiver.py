#!/usr/bin/env python3
"""Receive framed host BGR video on stdin and publish real camera frames."""
import struct
import sys
import rclpy
from sensor_msgs.msg import Image


def exact(size):
    data = bytearray()
    while len(data) < size:
        chunk = sys.stdin.buffer.read(size - len(data))
        if not chunk:
            raise EOFError
        data.extend(chunk)
    return bytes(data)


def main():
    rclpy.init()
    node = rclpy.create_node('mac_astra_rgb_bridge')
    pub = node.create_publisher(Image, '/astra/rgb/image_raw', 5)
    count = 0
    try:
        while rclpy.ok():
            width, height, length, stamp = struct.unpack('!IIIQ', exact(20))
            if not (0 < width <= 4096 and 0 < height <= 2160 and length == width * height * 3):
                raise ValueError('Invalid frame dimensions')
            msg = Image()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1000000000)
            msg.header.frame_id = 'astra_rgb_optical_frame'
            msg.width, msg.height, msg.step = width, height, width * 3
            msg.encoding = 'bgr8'
            msg.data = exact(length)
            pub.publish(msg)
            count += 1
            if count % 100 == 1:
                print(f'Published {count} Astra RGB frames', file=sys.stderr, flush=True)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
