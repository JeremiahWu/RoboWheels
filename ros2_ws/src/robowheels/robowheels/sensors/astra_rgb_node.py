#!/usr/bin/env python3
"""Astra Pro RGB camera node (UVC fallback, no Orbbec SDK needed).

The Astra Pro's color sensor (USB PID 2bc5:0501) enumerates as a plain
UVC camera (/dev/video0 on the RDK X5) and works with OpenCV directly.
The depth/IR part (PID 2bc5:0403) is vendor-specific and needs the
official driver - see launch/astra_camera.launch.py.

Publishes:
  /astra/rgb/image_raw   (sensor_msgs/Image)
  /astra/rgb/camera_info (sensor_msgs/CameraInfo)

Parameters (also in config/astra_rgb.yaml):
  device:         0        # V4L2 device index (or /dev/videoX path)
  width:          1280
  height:         720
  fps:            30
  fourcc:         MJPG     # MJPG 1280x720@30 or YUYV 640x480@30
  encoding:       bgr8
  frame_id:       astra_rgb
  fx/fy/cx/cy:    intrinsics placeholder (calibrate later)
"""
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


class AstraRgbNode(Node):

    def __init__(self):
        super().__init__('astra_rgb_node')
        p = self.declare_parameter

        self.device = p('device', 0).value
        self.width = p('width', 1280).value
        self.height = p('height', 720).value
        self.fps = p('fps', 30.0).value
        fourcc = p('fourcc', 'MJPG').value
        self.encoding = p('encoding', 'bgr8').value
        self.frame_id = p('frame_id', 'astra_rgb').value
        self.fx = p('fx', 700.0).value
        self.fy = p('fy', 700.0).value
        self.cx = p('cx', self.width / 2.0).value
        self.cy = p('cy', self.height / 2.0).value

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, '/astra/rgb/image_raw', 10)
        self._info_pub = self.create_publisher(
            CameraInfo, '/astra/rgb/camera_info', 10)

        self._cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f'cannot open camera device {self.device}')
        self._cap.set(cv2.CAP_PROP_FOURCC,
                      cv2.VideoWriter_fourcc(*fourcc))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        w = self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(
            f'astra_rgb ready: {fourcc} {int(w)}x{int(h)} '
            f'@~{self._cap.get(cv2.CAP_PROP_FPS):.1f} fps, '
            f'publishing /astra/rgb/image_raw ({self.encoding})')

        self._timer = self.create_timer(1.0 / self.fps, self._grab_cb)

    def _grab_cb(self):
        ok, frame = self._cap.read()
        if not ok:
            self.get_logger().warn('frame grab failed', throttle_duration_sec=5.0)
            return
        stamp = self.get_clock().now().to_msg()
        msg = self._bridge.cv2_to_imgmsg(frame, encoding=self.encoding)
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self._pub.publish(msg)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = frame.shape[1]
        info.height = frame.shape[0]
        info.distortion_model = 'plumb_bob'
        info.k = [self.fx, 0.0, self.cx,
                  0.0, self.fy, self.cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0,
                  0.0, self.fy, self.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info)

    def shutdown(self):
        if hasattr(self, '_cap') and self._cap is not None:
            self._cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = AstraRgbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
