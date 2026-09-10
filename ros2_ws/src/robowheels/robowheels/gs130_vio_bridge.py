#!/usr/bin/env python3
"""GS130WI stereo+IMU -> VIO convention bridge.

Verified live on the RDK X5 (2026-09-10) with tros mipi_cam in dual
channel mode (gdc on, nv12): the driver publishes per-eye images and a
100 Hz IMU:

    /image_left_raw   816x960 nv12   (Y plane = first 640 rows)
    /image_right_raw  816x960 nv12
    /imu_data         sensor_msgs/Imu (ICM-42688, ~100 Hz)

VIO stacks (VINS-Fusion, FAST-LIVO, OpenVINS) expect mono images and
standard topic names.  This node converts:

    /image_left_raw   ->  /cam0/image_raw    (mono8 816x640)
    /image_right_raw  ->  /cam1/image_raw    (mono8 816x640)
    /imu_data         ->  /imu0              (Imu, frame imu0)
    camera_info       ->  /cam0/camera_info, /cam1/camera_info

and publishes static TFs imu0->cam0 / imu0->cam1 (baseline along +x,
centered on the IMU).

Parameters: config/gs130_vio.yaml
"""
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import Image, Imu, CameraInfo
from tf2_ros import StaticTransformBroadcaster


class Gs130VioBridge(Node):

    def __init__(self):
        super().__init__('gs130_vio_bridge')
        p = self.declare_parameter

        self.flip_eyes = p('flip_eyes', False).value
        self.baseline = p('baseline', 0.07).value       # m, GS130WI
        self.imu_frame = p('imu_frame', 'imu0').value
        self.cam0_frame = p('cam0_frame', 'cam0').value
        self.cam1_frame = p('cam1_frame', 'cam1').value
        self.publish_static_tf = p('publish_static_tf', True).value
        # T_imu<-cam (optical frame: x right, y down, z forward);
        # cameras at +/- baseline/2 along imu x
        self.T_imu_cam0 = p('T_imu_cam0',
                            [-self.baseline / 2.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 1.0]).value
        self.T_imu_cam1 = p('T_imu_cam1',
                            [self.baseline / 2.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 1.0]).value

        self._bridge = CvBridge()

        # outputs
        self._pub_cam0 = self.create_publisher(Image, '/cam0/image_raw', 10)
        self._pub_cam1 = self.create_publisher(Image, '/cam1/image_raw', 10)
        self._pub_imu0 = self.create_publisher(Imu, '/imu0', 10)
        self._pub_info0 = self.create_publisher(
            CameraInfo, '/cam0/camera_info', 10)
        self._pub_info1 = self.create_publisher(
            CameraInfo, '/cam1/camera_info', 10)

        # inputs (verified topic names, see module docstring).
        # flip_eyes swaps the left/right assignment (cam0/cam1 convention)
        cam0_topic = '/image_right_raw' if self.flip_eyes else '/image_left_raw'
        cam1_topic = '/image_left_raw' if self.flip_eyes else '/image_right_raw'
        self._sub_left = self.create_subscription(
            Image, cam0_topic, self._make_img_cb(
                self._pub_cam0, self.cam0_frame), 10)
        self._sub_right = self.create_subscription(
            Image, cam1_topic, self._make_img_cb(
                self._pub_cam1, self.cam1_frame), 10)
        self._sub_imu = self.create_subscription(
            Imu, '/imu_data', self._imu_cb, 10)
        self._sub_info0 = self.create_subscription(
            CameraInfo, cam0_topic + '/camera_info', self._info0_cb, 10)
        self._sub_info1 = self.create_subscription(
            CameraInfo, cam1_topic + '/camera_info', self._info1_cb, 10)

        if self.publish_static_tf:
            self._tf_pub = StaticTransformBroadcaster(self)
            self._tf_pub.sendTransform(
                [self._make_tf(self.cam0_frame, self.T_imu_cam0),
                 self._make_tf(self.cam1_frame, self.T_imu_cam1)])

        self.get_logger().info(
            f'gs130_vio_bridge ready: cam0<-{cam0_topic}, '
            f'cam1<-{cam1_topic} (mono8), /imu_data -> /imu0, '
            f'baseline={self.baseline} m')

    def _make_img_cb(self, pub, frame):

        def cb(msg):
            # cv_bridge has no nv12 support: take the Y plane directly.
            # nv12 layout: Y plane (w x y_rows) then interleaved UV;
            # the declared height is y_rows * 1.5.
            w, h = msg.width, msg.height
            y_rows = h * 2 // 3
            expected = w * h * 3 // 2
            if len(msg.data) != expected:
                self.get_logger().error(
                    f'{msg.width}x{msg.height} nv12: got '
                    f'{len(msg.data)}B, expected {expected}B')
                return
            y = np.frombuffer(msg.data, dtype=np.uint8,
                              count=w * y_rows).reshape(y_rows, w)
            out = self._bridge.cv2_to_imgmsg(y, encoding='mono8')
            out.header = msg.header
            out.header.frame_id = frame
            pub.publish(out)

        return cb

    def _make_tf(self, child, t):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.imu_frame
        tf.child_frame_id = child
        tf.transform.translation = Vector3(x=t[0], y=t[1], z=t[2])
        tf.transform.rotation = Quaternion(x=t[3], y=t[4], z=t[5], w=t[6])
        return tf

    def _imu_cb(self, msg):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.imu_frame
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._pub_imu0.publish(out)

    def _info0_cb(self, msg):
        msg.header.frame_id = self.cam0_frame
        self._pub_info0.publish(msg)

    def _info1_cb(self, msg):
        msg.header.frame_id = self.cam1_frame
        self._pub_info1.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Gs130VioBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
