"""Normalize X5 StereoNet depth (uint16 millimeters) and optical XYZ."""

import copy
from array import array
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer


class StereoNetAdapter(Node):
    def __init__(self):
        super().__init__("stereonet_adapter")
        self.bridge = CvBridge()
        self.info = None
        self.depth_pub = self.create_publisher(Image, "/stereo/depth", 1)
        self.image_pub = self.create_publisher(Image, "/perception/rectified/image", 1)
        self.info_pub = self.create_publisher(
            CameraInfo, "/perception/rectified/camera_info", 1
        )
        self.cloud_pub = self.create_publisher(PointCloud2, "/stereo/points", 1)
        self.create_subscription(
            CameraInfo,
            "/StereoNetNode/stereonet_depth/camera_info",
            self.info_cb,
            qos_profile_sensor_data,
        )
        self.images = Subscriber(
            self,
            Image,
            "/StereoNetNode/rectify_left_image",
            qos_profile=qos_profile_sensor_data,
        )
        self.depths = Subscriber(
            self,
            Image,
            "/StereoNetNode/stereonet_depth",
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = TimeSynchronizer([self.images, self.depths], 5)
        self.sync.registerCallback(self.receive)

    def info_cb(self, msg):
        self.info = msg

    def receive(self, image, depth):
        info = self.info
        if (
            info is None
            or (info.width, info.height) != (depth.width, depth.height)
            or (image.width, image.height) != (depth.width, depth.height)
        ):
            return
        if depth.encoding not in ("mono16", "16UC1"):
            return
        K = np.asarray(info.k).reshape(3, 3)
        if not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
            return
        meters = (
            self.bridge.imgmsg_to_cv2(depth, "passthrough").astype(np.float32) * 0.001
        )
        meters[(meters < 0.2) | (meters > 5.0)] = np.nan
        # Canonical frame denotes the same left optical axes for all outputs.
        header = copy.deepcopy(depth.header)
        header.frame_id = "camera_optical_frame"
        out = Image(
            header=header,
            height=meters.shape[0],
            width=meters.shape[1],
            encoding="32FC1",
            step=meters.shape[1] * 4,
            data=array("B", meters.astype("<f4").tobytes()),
        )
        self.depth_pub.publish(out)
        eye = copy.copy(image)
        eye.header = header
        self.image_pub.publish(eye)
        ci = copy.deepcopy(info)
        ci.header = header
        self.info_pub.publish(ci)
        vv, uu = np.indices(meters.shape)
        valid = np.isfinite(meters) & (vv % 4 == 0) & (uu % 4 == 0)
        z = meters[valid]
        x = (uu[valid] - K[0, 2]) * z / K[0, 0]
        y = (vv[valid] - K[1, 2]) * z / K[1, 1]
        xyz = np.column_stack((x, y, z)).astype("<f4")
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(xyz)
        cloud.fields = [
            PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate(("x", "y", "z"))
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.width * 12
        cloud.is_dense = True
        cloud.data = array("B", xyz.tobytes())
        self.cloud_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = StereoNetAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
