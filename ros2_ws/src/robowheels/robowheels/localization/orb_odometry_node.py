"""ORB + BPU stereo depth visual odometry, with explicit tracking health."""

import json
from array import array
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer
from scipy.spatial.transform import Rotation
from robowheels.localization.orb_odometry import OrbOdometry


class OrbOdometryNode(Node):
    def __init__(self):
        super().__init__("orb_odometry_node")
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        self.broadcast = self.declare_parameter("broadcast_stereonet_tf", False).value
        self.tf = TransformBroadcaster(self)
        self.engine = OrbOdometry()
        self.info = None
        self.pending = None
        self.last = None
        self.path = Path()
        self.status = self.create_publisher(String, "/localization/status", 1)
        self.odom = self.create_publisher(Odometry, "/localization/vo", 1)
        self.path_pub = self.create_publisher(Path, "/localization/vo_path", 1)
        self.debug = self.create_publisher(Image, "/perception/orb_features", 1)
        self.create_subscription(
            CameraInfo,
            "/perception/rectified/camera_info",
            self.info_cb,
            qos_profile_sensor_data,
        )
        self.images = Subscriber(
            self,
            Image,
            "/perception/rectified/image",
            qos_profile=qos_profile_sensor_data,
        )
        self.depths = Subscriber(
            self, Image, "/stereo/depth", qos_profile=qos_profile_sensor_data
        )
        self.sync = TimeSynchronizer([self.images, self.depths], 5)
        self.sync.registerCallback(self.pair_cb)
        self.create_timer(0.2, self.process)
        self.create_timer(1.0, self.watchdog)

    def info_cb(self, msg):
        self.info = msg

    def pair_cb(self, image, depth):
        self.pending = (image, depth)

    def watchdog(self):
        if self.last is None or time.monotonic() - self.last > 1.5:
            if self.engine.previous is not None:
                self.engine.reset()
            self.path = Path()
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path.header.frame_id = "vo_odom_" + str(self.engine.segment)
            self.path_pub.publish(self.path)
            self.status.publish(
                String(
                    data=json.dumps(
                        {"state": "NO_FRESH_SYNCHRONIZED_RGBD", "imu_fused": False}
                    )
                )
            )

    def process(self):
        if self.pending is None or self.info is None:
            return
        image, depth = self.pending
        self.pending = None
        stamp = image.header.stamp.sec + image.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if not -0.1 <= age <= 1.0:
            return
        if (image.width, image.height) != (depth.width, depth.height) or (
            image.width,
            image.height,
        ) != (self.info.width, self.info.height):
            self.status.publish(String(data="IMAGE_DEPTH_INTRINSICS_SIZE_MISMATCH"))
            return
        if (
            image.header.frame_id != depth.header.frame_id
            or self.info.header.frame_id != image.header.frame_id
        ):
            self.status.publish(String(data="OPTICAL_FRAME_MISMATCH"))
            return
        if image.encoding.lower() == "nv12":
            gray = (
                np.frombuffer(
                    image.data, dtype=np.uint8, count=image.step * image.height
                )
                .reshape(image.height, image.step)[:, : image.width]
                .copy()
            )
        else:
            gray = self.bridge.imgmsg_to_cv2(image, "mono8")
        # The BPU adapter normalizes depth to float32 meters.
        if depth.encoding != "32FC1":
            self.status.publish(String(data="DEPTH_MUST_BE_32FC1_METERS"))
            return
        z = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        K = np.asarray(self.info.k, dtype=float).reshape(3, 3)
        if K[0, 0] <= 0 or K[1, 1] <= 0 or not np.isfinite(K).all():
            return
        result = self.engine.update(gray, z, K, stamp)
        self.last = time.monotonic()
        shown = cv2.drawKeypoints(
            gray, result.pop("keypoints"), None, color=(0, 255, 0)
        )
        debug = Image(
            header=image.header,
            height=shown.shape[0],
            width=shown.shape[1],
            encoding="bgr8",
            step=shown.shape[1] * 3,
            data=array("B", shown.tobytes()),
        )
        self.debug.publish(debug)
        pose = result.pop("pose")
        self.status.publish(String(data=json.dumps(dict(result, imu_fused=False))))
        if pose is None:
            self.path = Path()
            self.path.header = image.header
            self.path.header.frame_id = "vo_odom_" + str(result["segment"])
            self.path_pub.publish(self.path)
            return
        m = Odometry()
        m.header.stamp = image.header.stamp
        m.header.frame_id = "vo_odom_" + str(result["segment"])
        m.child_frame_id = image.header.frame_id
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(
            float, pose[:3, 3]
        )
        q = Rotation.from_matrix(pose[:3, :3]).as_quat()
        (
            m.pose.pose.orientation.x,
            m.pose.pose.orientation.y,
            m.pose.pose.orientation.z,
            m.pose.pose.orientation.w,
        ) = map(float, q)
        # Conservative placeholder covariance, not a calibrated fusion covariance.
        for i in (0, 7, 14):
            m.pose.covariance[i] = 1.0
        for i in (21, 28, 35):
            m.pose.covariance[i] = 0.5
        for i in (0, 7, 14, 21, 28, 35):
            m.twist.covariance[i] = 1e6
        self.odom.publish(m)
        if self.broadcast:
            # StereoNet owns camera_link -> camera_optical_frame (standard ROS axes).
            # Publish its parent, rather than assigning optical frame a second parent.
            optical_from_link = np.eye(4)
            optical_from_link[:3, :3] = [[0, -1, 0], [0, 0, -1], [1, 0, 0]]
            world_link = pose @ optical_from_link
            tf = TransformStamped()
            tf.header = m.header
            tf.child_frame_id = "camera_link"
            (
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            ) = map(float, world_link[:3, 3])
            tq = Rotation.from_matrix(world_link[:3, :3]).as_quat()
            (
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w,
            ) = map(float, tq)
            self.tf.sendTransform(tf)
        if self.path.header.frame_id != m.header.frame_id:
            self.path = Path()
        self.path.header = m.header
        p = PoseStamped()
        p.header = m.header
        p.pose = m.pose.pose
        self.path.poses.append(p)
        self.path.poses = self.path.poses[-1000:]
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OrbOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
