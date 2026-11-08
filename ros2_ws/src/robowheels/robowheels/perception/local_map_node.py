"""Camera-relative obstacle preview; does not infer camera-to-base extrinsics."""

from array import array
import time
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from robowheels.planning.grid import ray_grid, signed_distance


class LocalMapNode(Node):
    def __init__(self):
        super().__init__("local_map_node")
        p = self.declare_parameter
        self.frame = p("frame", "camera_nav").value
        self.optical = p("optical_frame", "camera_link").value
        self.res = float(p("resolution", 0.05).value)
        self.size = int(p("size", 120).value)
        self.radius = float(p("inflation_radius", 0.18).value)
        self.low = float(p("min_height", -0.2).value)
        self.high = float(p("max_height", 0.5).value)
        self.timeout = float(p("timeout", 1.5).value)
        if (
            self.res <= 0
            or not 10 <= self.size <= 400
            or self.radius < 0
            or self.low >= self.high
        ):
            raise ValueError("Invalid local map dimensions or obstacle slice")
        self.esdf_pub = self.create_publisher(Image, "/perception/esdf", 1)
        self.pub = self.create_publisher(OccupancyGrid, "/perception/local_grid", 1)
        self.status = self.create_publisher(String, "/perception/status", 1)
        self.sub = self.create_subscription(
            PointCloud2,
            "/stereo/points",
            self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.last = None
        self.tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id = self.optical
        t.child_frame_id = self.frame
        t.header.stamp = self.get_clock().now().to_msg()
        # camera_nav: forward=optical Z, left=-optical X, up=-optical Y.
        t.transform.rotation.x = 0.5
        t.transform.rotation.y = -0.5
        t.transform.rotation.z = 0.5
        t.transform.rotation.w = 0.5
        self.tf.sendTransform(t)
        self.create_timer(0.5, self.watchdog)

    def publish(self, data, stamp):
        m = OccupancyGrid()
        m.header.frame_id = self.frame
        m.header.stamp = stamp
        m.info.resolution = self.res
        m.info.width = m.info.height = self.size
        m.info.origin.position.x = m.info.origin.position.y = -self.size * self.res / 2
        m.info.origin.orientation.w = 1.0
        m.data = array("b", data.ravel().tobytes())
        self.pub.publish(m)

    def receive(self, msg):
        age = (
            self.get_clock().now().nanoseconds
            - msg.header.stamp.sec * 10**9
            - msg.header.stamp.nanosec
        ) / 1e9
        if msg.header.frame_id != self.optical or not -0.1 <= age <= self.timeout:
            self.last = None
            return
        fields = {field.name: field for field in msg.fields}
        if not all(
            name in fields and fields[name].datatype == 7 for name in ("x", "y", "z")
        ):
            self.last = None
            return
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [(">" if msg.is_bigendian else "<") + "f4"] * 3,
                "offsets": [fields[name].offset for name in ("x", "y", "z")],
                "itemsize": msg.point_step,
            }
        )
        raw = np.ndarray(
            (msg.height, msg.width),
            dtype=dtype,
            buffer=msg.data,
            strides=(msg.row_step, msg.point_step),
        )
        stride = max(1, raw.size // 4000)
        pts = np.column_stack([raw[name].ravel()[::stride] for name in ("x", "y", "z")])
        grid = ray_grid(pts, self.size, self.res, self.low, self.high)
        esdf = signed_distance(grid, self.res)
        distance = Image()
        distance.header = copy.deepcopy(msg.header)
        distance.header.frame_id = self.frame
        distance.height = distance.width = self.size
        distance.encoding = "32FC1"
        distance.step = self.size * 4
        distance.data = array("B", esdf.astype("<f4").tobytes())
        self.esdf_pub.publish(distance)
        # Conservative footprint clearance; unknown cells remain unknown.
        grid[(grid == 0) & (esdf <= self.radius + self.res * 2**0.5 / 2)] = 100
        self.last = time.monotonic()
        self.publish(grid, msg.header.stamp)
        self.status.publish(
            String(data="PREVIEW: camera-relative map; base extrinsics not calibrated")
        )

    def watchdog(self):
        if self.last is None or time.monotonic() - self.last > self.timeout:
            stamp = self.get_clock().now().to_msg()
            self.publish(np.full((self.size, self.size), -1, dtype=np.int8), stamp)
            blank = Image(
                height=self.size,
                width=self.size,
                encoding="32FC1",
                step=self.size * 4,
                data=array(
                    "B", np.full((self.size, self.size), np.nan, dtype="<f4").tobytes()
                ),
            )
            blank.header.frame_id = self.frame
            blank.header.stamp = stamp
            self.esdf_pub.publish(blank)
            self.status.publish(String(data="NO_FRESH_CLOUD"))


def main(args=None):
    rclpy.init(args=args)
    n = LocalMapNode()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
