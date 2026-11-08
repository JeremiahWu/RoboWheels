#!/usr/bin/env python3
"""GS130WI stereo point cloud node (self-contained, OpenCV CPU matching).

Takes the GDC-rectified stereo pair from the tros mipi_cam driver
(/image_left_raw, /image_right_raw, nv12), runs stereo matching
(SGBM or BM) and publishes:

  /stereo/points    sensor_msgs/PointCloud2  (x/y/z + intensity,
                    frame: camera_link by default)
  /stereo/depth     sensor_msgs/Image 32FC1 (meters)
  /stereo/disparity sensor_msgs/Image 32FC1 (pixels, optional)
  static TF imu0 -> camera_link

This replaces the external hobot_stereonet (BPU) pipeline with a
self-contained implementation.  Stereonet stays available as the
high-accuracy/high-framerate alternative:
  bash /opt/tros/humble/share/hobot_stereonet/script/run_stereo.sh

Calibration (measured 2026-09-10 from the mipi_cam driver log):
  rectify [f, cx, cy, baseline] = [511.53, 539.059, 681.172, 0.06889]
  in the 1088x1280 plane.  The 816x960 gdc output is an exact 0.75x
  downscale, so the output-plane intrinsics are:
  f = 511.53*0.75 = 383.65, cx = 404.29, cy = 510.88.
  All values are parameters (config/stereo_pc.yaml) - if the cloud
  looks squashed/stretched, adjust fx/fy first.

The Astra Pro depth point cloud is NOT generated here: its depth/IR
sensor needs the Orbbec SDK driver (currently incompatible with this
camera's firmware, see HARDWARE.md); once fixed, /astra/depth/points
comes directly from that driver.
"""
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

import cv2


class StereoPointCloudNode(Node):

    def __init__(self):
        super().__init__('stereo_pointcloud_node')
        p = self.declare_parameter

        # ---- topics ----
        self.left_topic = p('left_topic', '/image_left_raw').value
        self.right_topic = p('right_topic', '/image_right_raw').value
        self.points_topic = p('points_topic', '/stereo/points').value
        self.depth_topic = p('depth_topic', '/stereo/depth').value
        self.disp_topic = p('disparity_topic', '/stereo/disparity').value
        self.publish_disparity = p('publish_disparity', True).value

        # ---- calibration (output plane, see module docstring) ----
        self.fx = p('fx', 383.65).value
        self.fy = p('fy', 383.65).value
        self.cx = p('cx', 404.29).value
        self.cy = p('cy', 510.88).value
        self.baseline = p('baseline', 0.06889).value

        # ---- matching ----
        self.downsample = int(p('downsample', 2).value)
        matcher = p('matcher', 'sgbm').value
        self.num_disparities = int(p('num_disparities', 96).value)
        self.block_size = int(p('block_size', 7).value)
        self.min_disparity = int(p('min_disparity', 0).value)
        self.uniqueness = int(p('uniqueness_ratio', 10).value)
        self.speckle_window = int(p('speckle_window', 100).value)
        self.speckle_range = int(p('speckle_range', 2).value)
        self.min_depth = p('min_depth', 0.2).value
        self.max_depth = p('max_depth', 5.0).value
        self.cloud_step = int(p('cloud_step', 2).value)
        self.sync_slop = p('sync_slop', 0.05).value

        # ---- frames ----
        self.cloud_frame = p('cloud_frame', 'camera_link').value
        self.imu_frame = p('imu_frame', 'imu0').value
        self.publish_tf = p('publish_tf', True).value
        self.T_imu_cam = p('T_imu_cam',
                           [-0.035, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]).value

        # ---- matcher ----
        if matcher == 'sgbm':
            self._matcher = cv2.StereoSGBM_create(
                minDisparity=self.min_disparity,
                numDisparities=self.num_disparities,
                blockSize=self.block_size,
                P1=8 * 3 * self.block_size ** 2,
                P2=32 * 3 * self.block_size ** 2,
                uniquenessRatio=self.uniqueness,
                speckleWindowSize=self.speckle_window,
                speckleRange=self.speckle_range,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        elif matcher == 'bm':
            self._matcher = cv2.StereoBM_create(
                numDisparities=self.num_disparities,
                blockSize=self.block_size)
        else:
            raise ValueError(f'unknown matcher {matcher}')

        # ---- ros plumbing ----
        self._bridge = CvBridge()
        img_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        self._sub_l = Subscriber(self, Image, self.left_topic,
                                 qos_profile=img_qos)
        self._sub_r = Subscriber(self, Image, self.right_topic,
                                 qos_profile=img_qos)
        self._sync = ApproximateTimeSynchronizer(
            [self._sub_l, self._sub_r], 10, self.sync_slop)
        self._sync.registerCallback(self._pair_cb)

        self._pub_points = self.create_publisher(
            PointCloud2, self.points_topic, 10)
        self._pub_depth = self.create_publisher(Image, self.depth_topic, 10)
        self._pub_disp = (self.create_publisher(Image, self.disp_topic, 10)
                          if self.publish_disparity else None)

        if self.publish_tf:
            self._tf_pub = StaticTransformBroadcaster(self)
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.imu_frame
            tf.child_frame_id = self.cloud_frame
            tf.transform.translation = Vector3(
                x=self.T_imu_cam[0], y=self.T_imu_cam[1], z=self.T_imu_cam[2])
            tf.transform.rotation = Quaternion(
                x=self.T_imu_cam[3], y=self.T_imu_cam[4],
                z=self.T_imu_cam[5], w=self.T_imu_cam[6])
            self._tf_pub.sendTransform(tf)

        # worker thread: process only the newest pair (drop frames when
        # matching is slower than the camera)
        self._lock = threading.Lock()
        self._latest = None
        self._running = True
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

        ds = self.downsample
        self.get_logger().info(
            f'stereo_pointcloud ready: {matcher} ds={ds} '
            f'f={self.fx / ds:.1f} cx={self.cx / ds:.1f} '
            f'cy={self.cy / ds:.1f} b={self.baseline} '
            f'ndisp={self.num_disparities} -> {self.points_topic}')

    # ---- image pair handling ----

    def _pair_cb(self, left, right):
        try:
            yl = self._nv12_y(left)
            yr = self._nv12_y(right)
        except ValueError as exc:
            self.get_logger().error(f'bad image: {exc}')
            return
        if yl.shape != yr.shape:
            self.get_logger().error(
                f'shape mismatch L{yl.shape} R{yr.shape}')
            return
        with self._lock:
            self._latest = (left.header.stamp, yl, yr)

    @staticmethod
    def _nv12_y(msg):
        w, h = msg.width, msg.height
        y_rows = h
        if len(msg.data) != w * h * 3 // 2:
            raise ValueError(f'{w}x{h} nv12 needs {w * h * 3 // 2}B, '
                             f'got {len(msg.data)}')
        return np.frombuffer(msg.data, dtype=np.uint8,
                             count=w * y_rows).reshape(y_rows, w)

    def _loop(self):
        while self._running:
            with self._lock:
                pair = self._latest
                self._latest = None
            if pair is not None:
                try:
                    self._process(pair)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f'processing failed: {exc}')
            else:
                import time
                time.sleep(0.01)

    # ---- stereo -> depth -> cloud ----

    def _process(self, pair):
        stamp, yl, yr = pair
        ds = self.downsample
        if ds > 1:
            yl = cv2.resize(yl, (yl.shape[1] // ds, yl.shape[0] // ds),
                            interpolation=cv2.INTER_LINEAR)
            yr = cv2.resize(yr, (yr.shape[1] // ds, yr.shape[0] // ds),
                            interpolation=cv2.INTER_LINEAR)

        # SGBM/BM output is fixed-point S16 (1/16 px); filterSpeckles
        # only accepts CV_16SC1, so filter before the float conversion
        disp16 = self._matcher.compute(yl, yr)
        disp16, _ = cv2.filterSpeckles(disp16, 0, self.speckle_window,
                                       self.speckle_range)
        disp = disp16.astype(np.float32) / 16.0

        # disparity -> depth
        f = self.fx / ds
        cx = self.cx / ds
        cy = self.cy / ds
        with np.errstate(divide='ignore', invalid='ignore'):
            depth = f * self.baseline / disp
        valid = (disp > self.min_disparity + 1) & np.isfinite(depth) \
            & (depth > self.min_depth) & (depth < self.max_depth)

        # publish depth image
        depth_out = np.where(valid, depth, 0.0).astype(np.float32)
        now = self.get_clock().now().to_msg()
        dmsg = self._bridge.cv2_to_imgmsg(depth_out, encoding='32FC1')
        dmsg.header.stamp = stamp
        dmsg.header.frame_id = self.cloud_frame
        self._pub_depth.publish(dmsg)

        if self._pub_disp is not None:
            disp_out = np.where(valid, disp, 0.0).astype(np.float32)
            dsp = self._bridge.cv2_to_imgmsg(disp_out, encoding='32FC1')
            dsp.header.stamp = stamp
            dsp.header.frame_id = self.cloud_frame
            self._pub_disp.publish(dsp)

        # build point cloud (subsampled)
        vv, uu = np.nonzero(valid[::self.cloud_step, ::self.cloud_step])
        if vv.size == 0:
            return
        uu = uu * self.cloud_step
        vv = vv * self.cloud_step
        z = depth[vv, uu]
        x = (uu - cx) * z / f
        y = (vv - cy) * z / f
        intensity = yl[vv, uu].astype(np.float32)

        pts = np.empty(vv.size, dtype=[
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
            ('intensity', '<f4')])
        pts['x'] = x
        pts['y'] = y
        pts['z'] = z
        pts['intensity'] = intensity

        header = Header(stamp=stamp, frame_id=self.cloud_frame)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(header, fields, pts)
        cloud.header = header
        self._pub_points.publish(cloud)

    def shutdown(self):
        self._running = False
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = StereoPointCloudNode()
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
