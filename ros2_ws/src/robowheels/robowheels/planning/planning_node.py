"""A* local path preview. No velocity publisher or motor interface."""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from robowheels.planning.grid import astar, signed_distance


class PlanningNode(Node):
    def __init__(self):
        super().__init__("planning_node")
        self.grid = None
        self.goal = None
        self.received = 0.0
        self.pub = self.create_publisher(Path, "/planning/path", 1)
        self.status = self.create_publisher(String, "/planning/status", 1)
        self.create_subscription(
            OccupancyGrid, "/perception/local_grid", self.map_cb, 1
        )
        self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_cb, qos_profile_sensor_data
        )
        self.create_timer(0.5, self.plan)

    def map_cb(self, msg):
        self.grid = msg
        self.received = time.monotonic()

    def goal_cb(self, msg):
        self.goal = msg

    def plan(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        g = self.grid
        reason = "WAITING_FOR_GRID_AND_GOAL"
        if g is not None:
            path.header.frame_id = g.header.frame_id
            age = (
                self.get_clock().now().nanoseconds
                - g.header.stamp.sec * 10**9
                - g.header.stamp.nanosec
            ) / 1e9
            q = g.info.origin.orientation
            if time.monotonic() - self.received > 1.5 or not -0.1 <= age <= 1.5:
                reason = "STALE_GRID"
            elif (
                not 0 < g.info.resolution < 10
                or not 0 < g.info.width <= 400
                or not 0 < g.info.height <= 400
                or not all(
                    math.isfinite(v)
                    for v in (g.info.origin.position.x, g.info.origin.position.y)
                )
                or len(g.data) != g.info.width * g.info.height
                or abs(q.w - 1) > 1e-6
                or abs(q.x) + abs(q.y) + abs(q.z) > 1e-6
            ):
                reason = "INVALID_GRID"
            elif self.goal is not None:
                if self.goal.header.frame_id != g.header.frame_id:
                    reason = "GOAL_FRAME_MISMATCH: use camera_nav; no base/global transform assumed"
                elif not all(
                    math.isfinite(v)
                    for v in (self.goal.pose.position.x, self.goal.pose.position.y)
                ):
                    reason = "INVALID_GOAL"
                else:

                    def cell(x, y):
                        return (
                            math.floor(
                                (x - g.info.origin.position.x) / g.info.resolution
                            ),
                            math.floor(
                                (y - g.info.origin.position.y) / g.info.resolution
                            ),
                        )

                    data = np.array(g.data, dtype=np.int8).reshape(
                        g.info.height, g.info.width
                    )
                    clearance = signed_distance(data, g.info.resolution)
                    cells = astar(
                        data,
                        cell(0, 0),
                        cell(self.goal.pose.position.x, self.goal.pose.position.y),
                        clearance=clearance,
                        clearance_weight=0.15,
                    )
                    for x, y in cells:
                        pose = PoseStamped()
                        pose.header = path.header
                        pose.pose.orientation.w = 1.0
                        pose.pose.position.x = (
                            g.info.origin.position.x + (x + 0.5) * g.info.resolution
                        )
                        pose.pose.position.y = (
                            g.info.origin.position.y + (y + 0.5) * g.info.resolution
                        )
                        path.poses.append(pose)
                    reason = "PREVIEW_PATH" if cells else "NO_KNOWN_FREE_PATH"
        self.pub.publish(path)
        self.status.publish(String(data=reason))


def main(args=None):
    rclpy.init(args=args)
    n = PlanningNode()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
