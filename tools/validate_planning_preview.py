#!/usr/bin/env python3
"""ROS integration checks in isolated topics; no hardware or velocity commands."""

import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

rclpy.init()
n = Node("validate_planning_preview")
latest = {}
pub = n.create_publisher(OccupancyGrid, "/validation/grid", 1)
goals = n.create_publisher(PoseStamped, "/validation/goal", 1)
n.create_subscription(Path, "/validation/path", lambda m: latest.update(path=m), 1)
n.create_subscription(
    String, "/validation/status", lambda m: latest.update(status=m.data), 1
)
args = [
    sys.executable,
    "-c",
    "from robowheels.planning.planning_node import main; main()",
    "--ros-args",
    "-r",
    "__node:=validation_planner",
    "-r",
    "/perception/local_grid:=/validation/grid",
    "-r",
    "/goal_pose:=/validation/goal",
    "-r",
    "/planning/path:=/validation/path",
    "-r",
    "/planning/status:=/validation/status",
]
process = subprocess.Popen(args, stdout=subprocess.DEVNULL)


def spin(seconds, send=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if send:
            send()
        rclpy.spin_once(n, timeout_sec=0.1)


def send():
    grid.header.stamp = n.get_clock().now().to_msg()
    pub.publish(grid)
    goals.publish(goal)


try:
    grid = OccupancyGrid()
    grid.header.frame_id = "camera_nav"
    grid.info.width = grid.info.height = 40
    grid.info.resolution = 0.1
    grid.info.origin.position.x = grid.info.origin.position.y = -2.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 1600
    goal = PoseStamped()
    goal.header.frame_id = "camera_nav"
    goal.pose.position.x = 1.0
    goal.pose.orientation.w = 1.0
    spin(4, send)
    assert len(latest["path"].poses) > 2, latest.get("status")
    grid.data = [100] * 1600
    spin(1.5, send)
    assert not latest["path"].poses and latest["status"] == "NO_KNOWN_FREE_PATH"
    grid.data = [0] * 1600
    goal.header.frame_id = "wrong_frame"
    spin(1.5, send)
    assert not latest["path"].poses and latest["status"].startswith(
        "GOAL_FRAME_MISMATCH"
    )
    goal.header.frame_id = "camera_nav"
    spin(1, send)
    spin(2.5)
    assert not latest["path"].poses and latest["status"] == "STALE_GRID", latest
    print(
        "Passed ROS path output, occupied map rejection, wrong-frame rejection and stale-path clearing"
    )
finally:
    process.terminate()
    process.wait(timeout=5)
    n.destroy_node()
    rclpy.shutdown()
