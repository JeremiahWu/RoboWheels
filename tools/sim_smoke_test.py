#!/usr/bin/env python3
"""Integration check against a running simulation. Publishes motion commands."""
import json
import math
import re
import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock


def ground_truth():
    result = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', '/world/robowheels/dynamic_pose/info'],
                            capture_output=True, text=True, timeout=8, check=True)
    pose = re.search(r'name: "robowheels".*?position \{(.*?)\}.*?orientation \{(.*?)\}', result.stdout, re.S)
    assert pose, 'Gazebo model pose missing'
    values = []
    for block, axes in zip(pose.groups(), ('xyz', 'xyzw')):
        fields = dict(re.findall(r'([xyzw]):\s*([-+\deE.]+)', block))
        values.append([float(fields.get(axis, 0)) for axis in axes])
    return values


def main():
    rclpy.init()
    node = Node('robowheels_sim_smoke_test')
    received = {}
    subscriptions = [node.create_subscription(kind, topic, lambda msg, key=key: received.__setitem__(key, msg), 10)
                     for kind, topic, key in [(Odometry, '/odom', 'odom'), (JointState, '/joint_states', 'joints'),
                                              (Twist, '/sim/cmd_vel', 'command'), (Clock, '/clock', 'clock')]]
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    def run(seconds, linear=None, angular=0.0):
        end = time.monotonic() + seconds
        command = Twist()
        command.linear.x = float(linear or 0)
        command.angular.z = float(angular)
        while time.monotonic() < end:
            if linear is not None:
                publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        run(4)
        assert all(k in received for k in ('odom', 'joints', 'command', 'clock')), f'Missing topics: {received.keys()}'
        start = received['odom'].pose.pose.position
        initial = (start.x, start.y)
        initial_truth, _ = ground_truth()
        run(2, .12)
        run(1)
        stop = received['odom'].pose.pose.position
        distance = math.hypot(stop.x-initial[0], stop.y-initial[1])
        actual, orientation = ground_truth()
        actual_distance = math.hypot(actual[0]-initial_truth[0], actual[1]-initial_truth[1])
        assert actual_distance > .08, f'Wheels spin but chassis did not move: {actual_distance}'
        assert abs(actual[2]) < .02 and abs(orientation[0]) < .15 and abs(orientation[1]) < .15, 'Chassis fell or tipped'
        assert distance > .08, f'No forward movement: {distance}'
        assert received['command'].linear.x == 0, 'Watchdog did not stop stale command'
        assert abs(received['odom'].twist.twist.linear.x) < .03, 'Wheels did not stop'
        before = received['odom'].pose.pose.orientation
        yaw0 = math.atan2(2*(before.w*before.z+before.x*before.y), 1-2*(before.y**2+before.z**2))
        run(2, 0, .5)
        run(1)
        after = received['odom'].pose.pose.orientation
        yaw1 = math.atan2(2*(after.w*after.z+after.x*after.y), 1-2*(after.y**2+after.z**2))
        rotation = math.atan2(math.sin(yaw1-yaw0), math.cos(yaw1-yaw0))
        assert rotation > .2, f'No positive yaw response: {rotation}'
        assert {'left_wheel_joint','right_wheel_joint'} <= set(received['joints'].name)
        assert received['clock'].clock.sec > 0
        run(.2, 4, 8)
        assert received['command'].linear.x <= .5 and received['command'].angular.z <= 2
        run(.2, 0)
        result = {'passed': True, 'forward_distance_m': distance, 'positive_yaw_rad': rotation,
                  'physical_forward_distance_m': actual_distance, 'physical_height_m': actual[2],
                  'joint_names': list(received['joints'].name), 'watchdog_stopped': True,
                  'sim_time_sec': received['clock'].clock.sec}
        print(json.dumps(result, indent=2))
    finally:
        publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
