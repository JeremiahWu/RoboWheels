#!/usr/bin/env python3
"""Match the real base's 500 ms command timeout, with bounded commands."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Watchdog(Node):
    def __init__(self):
        super().__init__('sim_cmd_vel_watchdog')
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.last_received = None
        self.command = Twist()
        self.publisher = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self.receive, 10)
        self.create_timer(0.02, self.tick)

    def receive(self, msg):
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            self.command = Twist()
            self.last_received = None
            return
        linear = self.get_parameter('max_linear_velocity').value
        angular = self.get_parameter('max_angular_velocity').value
        self.command = Twist()
        self.command.linear.x = max(-linear, min(linear, msg.linear.x))
        self.command.angular.z = max(-angular, min(angular, msg.angular.z))
        self.last_received = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        age = (now - self.last_received).nanoseconds / 1e9 if self.last_received is not None else float('inf')
        valid = 0 <= age <= self.get_parameter('timeout').value
        self.publisher.publish(self.command if valid else Twist())


def main():
    rclpy.init()
    node = Watchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
