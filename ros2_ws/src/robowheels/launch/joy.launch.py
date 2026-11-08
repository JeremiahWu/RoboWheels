"""Gamepad -> /cmd_vel (joy + teleop_twist_joy)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('robowheels'), 'config', 'joy.yaml')
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0')

    return LaunchDescription([
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0',
                              description='joystick device node'),
        Node(package='joy_linux', executable='joy_linux_node', name='joy_node',
             output='screen',
             parameters=[{'dev': joy_dev, 'deadzone': 0.08,
                          'autorepeat_rate': 20.0}]),
        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop_twist_joy_node',
             output='screen',
             parameters=[params],
             remappings=[('/cmd_vel', '/cmd_vel')]),
    ])
