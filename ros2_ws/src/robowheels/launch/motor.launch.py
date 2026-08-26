import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('robowheels'), 'config', 'motor.yaml')
    return LaunchDescription([
        Node(package='robowheels',
             executable='motor_driver',
             name='motor_driver',
             output='screen',
             parameters=[params]),
    ])
