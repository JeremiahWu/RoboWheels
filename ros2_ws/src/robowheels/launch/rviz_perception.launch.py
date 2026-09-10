"""Launch RViz2 with the perception config (IR + stereo point cloud).

Run on a PC on the same network / ROS_DOMAIN_ID as the X5, or on the
X5 itself if rviz2 + a display are available:

    ros2 launch robowheels rviz_perception.launch.py

The config lives in config/rviz/perception.rviz (fixed frame
camera_link, stereonet point cloud + IR + depth/rectified images).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robowheels'),
        'config', 'rviz', 'perception.rviz')
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen', arguments=['-d', config]),
    ])
