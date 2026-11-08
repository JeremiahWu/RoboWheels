"""Camera-relative perception/planning preview; never starts base control."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robowheels")
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "gs130.launch.py")
                ),
                launch_arguments={"mipi_lpwm_enable": "true"}.items(),
                condition=IfCondition(LaunchConfiguration("start_camera")),
            ),
            Node(
                package="robowheels",
                executable="stereo_pointcloud_node",
                output="screen",
                parameters=[os.path.join(share, "config", "stereo_pc.yaml")],
            ),
            Node(package="robowheels", executable="local_map_node", output="screen"),
            Node(package="robowheels", executable="planning_node", output="screen"),
        ]
    )
