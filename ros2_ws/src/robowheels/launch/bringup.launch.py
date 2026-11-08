"""Compose independently selectable hardware launch files.

Sensors start by default. Base actuation requires enable_motor:=true;
joy additionally requires enable_joy:=true. The unconnected Astra stays off.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("robowheels")
    actions = []
    for name, default, file in [
        ("motor", "false", "motor"),
        ("joy", "false", "joy"),
        ("gs130", "true", "gs130"),
        ("astra", "false", "astra_camera"),
    ]:
        actions.append(DeclareLaunchArgument("enable_" + name, default_value=default))
        kwargs = (
            {"launch_arguments": {"mipi_lpwm_enable": "true"}.items()}
            if name == "gs130"
            else {}
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", file + ".launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("enable_" + name)),
                **kwargs,
            )
        )
    return LaunchDescription(actions)
