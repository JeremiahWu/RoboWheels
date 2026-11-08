"""Launch installed hobot_vio only with an explicitly calibrated GS130WI config."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    config = LaunchConfiguration("config").perform(context)
    if not config or not os.path.isfile(config):
        raise RuntimeError(
            "Provide config:=/absolute/path/to/calibrated-gs130wi.yaml; RealSense defaults are not suitable."
        )
    if LaunchConfiguration("calibration_verified").perform(context).lower() != "true":
        raise RuntimeError(
            "Verify GS130WI intrinsics, T_CtoI, IMU units and timing before calibration_verified:=true."
        )
    return [
        Node(
            package="hobot_vio",
            executable="hobot_vio",
            output="screen",
            parameters=[
                {
                    "path_config": config,
                    "image_topic": "/cam0/image_raw",
                    "imu_topic": "/imu0",
                    "sample_gap": 1,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=""),
            DeclareLaunchArgument("calibration_verified", default_value="false"),
            OpaqueFunction(function=start),
        ]
    )
