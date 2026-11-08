"""X5 BPU stereo + ORB VO + 2D ESDF preview. No base actuation."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robowheels")
    stereo = get_package_share_directory("hobot_stereonet")
    mipi = get_package_share_directory("mipi_cam")
    return LaunchDescription(
        [
            SetEnvironmentVariable("OPENBLAS_NUM_THREADS", "1"),
            SetEnvironmentVariable("OMP_NUM_THREADS", "1"),
            # Match model dimensions to avoid double intrinsic scaling in StereoNet 2.5.5.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mipi, "launch", "mipi_cam_dual_channel.launch.py")
                ),
                launch_arguments={
                    "mipi_image_width": "544",
                    "mipi_image_height": "448",
                    "mipi_image_framerate": "10.0",
                    "mipi_out_format": "nv12",
                    "mipi_gdc_enable": "true",
                    "mipi_lpwm_enable": "true",
                    "mipi_channel": "2",
                    "mipi_channel2": "0",
                    "mipi_frame_ts_type": "realtime",
                    "log_level": "warn",
                }.items(),
            ),
            Node(
                package="robowheels",
                executable="gs130_vio_bridge",
                parameters=[
                    os.path.join(share, "config", "gs130_vio.yaml"),
                    {"publish_static_tf": False},
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(stereo, "launch", "stereonet_model.launch.py")
                ),
                launch_arguments={
                    "stereonet_model_file_path": os.path.join(
                        stereo, "config", "DStereoV2.4_int8_544_448_96.bin"
                    ),
                    "camera_info_topic": "/image_right_raw/camera_info",
                    "left_camera_info_topic": "/image_left_raw/camera_info",
                    "publish_pcd_enabled": "false",
                    "publish_visual_enabled": "false",
                    "publish_origin_enable": "false",
                    "infer_thread_num": "1",
                    "log_level": "warn",
                }.items(),
            ),
            Node(package="robowheels", executable="stereonet_adapter", output="screen"),
            Node(
                package="robowheels",
                executable="orb_odometry_node",
                parameters=[{"broadcast_stereonet_tf": True}],
                output="screen",
            ),
            Node(
                package="robowheels",
                executable="local_map_node",
                parameters=[{"optical_frame": "camera_optical_frame"}],
                output="screen",
            ),
            Node(package="robowheels", executable="planning_node", output="screen"),
        ]
    )
