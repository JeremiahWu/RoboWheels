"""GS130WI stereo point cloud bringup.

Starts mipi_cam (dual channel, gdc on, nv12) + our self-contained
stereo_pointcloud_node (OpenCV SGBM -> /stereo/points + /stereo/depth).

Usage:
  ros2 launch robowheels stereo_pointcloud.launch.py
  ros2 launch robowheels stereo_pointcloud.launch.py mipi_image_framerate:=30.0

Note: this launch owns the mipi camera - do not run it together with
gs130.launch.py (its own mipi_cam instance) or hobot_stereonet's
run_stereo.sh (also owns the camera).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robowheels')
    mipi_share = get_package_share_directory('mipi_cam')

    return LaunchDescription([
        DeclareLaunchArgument('mipi_image_width', default_value='816'),
        DeclareLaunchArgument('mipi_image_height', default_value='960'),
        DeclareLaunchArgument('mipi_image_framerate', default_value='10.0'),
        DeclareLaunchArgument('mipi_gdc_enable', default_value='True'),
        DeclareLaunchArgument('mipi_lpwm_enable', default_value='False'),
        DeclareLaunchArgument('mipi_frame_ts_type', default_value='realtime'),
        DeclareLaunchArgument('log_level', default_value='warn'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mipi_share, 'launch',
                             'mipi_cam_dual_channel.launch.py')),
            launch_arguments={
                'mipi_out_format': 'nv12',
                'mipi_image_width': LaunchConfiguration('mipi_image_width'),
                'mipi_image_height': LaunchConfiguration('mipi_image_height'),
                'mipi_image_framerate':
                    LaunchConfiguration('mipi_image_framerate'),
                'mipi_gdc_enable': LaunchConfiguration('mipi_gdc_enable'),
                'mipi_lpwm_enable': LaunchConfiguration('mipi_lpwm_enable'),
                'mipi_frame_ts_type':
                    LaunchConfiguration('mipi_frame_ts_type'),
                'log_level': LaunchConfiguration('log_level'),
            }.items()),

        Node(package='robowheels',
             executable='stereo_pointcloud_node',
             name='stereo_pointcloud_node',
             output='screen',
             parameters=[os.path.join(share, 'config', 'stereo_pc.yaml')]),
    ])
