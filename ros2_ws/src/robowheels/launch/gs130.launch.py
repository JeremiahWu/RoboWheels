"""GS130WI stereo IMU camera bringup for VIO.

Starts:
  1. tros mipi_cam (dual channel, gdc on, nv12) - publishes
     /image_combine_raw, /image_left_raw, /image_right_raw, /imu_data
  2. gs130_vio_bridge - converts to VIO conventions
     /cam0/image_raw, /cam1/image_raw, /imu0 + camera_info + static TF

Verified on the RDK X5 (2026-09-10): camera EEPROM calibration is read
automatically (camType 17), IMU (ICM-42688) publishes ~100 Hz.

Usage:
  ros2 launch robowheels gs130.launch.py
  ros2 launch robowheels gs130.launch.py mipi_image_width:=1088 \
      mipi_image_height:=1280 mipi_image_framerate:=30.0

Notes:
  - requires system version >= 3.4.1 and tros-humble-mipi-cam >= 2.5.0
  - camera module switch must be in EXT mode
  - the camera's black sync wire goes to 40PIN pin 37; for LPWM
    hardware sync run with mipi_lpwm_enable:=true (needs the wire)
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
        DeclareLaunchArgument('mipi_out_format', default_value='nv12'),
        DeclareLaunchArgument('mipi_image_width', default_value='816'),
        DeclareLaunchArgument('mipi_image_height', default_value='960'),
        DeclareLaunchArgument('mipi_image_framerate', default_value='10.0'),
        DeclareLaunchArgument('mipi_gdc_enable', default_value='True'),
        DeclareLaunchArgument('mipi_lpwm_enable', default_value='False'),
        DeclareLaunchArgument('mipi_channel', default_value='2'),
        DeclareLaunchArgument('mipi_channel2', default_value='0'),
        DeclareLaunchArgument('mipi_frame_ts_type', default_value='realtime'),
        DeclareLaunchArgument('log_level', default_value='warn'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mipi_share, 'launch',
                             'mipi_cam_dual_channel.launch.py')),
            launch_arguments={
                'mipi_out_format': LaunchConfiguration('mipi_out_format'),
                'mipi_image_width': LaunchConfiguration('mipi_image_width'),
                'mipi_image_height': LaunchConfiguration('mipi_image_height'),
                'mipi_image_framerate':
                    LaunchConfiguration('mipi_image_framerate'),
                'mipi_gdc_enable': LaunchConfiguration('mipi_gdc_enable'),
                'mipi_lpwm_enable': LaunchConfiguration('mipi_lpwm_enable'),
                'mipi_channel': LaunchConfiguration('mipi_channel'),
                'mipi_channel2': LaunchConfiguration('mipi_channel2'),
                'mipi_frame_ts_type':
                    LaunchConfiguration('mipi_frame_ts_type'),
                'log_level': LaunchConfiguration('log_level'),
            }.items()),

        Node(package='robowheels',
             executable='gs130_vio_bridge',
             name='gs130_vio_bridge',
             output='screen',
             parameters=[os.path.join(share, 'config', 'gs130_vio.yaml')]),
    ])
