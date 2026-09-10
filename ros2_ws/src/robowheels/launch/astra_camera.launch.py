"""Astra Pro bringup.

Default path: official Orbbec driver (ros-humble-orbbec-camera) which
handles BOTH the RGB sensor and the depth/IR sensor (PID 2bc5:0403),
publishing /color/image_raw, /depth/image_raw, /ir/image_raw,
/depth/points and all camera_info topics.

If the official driver does not enumerate this classic Astra Pro (the
driver is built on OrbbecSDK v2; very old Astra Pro units may need the
OpenNI2-based driver instead), run with:

    ros2 launch robowheels astra_camera.launch.py use_sdk:=false

which starts the UVC-only RGB fallback node (astra_rgb_node).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _use_sdk(context, *args, **kwargs):
    return bool(LaunchConfiguration('use_sdk').perform(context).lower()
                in ('true', '1', 'yes'))


def generate_launch_description():
    share = get_package_share_directory('robowheels')

    use_sdk = LaunchConfiguration('use_sdk')
    enable_point_cloud = LaunchConfiguration('enable_point_cloud')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_ir = LaunchConfiguration('enable_ir')

    sdk_node = Node(
        package='orbbec_camera',
        executable='orbbec_camera_node',
        name='orbbec_camera',
        output='screen',
        parameters=[{
            'camera_name': 'astra',
            'enable_color': True,
            'enable_depth': enable_depth,
            'enable_ir': enable_ir,
            'enable_point_cloud': enable_point_cloud,
            'enable_accel': False,
            'enable_gyro': False,
            'depth_registration': True,
            'color_width': 1280,
            'color_height': 720,
            'color_fps': 30,
            'color_format': 'MJPG',
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30,
            'depth_format': 'Y16',
            'ir_width': 640,
            'ir_height': 480,
            'ir_fps': 30,
            'ir_format': 'Y16',
            'depth_frame_id': 'astra_depth_frame',
            'color_frame_id': 'astra_rgb_frame',
            'ir_frame_id': 'astra_ir_frame',
        }],
    )

    rgb_fallback = Node(
        package='robowheels',
        executable='astra_rgb_node',
        name='astra_rgb_node',
        output='screen',
        parameters=[os.path.join(share, 'config', 'astra_rgb.yaml')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sdk', default_value='true',
                              description='use official orbbec_camera '
                                          'driver (RGB+IR+depth)'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('enable_ir', default_value='true'),
        DeclareLaunchArgument('enable_point_cloud', default_value='false'),
        OpaqueFunction(function=lambda ctx: [sdk_node if _use_sdk(ctx)
                                             else rgb_fallback]),
    ])
