"""RoboWheels full bringup (RDK X5).

Starts:
  - motor_driver   (AT8236 + encoders + odom, subscribes /cmd_vel)
  - joy + teleop_twist_joy  (gamepad -> /cmd_vel)
  - gs130.launch   (GS130WI stereo+IMU -> VIO topics)     [default on]
  - astra_camera   (Astra Pro RGB + IR + depth)           [default on]

Usage:
  ros2 launch robowheels bringup.launch.py
  ros2 launch robowheels bringup.launch.py enable_gs130:=false \
      enable_astra:=false enable_joy:=false

Environment (important - the tros mipi_cam driver lives in /opt/tros):
  source /opt/ros/humble/setup.bash
  source /opt/tros/humble/setup.bash
  source ~/RoboWheels/ros2_ws/install/setup.bash
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(ctx, name):
    return LaunchConfiguration(name).perform(ctx).lower() in (
        'true', '1', 'yes')


def generate_launch_description():
    share = get_package_share_directory('robowheels')

    enable_joy = LaunchConfiguration('enable_joy')
    enable_gs130 = LaunchConfiguration('enable_gs130')
    enable_astra = LaunchConfiguration('enable_astra')

    motor = Node(package='robowheels', executable='motor_driver',
                 name='motor_driver', output='screen',
                 parameters=[os.path.join(share, 'config', 'motor.yaml')])

    joy = Node(package='joy', executable='joy_node', name='joy_node',
               output='screen',
               parameters=[{'device_id': 0, 'deadzone': 0.05,
                            'autorepeat_rate': 20.0}],
               condition=IfCondition(enable_joy))

    teleop = Node(package='teleop_twist_joy', executable='teleop_node',
                  name='teleop_twist_joy_node', output='screen',
                  parameters=[os.path.join(share, 'config', 'joy.yaml')],
                  condition=IfCondition(enable_joy))

    return LaunchDescription([
        DeclareLaunchArgument('enable_joy', default_value='true'),
        DeclareLaunchArgument('enable_gs130', default_value='true'),
        DeclareLaunchArgument('enable_astra', default_value='true'),

        motor,
        joy,
        teleop,

        # cameras are launched via their own launch files so their args
        # stay available; wrap in GroupAction to keep the include clean
        GroupAction(
            actions=[
                _include_gs130(share),
            ],
            condition=IfCondition(enable_gs130),
        ),
        GroupAction(
            actions=[
                _include_astra(share),
            ],
            condition=IfCondition(enable_astra),
        ),
    ])


def _include_gs130(share):
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'gs130.launch.py')))


def _include_astra(share):
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'astra_camera.launch.py')))
