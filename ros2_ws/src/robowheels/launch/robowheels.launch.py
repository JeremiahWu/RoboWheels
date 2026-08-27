"""Full RoboWheels stack.

Nodes: motor_driver, imu_publisher, joy (joy_node), joy_controller,
obstacle_avoid. The COIN-D4 lidar (cspc_lidar, separate workspace) is
included when its share directory is found.

Usage:
    ros2 launch robowheels robowheels.launch.py            # with lidar
    ros2 launch robowheels robowheels.launch.py with_lidar:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                             IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robowheels')
    motor_params = os.path.join(share, 'config', 'motor.yaml')
    control_params = os.path.join(share, 'config', 'control.yaml')

    with_lidar = LaunchConfiguration('with_lidar')
    with_lidar_arg = DeclareLaunchArgument(
        'with_lidar', default_value='true',
        description='also launch the COIN-D4 cspc_lidar driver')

    # cspc_lidar lives in its own workspace (drivers/cspc_lidar_sdk_ros2)
    lidar_share = None
    for base in ('/root/RoboWheels/drivers/cspc_lidar_sdk_ros2/install',
                 '/root/RoboWheels/ros2_ws/install'):
        cand = os.path.join(base, 'cspc_lidar', 'share', 'cspc_lidar')
        if os.path.isdir(cand):
            lidar_share = cand
            break

    actions = [
        with_lidar_arg,
        Node(package='robowheels', executable='motor_driver',
             name='motor_driver', output='screen',
             parameters=[motor_params]),
        Node(package='robowheels', executable='imu_publisher',
             name='imu_publisher', output='screen',
             parameters=[control_params]),
        Node(package='joy', executable='joy_node', name='joy_node',
             output='screen',
             parameters=[{'autorepeat_rate': 0.0, 'deadzone': 0.05}]),
        Node(package='robowheels', executable='joy_controller',
             name='joy_controller', output='screen',
             parameters=[control_params]),
        Node(package='robowheels', executable='obstacle_avoid',
             name='obstacle_avoid', output='screen',
             parameters=[control_params]),
    ]

    if lidar_share is not None:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(lidar_share, 'launch', 'lidar_launch.py')),
                condition=IfCondition(with_lidar)))
    else:
        actions.append(LogInfo(
            msg='cspc_lidar not found in either workspace install - '
                'start the lidar manually'))

    return LaunchDescription(actions)
