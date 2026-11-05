"""Start a headless Fortress world and the ROS interfaces; GUI is optional."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description = Path(get_package_share_directory('robowheels_description'))
    simulation = Path(get_package_share_directory('robowheels_sim'))
    robot = (description / 'urdf/robot.urdf').read_text()
    # Absolute mesh paths also work in Gazebo after URDF -> SDF conversion.
    gazebo_robot = robot.replace('package://robowheels_description/', str(description) + '/')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', str(description.parent) + os.pathsep + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')),
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-s', str(simulation / 'worlds/empty.sdf')], output='screen'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-g'], condition=IfCondition(LaunchConfiguration('gui')), output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'robowheels', '-name', 'robowheels', '-string', gazebo_robot, '-z', '0.01'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(simulation / 'config/bridge.yaml'), 'use_sim_time': True}], output='screen'),
        Node(package='robowheels_sim', executable='cmd_vel_watchdog.py',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='robowheels_sim', executable='web_viewer.py',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             parameters=[{'port': 8765, 'address': '0.0.0.0', 'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('foxglove')), output='screen'),
    ])
