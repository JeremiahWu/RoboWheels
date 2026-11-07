"""Kinematic ros2_control mock hardware; independent of the Gazebo mode."""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description = Path(get_package_share_directory('robowheels_description'))
    sim = Path(get_package_share_directory('robowheels_sim'))
    robot = (description/'urdf/robot_control.urdf').read_text()
    native = os.environ.get('ROBOWHEELS_DISPLAY_MODE') == 'xquartz'
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[str(sim/'config/controllers.yaml')],
             remappings=[('~/robot_description','/robot_description'),
                         ('/base_controller/cmd_vel_unstamped','/cmd_vel'),
                         ('/base_controller/odom','/odom')], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster','--controller-manager-timeout','60'], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['arm_controller','--controller-manager-timeout','60'], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['base_controller','--controller-manager-timeout','60'], output='screen'),
        Node(package='robowheels_sim', executable='control_web.py', output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d',str(sim/'config' / ('control_native.rviz' if native else 'control.rviz'))],
             prefix='/opt/VirtualGL/bin/vglrun -nodl -d :1 -c proxy' if native else '', output='screen'),
    ])
