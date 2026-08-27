#!/bin/bash
# RoboWheels full-stack launcher: sources all workspaces and starts
# motor_driver + imu_publisher + joy + joy_controller + obstacle_avoid
# + cspc_lidar. Usage:
#   tools/start_stack.sh             # with lidar
#   tools/start_stack.sh nolidar     # skip the lidar
set -e

source /opt/ros/humble/setup.bash
source /root/RoboWheels/drivers/cspc_lidar_sdk_ros2/install/setup.bash
source /root/RoboWheels/ros2_ws/install/setup.bash

if [ "$1" = "nolidar" ]; then
    exec ros2 launch robowheels robowheels.launch.py with_lidar:=false
else
    exec ros2 launch robowheels robowheels.launch.py
fi
