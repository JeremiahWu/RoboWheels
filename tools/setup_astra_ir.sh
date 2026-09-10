#!/usr/bin/env bash
# Astra Pro IR/depth driver setup - OpenNI2 route (ros2_astra_camera).
#
# Why: verified 2026-09-10 that OrbbecSDK v2 (ros-humble-orbbec-camera
# 2.8.6) does NOT drive the classic Astra Pro (2bc5:0403): it enumerates
# the device but the vendor handshake fails ("Device response size(0) <
# response header size(10)").  Alternative A: update the camera firmware
# with OrbbecView on a PC and retry the v2 driver.  Alternative B (this
# script): build the OpenNI2-based driver.
#
# Steps are from the official ros2_astra_camera README (master branch),
# adapted for Humble.  You must download openNISDk_ROS2_*.tar.gz from
#   https://github.com/orbbec/ros2_astra_camera/releases
# and pass its path as the first argument.
set -e

SDK_TARBALL=${1:?usage: setup_astra_ir.sh /path/to/openNISDk_ROS2_xxx.tar.gz}
WS=${2:-$HOME/RoboWheels/ros2_ws}

echo ">> installing system deps ..."
sudo apt-get install -y libgflags-dev libgoogle-glog-dev libusb-1.0-0-dev \
    libeigen3-dev \
    ros-humble-image-geometry ros-humble-camera-info-manager \
    ros-humble-image-transport ros-humble-image-publisher

echo ">> building libuvc ..."
cd /tmp
if [ ! -d libuvc ]; then git clone https://github.com/libuvc/libuvc.git; fi
cd libuvc && mkdir -p build && cd build
cmake .. && make -j"$(nproc)" && sudo make install && sudo ldconfig

echo ">> extracting OpenNI2 SDK into workspace src ..."
mkdir -p "$WS/src"
tar -xzf "$SDK_TARBALL" -C "$WS/src"

echo ">> udev rules ..."
if [ -d "$WS/src/ros2_astra_camera/astra_camera/scripts" ]; then
    (cd "$WS/src/ros2_astra_camera/astra_camera/scripts" && sudo bash install.sh)
    sudo udevadm control --reload-rules && sudo udevadm trigger
fi

echo ">> building ..."
cd "$WS"
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ">> done. start with:"
echo "   source $WS/install/setup.bash"
echo "   ros2 launch astra_camera astra.launch.xml"
