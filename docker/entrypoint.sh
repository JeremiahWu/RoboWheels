#!/bin/bash
# RoboWheels container entrypoint: sources ROS2 + TROS + the robowheels
# workspace, exposes the uv venv to the system python that runs the
# ros2 CLI, then executes the given command.
set -e

source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /opt/robowheels/ros2_ws/install/setup.bash 2>/dev/null || true

# the ros2 CLI runs with the system python3; make the uv-managed
# packages (Hobot.GPIO, pyserial, ...) visible to it
export PYTHONPATH="/opt/venv/lib/python3.10/site-packages:${PYTHONPATH}"
export PATH="/opt/venv/bin:${PATH}"

exec "$@"
