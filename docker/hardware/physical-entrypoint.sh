#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export DISPLAY=:1
export PYTHONPATH="/physical/python:${PYTHONPATH:-}"
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
cleanup() { jobs -pr | xargs -r kill 2>/dev/null || true; }
trap cleanup EXIT TERM INT
Xvfb :1 -screen 0 1600x1000x24 -s 0 -nolisten tcp &
for i in {1..50}; do test -S /tmp/.X11-unix/X1 && break; sleep .1; done
openbox >/tmp/openbox.log 2>&1 &
x11vnc -display :1 -noxdamage -localhost -forever -shared -nopw -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
websockify --web /usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 &
python3 /physical/physical_arm_model.py /ws/install/robowheels_description/share/robowheels_description/urdf/robot_control.urdf /tmp/physical.urdf
python3 - <<'PY'
from pathlib import Path
import yaml
Path('/tmp/physical-params.yaml').write_text(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': Path('/tmp/physical.urdf').read_text()}}}))
PY
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file /tmp/physical-params.yaml >/tmp/physical-state-publisher.log 2>&1 &
python3 -m robowheels.arms.ros_shadow --token-file /run/servo-token >/tmp/servo-shadow.log 2>&1 &
rviz2 -d /physical/physical_arms.rviz >/tmp/physical-rviz.log 2>&1 &
wait -n
