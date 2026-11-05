#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export DISPLAY=:1
Xvfb :1 -screen 0 1600x1000x24 -s 0 -nolisten tcp &
for attempt in $(seq 1 50); do
    test -S /tmp/.X11-unix/X1 && break
    sleep 0.1
done
openbox > /tmp/openbox.log 2>&1 &
x11vnc -display :1 -localhost -forever -shared -nopw -rfbport 5900 > /tmp/x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc 0.0.0.0:6080 localhost:5900 > /tmp/novnc.log 2>&1 &
exec ros2 launch robowheels_sim control.launch.py
