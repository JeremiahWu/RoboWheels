#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
target_display="${DISPLAY:-:1}"
export DISPLAY=:1
Xvfb :1 -screen 0 1600x1000x24 -s 0 -nolisten tcp &
for attempt in $(seq 1 50); do
    test -S /tmp/.X11-unix/X1 && break
    sleep 0.1
done
if [[ "${ROBOWHEELS_DISPLAY_MODE:-browser}" == "xquartz" ]]; then
    export DISPLAY="$target_display"
    test -r "${XAUTHORITY:?XAUTHORITY must point to the mounted XQuartz cookie}"
    timeout 10 xdpyinfo -display "$DISPLAY" > /tmp/xquartz-display.log
    DISPLAY=:1 timeout 15 glxinfo -B > /tmp/virtualgl-renderer.log
else
    openbox > /tmp/openbox.log 2>&1 &
    x11vnc -display :1 -localhost -forever -shared -nopw -rfbport 5900 > /tmp/x11vnc.log 2>&1 &
    websockify --web=/usr/share/novnc 0.0.0.0:6080 localhost:5900 > /tmp/novnc.log 2>&1 &
fi
exec ros2 launch robowheels_sim control.launch.py
