#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
Xvfb :1 -s 0 -screen 0 1600x1000x24 -nolisten tcp &
for i in {1..50}; do test -S /tmp/.X11-unix/X1 && break; sleep .1; done
if [[ "${ROBOWHEELS_DEBUG_DISPLAY:-web}" == web ]]; then
  export DISPLAY=:1
  openbox >/tmp/openbox.log 2>&1 &
  x11vnc -display :1 -noxdamage -localhost -forever -shared -nopw -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
  websockify --web /usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 &
  exec rviz2 -d /debug/rdk_debug.rviz
else
  exec /opt/VirtualGL/bin/vglrun -nodl -d :1 -c proxy rviz2 -d /debug/rdk_debug.rviz
fi
