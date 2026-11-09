#!/bin/bash
# Double-click in Finder to open real X5 telemetry. No motor commands are sent.
set -euo pipefail
export PATH="/Applications/Docker.app/Contents/Resources/bin:/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin:$PATH"
TASK_ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$TASK_ROOT"
trap 'echo; echo "启动失败，请保留此窗口中的提示以便排查。"; read -r -p "按回车关闭…" _' ERR
if ! docker info >/dev/null 2>&1; then
  echo "正在启动 Docker Desktop…"
  open -a Docker
  for task_attempt in {1..90}; do
    docker info >/dev/null 2>&1 && break
    sleep 1
  done
  docker info >/dev/null 2>&1 || { echo "Docker 尚未就绪。"; false; }
fi
echo "正在连接 RDK X5 并启动点云窗口…"
python3 scripts/rdk_debug.py start
TASK_URL='http://localhost:16081/vnc.html?autoconnect=1&resize=scale'
for task_attempt in {1..40}; do
  curl -fsS http://localhost:16081/vnc.html >/dev/null 2>&1 && break
  sleep .5
done
curl -fsS http://localhost:16081/vnc.html >/dev/null
open "$TASK_URL"
echo "已打开真机 RViz 点云。关闭网页不会停止 X5 上的传感器。"
