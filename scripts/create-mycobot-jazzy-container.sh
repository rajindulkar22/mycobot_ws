#!/usr/bin/env bash
# Create the parallel Jazzy simulation container (does not remove mycobot-humble).

set -euo pipefail

if docker ps -a --format '{{.Names}}' | grep -qx 'mycobot-jazzy'; then
  echo "Container mycobot-jazzy already exists."
  echo "Start it with: docker start -ai mycobot-jazzy"
  exit 0
fi

xhost +local:docker

docker run -it \
  --name mycobot-jazzy \
  --network host \
  --ipc host \
  --device=/dev/dri \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${HOME}/mycobot_ws:/root/mycobot_ws" \
  osrf/ros:jazzy-desktop-full-noble
