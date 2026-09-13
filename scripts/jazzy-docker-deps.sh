#!/usr/bin/env bash
# One-shot apt install for the mycobot-jazzy Docker container (Noble + ROS 2 Jazzy).
# Run inside the container after: source /opt/ros/jazzy/setup.bash

set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

apt-get update

apt-get install -y --no-install-recommends \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3.12-venv \
  python3-vcstool \
  git \
  wget \
  curl \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-moveit \
  ros-jazzy-moveit-configs-utils \
  ros-jazzy-moveit-planners \
  ros-jazzy-moveit-ros-planning-interface \
  ros-jazzy-moveit-ros-move-group \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-gripper-controllers \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-control-msgs \
  ros-jazzy-shape-msgs \
  ros-jazzy-rclcpp-action

rosdep init 2>/dev/null || true
rosdep update

echo ""
echo "Jazzy dependencies installed."
echo "Next:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  cd /root/mycobot_ws && colcon build --symlink-install"
echo "  bash /root/mycobot_ws/src/scripts/yolo-env-jazzy.sh"
