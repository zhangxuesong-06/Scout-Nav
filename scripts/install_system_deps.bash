#!/usr/bin/env bash
# 在宿主机终端中手动运行；不修改 Docker、CAN、udev 或 ~/.bashrc。
set -eo pipefail
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git libasio-dev can-utils \
  python3-colcon-common-extensions python3-numpy python3-opencv python3-yaml \
  python3-pytest python3-venv \
  ros-humble-rtabmap-ros ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-cv-bridge ros-humble-message-filters ros-humble-tf2-ros \
  ros-humble-image-transport ros-humble-image-publisher \
  ros-humble-ros-gz ros-humble-rmw-cyclonedds-cpp \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-rviz2 \
  ros-humble-camera-info-manager ros-humble-diagnostic-updater \
  ros-humble-backward-ros libgflags-dev libgoogle-glog-dev \
  libusb-1.0-0-dev libdw-dev nlohmann-json3-dev
