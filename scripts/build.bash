#!/usr/bin/env bash
set -eo pipefail
SCOUT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
if [[ -d "$SCOUT_ROOT/deps/asio/usr/include" ]]; then
  export CPLUS_INCLUDE_PATH="$SCOUT_ROOT/deps/asio/usr/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
fi
cd "$SCOUT_ROOT"
# 先显式构建已准备的包；底盘上游就绪后也会加入。
packages=(orbbec_camera scout_nav_bringup scout_visual_follow scout_sim)
if [[ -f src/scout_ros2/scout_base/package.xml && -f src/ugv_sdk/package.xml ]]; then
  packages+=(scout_base)
fi
colcon build --executor sequential --packages-up-to "${packages[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DINSTALL_UDEV_RULES=OFF
