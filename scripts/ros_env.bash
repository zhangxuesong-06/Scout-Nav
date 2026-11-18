# source ~/scout/scripts/ros_env.bash
# 只使用 Humble 和 Scout，避免 ~/.bashrc 中其他机器人 overlay 污染。
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
_scout_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$_scout_root/install/local_setup.bash" ]]; then
  source "$_scout_root/install/local_setup.bash"
fi
unset _scout_root
