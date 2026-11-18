# Source this file in a terminal dedicated to offline YOLO training.
_scout_train_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$_scout_train_root/.venv-yolo-train/bin/python" ]]; then
  printf 'Training environment missing; see %s/YOLO_TRAINING.md\n' "$_scout_train_root" >&2
  unset _scout_train_root
  return 1
fi

# This venv uses pip CUDA libraries and does not need the ROS overlays from .bashrc.
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
export PYTHONNOUSERSITE=1
source "$_scout_train_root/.venv-yolo-train/bin/activate"
unset _scout_train_root
