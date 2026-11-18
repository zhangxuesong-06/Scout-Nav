# Source this file before running the ROS 2 visual follower with the CPU venv.
_scout_vision_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$_scout_vision_root/.venv-vision/bin/python" ]]; then
  printf 'Vision environment missing; see %s/README.md\n' "$_scout_vision_root" >&2
  unset _scout_vision_root
  return 1
fi
source "$_scout_vision_root/scripts/ros_env.bash"
# Keep ROS / apt packages visible, but exclude unrelated ~/.local packages.
export PYTHONNOUSERSITE=1
source "$_scout_vision_root/.venv-vision/bin/activate"
unset _scout_vision_root
