#!/usr/bin/env bash
# Real grape-following bringup. Select a target ID in the window to begin following.
set -eo pipefail
_scout_follow_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_scout_follow_root/scripts/yolo_train_env.bash"
source "$_scout_follow_root/scripts/ros_env.bash"
export YOLO_OFFLINE=true ROS_LOG_DIR="$_scout_follow_root/log/follow"
_scout_follow_profile="${SCOUT_PROFILE:-$_scout_follow_root/config/robot.yaml}"
for _scout_follow_arg in "$@"; do
  if [[ "$_scout_follow_arg" == profile:=* ]]; then
    _scout_follow_profile="${_scout_follow_arg#profile:=}"
  fi
done
if [[ " $* " != *" --show-args "* && " $* " != *" -s "* ]]; then
  # Check identity and configuration before asking sudo to bring up a DOWN port.
  _scout_follow_port="$(python - "$_scout_follow_profile" <<'PY'
import json, subprocess, sys
from ament_index_python.packages import get_package_prefix
from scout_nav_bringup.profile import mount_values, read_profile, require_base, require_camera
p = read_profile(sys.argv[1])
require_camera(p)
mount_values(p, allow_approximate=True)
get_package_prefix('scout_base')
port = require_base(p, require_up=False)
link = json.loads(subprocess.check_output(['ip', '-details', '-json', 'link', 'show', port]))[0]
if ('UP' in link['flags']
        and link['linkinfo']['info_data'].get('bittiming', {}).get('bitrate') != 500000):
    raise SystemExit('Scout CAN must use 500000 bit/s; refusing to change an active port automatically')
print(port)
PY
)"
  _scout_follow_nodes="$(ros2 node list --no-daemon 2>/dev/null)"
  if printf '%s\n' "$_scout_follow_nodes" | rg -q '^/(nav_camera/|scout/(base|visual_follower|motion_gate)$|nav_camera_mount$)'; then
    printf 'A Scout/camera node is already running. Stop that launch first:\n%s\n' "$_scout_follow_nodes" >&2
    exit 1
  fi
  _scout_follow_flags="$(cat "/sys/class/net/$_scout_follow_port/flags")"
  if (( (_scout_follow_flags & 1) == 0 )); then
    sudo ip link set "$_scout_follow_port" up type can bitrate 500000
  fi
fi
exec ros2 launch scout_visual_follow follow.launch.py \
  profile:="$_scout_follow_profile" start_hardware:=true auto_enable:=true \
  allow_approximate_mount:=true \
  model_path:="$_scout_follow_root/models/grapes/wgisd-grape-cluster-detection-v1.pt" \
  python_executable:="$_scout_follow_root/.venv-yolo-train/bin/python" "$@"
