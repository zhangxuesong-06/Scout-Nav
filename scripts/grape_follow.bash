#!/usr/bin/env bash
# RGB-D grape following preview; camera / mount TF must be started separately.
set -eo pipefail
_scout_grape_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_scout_grape_root/scripts/yolo_train_env.bash"
source "$_scout_grape_root/scripts/ros_env.bash"
export YOLO_OFFLINE=true
export ROS_LOG_DIR="$_scout_grape_root/log/grape_follow"
exec ros2 launch scout_visual_follow preview.launch.py \
  model_path:="$_scout_grape_root/models/grapes/wgisd-grape-cluster-detection-v1.pt" \
  python_executable:="$_scout_grape_root/.venv-yolo-train/bin/python" \
  confidence:=0.10 imgsz:=800 min_box_area:=200 device:=0 show_window:=true \
  depth_min_valid_pixels:=20 depth_min_valid_ratio:=0.15 "$@"
