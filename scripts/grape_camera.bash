#!/usr/bin/env bash
set -e
_scout_grape_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_scout_grape_root/scripts/yolo_train_env.bash"
export YOLO_OFFLINE=true
exec python -u "$_scout_grape_root/scripts/grape_camera.py" "$@"
