#!/usr/bin/env bash
# Standalone RGB-D mapping mission in the isolated Gazebo domain.
set -eo pipefail
SCOUT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCOUT_ROOT/scripts/sim_env.bash"
if [[ -n "${SCOUT_RTABMAP_PREFIX:-}" ]]; then
  p="${SCOUT_RTABMAP_PREFIX}"
  [[ -f "$p/share/ament_index/resource_index/packages/rtabmap_slam" ]] || {
    echo "rtabmap_slam not found in SCOUT_RTABMAP_PREFIX: $p" >&2; exit 1;
  }
  export AMENT_PREFIX_PATH="$p${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$p/lib:$p/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export PYTHONPATH="$p/local/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
export PYTHONNOUSERSITE=1 YOLO_OFFLINE=true
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}" MKL_NUM_THREADS="${MKL_NUM_THREADS:-2}"
export LP_NUM_THREADS="${LP_NUM_THREADS:-4}"
exec ros2 launch scout_sim mapping_mission.launch.py \
  database:="$SCOUT_ROOT/maps/sim-$(date +%Y%m%d-%H%M%S-%N).db" \
  profile:="$SCOUT_ROOT/config/robot.yaml" \
  python_executable:="$SCOUT_ROOT/.venv-yolo-train/bin/python" \
  model_path:="$SCOUT_ROOT/models/grapes/wgisd-grape-cluster-detection-v1.pt" "$@"
