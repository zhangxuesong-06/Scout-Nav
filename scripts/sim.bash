#!/usr/bin/env bash
set -eo pipefail
SCOUT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCOUT_ROOT/scripts/sim_env.bash"
export PYTHONNOUSERSITE=1
exec ros2 launch scout_sim simulation.launch.py "$@"
