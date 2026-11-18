#!/usr/bin/env bash
set -eo pipefail
SCOUT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCOUT_ROOT/scripts/ros_env.bash"
export ROS_LOG_DIR="$SCOUT_ROOT/log/offline_tests"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONNOUSERSITE=1
cd "$SCOUT_ROOT"
/usr/bin/python3 -m pytest -q tests "$@"
