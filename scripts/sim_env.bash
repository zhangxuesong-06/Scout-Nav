#!/usr/bin/env bash
# Source in every simulation terminal, rather than the hardware shell profile.
SCOUT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCOUT_ROOT/scripts/ros_env.bash"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export IGN_PARTITION=scout_sim_42
export ROS_LOG_DIR="$SCOUT_ROOT/log/simulation"
mkdir -p "$ROS_LOG_DIR"
