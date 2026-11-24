#!/usr/bin/env bash
# Source in every simulation terminal, rather than the hardware shell profile.
SCOUT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCOUT_ROOT/scripts/ros_env.bash"
_scout_sim_domain="${SCOUT_SIM_DOMAIN:-42}"
case "$_scout_sim_domain" in
  42|19[0-9]) ;;
  *) printf 'SCOUT_SIM_DOMAIN must be 42 or 190–199\n' >&2; unset _scout_sim_domain; return 1 ;;
esac
export ROS_DOMAIN_ID="$_scout_sim_domain"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export IGN_PARTITION="scout_sim_$_scout_sim_domain"
export CYCLONEDDS_URI="file://$SCOUT_ROOT/config/cyclonedds_sim.xml"
unset _scout_sim_domain
export ROS_LOG_DIR="$SCOUT_ROOT/log/simulation"
mkdir -p "$ROS_LOG_DIR"
