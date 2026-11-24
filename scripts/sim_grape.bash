#!/usr/bin/env bash
# Manipulate photo fixtures in the currently running mapping_course simulation.
set -eo pipefail
SCOUT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCOUT_ROOT/scripts/sim_env.bash"
exec /usr/bin/python3 -m scout_sim.grape_fixture "$@"
