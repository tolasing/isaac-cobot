#!/bin/bash
# Wrapper around the installed /isaac-sim/isaac-sim.newton.sh that also
# patches sys.executable before you touch anything, via Kit's own --exec
# flag -- see scripts/newton_sys_executable_patch.py for why this is
# needed. Does NOT modify /isaac-sim/isaac-sim.newton.sh (or anything else
# under /isaac-sim) at all; it just calls the real, untouched launcher with
# one extra flag. Forwards any other args through unchanged, so this is a
# drop-in replacement for however you'd normally invoke isaac-sim.newton.sh
# (GUI or headless with --no-window).
#
# Usage (GUI):
#   scripts/launch_isaac_sim_newton.sh
# Usage (headless, same flags the real launcher accepts):
#   scripts/launch_isaac_sim_newton.sh --no-window --/app/fastShutdown=true

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_NEWTON_SH="/isaac-sim/isaac-sim.newton.sh"

if [ ! -f "$ISAAC_SIM_NEWTON_SH" ]; then
    echo "error: $ISAAC_SIM_NEWTON_SH not found -- is ISAACSIM_ROOT_PATH still /isaac-sim?" >&2
    exit 1
fi

exec "$ISAAC_SIM_NEWTON_SH" --exec "$SCRIPT_DIR/newton_sys_executable_patch.py" "$@"
