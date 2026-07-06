#!/bin/bash
# Wrapper around the installed /isaac-sim/isaac-sim.sh (the base, non-Newton
# app -- isaacsim.exp.full.kit, PhysX by default) for convenience/consistency
# with scripts/launch_isaac_sim_newton.sh. Unlike that wrapper, this one does
# NOT inject the sys.executable patch -- isaacsim.exp.full.kit doesn't load
# Newton by default, so the MuJoCo/glfw bug that patch works around isn't in
# play here. Does not modify /isaac-sim/isaac-sim.sh; just forwards to it.
#
# Usage (GUI):
#   scripts/launch_isaac_sim.sh
# Usage (headless, same flags the real launcher accepts):
#   scripts/launch_isaac_sim.sh --no-window --/app/fastShutdown=true

set -e

ISAAC_SIM_SH="/isaac-sim/isaac-sim.sh"

if [ ! -f "$ISAAC_SIM_SH" ]; then
    echo "error: $ISAAC_SIM_SH not found -- is ISAACSIM_ROOT_PATH still /isaac-sim?" >&2
    exit 1
fi

exec "$ISAAC_SIM_SH" "$@"
