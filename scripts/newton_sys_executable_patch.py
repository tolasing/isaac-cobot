"""Patches sys.executable for Newton/MuJoCo/glfw compatibility.

Meant to be run via Kit's --exec flag (see scripts/launch_isaac_sim_newton.sh),
executed once inside an already-running Kit app -- NOT a standalone script,
does not create a SimulationApp, and deliberately does not exit the process
afterward (unlike this repo's diagnostic scratchpad scripts): the whole
point is for the interactive GUI session to keep running normally after
this runs, just with the patch already applied.

Root cause (see CLAUDE.md's `assets/mefron/` entry for the full writeup):
Newton's default solver is MuJoCo, whose bundled `mujoco` package
unconditionally imports an OpenGL rendering submodule even for headless
physics; that submodule's `glfw` dependency spawns
`subprocess.Popen([sys.executable, ...])` to safety-check a library version.
`sys.executable` is '' whenever Python is embedded inside Kit's own C++
process (true for ANY `kit/kit <app>.kit` launch -- the interactive GUI
included, not just headless --exec runs) -- unrelated to any asset, scene,
or instancing choice; it happens the moment Newton tries to build a MuJoCo
solver at all. Without this patch, that manifests as:
    [Newton] Initialization failed: [Errno 13] Permission denied: ''
"""

import sys

if not sys.executable:
    sys.executable = "/isaac-sim/kit/python/bin/python3"
    print("[isaac-cobot] patched sys.executable for Newton/MuJoCo/glfw compatibility", flush=True)
else:
    print(f"[isaac-cobot] sys.executable already set ({sys.executable}), no patch needed", flush=True)
