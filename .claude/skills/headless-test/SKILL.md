---
description: How to actually run Isaac Sim/cuRobo code and tests in this repo's environment — pytest and pip don't work here, and SimulationApp has repo-specific bootstrap gotchas. Use before writing a new headless script or reaching for pytest.
---

# Running things headlessly in this repo

`pytest` and `pip install` are **broken** in this environment (torch ABI
mismatch breaks `ninja`-based JIT compilation for cuRobo's CUDA kernels;
`pip` itself doesn't work). Don't try to install/fix them — work around
them:

- Existing regression scripts: run directly via
  `${ISAACSIM_ROOT_PATH}/python.sh scripts/test_teleop_headless.py --headless`
  (mefron/Franka path) or the `dobot`-branch equivalent for the CR5+gripper
  testbed. Check the script's own `_MAX_ITERATIONS`/similar constants
  before assuming a timeout means failure — a derated/dilated trajectory
  can need far more waypoints than an un-derated one.
- `tests/test_configs.py`: no pytest runner — call its functions directly
  from a `python.sh` one-liner or a throwaway script.
- New one-off diagnostic/audit scripts: write to the scratchpad dir, run
  via `/isaac-sim/python.sh <script>.py`, delete when done. Don't leave
  throwaway scripts in the repo.

## Boilerplate every headless script needs

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True}, experience="")
# ^ experience="" is required, not optional — passing the full experience
#   (isaacsim.exp.full.kit, needed only for GUI debug-viz menus) breaks
#   cuRobo's `from packaging import version` import. If you need cuRobo
#   in a *non*-headless script, see kit_bootstrap.preload_real_packaging().

from pxr import Usd, UsdPhysics, ...  # imports must come after SimulationApp exists

stage = Usd.Stage.Open(path)  # or omni.usd.get_context().open_stage()
for _ in range(60):
    simulation_app.update()  # let async reference/import resolution settle
    # (see robot-import-checklist skill — URDF import population is async)

...

simulation_app.close()
```

## Ordering gotchas that produce confusing errors, not clean failures

- Define `/physicsScene` (`UsdPhysics.Scene.Define(stage, "/physicsScene")`)
  and let any import/reference finish settling **before** calling
  `timeline.play()`. Playing too early corrupts PhysX's tensor
  simulationView — the symptom is `AttributeError: 'NoneType' object has
  no attribute 'link_names'` on the *next* `SingleArticulation(...)`, not
  an error at the `play()` call itself.
- A `SingleArticulation` only stays valid across Play→Stop→Play if you
  rebuild it on every fresh Play — track the not-playing→playing
  transition explicitly.
- If a bare/anonymous stage has no `/World` yet, `MovePrim` inside an
  import helper can silently no-op (it doesn't check its own return
  status) — define the destination's parent (`UsdGeom.Xform.Define(stage,
  "/World")`) before importing anything into it.
