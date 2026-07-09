# CLAUDE.md

Project-specific context for **isaac-cobot**. Kept to the essentials for
working in the sim day-to-day — full history, forensic detail, and "why"
narratives live in `docs/` (linked below) rather than here.

## What this repo is

An NVIDIA Isaac Sim project using cuRobo for collision-aware motion
planning. **There is no physical robot hardware** — everything targets
Isaac Sim only; treat all sim behavior (contact dynamics, motion timing,
gripper interaction) as illustrative, not validated against real hardware.

The active work is a scanner-assembly pick-and-place task built on
`assets/mefron/` — a hand-authored scene (`mefron.usd`: factory floor,
packing tables, and a scanner-assembly CAD mockup — `finger_print_scanner`,
`main_holder`, `screen`, `backpanel_support`). A Franka Panda (cuRobo's own
bundled config) is mounted into this scene and driven by an interactive
cuRobo drag-teleop loop (`motion_gen.plan_single()`); grasp and
assembly-placement poses are derived by manually jogging the robot to a
good pose in the GUI and reading back the relative transform, not measured
on real hardware.

## Where things live

- **This file** — current state, and gotchas that will immediately break
  something if you don't know them.
- `docs/mefron-history.md` — full chronological bug/fix log for every
  mefron script, plus the cuRobo/PhysX/URDF-importer Conventions this file
  only summarizes.
- `docs/grasp-and-assembly-offsets.md` — how the grasp/assembly relative-
  pose constants were derived (`compute_relative_pose()` methodology, the
  abandoned Grasp Editor investigation, the open grasp-centering problem).
- `docs/docker-and-devcontainer.md` — Docker/devcontainer environment setup
  (generic Isaac Sim/cuRobo infra, not scene-specific).
- `examples/curobo_reference/` — pristine, unmodified copy of cuRobo's own
  interactive teleop demo. **Do not modify these two files**; write a
  separate script instead (`scripts/mefron.py` is exactly that). See
  `docs/mefron-history.md` for the environment fixes needed to run it and
  a license caveat on its header comments.

## Active script + current state

`scripts/mefron.py` is the live script: mounts cuRobo's bundled Franka
Panda onto `assets/mefron/`'s SEKTION-cabinet mount plate (world position
`[2.74097, -4.782, 0.7924]`), runs a drag-follow teleop loop, and provides
G(rasp)/P(lace) keys that snap the teleop target to a live-computed pose
for `finger_print_scanner`/`main_holder`, plus C/O keys for the gripper.
Opens `mefron.usd` directly via `open_stage()`.

`scripts/build_scene_mefron.py` (+ `configs/scene/mefron_layout.yaml`) is
architecturally preferred — same goal, but a fresh anonymous stage with
`mefron.usd` referenced in, which avoids most of `mefron.py`'s bugs by
construction — but is currently **dormant**: deriving the grasp/assembly
offsets requires temporarily reparenting prims in the Stage tree, which
only works when `mefron.usd` is opened directly, so active work happens in
`mefron.py`. See `docs/mefron-history.md` for both files' full histories.

Current constants (`scripts/mefron.py`):
- `ASSEMBLY_RELATIONSHIPS["finger_print_scanner_on_main_holder"]`:
  `local_position=[-0.05765001316747483, 0.02068996147910942,
  0.01500000425999065]`, `local_orientation_wxyz=[1.0, 0.0, 0.0, 0.0]`.
- `GRASP_OFFSET_POSITION=[0.00027002069774515104, -0.021693730387954874,
  -0.1271989186209571]`, `GRASP_OFFSET_ORIENTATION_WXYZ=
  [-2.1523912431273915e-05, -8.089888886539503e-06, 5.762411090611313e-06,
  0.9999999997190347]`.
- `_TELEOP_VELOCITY_SCALE = _TELEOP_ACCELERATION_SCALE = 0.2`,
  `GRIPPER_CLOSE_SPEED = 0.02` m/s, `GRIPPER_DRIVE_STIFFNESS = 10000.0`.

Currently open issues (see the linked docs for full diagnosis):
- **Grasp-centering**: `finger_print_scanner` isn't equidistant from both
  fingertips at grasp time, so one finger contacts first and shifts the
  part sideways. Not a joint/drive asymmetry (explicitly ruled out) — see
  `docs/grasp-and-assembly-offsets.md`.
- Assembly placement still lands off on the Y axis — likely the same
  grasp-centering issue, not confirmed.
- `attach_objects_to_robot()`/`detach_object_from_robot()` (cuRobo's
  carried-object collision awareness) is not yet wired to the C/O keys —
  `franka.yml` already has a spare `attached_object` link ready for it.
- `main_holder`'s convex-decomposition collision tuning (fixes sinking +
  lost mounting studs) is researched but not yet applied/saved to
  `mefron.usd` — see `docs/mefron-history.md`.

## Must-know gotchas

- **`PhysicsScene` required.** `SingleArticulation.initialize()` (and
  anything else needing a PhysX simulation view) silently breaks without
  a real `PhysicsScene` prim on the stage — nothing in this repo's
  robot-import path creates one automatically. Define one explicitly:
  `UsdPhysics.Scene.Define(stage, "/physicsScene")`.
- **`timeline.play()` timing.** Calling it before `/physicsScene` exists,
  or before a long blocking call like `motion_gen.warmup()` (~30s, no
  `simulation_app.update()` of its own) finishes, corrupts PhysX's tensor
  simulationView — symptom: `AttributeError: 'NoneType' object has no
  attribute 'link_names'` on the next `SingleArticulation(...)`. Only
  drive `timeline.play()` yourself after both are done; normally a human
  clicks Play in the GUI instead.
- **Stop/Play rebuild.** A `SingleArticulation` is only valid for the
  PhysX simulation view that existed when it was built — clicking Stop
  tears that view down, and reusing a pre-Stop `SingleArticulation` after
  a later Play leaves the robot permanently unresponsive. Any interactive
  loop must track not-playing→playing transitions and rebuild
  `robot`/`idx_list`/`articulation_controller` on *every* fresh Play (see
  `run_teleop_loop()` in `scripts/mefron.py`).
- **cuRobo plans in the robot's base-link frame, never world space.** Any
  USD world pose (e.g. a dragged teleop target) must go through
  `robot_base_pose.compute_local_pose(world_pose)` first, where
  `robot_base_pose` comes from wherever the robot was actually mounted.
- **URDF importer + file-backed stages.** Importing a robot into a stage
  opened directly from a real `.usd` file (`open_stage()`, what
  `mefron.py` does) makes the importer write a disk-persisted, multi-layer
  "Robot Description" (`configuration/` folder) as a side effect — this
  breaks `CopyPrim`-based duplication of anything under the robot (use
  `prim.GetReferences().AddInternalReference()` instead). A fresh
  anonymous stage with content brought in via `add_reference_to_stage()`
  (what `build_scene_mefron.py` does) avoids this entirely. Full detail:
  `docs/mefron-history.md`.
- **`SimulationApp` full experience breaks cuRobo's `packaging` import.**
  Passing `experience=.../isaacsim.exp.full.kit` (needed for the Physics
  debug-visualization menu) makes `from packaging import version` resolve
  to a broken internal copy. Fix: pre-load `packaging`/`packaging.version`
  from real `site-packages` before anything else imports cuRobo — see the
  top of `scripts/mefron.py`. Full root-cause: `docs/mefron-history.md`.
- **`ninja`/`pip` are broken in this environment.** cuRobo's CUDA kernels
  JIT-compile (needs `ninja`) on a torch ABI mismatch; `pip install`
  itself doesn't work here. Fixed via `apt-get install ninja-build` — see
  `docs/docker-and-devcontainer.md`.

## Pinned versions

Isaac Sim `5.1.0`, cuRobo commit `ebb71702f3f70e767f40fd8e050674af0288abe8`,
torch `2.11.0+cu128` (CUDA 12.8). Dev GPU: RTX PRO 4000 Blackwell (sm_120)
— `TORCH_CUDA_ARCH_LIST` must be `12.0+PTX` for this GPU.
