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
- `scripts/mefron_lib/` — shared package backing every mefron entry-point
  script (`mefron.py`, `mefron_gripper_probe.py`,
  `mefron_grasp_editor_scene.py`, `franka_grasp_editor_scene.py`,
  `test_mefron_*_headless.py`): `kit_bootstrap.py` (packaging preload +
  stale-config cleanup, stdlib-only so it's safe to import before
  `SimulationApp` exists), `config.py` (all constants), `grasp.py` (pose
  math), `robot.py` (mount/friction/drive), `teleop.py` (keyboard control +
  `run_teleop_loop()`). `mefron2.py` (dormant/superseded, see below) keeps
  its own diverged copies of everything except the packaging-preload block.

## Active script + current state

`scripts/mefron.py` is the live script, now a thin entry point over
`scripts/mefron_lib/`: mounts cuRobo's bundled Franka Panda onto
`assets/mefron/`'s SEKTION-cabinet mount plate (world position
`[2.74097, -4.782, 0.7924]`), runs a drag-follow teleop loop, and provides
J(grasp, via an NVIDIA Grasp Editor-exported pose)/P(lace) keys that snap
the teleop target to a live-computed pose, plus C/O keys for the gripper.
P's placement pose is computed by measuring the CURRENT live gripper-to-
part offset (not a fixed constant) and applying it to `finger_print_scanner`'s
live-computed target pose on `main_holder` — so it self-corrects to
whatever grasp J actually produced. There is no G key: an earlier
hand-derived-constant grasp-approach pose has been removed in favor of J.
Opens `mefron.usd` directly via `open_stage()`.

`scripts/mefron_gripper_probe.py` imports just the Franka hand +
`panda_leftfinger`/`panda_rightfinger` + `ee_link` (no arm, no motion_gen)
onto its own free-floating `base_link`, for dragging into place against a
part mesh in Stop mode to measure a grasp pose directly, without cuRobo
IK/teleop in the way.

`scripts/build_scene_mefron.py` (+ `configs/scene/mefron_layout.yaml`) is
architecturally preferred — same goal, but a fresh anonymous stage with
`mefron.usd` referenced in, which avoids most of `mefron.py`'s bugs by
construction — but is currently **dormant**: deriving the grasp/assembly
offsets requires temporarily reparenting prims in the Stage tree, which
only works when `mefron.usd` is opened directly, so active work happens in
`mefron.py`. See `docs/mefron-history.md` for both files' full histories.

Current constants (`scripts/mefron_lib/config.py`):
- `ASSEMBLY_RELATIONSHIPS["finger_print_scanner_on_main_holder"]`:
  `local_position=[-0.05765, 0.02069, 0.01565]`,
  `local_orientation_wxyz=[1.0, 0.0, 0.0, 0.0]`. (There is no
  `GRASP_OFFSET_POSITION`/`ORIENTATION_WXYZ` anymore — P measures the
  live grasp offset instead of using a fixed constant; see above.)
- `_TELEOP_VELOCITY_SCALE = _TELEOP_ACCELERATION_SCALE = 0.5`,
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
  `run_teleop_loop()` in `scripts/mefron_lib/teleop.py`).
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
  from real `site-packages` before anything else imports cuRobo — see
  `scripts/mefron_lib/kit_bootstrap.py`'s `preload_real_packaging()`,
  called from the top of every mefron entry-point script. Full root-cause:
  `docs/mefron-history.md`.
- **`ninja`/`pip` are broken in this environment.** cuRobo's CUDA kernels
  JIT-compile (needs `ninja`) on a torch ABI mismatch; `pip install`
  itself doesn't work here. Fixed via `apt-get install ninja-build` — see
  `docs/docker-and-devcontainer.md`.

## Pinned versions

Isaac Sim `5.1.0`, cuRobo commit `ebb71702f3f70e767f40fd8e050674af0288abe8`,
torch `2.11.0+cu128` (CUDA 12.8). Dev GPU: RTX PRO 4000 Blackwell (sm_120)
— `TORCH_CUDA_ARCH_LIST` must be `12.0+PTX` for this GPU.
