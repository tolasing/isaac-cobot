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
`assets/mefron/`'s `ur10_mount` pedestal (world position
`[2.625260866887235, -4.7019853821770115, 0.8093334035921127]` -- replaced the original SEKTION-cabinet
mount plate once the packing table was swapped for a conveyor line; arm 2 mounts on the
paired `ur10_mount_01` pedestal the same way), runs a drag-follow teleop loop, and provides
one grasp key per `config.GRASP_TARGETS` entry (J: `finger_print_scanner`,
B: `backpanel_support`, via NVIDIA Grasp Editor-exported poses) plus P(lace),
all snapping the teleop target to a live-computed pose, plus C/O keys for
the gripper. Pressing a grasp key also stages that object's own yaml-specified
finger widths (`cspace_position`/`pregrasp_cspace_position`) onto the
`GripperKeyboardControl` instance and immediately opens the gripper to its
pregrasp width — so C/O ramp toward whichever object was grasped last, not
one fixed global width. P's placement pose is computed by measuring the
CURRENT live gripper-to-part offset (not a fixed constant) and applying it
to `finger_print_scanner`'s live-computed target pose on `main_holder` — so
it self-corrects to whatever grasp J actually produced (P is not yet
generalized to `backpanel_support`). There is no G key: an earlier
hand-derived-constant grasp-approach pose has been removed in favor of
J/B. Opens `mefron.usd` directly via `open_stage()`.

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
- `GRASP_TARGETS`: dict keyed by object name (`finger_print_scanner`,
  `backpanel_support`), each entry holding the keyboard `key` (a
  `carb.input.KeyboardInput` attribute name, resolved via `getattr` in
  `teleop.py` since `config.py` stays omni/curobo-import-free), `yaml_path`,
  `grasp_name`, and `part_prim_path`. Replaces the old singular
  `GRASP_EDITOR_YAML_PATH`/`GRASP_EDITOR_GRASP_NAME` constants. Finger
  widths themselves aren't stored here — `grasp.compute_grasp_finger_widths_from_file()`
  reads `cspace_position`/`pregrasp_cspace_position` from the yaml live.
- `ASSEMBLY_RELATIONSHIPS["finger_print_scanner_on_main_holder"]`:
  `local_position=[-0.05765, 0.02069, 0.01565]`,
  `local_orientation_wxyz=[1.0, 0.0, 0.0, 0.0]`. (There is no
  `GRASP_OFFSET_POSITION`/`ORIENTATION_WXYZ` anymore — P measures the
  live grasp offset instead of using a fixed constant; see above.)
- `_TELEOP_VELOCITY_SCALE = _TELEOP_ACCELERATION_SCALE = 0.5`,
  `GRIPPER_CLOSE_SPEED = 0.02` m/s, `GRIPPER_DRIVE_STIFFNESS = 10000.0`.
  `GRIPPER_OPEN_POSITION`/`GRIPPER_CLOSED_POSITION` are now only the
  *default* widths a fresh `GripperKeyboardControl` starts with, before any
  grasp key has been pressed — each grasp key overrides them for the rest
  of the run via `set_grasp_widths()`.
- `OBSTACLE_PRIM_PATHS`: no longer references `packing_table`/
  `packing_table_01` (removed from the scene when it became a conveyor
  line) — now includes both `ur10_mount`/`ur10_mount_01` pedestals instead,
  so each arm treats the other arm's own pedestal as an obstacle too (they
  now sit only ~0.65m apart). Does **not** yet include the new conveyor
  belt/container prims — confirmed live that adding them made cuRobo's
  mesh-collision-world construction hang for over an hour with zero
  progress (each top-level `ConveyorBelt_*` Xform recurses into dozens of
  child meshes; real conveyor-line CAD assemblies are far more complex than
  the single `packing_table` prop they replaced). See "Currently open
  issues" below.

Currently open issues (see the linked docs for full diagnosis):
- **Grasp-centering**: `finger_print_scanner` isn't equidistant from both
  fingertips at grasp time, so one finger contacts first and shifts the
  part sideways. Not a joint/drive asymmetry (explicitly ruled out) — see
  `docs/grasp-and-assembly-offsets.md`.
- **Assembly placement (P) still doesn't land cleanly.** Tried a proper
  lift/translate/rotate/descend redesign 2026-07-17 (the original 2-stage lift baked the *final*
  X/Y into the lift waypoint, so cuRobo swung sideways instead of lifting straight; a 3-stage
  version fixed that but its combined rotate+translate leg made cuRobo hold the old orientation
  until the very end of that leg and snap to final right at the align→descend handoff, a violent
  kickstart) but reverted all of it after finding a deeper, unrelated issue: `ASSEMBLY_LIFT_HEIGHT`
  is a fixed world-frame Z constant — unlike every other pose in this system, which is computed
  relative to a live prim (`main_holder` for `ASSEMBLY_RELATIONSHIPS`, the part itself for grasp
  approach) — so moving `main_holder` (or repositioning the assembly generally) breaks the staged
  sequence outright, since the fixed height has no relationship to wherever things actually are.
  Next attempt should make the lift clearance relative instead — e.g. a margin above whichever of
  the current/final Z is higher — rather than an absolute world height. May or may not also be the
  same grasp-centering issue below; not confirmed either way.
- `attach_objects_to_robot()`/`detach_object_from_robot()` (cuRobo's
  carried-object collision awareness) is not yet wired to the C/O keys —
  `franka.yml` already has a spare `attached_object` link ready for it.
- `main_holder`'s convex-decomposition collision tuning (fixes sinking +
  lost mounting studs) is researched but not yet applied/saved to
  `mefron.usd` — see `docs/mefron-history.md`.
- **Conveyor line has no collision awareness yet.** The new
  `ConveyorBelt_*`/`container_h20*` prims (added when the packing table
  became a conveyor line) are deliberately left out of
  `OBSTACLE_PRIM_PATHS` — confirmed live 2026-07-18 that including even
  just the 5 conveyor + 4 container top-level Xforms made
  `get_obstacles_from_stage()`'s mesh-collision-world construction hang
  (past its own "Creating new Mesh cache: 95" log line) for over an hour
  with zero forward progress, steady CPU/GPU load, and no crash/OOM to
  even signal failure -- had to be killed. Each top-level Xform recurses
  into every child mesh (9 prims -> ~95 individual meshes), and real
  conveyor-line CAD assemblies (rollers, frame, guards, motor housing --
  see the 13-113MB per-file sizes under `Conveyors/`) are far more
  geometrically complex than the single `packing_table` prop they
  replaced -- well past what cuRobo's mesh-based collision checker can
  preprocess in reasonable time. Next attempt should use primitive/cuboid
  obstacle approximations instead of the raw CAD meshes (cuRobo's
  `WorldConfig` supports cuboid obstacles directly), or narrow to specific
  lightweight sub-prims rather than whole assemblies -- not the raw
  top-level Xforms.

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
- **This importer side effect also silently rewrites `mefron.usd` itself
  on every run — not just a stray Ctrl+S risk, an unconditional one.**
  Confirmed live: running any mefron entry-point script (including a
  plain headless verification run with zero explicit save call anywhere
  in this repo's code) changes `mefron.usd`'s on-disk hash every time,
  growing it by however much the just-imported Franka(s) add. Setting the
  stage's edit target to its session layer (`stage.GetSessionLayer()`,
  USD's own built-in "never included in a save" mechanism) does **not**
  prevent this — confirmed by testing against a scratch copy of the
  scene — so the importer is authoring directly onto the root layer
  regardless of the current edit target, not respecting `EditTarget` at
  all. No real fix currently applied; the actual fix would be the
  anonymous-stage-plus-reference architecture above, which isn't usable
  yet for the reason given in `build_scene_mefron.py`'s own section
  above. Until then: (a) treat any diff on `mefron.usd`/
  `configuration/*.usd` after a run as expected noise, not a sign of a
  new bug, (b) never blindly `git checkout` this file — it may carry
  real hand-placed scene edits (e.g. mount-plate positions) alongside the
  incidental Franka bake-in, so reconcile by hand, (c) when testing
  headlessly, prefer running against a scratch copy of
  `assets/mefron/factory floor/` rather than the real file, to avoid
  adding to the noise.
- **Accumulated stray robot prims from the above break rendering, not
  physics.** If a stray Save (or Ctrl+S) ever catches a session mid-run,
  whatever's currently mounted (`/World/Franka`, `/World/Franka2`,
  `/World/Franka3`, and/or the historical intermediate `/panda` path —
  see `mount_franka()`'s docstring for why `/panda` specifically) gets
  baked into `mefron.usd`'s saved root layer as an orphaned leftover.
  Confirmed live: leaving these live during `mefron.py`'s 120-frame
  post-`open_stage()` settle pump lets PhysX/Fabric/Hydra partially
  register the *stale* prims before `mount_franka()`'s own per-path
  cleanup (which only runs right before *that* specific import, too late)
  ever gets a chance to delete+reimport at the same path — the fresh
  reimport's physics/motion-planning end up correct (each arm is driven
  by its own exact, freshly-created prim_path), but the viewport visibly
  desyncs from it: cuRobo successfully plans and drives the joints, the
  arm just never appears to move. Fixed by `robot.clear_stray_robot_prims()`,
  called in `mefron.py` right after `open_stage()` and before the settle
  pump — in-memory `DeletePrims` only, so (like the rewrite itself) this
  needs to run on every fresh `open_stage()`, not just once. Full
  diagnosis: `docs/mefron-history.md`.
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
