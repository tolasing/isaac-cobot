# CLAUDE.md

Project-specific context for **isaac-cobot**. Kept to the essentials for
working in the sim day-to-day — full history, forensic detail, and "why"
narratives live in `docs/` (linked below) rather than here.

## Code style

- Comments/docstrings: 2 lines normal, **4 lines hard max**. Longer
  rationale belongs in `docs/` (usually `docs/mefron-history.md`), linked
  with a short pointer instead of inlined.

## What this repo is

An NVIDIA Isaac Sim project using cuRobo for collision-aware motion
planning. **There is no physical robot hardware** — everything targets
Isaac Sim only; treat all sim behavior as illustrative, not validated
against real hardware.

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
  only summarizes, plus full detail on the open issues below.
- `docs/grasp-and-assembly-offsets.md` — how the grasp/assembly relative-
  pose constants were derived, plus the open grasp-centering problem.
- `docs/docker-and-devcontainer.md` — Docker/devcontainer environment setup
  (generic infra, not scene-specific).
- `examples/curobo_reference/` — pristine, unmodified copy of cuRobo's own
  interactive teleop demo. **Do not modify these two files**; write a
  separate script instead (`scripts/mefron.py` is exactly that).
- `scripts/mefron_lib/` — shared package backing every mefron entry-point
  script: `kit_bootstrap.py` (packaging preload + stale-config cleanup),
  `config.py` (all constants), `grasp.py` (pose math), `robot.py`
  (mount/friction/drive), `teleop.py` (keyboard control +
  `run_teleop_loop()`), `conveyor.py` (conveyor belt control). `mefron2.py`
  (dormant/superseded) keeps its own diverged copies of everything except
  the packaging-preload block.

## Active script + current state

`scripts/mefron.py` is the live script, a thin entry point over
`scripts/mefron_lib/`: mounts cuRobo's bundled Franka Panda onto
`assets/mefron/`'s `ur10_mount` pedestal (arm 2 mounts on the paired
`ur10_mount_01` pedestal the same way), runs a drag-follow teleop loop, and
provides one grasp key per `config.GRASP_TARGETS` entry (J:
`finger_print_scanner`, B: `backpanel_support`, via NVIDIA Grasp
Editor-exported poses) plus P(lace), plus C/O for the gripper and V/L for
arm 2's suction attach/detach. Opens `mefron.usd` directly via
`open_stage()`.

Pressing a grasp key also stages that object's yaml-specified finger widths
onto `GripperKeyboardControl` and opens the gripper to pregrasp width — C/O
ramp toward whichever object was grasped last, not a fixed global width.
P's placement pose is computed by measuring the CURRENT live
gripper-to-part offset (not a fixed constant) and applying it to the live
target pose on `main_holder`.

`scripts/mefron_gripper_probe.py` imports just the Franka hand (no arm, no
motion_gen) onto its own free-floating `base_link`, for dragging into place
against a part mesh in Stop mode to measure a grasp pose directly.

`scripts/build_scene_mefron.py` (+ `configs/scene/mefron_layout.yaml`) is
architecturally preferred — a fresh anonymous stage with `mefron.usd`
referenced in, avoiding most of `mefron.py`'s bugs by construction — but is
currently **dormant**: deriving grasp/assembly offsets requires temporarily
reparenting prims, which only works when `mefron.usd` is opened directly.
See `docs/mefron-history.md` for both files' full histories.

Current constants (`scripts/mefron_lib/config.py`):
- `GRASP_TARGETS`: dict keyed by object name, each entry holding the
  keyboard `key`, `yaml_path`, `grasp_name`, `part_prim_path`. Finger
  widths aren't stored here — `grasp.compute_grasp_finger_widths_from_file()`
  reads them from the yaml live.
- `ASSEMBLY_RELATIONSHIPS["finger_print_scanner_on_main_holder"]`: the
  part's pose in `main_holder`'s local frame. P measures the live grasp
  offset rather than using a fixed constant.
- `_TELEOP_VELOCITY_SCALE = _TELEOP_ACCELERATION_SCALE = 0.5`,
  `GRIPPER_CLOSE_SPEED = 0.02` m/s, `GRIPPER_DRIVE_STIFFNESS = 10000.0`.
  `GRIPPER_OPEN_POSITION`/`GRIPPER_CLOSED_POSITION` are only the *default*
  widths before any grasp key is pressed — each grasp key overrides them.
- `OBSTACLE_PRIM_PATHS`: both `ur10_mount`/`ur10_mount_01` pedestals (arms
  sit ~0.65m apart, each treats the other's pedestal as an obstacle).
  Deliberately excludes the conveyor/container prims — see open issues.

## Currently open issues

Full investigation detail for all of these: `docs/mefron-history.md`.

- **Grasp-centering**: `finger_print_scanner` isn't equidistant from both
  fingertips at grasp time, so one finger contacts first and shifts the
  part sideways. Not a joint/drive asymmetry (ruled out) — see
  `docs/grasp-and-assembly-offsets.md`.
- **Assembly placement (P) still doesn't land cleanly.** A lift/rotate/
  descend redesign was tried and reverted after finding `ASSEMBLY_LIFT_HEIGHT`
  is a fixed world-Z constant with no relationship to where things actually
  are, unlike every other pose in this system. Next attempt: make lift
  clearance relative, not absolute.
- `attach_objects_to_robot()`/`detach_object_from_robot()` (carried-object
  collision awareness) isn't wired to the C/O keys yet — `franka.yml`
  already has a spare `attached_object` link ready for it.
- `main_holder`'s convex-decomposition collision tuning (fixes sinking +
  lost mounting studs) is researched but not yet applied/saved to
  `mefron.usd`.
- **Conveyor line has no collision awareness yet.** Adding the
  `ConveyorBelt_*`/`container_h20*` prims to `OBSTACLE_PRIM_PATHS` hung
  cuRobo's mesh-collision-world construction for over an hour with zero
  progress — real conveyor CAD is far more complex than the single
  `packing_table` prop it replaced. Needs cuboid obstacle approximations
  instead of raw CAD meshes, or narrower sub-prim selection.
- **Arm 2's suction release (L key) doesn't actually let go.** The real
  `SurfaceGripperManager` processes attach/detach as queued PhysX/USD
  actions on its own `onPhysicsStep`, so scripting the joint-enabled
  toggle directly from Python races its internal state (three variants
  tried, all reverted). Manual Stage-panel workaround still required.

## Must-know gotchas

Full root-cause detail for all of these: `docs/mefron-history.md`
(Conventions section) unless noted otherwise.

- **`PhysicsScene` required.** `SingleArticulation.initialize()` silently
  breaks without a real `PhysicsScene` prim — define one explicitly:
  `UsdPhysics.Scene.Define(stage, "/physicsScene")`.
- **`timeline.play()` timing.** Calling it before `/physicsScene` exists,
  or before `motion_gen.warmup()` finishes, corrupts PhysX's tensor
  simulationView (`AttributeError: 'NoneType' object has no attribute
  'link_names'` on the next `SingleArticulation(...)`). Only drive
  `timeline.play()` yourself after both are done.
- **Stop/Play rebuild.** A `SingleArticulation` is only valid for the PhysX
  view that existed when built — Stop tears that view down. Any
  interactive loop must track not-playing→playing transitions and rebuild
  `robot`/`idx_list`/`articulation_controller` on every fresh Play (see
  `run_teleop_loop()`).
- **cuRobo plans in the robot's base-link frame, never world space.** Any
  USD world pose must go through `robot_base_pose.compute_local_pose(...)`
  first.
- **URDF importer + file-backed stages.** Importing into a stage opened
  directly from a `.usd` file (`open_stage()`, what `mefron.py` does)
  makes the importer write a disk-persisted "Robot Description" as a side
  effect, breaking `CopyPrim`-based duplication (use
  `prim.GetReferences().AddInternalReference()` instead).
- **This importer side effect also silently rewrites `mefron.usd` itself on
  every run**, growing it by whatever was just imported — confirmed
  unconditional, not preventable via `stage.GetSessionLayer()`. Treat any
  diff on `mefron.usd`/`configuration/*.usd` after a run as expected noise;
  never blindly `git checkout` this file (it may carry real hand-placed
  scene edits too); prefer a scratch copy for headless testing.
- **Accumulated stray robot prims break rendering, not physics.** A stray
  Save mid-run can bake in orphaned robot prims that visibly desync the
  viewport from a correct physics/motion-planning state. Fixed by
  `robot.clear_stray_robot_prims()`, called right after `open_stage()` on
  every run (in-memory only, so it must keep running every time).
- **`SimulationApp` full experience breaks cuRobo's `packaging` import.**
  Pre-load `packaging`/`packaging.version` from real `site-packages`
  before anything imports cuRobo — see
  `kit_bootstrap.py`'s `preload_real_packaging()`.
- **`ninja`/`pip` are broken in this environment.** Fixed via `apt-get
  install ninja-build` — see `docs/docker-and-devcontainer.md`.

## Pinned versions

Isaac Sim `5.1.0`, cuRobo commit `ebb71702f3f70e767f40fd8e050674af0288abe8`,
torch `2.11.0+cu128` (CUDA 12.8). Dev GPU: RTX PRO 4000 Blackwell (sm_120)
— `TORCH_CUDA_ARCH_LIST` must be `12.0+PTX` for this GPU.
