# mefron scene — full history

Chronological bug/fix log for the mefron scanner-assembly scene and its
scripts. `CLAUDE.md` keeps only current state and the gotchas most likely
to bite immediately; this file has the full forensic detail — root causes,
what was ruled out, and how each fix was verified. See
`docs/grasp-and-assembly-offsets.md` for everything specific to *how* the
grasp/assembly relative-pose constants were derived (that content used to
live inside this file's `scripts/mefron.py` section but is cross-cutting
enough to warrant its own doc).

## `examples/curobo_reference/`

`motion_gen_reacher.py` + `helper.py`, fetched verbatim from cuRobo's own
GitHub repo at the exact pinned commit. A pristine reference copy of
cuRobo's official interactive teleop demo (drag a target cuboid, robot
follows via `MotionGen`) — **do not modify these two files**; write a
separate script instead if a robot- or scene-specific variant is needed
(`scripts/mefron.py` / `scripts/build_scene_mefron.py` are exactly that).

**Verified it runs** on this install with two environment fixes: (1) the
prebuilt `kinematics_fused_cu` kernel has a torch ABI mismatch here, and
cuRobo's JIT-compile fallback needs `ninja` — fixed by installing
`ninja-build` via `apt` (see `docs/docker-and-devcontainer.md`); (2) `pip`
is itself broken in this Isaac Sim install (`ModuleNotFoundError: No
module named 'pip._vendor.packaging._structures'`), so `ninja` had to be
fetched as a static binary instead of `pip install ninja` — anything
relying on pip inside the container is currently dead and worth fixing
separately.

**License note**: the overall cuRobo project is Apache-2.0, but these two
files' own header comments say "NVIDIA CORPORATION... strictly prohibited"
(proprietary-looking boilerplate that doesn't obviously match the
repo-level license) — not resolved; treat as internal reference/testing
use only until clarified, and don't redistribute beyond this repo without
checking.

## `assets/mefron/`

A hand-authored scene (its own factory floor, two
`packing_table`/`packing_table_01` copies, and a scanner-assembly CAD
mockup — `finger_print_scanner`, `main_holder`, `screen`,
`backpanel_support`), built directly in the Isaac Sim GUI. `factory
floor/mefron.usd` is the top-level stage file (`defaultPrim=/World`,
`metersPerUnit=1.0`). Untracked in git.

**Real, non-obvious finding**: importing a robot into this file *while
it's opened directly* (`omni.usd.get_context().open_stage()`, not
referenced in) makes Isaac Sim's URDF importer write a disk-persisted,
multi-layer "Robot Description" structure under `factory
floor/configuration/` (a multi-MB `mefron_base.usd` plus smaller
`mefron_physics.usd`/`mefron_sensor.usd`/`mefron_robot.usd` sublayers) —
confirmed via the importer's own log line ("Creating Asset in an in-memory
stage, will not create layered structure") that this behavior is
conditional on the stage having a real, file-backed root layer; it never
happens for a fresh anonymous, in-memory stage (see Conventions below).
This write happens automatically, with no save prompt, every time the
import runs (script or manual GUI import) — confirmed via file mtimes.
Now gitignored (`assets/mefron/*` in `.gitignore`) — it's a large
(~23MB), hand-authored, vendor-origin asset pack with no established
redistribution terms of its own.

**Correction to an earlier claim**: `mefron.usd`'s own *saved* root layer
was believed still pristine (`stage.Save()` is never called by any script
here) based on an earlier session's `Sdf.Layer.FindOrOpen()` check — but a
`/panda` prim spec is confirmed **still present** in the file's saved root
layer as of the session that did the T_H_S/T_S_G/Step-6 work (see
`docs/grasp-and-assembly-offsets.md`): every fresh `open_stage()` of this
file, from a brand-new process, immediately emits `Could not open asset
.../configuration/mefron_*.usd for payload introduced by
.../mefron.usd</panda{...}>` warnings — before any script code runs, so
this can only be coming from the file itself, not something a script
wrote in-memory this run. Not a live bug (these are non-fatal warnings,
not the "layer already exists" crash a *corrupted* configuration file
causes — see `clear_stale_robot_configuration()` below for that distinct
issue), but the file does need a manual GUI fix (open `mefron.usd`,
delete `/panda`, save) to actually clean up — no script here calls
`stage.Save()`, so nothing here can fix this from code.

## `scripts/mefron.py`

A standalone script that mounts cuRobo's bundled Franka Panda onto
`assets/mefron/`'s pre-authored SEKTION-cabinet mount plate
(`/World/sektion_cabinet_instanceable`, world position
`[2.74097, -4.782, 0.7924]` — same value `mefron_layout.yaml`'s
`cr5_mount.position` already uses; **corrected** from an earlier,
now-stale mount at `/World/Factory/Stage/Pedestal_plates/Cube_05` /
`(2.2025, -4.5025, 1.0018)`, which no longer exists after the
pedestal-to-SEKTION-table remount documented under
`configs/scene/mefron_layout.yaml` below), runs an interactive
drag-follow teleop loop (`run_teleop_loop()`, adapted from cuRobo's own
`motion_gen_reacher.py` reference pattern), and provides a scalable
pick-and-place assembly capability — pressing **G**/**P** snaps the
teleop target to a live-computed grasp-approach or assembly-placement
pose for `finger_print_scanner`/`main_holder`, recomputed from their
*current* world poses every time, not baked in once. Opens `mefron.usd`
directly via `open_stage()`. **Superseded by `scripts/build_scene_mefron.py`
below** for the base teleop capability, but kept working and documented
since it's the only script that can host `mefron.usd` reparenting
operations `build_scene_mefron.py`'s referenced-stage session can't (see
`docs/grasp-and-assembly-offsets.md`'s T_H_S section) — has its own real,
confirmed findings:

- `build_teleop_target()`'s usual `CopyPrim`-based approach for building a
  ghost/target copy of the robot's end-effector mesh produced a prim with
  a **completely empty bounding box** here (confirmed via
  `UsdGeom.BBoxCache`). Root cause: the `configuration/` multi-layer
  structure (see `assets/mefron/` above) means the source prim's visuals
  are composed across several layers, and `CopyPrim`'s shallow,
  spec-level copy can't correctly re-resolve a same-layer reference once
  relocated to a new prim path outside that layer stack. Confirmed this
  isn't fixable via `stage.SetEditTarget()` beforehand — the importer
  resets the edit target itself regardless of what it was set to. Fixed
  by replacing `CopyPrim` with an **internal USD reference**
  (`prim.GetReferences().AddInternalReference()`) — a live pointer at the
  already-*composed* result rather than a copy of raw specs, so it
  renders correctly no matter how many layers underlie the source.
  Confirmed live: non-empty bbox with real, plausible gripper-mesh
  extents.
- `mefron.usd` already has its own hand-authored `/PhysicsScene`
  (capitalized, at stage root). `run_teleop_loop()`'s physics-scene check
  only ever looks for the exact lowercase path `/physicsScene` and
  creates one unconditionally if that's missing — it has no way to know
  about the differently-named one already on the stage, so it creates a
  second, redundant scene. Confirmed live that having both active
  simultaneously breaks the robot's PhysX articulation view entirely:
  `isaacsim.core.prims.impl.articulation` logs "Physics Simulation View
  is not created yet" forever, `get_joints_state()` never returns
  non-`None`, and the robot never responds to the target no matter how
  long you wait. Fixed here by deactivating (not deleting — reversible,
  in-memory only, never touches the file on disk) the pre-existing
  `/PhysicsScene` right after opening the stage, so `run_teleop_loop()`'s
  own check finds neither path valid and creates exactly one canonical
  `/physicsScene` itself.
- Calling `timeline.play()` before cuRobo's ~30s blocking
  `motion_gen.warmup()` (which calls no `simulation_app.update()` of its
  own) leaves physics "playing" across a long unpumped real-time gap —
  confirmed live this corrupts PhysX's tensor simulationView by the time
  the drag loop's own `SingleArticulation` gets constructed, crashing
  with `AttributeError: 'NoneType' object has no attribute 'link_names'`
  (see Conventions below). Fixed by never calling `timeline.play()` in
  this script at all — a human clicks Play in the GUI, and
  `run_teleop_loop()` already waits for `is_playing()` itself.
- Clicking **Stop** in the GUI tears down PhysX's simulation view
  entirely — confirmed live that a `SingleArticulation` built before the
  Stop is left pointing at that now-destroyed view, and reusing it after
  a later Play produces the same endless "Physics Simulation View is not
  created yet" symptom, permanently. A `run_teleop_loop()` that only ever
  builds its `SingleArticulation` once, gated by `idx_list is None` and
  checked just on the very first Play, has no path to rebuild it on a
  *later* Play. Fixed in this file by tracking not-playing→playing
  transitions and rebuilding `robot`/`idx_list`/`articulation_controller`
  (and resetting all other per-session state) on *every* fresh Play, not
  just the first. **Verified live**: a scripted test drove a fake drag
  (`plan_single success=True`, `0.29m` real end-effector movement), then
  called `timeline.stop()`/`timeline.play()` again in the same process
  and drove a second fake drag — `plan_single success=True` again,
  `0.34m` movement.
- **A corrupted-`configuration/`-file crash was found and fixed at the
  source.** A previous crash left truncated
  `configuration/mefron_{base,physics,robot,sensor}.usd` stub files on
  disk — these broke every subsequent import into `mefron.usd` the same
  way, forever, since USD caches `Sdf.Layer` objects by identifier and a
  truncated file makes `open_stage()` itself crash with "a layer already
  exists" while resolving `/panda`'s broken payload references, before
  any script code even runs. `clear_stale_robot_configuration()` deletes
  any pre-existing files under that directory — but it must run
  **before** `open_stage()`, not after: the first version of this fix ran
  it after and still crashed, since USD had already cached the broken
  `Sdf.Layer` objects while opening the stage.
- **Grasp-physics and trajectory-pacing parity, ported from
  `build_scene_mefron.py`.** Added `GripperKeyboardControl` (C/O keys), a
  gripper friction material (`GRIPPER_STATIC_FRICTION=0.9`/
  `GRIPPER_DYNAMIC_FRICTION=0.8` on `/World/finger_print_scanner`), a
  stiffened finger drive (`GRIPPER_DRIVE_STIFFNESS=10000.0`), and
  `interpolation_dt`-gated trajectory playback (real elapsed time via
  `time.time()`, not one waypoint per render frame — see
  `build_scene_mefron.py`'s own entry for the FPS-vs-`interpolation_dt`
  mechanism this fixes). Ported as duplicated logic, not shared code,
  matching this file's own established convention of not importing
  `build_scene_mefron.py` (different stage types).
- **`robot.initialize()` crashed with `AttributeError: 'NoneType' object
  has no attribute 'create_articulation_view'`**, even after a
  settle-frame delay and a forced post-warmup `timeline.stop()`. Root
  cause, confirmed by reading `isaacsim.core.simulation_manager`'s actual
  source: `SingleArticulation.initialize()` depends on
  `SimulationManager.get_physics_sim_view()`, which is only ever set via
  one specific chain — timeline PLAY event → `_warm_start()` → gated
  behind the carb setting `/app/player/playSimulations` → if true,
  `initialize_physics()` → dispatches `PHYSICS_WARMUP` →
  `_create_simulation_view()` actually sets the view. That setting is a
  real, user-facing toggle in the Play button's own toolbar dropdown
  (alongside Play Animations/Audio/Computegraph) — if it's off,
  `timeline.is_playing()` still correctly returns `True`, but the
  simulation view never gets created, and no amount of Play-timing or
  settle frames fixes it. Fixed by forcing it on explicitly at the top of
  `main()`: `carb.settings.get_settings().set_bool("/app/player/playSimulations", True)`.
  Confirmed via a headless regression test (`scripts/test_mefron_teleop_headless.py`).
- **`/app/player/useFixedTimeStepping`, forced on in `main()`.** Decouples
  physics stepping from real elapsed wall-clock time between
  `simulation_app.update()` calls. Without it, a heavier per-frame Python
  cost (three arms' worth of cuRobo planning here, vs. near-zero in a
  lightweight demo like the standalone Conveyor Builder scene) makes
  physics take bigger/bunched-up catch-up steps to keep pace with real
  time, which destabilizes friction-coupled mechanisms like
  `ConveyorBelt_A24`'s `PhysxSurfaceVelocityAPI` — confirmed live
  2026-07-21: an identical belt/jig/friction setup was smooth in the
  Conveyor Builder demo but vibrated/rotated under `mefron.py`'s own
  heavier loop. Forces every `update()` call to advance physics by exactly
  one fixed-size step regardless of how long the Python code took.

→ See `docs/grasp-and-assembly-offsets.md` for how T_H_S/T_S_G (the
grasp and assembly relative-pose constants) were derived, the abandoned
Grasp Editor investigation, the Step 6 G/P keybinding wiring and its
debounce-ordering bug, and the open grasp-centering problem.

- **A later session tried moving robot/friction ownership into
  `mefron.usd` itself (`scripts/mefron2.py`), then reverted back to this
  file.** The user deleted the manually-placed Franka and its
  gripper-friction material back out of `mefron.usd` in the GUI and asked
  to return to this script's own code-driven `mount_franka()`/
  `apply_gripper_friction()`/`stiffen_gripper_drive()` pipeline ("lets go
  back to mefron.py i deleted the franka from mefron.usd and gripper
  friction do this reimport and physciacs from script itself"). This file
  is the active script again; the fixes below all landed here, not in
  `mefron2.py`.
- **Motion was only uniformly slow in *relative* terms — far targets
  still moved fast enough to disturb a carried load — fixed via
  `velocity_scale`/`acceleration_scale`, not `time_dilation_factor`.**
  `_TELEOP_TIME_DILATION_FACTOR` (`0.3`, a `MotionGenPlanConfig`-level
  setting) only uniformly re-times an *already-planned* trajectory after
  the fact — confirmed via cuRobo's own source that this can't change the
  plan's relative speed profile, only stretch it. Fixed by capping the
  velocity/acceleration *limits the trajectory optimizer plans within*
  instead, via `MotionGenConfig.load_from_robot_config()`'s own
  `velocity_scale`/`acceleration_scale` kwargs (set once, at `MotionGen`
  construction time, in `setup_motion_gen()`). New module constants
  `_TELEOP_VELOCITY_SCALE = 0.2` / `_TELEOP_ACCELERATION_SCALE = 0.2` —
  deliberately kept in the `0.1`–`0.25` band: confirmed via direct source
  read that cuRobo treats `scale <= 0.25` as a first-class case,
  automatically swapping in a dedicated `finetune_trajopt_slow.yml`
  tuning file made for slow trajectories and increasing the trajectory's
  own time budget (`maximum_trajectory_dt`) to compensate, whereas going
  below `0.1` would additionally require setting `maximum_trajectory_dt`
  by hand. Scaling both velocity **and** acceleration is deliberate: a
  low acceleration limit specifically damps sudden starts/stops/direction
  changes, which is what actually shakes a friction-held carried object
  loose, not just top speed.
- **Gripper was closing too fast even with drive stiffness already tuned
  up for grip strength.** Root cause: the gripper block in
  `run_teleop_loop()` commanded `GRIPPER_OPEN_POSITION`/
  `GRIPPER_CLOSED_POSITION` as an instant position-target jump every
  frame — with `stiffen_gripper_drive()`'s already-high
  stiffness/damping, a stiff position drive tracks a sudden setpoint
  change aggressively, snapping the fingers shut almost immediately.
  Fixed by ramping the *commanded setpoint* itself gradually instead of
  jumping straight to the target: new `gripper_setpoint`/
  `last_gripper_time` loop-local state, advanced toward the target by at
  most `GRIPPER_CLOSE_SPEED` (new module constant, `0.02` m/s — the full
  `0.04`m open↔closed travel takes about 2 seconds) times real elapsed
  wall-clock time each frame. The drive itself is unchanged, so the same
  strong holding force applies once fully closed — only the approach to
  that setpoint is gradual now.
- **Confirmed via source but not yet wired in: cuRobo has no awareness
  that the robot is carrying `finger_print_scanner` once grasped**, which
  is at least part of why a planned move crashes the carried part into
  `main_holder` instead of avoiding it. Confirmed via direct source read:
  `MotionGen` exposes `attach_objects_to_robot()`/
  `detach_object_from_robot()` specifically for this (treating a grasped
  object as rigidly attached to the robot's own collision body for the
  rest of planning, until detached), and `franka.yml` already has a
  pre-built `attached_object` link with 4 spare collision spheres sized
  for exactly this purpose, currently unused. The natural wiring point is
  the existing C/O gripper keybindings in `run_teleop_loop()` (call
  `attach_objects_to_robot()` on close, `detach_object_from_robot()` on
  open), but this is **not implemented** — paused mid-decision to check
  how placement accuracy behaves first, and the session moved on to the
  grasp-centering problem (see `docs/grasp-and-assembly-offsets.md`)
  before returning to it.

## Third arm (mounting a 3rd Franka)

A first attempt (mount + bare-hand cuRobo teleop, no tool) was committed as
`b9cb4c1 BROKEN: Mount arm 3 on ur10_mount_02 with bare-hand cuRobo teleop,
no tool` and reverted back to `a19b672` after live testing showed all three
arms going unresponsive — arm 3's own mount, `OBSTACLE_PRIM_PATHS` entries,
`motion_gen` warmup, and `arms` list wiring were all added in one shot, so
isolating the actual cause took several wrong turns before landing on the
real one. Recorded here since the wrong turns are as useful as the right
answer for next time this scale of change comes up:

- **First guess, ruled out**: a PhysX simulation-view failure (from arm 2's
  already-known nested-`RigidBodyAPI` xformstack warning on its suction
  gripper, see `scripts/mefron_lib/robot.py`'s `attach_suction_gripper()`
  docstring) silently starving `get_joints_state()` for every arm.
  Disproved by live console output: `[Warning] [curobo] Couldn't find
  solution with 10 attempts, resetting seeds` showed `plan_single()` was
  actually running, not silently blocked on empty joint state.
- **Second guess, ruled out**: arm 3's body/pedestal (mounted only ~1.13m
  from arm 2 — within two Franka reach envelopes) was read as in-collision
  against arm 2's own resting pose, so *every* plan attempt failed at the
  start state. Disproved once `[mefron] armX teleop plan_single
  success=True` started printing for all three arms — planning was
  genuinely succeeding.
- **Third guess, ruled out**: a frame-rate/throughput problem —
  `_step_arm()`'s trajectory playback (`teleop.py`, the `cmd_plan`
  execution block) advances at most one waypoint per rendered frame
  (an `if`, not a `while`, gated on `interpolation_dt`), so a 3rd full
  Franka's added PhysX/render load could in principle throttle plan
  execution to a crawl. Never actually confirmed or fixed — superseded by
  the finding below before this was tested to conclusion. May still be
  worth revisiting as a secondary/compounding factor if arm count keeps
  growing.
- **Fourth guess, ruled out**: arm 1/arm 2/arm 3 sharing the identical
  bundled `franka_panda.urdf` (`config.FRANKA_URDF_RELATIVE_PATH`) was the
  cause — tested directly by giving arm 3 a byte-identical duplicate URDF
  with a unique `<robot name="panda_arm3">` (avoiding the shared
  intermediate `/panda` `MovePrim` staging path every import goes through,
  see `mount_franka()`'s docstring) and absolute mesh paths so it didn't
  need its own `meshes/` copy. Still broken with a fully independent robot
  identity — ruled out cleanly.
- **Real root cause, confirmed live**: the user reported the arm was
  "moving programmatically" (cuRobo's plan executed, joint drives were
  correctly commanded) but stayed visually frozen in the viewport — a
  render/physics desync, not a planning or execution failure. Given
  CLAUDE.md's already-documented finding that *every* mefron entry-point
  run silently rewrites `mefron.usd`'s saved root layer (growing it by
  whatever the just-imported Franka(s) add, with no explicit save call
  anywhere in this repo's code), a session that had accumulated enough
  runs — or ever caught a stray Ctrl+S mid-import — had orphaned
  `/World/Franka`, `/World/Franka2`, and the historical intermediate
  `/panda` prim specs baked directly into the file, sitting there the
  moment `open_stage()` returned, confirmed by directly inspecting the
  Stage panel on a fresh open (all three present as top-level siblings of
  `/Environment` and `/PhysicsScene`, before any script had run). Manually
  deleting these three stray prims before running `mefron.py` fixed
  everything immediately.

  Mechanism: `mount_franka()` already deletes whatever's at its own target
  path right before importing into it — but only right before *that*
  specific import call. `mefron.py`'s `open_stage()` is followed by a
  120-frame settle pump (needed for the file's own async content
  resolution) *before* any `mount_franka()` call runs at all — during
  those 120 frames, the stale leftover prims are live, valid,
  physics-schema-bearing articulations, long enough for PhysX/Fabric/Hydra
  to partially register them before `mount_franka()`'s delete+reimport at
  the same path ever gets a chance to run. The fresh reimport's own
  physics/motion-planning ends up correct regardless (each arm is driven
  by its own exact, freshly-created prim_path, read directly off the live
  stage), but the render layer visibly desyncs from it — exactly
  "cuRobo moves it, the viewport doesn't show it."

  Fixed by `robot.clear_stray_robot_prims()`: deletes
  `config.ROBOT_PRIM_PATH`/`ROBOT_2_PRIM_PATH`/`ROBOT_3_PRIM_PATH` and the
  historical `/panda` path (whichever currently resolve to a valid prim)
  via `DeletePrims`, called in `mefron.py` immediately after
  `open_stage()` and *before* the 120-frame settle pump — not just before
  each `mount_franka()` call, which is too late for this specific failure
  mode. In-memory only, no `stage.Save()` — matches this file's existing
  policy of never persisting `mefron.usd` from script code, so (like the
  rewrite itself) this needs to run on every fresh `open_stage()`, not
  just once ever. **Confirmed live: fixed it.**

## `scripts/mefron_lib/` (package split)

`mefron.py` had grown into one file holding constants, robot mounting,
grasp/assembly pose math, keyboard control, and the teleop loop together.
Split into a proper package, `scripts/mefron_lib/` (`kit_bootstrap.py`,
`config.py`, `grasp.py`, `robot.py`, `teleop.py`), with `mefron.py` reduced
to a thin entry point. Also applied to `mefron_gripper_probe.py`,
`mefron_grasp_editor_scene.py`, `franka_grasp_editor_scene.py`, and both
`test_mefron_*_headless.py` files, all of which previously did `import
mefron` purely to reach its module-level functions/constants and to
trigger its packaging-preload side effect. `mefron2.py` (dormant/
superseded, see below) only had its duplicated packaging-preload block
swapped for `kit_bootstrap.preload_real_packaging()` — its other, already-
diverged logic was left alone rather than forced onto the new shared
modules' signatures.

Two non-cosmetic things fell out of this, not just file-moving:

- **`mefron.simulation_app = simulation_app` monkey-patching, eliminated.**
  The four dependent scripts each set this on the imported `mefron` module
  before calling any of its functions, because the old `run_teleop_loop()`
  referenced the bare module-global `simulation_app` (`while
  simulation_app.is_running(): simulation_app.update()`) — grep confirmed
  this was the *only* moved function relying on that global (`mount_franka()`/
  `apply_gripper_friction()`/`stiffen_gripper_drive()` don't touch it).
  `mefron_lib.teleop.run_teleop_loop()` now takes `simulation_app` as an
  explicit first parameter instead, so every caller passes its own local
  variable directly and the monkey-patch requirement disappears entirely.
- **A real bug caught while wiring up `mefron_gripper_probe.py`'s
  `spawn_gripper_probe()`**: importing the hand-only probe while
  `finger_print_scanner` was still selected in the Stage tree nested the
  new prim under it (`/World/finger_print_scanner/GripperProbe` instead of
  `/World/GripperProbe`) — confirmed live via the PhysX warning "Rigid Body
  ... missing xformstack reset when child of another enabled rigid body in
  hierarchy." `import_cr5()`'s `URDFParseAndImportFile` call parents under
  whatever's currently selected when a specific destination isn't otherwise
  forced; `spawn_gripper_probe()` now explicitly clears the Stage-tree
  selection (and deletes any stale prim already at the destination path)
  before importing, so it lands under `/World` regardless of prior
  selection state.

## `scripts/mefron_lib/kit_experience.py` — the viewport-blanking regression

A prior version of `enable_full_experience_extensions()` scaled back to a
small hand-picked subset (just `omni.physx.bundle`, for the Physics
debug-viz menu) after observing the full
`config.FULL_EXPERIENCE_EXTRA_EXTENSIONS` list (~122 names) blank the
viewport shortly after enabling. That subset traded away real
functionality piecemeal and unpredictably (Script Editor, then
`isaacsim.robot_setup.grasp_editor`'s `import_grasps_from_file()` needed
manual Window→Extensions enabling or crashed outright with
`ModuleNotFoundError`) — so it went back to enabling the full list.

Root cause of the "blanking" (confirmed by reading `isaacsim.app.setup`'s
own source, not guessed): it isn't a rendering/menu issue at all —
`isaacsim.app.setup.CreateSetupExtension.on_startup()` reads the carb
setting `/isaac/startup/create_new_stage`, and if true (its own
`extension.toml` default), schedules an async task a few frames out that
unconditionally opens a brand-new stage, discarding whatever's loaded.
Since `enable_full_experience_extensions()` runs after `mefron.usd` is
already open and both Frankas are mounted, that fires late and wipes the
running scene — not "too many extensions at once". Forcing the setting
off (`carb.settings.get_settings().set_bool("/isaac/startup/create_new_stage",
False)`) before enabling fixes it without dropping anything from the full
list.

## `scripts/mefron_lib/config.py` — constants derivation notes

**`GRIPPER_OPEN_POSITION`/`GRIPPER_CLOSED_POSITION` narrowing.** Narrowed
from the full 0–0.04m stroke to bracket `finger_print_scanner`'s actual
12mm grip width (measured via `UsdGeom.BBoxCache` local bound) — the full
stroke let one finger contact and drag the part sideways well before the
other closed the remaining distance. `CLOSED` is the symmetric half-width
(6mm/side); `OPEN` adds a 4mm/side clearance margin for approach.

**`pcb_assembly`'s K key, retired 2026-07-22, root cause.** Tried forcing
`MotionGenPlanConfig`'s `use_start_state_as_retract` to `False`
(regularize IK's null-space/redundant-branch choice toward `robot_cfg`'s
fixed `retract_config` instead of the arm's current joint state), on the
theory it would stop the gripper twisting in place when K was pressed
right after a B (`backpanel_support`) grasp+place cycle left the arm far
from `retract_config`. Confirmed live this made things WORSE — every
part's grasp/approach after B got pulled toward whichever branch is
nearest `retract_config` regardless of current distance from it, instead
of only occasionally diverging for specific target/current-pose pairs
like before. Reverted; root cause of the original twisting itself was
never resolved — switching K's object (`pcb_assembly`) to arm 2's suction
cup instead sidesteps the bug rather than fixing it.

**Suction/screwdriver asset alignment.** Both
`robots/franka_panda/Props/suction gripper.usd` (custom-designed in
SolidWorks for this exact Franka flange — Ø63mm mount face matching
Franka's ISO 9409-1-50 flange OD, Ø50mm suction tip) and
`robots/grippers/electric_screwdriver.usd` mount as `panda_hand` children
needing zero offset/rotation correction: verified live that both assets'
own root Xforms are already coincident with `panda_hand`'s frame
(origin = flange point, +Z toward the fingers) — unlike the earlier
borrowed `robots/ur10_suction/short_gripper.usd`, which needed a solved
offset+rotation because its internal "wrist" frame didn't line up.
`SCREWDRIVER_LOCAL_ORIENTATION_WXYZ` is the user's live-jogged GUI pose
(Orient X/Y/Z = 90/45/90°, USD `rotateXYZ` = Rx·Ry·Rz), converted to wxyz.

**Suction approach pose derivation (`screen`, `pcb_assembly`).** Both
derived the same way as `docs/grasp-and-assembly-offsets.md`'s
`compute_relative_pose()` methodology: hand-jog the suction target against
the live part in the GUI until the cup tip sits correctly, then read back
target-wrt-part. `screen`'s pose supersedes a first-pass bbox-top
derivation (`scripts/mefron_screen_approach_probe.py`) that was wrong
twice over — it ignored the 100mm cup length, and assumed local +Z was
"up" when `screen`'s own frame is flipped ~180° about X. `pcb_assembly`'s
first-pass Z (0.10492) reported attached but didn't actually lift the
part; the current value (~1cm further out) lifts cleanly.

**Conveyor: OmniGraph route, not direct PhysX writes.**
`conveyor.setup_conveyor_belt_graph()` drives `ConveyorBelt_A24` through
Isaac Sim's `isaacsim.asset.gen.conveyor` OmniGraph node
(`CreateConveyorBelt`), not a direct `PhysxSurfaceVelocityAPI` write. This
exact route was tried once before (Create > Isaac Sim > Conveyor) and
abandoned 2026-07-20 after two live-confirmed failures: the node's own
"Enabled" input ended up unchecked (silently means the node never computes
at all), and deleting the graph left a nonzero `surfaceVelocity` value
permanently orphaned on the belt's USD spec (authored directly on the
rigid body, not cleared by removing the graph that drove it). That session
fell back to the direct-PhysX write, which worked but bypassed "the right
way." This retry engineers around both failures instead of repeating them:
`setup_conveyor_belt_graph()` explicitly forces `inputs:enabled` to `True`
and reads it back, and deletes any stray graph plus re-zeros
`surfaceVelocity` before rebuilding. The native node isn't a different
physics mechanism than the direct-write fallback — its own changelog
confirms it writes to the same `PhysxSurfaceVelocityAPI` attribute, just
wrapped in an ActionGraph (`OnPlaybackTick` → `IsaacConveyor`) for
authoring convenience. `CONVEYOR_LOCAL_VELOCITY_DIRECTION`'s axis flipped
between local X and Y more than once across earlier graph
deletions/recreations — treat the current value as a starting point, not
a settled fact.

**`FULL_EXPERIENCE_EXTRA_EXTENSIONS` load-order crash.** Mounting a second
Franka (a second native URDF import in one process) crashes Kit's
`isaacsim.asset.importer.urdf` plugin if these extensions are already
loaded at import time — confirmed live. Enabling them AFTER both Frankas
are mounted reproduces the identical final feature set with zero crash
(confirmed live: all 122 enable cleanly). See `robot.mount_franka()`'s
docstring and `kit_experience.enable_full_experience_extensions()`.

## `scripts/mefron_lib/conveyor.py` — investigation detail

**`ConveyorControl`'s state machine + `reset()`.** A '1' press toggles
between a full back-to-front or front-to-back run (forward then back,
reversing direction each time) — a press mid-transit is ignored (not
queued, not a reversal): this belt's only measured, confirmed-safe
behavior is a full run, and half-way reversals were never asked for or
tested. `reset()` matters because `ConveyorControl` is built once, outside
the per-Play arm-state rebuild: without it, a '1' press queued before a
Stop would fire the instant the next Play starts with no new keypress at
all, and once "moving" it silently ignores every further press until it
reaches an end, making the belt look completely unresponsive. Stop reverts
the stage (and the jig) to its initial position, so `reset()` sets
`state="back"` and re-zeros the velocity variable to match, covering Stop
also reverting the graph's own authored default.

**Conveyor vibration, root cause (2026-07-21).** `_jig_world_y()` measures
`main_holder_jig`'s live Y every frame during transit via a fresh
`SingleXFormPrim`. `main_holder_jig`'s `xformOpOrder` is `[translate,
orient, scale, scale:unitsResolve]` (the last op compensates the source
asset's `metersPerUnit=0.001` down to this stage's meters), but
`SingleXFormPrim`'s default (`reset_xform_properties=True`) forces every
prim it wraps down to exactly `[translate, orient, scale]` post-init,
silently stripping `unitsResolve`. Since a fresh `SingleXFormPrim`
constructs every frame during transit, leaving the default on was
re-stripping that op every single frame while the belt moved, violently
disrupting the jig's effective scale/transform each step — confirmed live
as the actual cause of the vibration/rotation (a manual edit of the
graph's own Velocity variable, which never touches `SingleXFormPrim`,
moved the same jig smoothly). Fixed by passing
`reset_xform_properties=False`.

## `scripts/test_mefron_teleop_headless.py`

Headless regression test for `mefron.py`'s `run_teleop_loop()` (reuses
`mefron.py`'s own functions as a library, fakes a target drag via
monkeypatching `target.get_world_pose()`). Used to confirm the
`/app/player/playSimulations` fix above. **Verified**: `plan_single
success=True`, real joint-position deltas.

## `scripts/test_mefron_assembly_headless.py`

Headless regression test for the Step 6 `compute_grasp_approach_pose()`/
`compute_assembly_grasp_target()` G/P one-shot snap requests (see
`docs/grasp-and-assembly-offsets.md`), simulating a keypress by calling
`gripper_control.request_grasp_approach()`/`request_assembly_target()`
directly rather than a real keyboard event. Runs both phases in sequence
in one process. **Verified**: caught the debounce-ordering bug via its
own failure (sane pose math, zero `plan_single` calls) before the fix,
then passed cleanly after.

## `configs/scene/mefron_layout.yaml` + `scripts/build_scene_mefron.py`

Originally written as the **preferred** approach for the mefron scene, in
place of `scripts/mefron.py` above. Same overall goal (mount the Franka,
run cuRobo teleop) but built as a fresh, anonymous `SimulationApp` stage
with `mefron.usd` brought in via `add_reference_to_stage()` (under
`/World/Factory`), not opened directly (see Conventions below for why
this pattern generally sidesteps a whole class of URDF-import bug). This
one architectural difference avoids essentially every bug found in
`scripts/mefron.py` above *by construction*, confirmed live (see below) —
**but in practice, all of this session's active interactive work
(grasp/assembly tuning, speed/gripper fixes, pose re-derivations)
happened directly in `scripts/mefron.py`, not this file**, because
deriving T_H_S required temporarily reparenting `finger_print_scanner`
under `main_holder` in the Stage tree, which only works against
`mefron.usd` opened directly — `build_scene_mefron.py`'s own
referenced-stage session hits the "Cannot move/rename ancestral prim"
restriction for that. Treat `build_scene_mefron.py` as verified-and-working
but currently dormant, and `scripts/mefron.py` as the actually-active
script, until/unless something forces a switch back:

- Since the stage's root layer stays anonymous/in-memory, the URDF
  importer never triggers the file-backed "Robot Description" multi-layer
  write (see `assets/mefron/` above) — `build_teleop_target()`'s
  original, unmodified `CopyPrim` approach produces a target with real
  geometry on the first try, no internal-reference workaround needed.
- `mefron.usd`'s own `/PhysicsScene` lives at `/PhysicsScene` (a sibling
  of `/World`, not nested inside it) in the source file —
  `add_reference_to_stage()` only brings in the referenced prim's own
  subtree (mefron's `/World` and everything under it), so this sibling
  prim is never pulled onto the new stage at all. No duplicate-scene
  conflict to work around; `run_teleop_loop()`'s unmodified
  `/physicsScene` check just creates the one and only scene.
- Referencing `mefron.usd` under `/World/Factory` nests its own content
  one level deeper than opening it directly would: mefron's own
  `/World/Factory` (its internal factory floor) becomes
  `/World/Factory/Factory` here, and its `/World` siblings
  (`packing_table`, `finger_print_scanner`, etc.) become
  `/World/Factory/packing_table` etc. Confirmed empirically — world
  positions of nested content are unaffected (both stages are
  meters-native, no scale reconciliation needed), only prim *paths*
  shift.
- The Stop→Play stale-`SingleArticulation` fix (see `scripts/mefron.py`
  above) is ported here too and **re-verified independently** in this
  file's own architecture: first-play fake-drag `0.2886` rad
  end-effector movement, then a real `timeline.stop()`/`play()` cycle
  in-process, then a second fake-drag `0.3378` rad movement — both
  `plan_single success=True`.

Also loads `SimulationApp` with the **full** `isaacsim.exp.full.kit`
experience (same one `isaac-sim.sh` itself launches) instead of
`SimulationApp`'s own default minimal `isaacsim.exp.base.python.kit`, for
interactive (non-`--headless`) runs only — the base experience is missing
most UI extensions, including the Physics debug-visualization menu needed
to view collision meshes. **Real bug found and fixed**: switching to the
full experience broke cuRobo's own `from packaging import version`
(inside `curobo/util/torch_utils.py`) with `FileNotFoundError:
.../omni.services.pip_archive-.../pip_prebundle/packaging/_structures.py`
— a *different* extension bundles its own incomplete internal `packaging`
copy (missing `_structures.py`, an older `packaging` release than the
real one) that somehow takes priority under the full experience.
Confirmed this is **not** a simple `sys.path`-ordering shadow: a full
`sys.path` dump under the full experience never contains any path under
that extension at all, yet `importlib.util.find_spec("packaging")` still
resolves there — some other, non-path-based resolution (almost certainly
a custom `sys.meta_path` finder the extension system registers) is
responsible, and it turned out to intercept `packaging.version`
specifically by name too, ignoring the parent module's own `__path__`
even after pre-registering a correct `packaging` in `sys.modules`. Fixed
by explicitly pre-loading *both* `packaging` and `packaging.version` from
their real `site-packages` location and setting the latter as a plain
attribute on the former, so `from packaging import version` resolves via
attribute lookup alone — confirmed live this survives the full experience
and reaches `curobo motion_gen: READY` same as before. Applied to both
this file and `scripts/mefron.py` for consistency.

- **Mount remount: pedestal → SEKTION table.** The old
  `Pedestal_plates/Cube_05` mount plate was removed from `mefron.usd` in
  the GUI and replaced with a pre-authored SEKTION cabinet table
  (`/World/sektion_cabinet_instanceable`, a `/World` sibling of `Factory`
  in mefron.usd's own raw hierarchy — becomes
  `/World/Factory/sektion_cabinet_instanceable` here after this file's
  own one-level nesting, see above). `cr5_mount.position`
  (`[2.74097, -4.782, 0.7924]`) was read directly off a manually-placed
  Franka copy's Property-panel Translate/Orient in the GUI, not
  independently re-derived via a `get_world_pose()`/`BBoxCache` script
  like most other poses in this project — worth re-checking first if the
  robot ends up floating/clipping through the table. `cr5_mount.pedestal`
  was renamed to **`cr5_mount.mount_surface`** in `mefron_layout.yaml`
  (`get_teleop_obstacles()` and `main()`'s status-print list both use the
  new key). `teleop_target.position` was carried forward
  **algebraically** (old target minus old mount, applied to the new
  mount position) rather than re-derived from scratch — valid only
  because `cr5_mount.orientation_wxyz` is unchanged (still identity);
  recompute properly, don't just shift, if the mount orientation ever
  changes.
- **Grasp-physics fixes: `apply_gripper_friction()` /
  `stiffen_gripper_drive()`.** Read-only inspection of `mefron.usd` found
  **zero** `PhysxMaterialAPI` authored anywhere (not on
  `finger_print_scanner`/`main_holder`/`screen`, not a usable one on
  `backpanel_support`'s pure-render `Black_Paint_01` material, no
  `PhysicsScene`-level default), and confirmed the Franka side has none
  either (`franka_panda.urdf` has no friction tags, and the import path
  sets none) — both sides of every grasp contact were relying on PhysX's
  un-overridden engine default friction, which is what was causing
  `finger_print_scanner` to slip out of the gripper regardless of its
  mass. `apply_gripper_friction()` creates one shared material at
  `/World/GripperFrictionMaterial` (`GRIPPER_STATIC_FRICTION=0.9`/
  `GRIPPER_DYNAMIC_FRICTION=0.8`, restitution 0.0) via the real
  `omni.physx.scripts.utils.addRigidBodyMaterial()`/
  `physicsUtils.add_physics_material_to_prim()` helpers, bound to both
  Franka fingertip links and everything listed in the new
  `high_friction_prim_paths` config key (currently just
  `finger_print_scanner`). Separately, a headless inspection of the
  actual imported joint prims (`/World/CR5/joints/panda_finger_joint1|2`,
  `UsdPhysics.DriveAPI` type `"linear"`) found `stiffness=625.0`/
  `damping=10.0` — not the configured `default_drive_strength=1047.2`/
  `default_position_drive_damping=52.36` (the URDF importer derives a
  different effective value for prismatic joints, and the URDF's own
  `<dynamics damping="10.0"/>` on these two joints wins over the
  importer's default damping), leaving most of the fingers' real
  `effort="20"` N ceiling unused for a typical 1-2cm grasp position error
  (~6-12N reached). `stiffen_gripper_drive()` raises both to
  `GRIPPER_DRIVE_STIFFNESS=10000.0`/`GRIPPER_DRIVE_DAMPING=200.0` via
  `UsdPhysics.DriveAPI` on both finger joints. Both fixes are
  **runtime-only, not persisted** to `mefron.usd`. Headless read-back
  confirmed the material's friction values and the joints' drive values
  land exactly as configured; **not yet live-tested** whether the
  combination actually produces a firm, non-slipping grip in the GUI.
- **Keyboard gripper control.** `GripperKeyboardControl` +
  `build_gripper_keyboard_control()` subscribe to real `carb.input`
  keyboard events (**C** closes, **O** opens), confirmed against this
  install's own stubs (`carb/input.pyi`, `omni/appwindow/_appwindow.pyi`)
  rather than assumed from memory. `GRIPPER_OPEN_POSITION=0.04`/
  `GRIPPER_CLOSED_POSITION=0.0` come from `franka_panda.urdf`'s actual
  joint limits (`panda_finger_joint1/2`, prismatic, `lower="0.0"
  upper="0.04"`). Wired into `run_teleop_loop()` via an optional
  `gripper_control` param, applied every playing frame *after* the arm's
  own `cmd_plan` block so it always wins that frame's write to the finger
  joints. Chosen over the `isaacsim.robot_setup.grasp_editor` tool
  because `franka.yml` already excludes the two finger joints from
  IK/trajopt entirely, so finger actuation was always going to be
  orthogonal to cuRobo regardless. **Verified headlessly** (joint-drive
  mechanism only, via the `set_closed()` test hook); **not yet verified**
  with a real interactive keypress in the GUI, and no permanent headless
  regression test exists yet for plain open/close specifically.
- **Trajectory playback was running too fast with an arrival
  oscillation — root-caused to a frame-vs-time mismatch, not a
  stiffness/damping problem.** `get_interpolated_plan()` spaces waypoints
  `interpolation_dt` seconds apart (`0.02s`), but this loop's own render
  rate (confirmed live at ~119 FPS, far above the 50Hz the plan assumes)
  has nothing to do with that — applying one waypoint per render *frame*
  instead of one per `interpolation_dt` played the whole trajectory back
  at roughly 2.4x its intended speed and cut its final
  deceleration-to-zero-velocity ramp short, leaving real residual
  velocity for the position-hold drive to absorb once `cmd_plan` ran out.
  Fixed by gating playback on real elapsed time (`time.time()`) against
  each plan's own `result.interpolation_dt` instead of one waypoint per
  render frame.

## `scripts/mefron_lib/robot.py` — mount/gripper investigation detail

**`remove_parallel_jaw_gripper()`: deactivate, not delete.** Converting arm
2 to a suction end-effector needed its finger links/joints gone, but
`omni.kit.commands.execute("DeletePrims", ...)` silently no-ops for these
specific prims — returns success, no error, but the prims stay
valid/active — since their specs live across the URDF importer's
disk-persisted, multi-layer `configuration/` stack rather than purely on
the current edit target. `Usd.Prim.SetActive(False)` authors directly on
the stage's current edit target regardless, and works.

**`hide_hand_housing()`: un-instancing before hiding.** The URDF importer
makes imported mesh geometry instanceable by default, and since all three
arms import the identical `franka_panda.urdf`, `panda_hand/visuals` across
all three can share one native-instancing prototype — authoring visibility
directly on an instance-proxy prim isn't a real per-instance override in
that case. Walking up to the nearest instance root and calling
`SetInstanceable(False)` un-shares that one arm's subtree from the
prototype first, so `MakeInvisible()` only affects that arm's own Franka.
`panda_hand` itself stays active (it's cuRobo's `franka.yml` `ee_link`) and
its collisions sub-scope stays active too (dropping it would stop the
other arm's planner from seeing it as an obstacle) — only the visuals
sub-scope is hidden.

**`attach_suction_gripper()`: baked-in collider/rigid-body on the custom
asset.** Unlike the borrowed UR10 asset, the custom SolidWorks-exported
suction flange comes in with a baked-in collider on its own mesh —
confirmed live 2026-07-17: a PhysX raycast from the cup tip along
`panda_hand`'s own +Z self-hit this asset's `Revolve1/Mesh` at distance
0.0, before ever reaching outward, silently breaking
`attach_surface_gripper_physics()`'s grab raycast. It also carries a
baked-in enabled `RigidBodyAPI` (PhysX logs "missing xformstack reset when
child of another enabled rigid body" once mounted — a nested-rigid-body
hierarchy, not merely a stray collider). Both are disabled via
`CollisionEnabled`/`RigidBodyEnabled = False` rather than removing the APIs
outright: a 2026-07-18 attempt at `prim.RemoveAPI(...)` (to also silence
the xformstack warning, which disabling alone doesn't since that check is
structural, on `HasAPI(RigidBodyAPI)`, not on whether the instance is
enabled) coincided with the suction gripper mesh going invisible in the
user's own GUI run; reverted back to disable-only to isolate whether
`RemoveAPI` was actually the cause before retrying. The xformstack warning
is expected to still appear with the current version — a known tradeoff
pending a fix that gets both.

**`attach_surface_gripper_physics()`: compliance tuning.** The visual
`suction_gripper` child (from `attach_suction_gripper()`) has zero physics
of its own — that function strips any collider/rigid-body its USD source
brings in, which is load-bearing here since this function authors the
*real* attach mechanism separately: one `UsdPhysics.Joint` tagged with
`IsaacAttachmentPointAPI`, plus `PhysicsLimitAPI`/`PhysicsDriveAPI`
compliance values taken directly from NVIDIA's own bundled reference
(`isaacsim.robot.surface_gripper`'s `data/SurfaceGripper_gantry.usda`) so
the joint actually constrains the grabbed object once attached — confirmed
live 2026-07-17 that without this, the joint is a fully-free D6 (the
schema's own documented default): the manager reports Closed/gripped
correctly, but nothing physically holds the object, so lifting leaves it
behind. `transX`/`transY` are locked (low > high, per `PhysicsLimitAPI`'s
own schema doc); `transZ` gets a small compliant range + spring (give
along the suction axis, like a real cup flexing slightly); `rotX`/`rotY`
get a looser spring (tilt compliance); `rotZ` is much stiffer (resists the
object spinning about the suction axis). This authoring happens before
Play/attach, so the low>high locked-axis convention is honored by the
initial parse — hot-patching an already-live joint mid-session would need
a tiny valid range instead, not an inverted one. `body1` is left unbound;
the `SurfaceGripperManager` rebinds it live to whatever it finds within
`max_grip_distance` at close time. `excludeFromArticulation` is the one
physics attribute that isn't optional: `panda_hand` is a real articulation
link, and without it PhysX tries to fold this joint into the Franka's own
reduced-coordinate solve instead of treating it as an auxiliary
maximal-coordinate constraint. Not `robot_schema.ApplyAttachmentPointAPI()`
for the schema apply itself: that helper calls
`Classes.ATTACHMENT_POINT_API.name` (the plain Enum's Python identifier)
instead of `.value` (the real schema name, `IsaacAttachmentPointAPI`) —
every sibling `Apply*()` in that module correctly uses `.value`, only this
one doesn't, so the real token is authored directly instead.

## `scripts/mefron2.py`

A simplified sibling of `scripts/mefron.py` built for a "everything
already baked into `mefron.usd`, no code-driven import" approach: assumes
the Franka and its gripper-friction material are already saved directly
into `mefron.usd` (via Isaac Sim's own GUI robot-asset import — NVIDIA's
bundled Nucleus Franka Panda asset, not this repo's URDF-import
pipeline), so this script does no import and no friction/drive-stiffness
authoring at all — only cuRobo setup, the draggable teleop target, and
the G/P/C/O controls, all ported from `mefron.py`. Two real,
confirmed-live differences from `mefron.py`'s equivalents were needed:

- `build_teleop_target()`'s first attempt — a plain `CopyPrim` from
  `panda_hand/geometry` — produced an **empty bounding box**, confirmed
  via `UsdGeom.BBoxCache`. Root cause: `geometry` is only
  `instanceable=True` metadata pointing at an instance; `CopyPrim`'s
  shallow, spec-level copy carries the instanceable flag but not the
  composition arc needed to resolve it, leaving a hollow shell. Fixed by
  resolving the actual instance-proxy `Mesh` prim underneath `geometry`
  first (via `Usd.TraverseInstanceProxies()`), then `CopyPrim`-ing from
  *that* already-resolved path — confirmed live this produces a correct
  non-empty bbox, and (separately, tested via a diagnostic script)
  `check_ancestral()==False` and a real `MovePrim` reparent succeeds,
  unlike the `AddInternalReference()` approach `mefron.py` uses for its
  own (differently-sourced) Franka.
- NVIDIA's bundled Franka asset bakes real `UsdPhysics.CollisionAPI` onto
  that same mesh prim (visuals and collision aren't split into separate
  prims the way a from-scratch URDF import keeps them), so the copied
  target inherited a real, live collider — confirmed via a real PhysX
  overlap query that it was already geometrically overlapping the actual
  robot's own nearby links. Since the resolved copy (unlike a plain
  instance proxy) is a genuine, editable prim,
  `RemoveAPI(UsdPhysics.CollisionAPI)` works directly on it with no
  further workaround needed.
- `MOUNT_POSITION`/`MOUNT_ORIENTATION_WXYZ` aren't hardcoded here since
  there's no mount step — `get_robot_base_pose()` reads the
  already-placed robot's real live world pose off the stage once at
  startup instead, correct regardless of exactly where the robot was
  manually placed when it was saved into `mefron.usd`.

**Verified working** at the time it was built: headless run reaches
`curobo motion_gen: READY`, all status paths `OK`, and a fake-drag test
gives `plan_single success=True` with a non-empty target bbox and no
articulation errors. **Now superseded**, not actively used: the user
later deleted the manually-placed Franka and its gripper-friction
material back out of `mefron.usd` in the GUI and asked to return to
`scripts/mefron.py`'s own code-driven pipeline instead. This file is kept
as a working artifact for its CopyPrim/instance-proxy-resolution
technique, but as of that revert it no longer matches what's actually
saved in `mefron.usd` (no Franka, no friction material) — it would need a
fresh Franka re-added to `mefron.usd` by hand (from `robots/franka_panda/`,
below) before it could run again; treat it as a reference, not as
ready-to-run.

## `robots/franka_panda/`

A local, Content-Browser-"Collect Asset"-vendored copy of NVIDIA's own
Nucleus-hosted Franka Panda asset (public, unauthenticated S3 bucket; see
the directory's own `SOURCE.md` for the exact URL), built specifically to
unblock `scripts/mefron2.py`: the Nucleus-hosted original's link geometry
is `instanceable=True`, and USD refuses to author anything — including
`SetInstanceable(False)` — onto a *read-only, Nucleus-backed* instance
proxy. Collecting the asset locally (which also pulls in every file it
references, unlike a plain `curl` of `franka.usd` alone) makes it a real,
locally-editable file instead. ~39MB, gitignored
(`robots/franka_panda/*` / `!robots/franka_panda/SOURCE.md`) — NVIDIA
Omniverse License Agreement content-pack terms (same as `assets/mefron/`).
Now effectively dormant along with `mefron2.py` itself, kept only because
that script still references it.

## `main_holder` convex-decomposition collision tuning

**Researched and confirmed against this Isaac Sim install's actual
schema, not yet applied.** Switching `main_holder`'s collider from Convex
Hull to Convex Decomposition (via GUI, needed for the same reason as
`finger_print_scanner`'s own collider — see the grasp-physics fixes
above) made the part sink slightly into the table and lose its small
mounting studs. A research **Workflow** (3 parallel research agents + 3
adversarial verify agents, every claim grounded against this install's
real schema files, not memory) confirmed:

- Schema: `PhysxSchema.PhysxConvexDecompositionCollisionAPI`
  (single-apply), applied alongside `UsdPhysics.MeshCollisionAPI` with
  `approximation="convexDecomposition"`. Real schema defaults:
  `hullVertexLimit=64`, `maxConvexHulls=32`, `minThickness=0.001`,
  `voxelResolution=500000`, `errorPercentage=10`, `shrinkWrap=False`.
- Mechanism (VHACD-family: voxelize → cluster → convex-hull-per-cluster →
  optional shrink-wrap re-projection): **sinking** happens because
  `shrinkWrap` defaults to `False`, so nothing re-projects the
  voxel-quantized hull back onto the true surface. **Small-feature loss**
  happens because `voxelResolution` is a budget spread over the *whole
  part's bounding box*, not per-feature — mm-scale studs on a much larger
  flat part can fail to rasterize at all, or get merged away during the
  volume-error-driven clustering step.
- `main_holder`'s actual collider prim (confirmed via headless
  inspection, not assumed by analogy):
  `/World/Factory/main_holder/tn__mainholder_kA` — `approximation` is
  `convexHull` on-disk in `mefron.usd` as of this check (any live GUI
  edit to `convexDecomposition` is session-local until saved).
- Recommended values, given to the user as a GUI walkthrough, **not**
  implemented in code — deliberately: the user pushed back on hardcoding
  per-part collision tuning as not scalable, and this is an
  asset-intrinsic property of `mefron.usd` itself: **Shrink Wrap → ON**
  (fixes sinking), **Voxel Resolution → ~3,000,000–5,000,000** (fixes
  stud loss; hard ceiling is 5,000,000), **Max Convex Hulls → ~128**
  (secondary, budget for small features), **Error Percentage → ~1–2**
  (secondary), Hull Vertex Limit and Min Thickness left at defaults —
  then **save `mefron.usd`** (`Ctrl+S`), the one fix in this
  investigation meant to persist into the asset file directly rather than
  be reproduced by code. **Not yet applied/tested** as of the last check
  (`mefron.usd` isn't tracked in git, so this can't be re-verified from
  the repo alone — check live before assuming it's still pending).

## Open issues — full investigation detail

CLAUDE.md keeps only a short pointer to each of these; this is the full detail.

**Assembly placement (P) redesign, reverted 2026-07-17.** Tried a proper
lift/translate/rotate/descend sequence (the original 2-stage lift baked the
*final* X/Y into the lift waypoint, so cuRobo swung sideways instead of
lifting straight; a 3-stage version fixed that but its combined
rotate+translate leg made cuRobo hold the old orientation until the very end
of that leg and snap to final right at the align→descend handoff, a violent
kickstart). Reverted all of it after finding a deeper, unrelated issue:
`ASSEMBLY_LIFT_HEIGHT` is a fixed world-frame Z constant — unlike every
other pose in this system, which is computed relative to a live prim
(`main_holder` for `ASSEMBLY_RELATIONSHIPS`, the part itself for grasp
approach) — so moving `main_holder` (or repositioning the assembly
generally) breaks the staged sequence outright. Next attempt should make the
lift clearance relative instead — e.g. a margin above whichever of the
current/final Z is higher — rather than an absolute world height. May or
may not also be the same grasp-centering issue (see
`docs/grasp-and-assembly-offsets.md`); not confirmed either way.

**Conveyor collision hang, confirmed live 2026-07-18.** Adding the new
`ConveyorBelt_*`/`container_h20*` prims to `OBSTACLE_PRIM_PATHS` (even just
the 5 conveyor + 4 container top-level Xforms) made
`get_obstacles_from_stage()`'s mesh-collision-world construction hang (past
its own "Creating new Mesh cache: 95" log line) for over an hour with zero
forward progress, steady CPU/GPU load, and no crash/OOM to even signal
failure — had to be killed. Each top-level Xform recurses into every child
mesh (9 prims → ~95 individual meshes), and real conveyor-line CAD
assemblies (rollers, frame, guards, motor housing — 13–113MB per file under
`Conveyors/`) are far more geometrically complex than the single
`packing_table` prop they replaced, well past what cuRobo's mesh-based
collision checker can preprocess in reasonable time. Next attempt should use
primitive/cuboid obstacle approximations instead of the raw CAD meshes
(cuRobo's `WorldConfig` supports cuboid obstacles directly), or narrow to
specific lightweight sub-prims rather than whole assemblies.

**Arm 2's suction release (L key) doesn't actually let go.** Pressing L
calls `open_gripper()` (the real `isaacsim.robot.surface_gripper` runtime
interface), which flips the manager's status to Open, but the object stays
physically stuck — the only working fix so far is manually selecting
`SurfaceGripperJoint` under `panda_hand` in the Stage panel and unchecking
its "Joint Enabled" property by hand every time. Tried scripting that exact
toggle (`UsdPhysics.Joint`'s `physics:jointEnabled`, via
`GetJointEnabledAttr()`) from `SurfaceGripperKeyboardControl.open()`/
`close()` across three variants, all confirmed live and all reverted:

1. `open()` sets `jointEnabled=False` right after `open_gripper()`;
   `close()` sets it back `True` right before `close_gripper()`. L then
   released correctly, but switching target objects mid-session (V on
   `pcb_assembly` right after a prior L+N/M cycle on `screen`) sent the arm
   violently snapping back toward `screen`'s location.
2. Same, but gated re-enabling on `is_closed()` polled once per frame from
   `_step_arm()` instead of doing it synchronously in `close()`. Deadlocked
   instead — V never attached again, apparently because the manager can't
   reach `Closed` status while its own joint is disabled.
3. Re-added `joint.GetBody1Rel().ClearTargets(True)` alongside the
   jointEnabled toggle (on the theory that a stale `body1` binding to the
   previous object was the cause of variant 1's snap) — no change, same
   violent snap-to-previous-object as variant 1.

Root cause per `isaacsim.robot.surface_gripper`'s own shipped headers
(`SurfaceGripperManager.h`/`SurfaceGripperComponent.h` —
`/isaac-sim/exts/isaacsim.robot.surface_gripper/include/...`): the real C++
`SurfaceGripperManager` tracks gripped objects, attachment points, and
per-joint settling counters (`m_settlingDelay = 10` physics steps) in its
own memory (`m_writeToUsd` defaults **false**) and processes attach/detach
as **queued** PhysX/USD actions drained on its own `onPhysicsStep`, not
synchronously within the Python call. `SurfaceGripperJoint` is also
registered as this manager's own attachment point
(`IsaacAttachmentPointAPI`), so it's independently watching that exact prim
for USD change notifications. Editing `jointEnabled`/`body1` on it directly
from Python races the manager's own deferred queue and its
`onComponentChange` listener, producing a different broken symptom each time
depending on exactly when the edit lands relative to the manager's own
processing — not a bug in the manager, but us fighting its ownership of
that prim. Current code is back to plain `open_gripper()`/`close_gripper()`
only (matching `isaacsim.robot.manipulators`' own `SurfaceGripper` wrapper
and the reference `OgnSurfaceGripper` node — neither ever touches the joint
directly), i.e. **the manual Stage-panel workaround is still required**.
Next attempt should look at whether the manual "uncheck" is even doing
anything physically real (vs. the elapsed time spent navigating the UI being
what actually lets the manager's own retry/settling logic clear itself)
before trying to automate it again, or look for a genuine reset/detach entry
point in `isaacsim.robot.surface_gripper._surface_gripper`'s interface
beyond `open_gripper()`/`close_gripper()`.

## Automatic tool changer (ATC branch)

Replaced the 3-Franka cell with one Franka + a scripted `UsdPhysics.
FixedJoint`-based tool changer (male coupler permanent on the wrist,
gripper/suction/screwdriver as detachable modules parked in a rack). Full
design, Isaac Sim prior-art survey, and open issues:
`docs/tool-changer.md`. Verified end-to-end headless
(`scripts/test_mefron_tool_changer_headless.py`, real GPU, both
`mefron.py --headless` and the dedicated test passing) on 2026-07-25, but
only after finding and fixing six real bugs by actually running the
mechanism under PhysX rather than by reading the USD Physics schema docs
— recorded in full in `docs/tool-changer.md`'s "Gotchas confirmed live"
section since they're reusable lessons, not just this feature's history.
**Gotcha 6 specifically was invisible to the headless test** (it only
checks poses/joints, not cross-robot visual reference integrity) and only
surfaced when the user opened the scene in the real GUI afterward and
found the main arm present in the Stage tree but not rendering — a
reminder that "the headless test passed" isn't the same claim as "nothing
is broken," only "nothing this test happens to check is broken":

1. A tool's own enabled collision fighting the wrist joint's pull against
   `panda_hand`'s collider (settled tens of cm short of the target instead
   of converging — first symptom noticed, took several other fixes before
   this was isolated as a distinct, independently-real cause).
2. Per-tool wrist joint paths, not one shared name (defensive fix for
   suspected stale-target resolution on same-path joint redefinition —
   applied before gotchas 3-5 were found, never reverted to re-isolate
   whether it was load-bearing on its own).
3. The gripper tool's `female_coupler` living under a plain organizing
   Xform instead of a real `RigidBodyAPI` link (`base_link`) — confirmed
   by direct `Usd.PrimRange` inspection that the wrapper prim itself
   carries neither `RigidBodyAPI` nor `ArticulationRootAPI`.
4. `electric_screwdriver.usd` carrying no baked-in `RigidBodyAPI`
   anywhere in its subtree at all (confirmed the same way) — unlike the
   suction gripper asset, which does.
5. A URDF-importer-synthesized `root_joint` fixing the hand-only tool's
   free-floating `base_link` to the world — `DeletePrims` silently
   no-ops on it (same class of gotcha as `remove_parallel_jaw_gripper()`'s
   existing finger-joint workaround); `SetActive(False)` is what actually
   removes it.
6. **Found after 1-5, from the real GUI, not from the headless test, and
   took two fix attempts**: the hand-only gripper tool's URDF reused
   `base_link`/`ee_link` — the exact same link names the main arm's own
   `franka_panda.urdf` uses for its root/tip links. The URDF importer's
   shared, disk-persisted "Robot Description" cache keys visuals by bare
   link name, not full prim path, so importing both in the same session
   let the tool's entry silently overwrite the main arm's own — arm
   stayed valid/correctly-posed (why headless checks missed it), but its
   visual mesh reference broke: present in the Stage tree, invisible in
   the viewport. A stray Save while the names still collided had also
   baked a stray top-level `/panda_gripper_only` prim directly into
   `mefron.usd` (same pre-`MovePrim`-staging mechanism as the
   already-documented `/panda` gotcha), which kept re-poisoning every
   subsequent run even after the name collision itself was fixed, until
   that stray prim was also cleaned up (`clear_stray_robot_prims()`).
   **First fix attempt (renaming the tool's links) fixed the reported
   symptom but not the actual bug**: a follow-up direct inspection
   (`Franka.GetChildren()` before/after spawning the tool, prompted by the
   user reporting the exact same "Franka orange, invisible" symptom again
   after the rename) found `panda_link0`–`8` disappearing from
   `/World/Franka` entirely, replaced by the tool's own link names — the
   shared cache corrupts the whole link *structure*, not just visual
   references, so no naming scheme fixes it from the importing side.
   **Real fix**: stop live-importing the gripper tool into `mefron.usd`
   at all — `scripts/vendor_gripper_tool.py` pre-bakes it into a
   standalone asset inside its own fresh anonymous stage (never opened
   from `mefron.usd`), referenced the same way as the suction/screwdriver
   tools. The link-renaming code was reverted as unnecessary once nothing
   shares the import anymore.

Debugging method worth repeating: rather than guessing from the schema
docs, wrote throwaway diagnostic scripts (not committed) that opened the
stage, spawned the tool(s), and printed live world poses frame-by-frame
plus `Usd.PrimRange` walks tagging which prims actually carry
`RigidBodyAPI`/`ArticulationRootAPI`/`CollisionAPI` — each fix's root
cause became obvious from that output in a way it wasn't from reasoning
about USD Physics semantics alone.

## Screw pick-and-place — weld-to-live-pose bug (ATC branch)

`attach_screw_to_wrist()` and `weld_screw_into_hole()`
(`scripts/mefron_lib/robot.py`) both originally welded a screw at
whatever pose it happened to be holding *live* — the arm's own settled
pose, not a nominal computed one — at the moment of the weld. Both
silently baked cuRobo's usual residual approach error into the joint as
a *permanent* misplacement, and both were found and fixed the same day,
~1h16m apart, by the same technique.

**Pick side, fixed 2026-08-04 (`a439f30`).** `attach_screw_to_wrist()`
measured the live `panda_hand`→screw offset and froze it into the tip
joint: `localPos0` read `(0.00065, -0.0009, 0.2887)` / `localRot0
(0.166, 0.093, -0.038)` deg instead of the nominal `(0, 0, 0.2868543)`/
identity — leaving the screw ~2mm off the bit axis for the whole carry
and eventual placement. Visible in the GUI as the screw not lining up
with the bit; reproduced on both screwdriver assets, ruling out tool
geometry as the cause. Fix: compose the nominal carry pose from the
docked tool's live pose plus `SCREW_CARRY_LOCAL_*`, move the screw onto
it, *then* weld — so the correction is one clean re-authoring rather
than a joint-solver-driven jitter.

**Place side, fixed the same day (`f0a305c`), "the place-side twin of
`a439f30`."** `weld_screw_into_hole()` had the identical bug: it read
the carried screw's live world pose (wherever the arm ended up after
6's descend leg) and froze that against `main_holder`, so `SCREW_HOLES`
only ever drove the target the arm was *asked* to reach, never where
the screw actually landed. Measured by reparenting two placed screws
under `main_holder`: off their configured pockets by `(-2.88, +1.47,
+4.82)mm` and `(-2.81, -0.79, +4.85)mm`, plus a few tenths of a degree.
Fix: weld at `compute_screw_hole_pose(hole_index)` — `main_holder`'s
live pose composed with `SCREW_HOLES[hole_index]` and pushed
`SCREW_HOLE_INSERTION_DEPTH` along the hole's own +Z — instead of the
screw's live pose.

**Deliberate tradeoff, both sides, kept intentionally.** The fix
doesn't make the arm more accurate — it relocates cuRobo's residual
from "permanently baked into the final position" to "a one-time,
~2mm-scale snap onto the nominal pose at the instant of the weld." A
clean-looking placement (or a screw sitting cleanly on the bit) is
therefore no longer evidence the arm actually arrived — per `f0a305c`:
"the bit still separates from the screw by that residual at the
instant of release." If the underlying arm accuracy itself ever needs
fixing, that's convergent arrival in `run_teleop_loop()` (see
`docs/ee-arrival-accuracy.md`'s ~3mm ee-arrival-shortfall
investigation), not another change here.

## Needs verification

- **ATC numpad tool-changing in the real GUI.** Headlessly verified that
  `robot.dock_tool_to_wrist()`/`undock_tool_to_rack()` correctly swap the
  `FixedJoint` and that the tool's `female_coupler` frame converges to the
  wrist (or rack) once PhysX settles, but the full numpad-key → cuRobo-
  planned-approach → dock sequence (`teleop.ToolChangerControl` +
  `_build_tool_change_queue()`) has not been exercised with real keyboard
  input or a live motion plan — only the underlying joint mechanics were
  driven directly. Also: `config.TOOL_CHANGE_TARGETS`' rack dock/approach
  positions are rough placeholders, not yet hand-jogged into place.
- **`build_scene_mefron.py`'s grasp-physics fixes**
  (`apply_gripper_friction()`, `stiffen_gripper_drive()`) — headless
  read-back confirmed the friction material and drive values land
  exactly as configured, but whether the combination actually produces a
  firm, non-slipping, non-dangling grasp on `finger_print_scanner` has
  not been tested live in the GUI.
- **Keyboard gripper open/close (`C`/`O`) in `build_scene_mefron.py`** —
  the underlying joint-drive mechanism is headlessly verified (via the
  `GripperKeyboardControl.set_closed()` test hook), but a real
  interactive keypress in the GUI hasn't been tried, and there's no
  permanent headless regression test for plain open/close specifically
  (only the separate G/P snap-to-pose request path in `mefron.py` has
  one, via `test_mefron_assembly_headless.py`).
- **`main_holder`'s convex-decomposition collision tuning** (Shrink Wrap
  on, Voxel Resolution ~3-5M, Max Convex Hulls ~128, Error Percentage
  ~1-2 — see above for the full research) is a recommendation only —
  apply it via the GUI, confirm live that it fixes the sinking/stud-loss
  symptoms, and save `mefron.usd`.
- **`attach_objects_to_robot()`/`detach_object_from_robot()` wiring in
  `scripts/mefron.py`** — confirmed via cuRobo source that this is the
  right mechanism for making planning aware of a carried
  `finger_print_scanner`, and `franka.yml` already has a pre-built
  `attached_object` link ready for it, but it's not wired into the C/O
  gripper keybindings yet. Paused pending the user's own check of "how
  precisely this placement works" — confirm that's resolved before
  implementing, in case it changes the approach.
- **`_TELEOP_VELOCITY_SCALE`/`_TELEOP_ACCELERATION_SCALE = 0.2` and
  `GRIPPER_CLOSE_SPEED = 0.02`** (both in `scripts/mefron.py`) were
  applied in direct response to the user reporting fast-target motion
  disturbing a carried load and the gripper snapping shut too quickly —
  the mechanism for both is confirmed correct against cuRobo/PhysX
  source, but neither was independently re-confirmed live afterward
  against the *original* complaints specifically. Worth a quick live
  re-check that both actually feel right before assuming these constants
  are final.

## Conventions

- cuRobo robot config files (a custom `configs/curobo/*.yml`, not needed
  for `franka.yml` itself since that ships inside cuRobo) can't use
  repo-relative paths directly for `urdf_path`/`asset_root_path`/
  `collision_spheres` — cuRobo's own loader always resolves those against
  its *own* bundled install directories unless the caller patches them to
  absolute paths first, before calling
  `MotionGenConfig.load_from_robot_config()`.
- `ninja` isn't installed in this Isaac Sim/cuRobo environment by
  default, and `pip install` doesn't work here at all
  (`ModuleNotFoundError: No module named 'pip._vendor.packaging._structures'`)
  — cuRobo's CUDA kernels fall back to a JIT compile (needs `ninja`) when
  the prebuilt `.so` has a torch ABI mismatch, which happened on this
  install. Fixed by installing `ninja-build` via `apt-get` (not `pip
  install ninja`, since pip itself is broken here) — see
  `docs/docker-and-devcontainer.md` for the full fix. Confirmed live this
  lets a headless cuRobo warmup JIT-compile all five of cuRobo's CUDA
  kernels cleanly and reach `curobo motion_gen: READY`, where it
  previously crashed with `undefined symbol:
  _ZN3c104cuda29c10_cuda_check_implementationEiPKcS2_ib` (the torch ABI
  mismatch) immediately followed by `RuntimeError: Ninja is required`.
- cuRobo's `MotionGen` (kinematics/IK/trajopt, `compute_kinematics()`,
  `plan_single()`) operates entirely in the **robot's own base-link
  frame**, never USD world space — any USD world pose (e.g. a dragged
  teleop target) must be transformed into that frame first via
  `robot_base_pose.compute_local_pose(world_pose)` (both
  `curobo.types.math.Pose` objects), where `robot_base_pose` comes from
  wherever the robot was actually mounted (`cr5_mount.position`/
  `orientation_wxyz`), not assumed to be the origin.
- `isaacsim.core.prims.SingleArticulation.initialize()` (and anything
  else that needs a PhysX simulation view) silently does nothing useful
  without an actual `PhysicsScene` prim on the stage — nothing in this
  repo's robot-import path creates one automatically.
  `isaacsim.core.api.World()` would create one automatically, but this
  repo deliberately avoids `World` for scripts that don't otherwise need
  it (see `run_teleop_loop()`'s own module comment) — where physics *is*
  needed, define one explicitly and minimally:
  `UsdPhysics.Scene.Define(stage, "/physicsScene")`.
- Calling `timeline.play()` before physics has a real chance to settle
  corrupts PhysX's tensor simulationView — confirmed live in two distinct
  ways (see `scripts/mefron.py` above): playing before `/physicsScene`
  even exists on the stage, and playing before a long blocking call
  (cuRobo's `motion_gen.warmup()`, ~30s, which calls no
  `simulation_app.update()` of its own) that leaves physics "playing"
  across an unpumped real-time gap. Both produce the identical downstream
  symptom: a later `SingleArticulation(...)` construction crashes with
  `AttributeError: 'NoneType' object has no attribute 'link_names'`. Any
  script driving `timeline.play()` itself (rather than leaving it to a
  human clicking Play in the GUI, this repo's usual pattern) needs to do
  so only *after* both the physics scene exists and any blocking warmup
  work is done.
- A `SingleArticulation` object is only valid for the specific PhysX
  simulation view that existed when it was constructed — clicking
  **Stop** in the GUI tears that view down entirely, and reusing a
  `SingleArticulation` built before the Stop after a later Play leaves it
  permanently broken (`get_joints_state()` never returns non-`None`
  again). Any interactive loop that only builds its `SingleArticulation`
  once (gated by e.g. `idx_list is None`, checked just on the first Play)
  needs to instead track not-playing→playing *transitions* and rebuild it
  (plus reset any other per-session state) on every fresh Play, not just
  the first — see `scripts/mefron.py`'s and
  `scripts/build_scene_mefron.py`'s `run_teleop_loop()` for the pattern.
- Isaac Sim's URDF importer behaves differently depending on whether the
  target stage's root layer is a real, file-backed USD file
  (`omni.usd.get_context().open_stage()`) or anonymous/in-memory (the
  default for a fresh `SimulationApp`, or content brought in via
  `add_reference_to_stage()` into such a stage). Confirmed live: only the
  file-backed case writes a disk-persisted, multi-layer "Robot
  Description" structure (a `configuration/` folder of sublayer `.usd`
  files) as a side effect of import, with no save prompt — and that
  extra layering breaks `CopyPrim`-based prim duplication (a shallow,
  spec-level copy that can't correctly re-resolve a same-layer reference
  once relocated across the resulting more complex layer stack). Prefer
  building scenes the way `scripts/build_scene_mefron.py` does — a fresh
  anonymous stage with external content brought in via
  `add_reference_to_stage()` — over opening an existing authored `.usd`
  file directly, when the script needs to import a robot into it; this
  sidesteps the whole class of bug rather than working around it (an
  internal USD reference, `prim.GetReferences().AddInternalReference()`,
  is the workaround if opening the file directly is unavoidable — see
  `scripts/mefron.py`'s `build_teleop_target()`).
- `SimulationApp`'s default experience (`isaacsim.exp.base.python.kit`)
  is missing most UI extensions, including the Physics debug-
  visualization menu (needed to view collision meshes in the viewport).
  Pass `experience=f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'`
  (the same experience `isaac-sim.sh` itself launches) to get the full
  menu bar for interactive runs — see `scripts/mefron.py`'s/
  `scripts/build_scene_mefron.py`'s `SimulationApp(...)` construction.
  **Gotcha, confirmed live**: doing this breaks cuRobo's own `from
  packaging import version` — see `scripts/build_scene_mefron.py`'s own
  entry above for the full root-cause investigation and fix.
