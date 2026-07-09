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

## Needs verification

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
