# CLAUDE.md

Project-specific context for **isaac-cobot**. Current state and the traps
that bite immediately; full history and "why" narratives live in `docs/`.

## Code style

- Comments and docstrings: **2 lines hard max.** Longer rationale belongs in
  `docs/` (usually `docs/mefron-history.md`), with a short pointer in its place.

## What this repo is

An NVIDIA Isaac Sim project using cuRobo for collision-aware motion planning.
**There is no physical robot hardware** — everything targets Isaac Sim only;
treat all sim behavior as illustrative, not validated against real hardware.

The active work is a scanner-assembly pick-and-place task built on
`assets/mefron/` — a hand-authored scene (`mefron.usd`: factory floor, packing
tables, and a scanner-assembly CAD mockup — `finger_print_scanner`,
`main_holder`, `screen`, `backpanel_support`). A Franka Panda (cuRobo's own
bundled config) is mounted into it and driven by an interactive cuRobo
drag-teleop loop (`motion_gen.plan_single()`). Grasp and assembly poses are
derived by jogging the robot to a good pose in the GUI and reading back the
relative transform, not measured on real hardware.

**This branch (`atc`)** replaces the old 3-separate-Frankas cell with **one
Franka fitted with an automatic tool changer**: a permanent male coupler on the
wrist, and gripper/suction/screwdriver as detachable modules parked in their own
rack until a tool-change key docks one. Full design: `docs/tool-changer.md`.

**This branch (`atc-laptop`)** is `atc` plus the 2026-08-08 cleanup, with the
tool-changer keys moved off the numpad to **Y/U/I** so it can be driven from a
laptop keyboard. That key remap is the only behavioral difference from `atc`.

**This branch (`atc-fairino`)** branches from `atc` @ `6374624` and is migrating
the cell off the Franka stand-in onto the real hardware — a **FAIRINO FR5** arm
(`robots/fr5/`) with a **DH Robotics PGC-140** gripper — in three GUI-verified
steps. **Steps 1 (the FR5 arm) and 2 (cuRobo) have landed.** cuRobo now loads
`configs/curobo/fr5.xrdf` — a cuMotion XRDF from the Lula editor, converted at
load time, *not* a `.yml` — and the full pipeline runs with teleop planning and
following (headless; live GUI still to confirm). **Step 3, the gripper, has
not:** the dockable gripper tool is still the Franka hand, so `Y` docks it with
placeholder offsets, and the grasp/screw/tool-changer harnesses still fail. Every
hand-jogged pose the swap invalidates is marked `STALE`/`UNVERIFIED` in
`config.py` rather than converted. Sequence, prior art on the `dobot` branch, and
the re-derivation checklist: `docs/fr5-migration.md`.

## Where things live

- **This file** — current state, and gotchas that break something if unknown.
- `docs/mefron-history.md` — full chronological bug/fix log, plus the
  cuRobo/PhysX/URDF-importer conventions this file only summarizes.
- `docs/grasp-and-assembly-offsets.md` — how the grasp/assembly constants were
  derived, the release weld, and the open grasp-centering problem.
- `docs/tool-changer.md` — the ATC's design, alternatives considered, screws,
  and open issues.
- `docs/part-feeders.md` — the per-part feeder belts, their photo-eyes, and the
  base-path→live-copy resolver every pick now goes through.
- `docs/ee-arrival-accuracy.md` — why the ee settles ~3mm short (measured,
  cuRobo ruled out), the live probes, and the verified offline FK chain.
- `docs/fr5-migration.md` — this branch's three-step FR5 + PGC-140 migration,
  what step 1 landed, and the re-derivation checklist. **Read before touching
  anything arm- or gripper-shaped.**
- `docs/docker-and-devcontainer.md` — environment setup (generic infra).
- `examples/curobo_reference/` — pristine copy of cuRobo's own teleop demo.
  **Do not modify those two files**; write a separate script instead.
- `robots/accessories/` — the dockable tools' CAD. Only the
  `*_with_tool_female.usd` pair is referenced; the rest are unreferenced.
- `robots/fr5/` — the vendored FAIRINO FR5 (`urdf/fairino5_v6.urdf` + STL
  meshes). Provenance, the `FR5WM` rejection, and the **no upstream license**
  finding: `robots/fr5/SOURCE.md`.
- `scripts/` — `mefron.py` (the only entry point) plus six
  `test_mefron_*_headless.py` regression harnesses. Everything else was deleted
  2026-08-08; see `docs/mefron-history.md` for what and why.
- `scripts/mefron_lib/` — `config.py` (all constants), `kit_bootstrap.py` /
  `kit_experience.py` (Kit startup order), `usd_util.py` (URDF import,
  references, fixed joints, collision toggles), `grasp.py` (pose math),
  `robot.py` (the arm itself), `toolchanger.py` (the ATC), `assembly.py` (the
  O/L release weld), `screws.py` (screw pick/place), `keyboard.py` (the five
  control objects), `motion.py` (cuRobo setup + waypoint queues), `teleop.py`
  (the per-frame loop), `conveyor.py`, `feeder.py` (the per-part feeder belts
  and the base-path→live-copy resolver).

## Active script + current state

`scripts/mefron.py` opens `mefron.usd` directly via `open_stage()`, mounts the
vendored **FR5** on the `ur10_mount` pedestal (`robot.mount_arm()`, at
`/World/FR5`), fits the ATC's male coupler, spawns and parks the three dockable
tools, and runs the drag-follow teleop loop.

**Mid-migration.** The arm and cuRobo are FR5; the **gripper tool is still the
Franka hand**, so everything below about C/O, grasp yamls and gripper docking is
what step 3 has to port. The FR5 ships a bare ISO flange, so
`remove_parallel_jaw_gripper()`/`hide_hand_housing()` have nothing to strip and
are no longer called, and the ATC's male coupler rides `wrist3_link` instead of
`panda_hand`. `--arm-only` stops right after the arm mount and hands the GUI over
(no ATC/cuRobo/teleop) — it also un-instances the link meshes so the Lula editor
can fit spheres. See `docs/fr5-migration.md`.

| Key | Action |
|---|---|
| `Y` / `U` / `I` | dock gripper / suction / screwdriver (`TOOL_CHANGE_TARGETS`) |
| G / J / B / K | gripper: approach a grasp (`GRASP_TARGETS`) |
| C / O | gripper: close / open — **O welds** (see below) |
| N / M | suction: approach (`SUCTION_TARGETS`) |
| V / L | suction: attach / release — **L welds** |
| 5 / 6 | screwdriver: pick the presented screw / place it in the next hole |
| P | place whatever was last grasped or approached |
| 1 (number row) | conveyor forward, press again for back |
| 2 (number row) | part feeders: advance now, skipping the 5s wait |

- **`main_holder` into the jig (`G`).** The base part is pickable too, and its
  `ASSEMBLY_RELATIONSHIPS` entry is the only one whose mount is
  **`main_holder_jig`**, not `main_holder` — so `G`/C/P/O seats it on the jig and
  the conveyor then carries the whole assembly. The seat offset was given as
  `z = -24` in the jig's own **mm-scale** frame → `-0.024` m here, and the jig is
  flipped 180° about Y, so that is 24mm *up* in world. Its relative orientation is
  **180° about Z** (`[0,0,0,1]`), not identity — that cancels the jig-vs-holder
  frame difference, so the holder seats facing the way it parks on the table.
  Everything else assembles in `main_holder`'s own frame off its *live* pose, so
  nothing needed re-deriving.
- **Screws.** 5 picks the screw on `/World/screw_presenter`; 6 carries it to the
  next of `SCREW_HOLES`' nine clearance holes in **`main_holder_back_cover`**
  and pops the next screw in. **No driving rotation** — deliberately out of
  scope. A screw is always joint-fixed to something (presenter → wrist → hole),
  never free-falling. Both welds use the *nominal* pose, not where the arm
  settled, so a clean placement is **not** evidence the arm arrived. Placement
  reads the cover's *live* pose, so doing it before the cover is assembled seats
  screws wherever it's parked (warned, not refused). See `docs/tool-changer.md`.
- **Part feeders (`ConveyorBelt_A06_02…06`).** Each sub-part has its own 1m belt
  queueing **GUI-placed copies** named `<base>`, `<base>_01`, … (Ctrl+D's own
  naming; move copies along **−Y only**, keeping X/Z, or the photo-eye's ray line
  misses them). 5s after a part leaves the pick spot, that belt runs until its
  light-beam photo-eye says the next copy has arrived, then ramps to a stop —
  measured 6.8–9.1mm from the authored pick spot on all five belts, headless.
  Nothing keys off the arm: the countdown starts when the **beam clears**, so the
  belt also won't run while the tool is standing in the station. A24 and its `1`
  key are untouched. Because a pick can land on any copy, every live pose read
  goes through `feeder.resolve_for_pick()` / `resolve_assembled()` — the copy at
  the station versus the copy already assembled, which are deliberately different
  answers (screws must go into the fitted cover while `K` moves to the next one).
  A grasp key can no longer reach an already-assembled copy. `FEEDER_SPEED` is
  **belt-local, not m/s** (these belts carry a 0.5 scale). Full design, and the
  two PhysX traps that make or break it: `docs/part-feeders.md`.
- **Gripper widths.** A grasp key stages that object's yaml-specified widths and
  opens to pregrasp width, so C/O ramp toward whichever object was last grasped.
  C/O writes the **docked tool's own** `panda_finger_joint1/2` DriveAPI every
  frame — those joints aren't in the arm's articulation at all.
- **Releasing welds the part.** Within `ASSEMBLY_WELD_MAX_DISTANCE` of its
  nominal pose, O/L snaps the part exactly onto that pose and joint-fixes it
  there; past that it's an ordinary release. A grasp key un-welds it first. The
  weld also turns the part's collision **off** by default, or the snap's
  interpenetration wakes later as a violent shake — cost: a part placed later
  passes through an already-welded one until its own weld fires.
  `ASSEMBLY_WELD_KEEP_COLLISION_PART_PRIM_PATHS` opts a part out; currently
  `main_holder_back_cover` alone. Confirmed non-shaking live 2026-08-07, but it
  *is* the configuration the default guards against — **re-check it after any
  change to `main_holder`'s (still untuned) collider.** Full design:
  `docs/grasp-and-assembly-offsets.md`.

Constants all live in `scripts/mefron_lib/config.py`. The ones that surprise:
`OBSTACLE_PRIM_PATHS` is still on its **debug value** (`ConveyorBelt_A06_01`
only — normally `main_holder_jig` + `tool_rack_gripper`, see open issues);
`SCREW_HOLE_INSERTION_DEPTH` is `0.00`, so a placed screw sits at the hole
mouth; `GRIPPER_OPEN_POSITION`/`CLOSED_POSITION` are only pre-grasp defaults;
`female_coupler_local_*` and `SURFACE_GRIPPER_LOCAL_POSITION` are unmeasured
placeholders; `FEEDER_SPEED` is belt-local, so it is half its value in m/s. All
three tools are hand-placed and baked into `mefron.usd`, so dock poses are read
off their live poses each run — move a tool in the GUI, no code change needed. The
feeders follow the same principle: their belts' photo-eye geometry, pick spots and
part queues are all derived from live poses at startup, so adding a copy or moving
a belt in the GUI needs no code change either.

## Currently open issues

Full investigation detail: `docs/mefron-history.md`.

- **`main_holder_on_main_holder_jig`'s ride check fails in the harness,
  unverified live.** `test_mefron_assembly_weld_headless.py
  --relationship=main_holder_on_main_holder_jig` welds cleanly (0.005m
  correction, 0.000m drift under gravity) but the welded holder does not follow a
  nudged jig — off by exactly the 0.15m nudge. Suspected harness artifact: it
  calls `sync_assembly_anchors()` **once** after a PhysX-side teleport and then
  simulates 90 frames, whereas `run_teleop_loop()` calls it every frame. Not
  compared against the baseline relationship, and not yet checked in the GUI —
  confirm on the conveyor before treating it as either a bug or a non-issue.
- **Part feeders: confirmed live in the GUI 2026-08-10.** All five belts also pass
  `test_mefron_feeder_headless.py`. The measurements in `docs/part-feeders.md`
  (6.8–9.1mm landing accuracy, the 0.5 belt-scale factor, the beam windows) are
  still headless numbers — the GUI confirmed the behaviour, not those figures.
  Which sub-cases were exercised on screen isn't recorded: the arm picking a
  *copy*, a screw going into an assembled copy, and whether a part tips on a full
  drain are worth re-checking before relying on them.
- **Two feeder belts stop on the pose fail-safe, not the photo-eye.**
  `backpanel_support` and `finger_print_scanner` have a notch at the ray line, so
  the depth test can miss them and `PartFeeder._overshot_station()` stops the belt
  instead, logging a WARNING every advance. Landing accuracy is unaffected (7-9mm).
  Widening `FEEDER_BEAM_ARRIVAL_EPSILON` from 0.003 to ~0.008 should let the beam
  confirm; **untested**, and it loosens the stop for the other three belts too.
- **Grasp-centering**: `finger_print_scanner` isn't equidistant from both
  fingertips at grasp time, so one finger contacts first and shifts the part
  sideways. Not a joint/drive asymmetry (ruled out).
- **Assembly placement (P) doesn't land cleanly — deliberately masked, not
  fixed.** The release weld snaps the part onto its nominal pose regardless of
  where the arm left it; accepted for this scene (visual pipeline, no VLA
  training). A lift/rotate/descend redesign was tried and reverted after finding
  `ASSEMBLY_LIFT_HEIGHT` is a fixed world-Z constant unrelated to where things
  actually are. If revisited: make lift clearance relative, not absolute.
- **The ee settles ~3mm short of `/World/target`,** along the approach axis.
  cuRobo is exact (`FK(commanded joints)` hits to 0.00mm/0.001°) — it's all
  joint tracking lag. `_STATIC_JOINT_VELOCITY_THRESHOLD` (0.5 rad/s) is 5x
  looser than the residual velocity at "arrival", so every `on_arrival` side
  effect fires early. See `docs/ee-arrival-accuracy.md`.
- **Conveyor line has no collision awareness.** Adding the
  `ConveyorBelt_*`/`container_h20*` prims to `OBSTACLE_PRIM_PATHS` hung cuRobo's
  mesh-collision-world construction for over an hour. Needs cuboid
  approximations or narrower sub-prim selection, not raw CAD meshes.
- **ATC: cuRobo sees neither the docked tool nor a carried screw.** The docked
  gripper tool carries no collision at all by construction, so it can't collide
  with the part it grips either. See `docs/tool-changer.md`'s open issues.
- **Screws: reach is tight.** The 27.5cm screwdriver puts the worst hover 0.809m
  from the mount against the Panda's ~0.855m envelope, so
  `SCREW_APPROACH_CLEARANCE` is 0.02 (not the rack's 0.15). An unreachable hole
  is a scene-layout fix, not a code one.
- `attach_objects_to_robot()`/`detach_object_from_robot()` (carried-object
  collision awareness) isn't wired to C/O yet — `franka.yml` already has a spare
  `attached_object` link ready.
- `main_holder`'s convex-decomposition collision tuning (fixes sinking + lost
  mounting studs) is researched but not applied/saved to `mefron.usd`.

## Must-know gotchas

Full root-cause detail: `docs/mefron-history.md` unless noted otherwise.

- **The URDF importer's drive arguments never reach the joints.** `import_urdf`'s
  `default_drive_strength`/`default_position_drive_damping` are silently ignored —
  the FR5 imported at stiffness 625 / **damping 0**, straight from its URDF's
  `<dynamics damping="0"/>`. Undamped drives ring: 6.04x velocity overshoot with
  0.03 rad position error, i.e. visible teleop jerk. Fixed in the vendored URDF
  (`damping="10.0"`, as cuRobo's own Franka declares — which is why the Franka
  never showed it); damping is the whole story, stiffness changes nothing.
  **Read drive gains back after any import.** `docs/fr5-migration.md`.
- **The FR5's all-zero joint config is a singularity.** It is also fully
  outstretched (wrist3 0.82m out, 0.05m up) — measured Jacobian condition number
  `inf`, so IK fails for essentially any target. `robot.apply_home_pose()` stages
  `FR5_HOME_JOINT_POSITIONS` (cond 8.2) instead, and `fr5.yml`'s
  `retract_config` must be seeded from it too. `docs/fr5-migration.md`.
- **`UsdPhysics` angular quantities are degrees**, while the URDF, cuRobo and
  this repo's own constants are radians. `apply_home_pose()` converts.
- **`PhysicsScene` required.** `SingleArticulation.initialize()` silently breaks
  without one — `UsdPhysics.Scene.Define(stage, "/physicsScene")`.
- **`timeline.play()` timing.** Calling it before `/physicsScene` exists or
  before `motion_gen.warmup()` finishes corrupts PhysX's tensor simulationView.
  Only drive it yourself after both are done.
- **Stop/Play rebuild.** A `SingleArticulation` is only valid for the PhysX view
  that existed when built. Any interactive loop must rebuild
  `robot`/`idx_list`/`articulation_controller` on every fresh Play.
- **Authoring a joint prim live mid-Play stales those same handles** —
  `apply_action()` keeps succeeding while the arm stops responding. Every live
  joint edit goes through `teleop._invalidate_articulation_handles()`.
- **cuRobo plans in the robot's base-link frame, never world space.** Any USD
  world pose must go through `robot_base_pose.compute_local_pose(...)` first.
- **URDF importer + file-backed stages.** Importing into a stage opened from a
  `.usd` file makes the importer write a disk-persisted "Robot Description",
  breaking `CopyPrim` duplication (use `AddInternalReference()` instead).
- **That side effect also rewrites `mefron.usd` itself on every run**, growing it
  — unconditional, not preventable via `stage.GetSessionLayer()`. Treat a diff
  on `mefron.usd`/`configuration/*.usd` after a run as expected noise; never
  blindly `git checkout` it (it may carry real hand-placed edits); prefer a
  scratch copy for headless testing.
- **Accumulated stray robot prims break rendering, not physics.** Fixed by
  `robot.clear_stray_robot_prims()`, called right after `open_stage()` every run
  (in-memory only, so it must keep running).
- **`SimulationApp` full experience breaks cuRobo's `packaging` import.**
  Pre-load it from real `site-packages` first — `kit_bootstrap.py`.
- **`ninja`/`pip` are broken here.** Fixed via `apt-get install ninja-build` —
  see `docs/docker-and-devcontainer.md`.
- **`DeletePrims` silently no-ops on an articulation-internal joint prim.**
  Confirmed for the Franka's finger joints and for a URDF-synthesized
  `root_joint` — `SetActive(False)` is what actually removes the constraint.
- **A `UsdPhysics.FixedJoint`'s target must resolve to a real `RigidBodyAPI`
  prim**, not just any descendant — an organizing Xform over an articulation's
  links, or a CAD asset with no baked-in body, silently fails to be pulled. See
  `docs/tool-changer.md`'s gotchas 3–4.
- **A jointed body's own enabled collision can fight the joint.** Overlapping
  colliders at both ends reach a contact-separation equilibrium short of the
  joint's target instead of converging. `docs/tool-changer.md`'s gotcha 2.
- **Redefining a `Joint` prim in place leaves PhysX solving against the stale
  `body1`,** even though USD reads correct. Use a fresh path per target.
- **Live-importing two robots into the same file-backed stage isn't safe,** no
  matter how they're named — the shared "Robot Description" cache corrupted the
  first robot's *link structure*. Pre-bake the second as a standalone asset and
  reference it. `docs/tool-changer.md`'s gotcha 6.
- **A "held" object that hangs in mid-air may just be a sleeping PhysX body.**
  One velocity write drops it. This misdiagnosed both the suction-release and
  "welded part follows the wrist" bugs — check for it before theorizing.
- **A resting part is asleep, and turning a KINEMATIC belt's surface velocity on
  does not wake it** — it just sits there while the belt runs under it. Same class
  as the gotcha above. `feeder._wake_bodies()` calls PhysX's own `wake_up()` on
  the belt's queue every frame while it feeds. `docs/part-feeders.md`.
- **`PhysxSurfaceVelocityAPI` must be applied BEFORE Play.** The `IsaacConveyor`
  node applies it itself when it first computes, but PhysX may never resync a body
  it already created — USD reads the right velocity while the belt drives nothing.
  Nondeterministic across runs and per belt within one run, which makes it look
  like anything but this. `feeder.prime_surface_velocity()`; `docs/part-feeders.md`.
- **`SingleRigidPrim.set_world_pose()` writes the PhysX/Fabric pose, not USD.**
  For a *kinematic* body nothing writes it back, so a `SingleXFormPrim` read (and
  anything built on one, like `feeder.belt_queue()`) still sees the old pose. A
  dynamic body's simulated transform does get written back.

## Pinned versions

Isaac Sim `5.1.0`, cuRobo commit `ebb71702f3f70e767f40fd8e050674af0288abe8`,
torch `2.11.0+cu128` (CUDA 12.8). Dev GPU: RTX PRO 4000 Blackwell (sm_120) —
`TORCH_CUDA_ARCH_LIST` must be `12.0+PTX` for this GPU.
