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

## Where things live

- **This file** — current state, and gotchas that break something if unknown.
- `docs/mefron-history.md` — full chronological bug/fix log, plus the
  cuRobo/PhysX/URDF-importer conventions this file only summarizes.
- `docs/grasp-and-assembly-offsets.md` — how the grasp/assembly constants were
  derived, the release weld, and the open grasp-centering problem.
- `docs/tool-changer.md` — the ATC's design, alternatives considered, screws,
  and open issues.
- `docs/ee-arrival-accuracy.md` — why the ee settles ~3mm short (measured,
  cuRobo ruled out), the live probes, and the verified offline FK chain.
- `docs/docker-and-devcontainer.md` — environment setup (generic infra).
- `examples/curobo_reference/` — pristine copy of cuRobo's own teleop demo.
  **Do not modify those two files**; write a separate script instead.
- `robots/accessories/` — the dockable tools' CAD. Only the
  `*_with_tool_female.usd` pair is referenced; the rest are unreferenced.
- `scripts/` — `mefron.py` (the only entry point) plus five
  `test_mefron_*_headless.py` regression harnesses. Everything else was deleted
  2026-08-08; see `docs/mefron-history.md` for what and why.
- `scripts/mefron_lib/` — `config.py` (all constants), `kit_bootstrap.py` /
  `kit_experience.py` (Kit startup order), `usd_util.py` (URDF import,
  references, fixed joints, collision toggles), `grasp.py` (pose math),
  `robot.py` (the arm itself), `toolchanger.py` (the ATC), `assembly.py` (the
  O/L release weld), `screws.py` (screw pick/place), `keyboard.py` (the five
  control objects), `motion.py` (cuRobo setup + waypoint queues), `teleop.py`
  (the per-frame loop), `conveyor.py`.

## Active script + current state

`scripts/mefron.py` opens `mefron.usd` directly via `open_stage()`, mounts
cuRobo's bundled Franka on the `ur10_mount` pedestal, strips that arm's *own*
hand (`remove_parallel_jaw_gripper()` + `hide_hand_housing()`, so `panda_hand`
terminates the wrist cleanly), fits the ATC's male coupler, spawns and parks the
three dockable tools, and runs the drag-follow teleop loop.

| Key | Action |
|---|---|
| `Y` / `U` / `I` | dock gripper / suction / screwdriver (`TOOL_CHANGE_TARGETS`) |
| J / B / K | gripper: approach a grasp (`GRASP_TARGETS`) |
| C / O | gripper: close / open — **O welds** (see below) |
| N / M | suction: approach (`SUCTION_TARGETS`) |
| V / L | suction: attach / release — **L welds** |
| 5 / 6 | screwdriver: pick the presented screw / place it in the next hole |
| P | place whatever was last grasped or approached |
| 1 (number row) | conveyor forward, press again for back |

- **Screws.** 5 picks the screw on `/World/screw_presenter`; 6 carries it to the
  next of `SCREW_HOLES`' nine clearance holes in **`main_holder_back_cover`**
  and pops the next screw in. **No driving rotation** — deliberately out of
  scope. A screw is always joint-fixed to something (presenter → wrist → hole),
  never free-falling. Both welds use the *nominal* pose, not where the arm
  settled, so a clean placement is **not** evidence the arm arrived. Placement
  reads the cover's *live* pose, so doing it before the cover is assembled seats
  screws wherever it's parked (warned, not refused). See `docs/tool-changer.md`.
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
placeholders. All three tools are hand-placed and baked into `mefron.usd`, so
dock poses are read off their live poses each run — move a tool in the GUI, no
code change needed.

## Currently open issues

Full investigation detail: `docs/mefron-history.md`.

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

## Pinned versions

Isaac Sim `5.1.0`, cuRobo commit `ebb71702f3f70e767f40fd8e050674af0288abe8`,
torch `2.11.0+cu128` (CUDA 12.8). Dev GPU: RTX PRO 4000 Blackwell (sm_120) —
`TORCH_CUDA_ARCH_LIST` must be `12.0+PTX` for this GPU.
