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

**This branch (`atc`)** replaces the old 3-separate-Frankas cell (one
arm each for gripper/suction/screwdriver) with **one Franka fitted with
an automatic tool changer**: a permanent male coupler on the wrist, and
the three tools as detachable modules parked in their own rack until a
numpad key docks one. See `docs/tool-changer.md` for the full design,
alternatives considered, and open issues.

## Where things live

- **This file** — current state, and gotchas that will immediately break
  something if you don't know them.
- `docs/mefron-history.md` — full chronological bug/fix log for every
  mefron script, plus the cuRobo/PhysX/URDF-importer Conventions this file
  only summarizes, plus full detail on the open issues below.
- `docs/grasp-and-assembly-offsets.md` — how the grasp/assembly relative-
  pose constants were derived, plus the open grasp-centering problem.
- `docs/tool-changer.md` — the ATC's design, alternatives considered
  (Robot Assembler, USD variants, `SurfaceGripper`), and open issues.
- `docs/ee-arrival-accuracy.md` — why the ee settles ~3mm short of the target
  (measured, cuRobo ruled out), the live Script Editor probes that measured it,
  and the candidate fixes. Also holds the verified offline FK chain.
- `docs/docker-and-devcontainer.md` — Docker/devcontainer environment setup
  (generic infra, not scene-specific).
- `examples/curobo_reference/` — pristine, unmodified copy of cuRobo's own
  interactive teleop demo. **Do not modify these two files**; write a
  separate script instead (`scripts/mefron.py` is exactly that).
- `robots/accessories/` — the dockable tools' CAD. Only the
  `*_with_tool_female.usd` pair is referenced (female coupler modeled onto
  the body); the plain `suction gripper.usd`/`electric_screwdriver.usd` and
  the newer `Delta inline screwdriver`/`delta_screwdriver_with_female_tool_head`
  are on disk but unreferenced.
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
`assets/mefron/`'s `ur10_mount` pedestal, strips that arm's *own* hand
(`remove_parallel_jaw_gripper()` + `hide_hand_housing()`, so `panda_hand`
terminates the wrist cleanly), fits it with the ATC's male coupler
(`robot.attach_tool_changer_male_coupler()`), spawns and parks the 3
dockable tools (`enable_gripper_tool_fingers()` right after), and runs a
drag-follow teleop loop. Numpad 1/2/3 (see `config.TOOL_CHANGE_TARGETS`)
sends the arm to dock/undock the gripper/suction/screwdriver tool at its own
rack (`docs/tool-changer.md`).
Once the matching tool is docked: J/B/K (via `config.GRASP_TARGETS`, NVIDIA
Grasp Editor-exported poses) and C/O for the gripper, N/M (via
`config.SUCTION_TARGETS`) and V/L for the suction cup, 5/6 for the
screwdriver; P places whichever was last grasped/approached either way.
Opens `mefron.usd` directly via `open_stage()`.

**Screws (screwdriver tool):** 5 picks the screw waiting on the presenter
(`/World/screw_presenter`), 6 carries it to the next of
`config.SCREW_HOLES`' ten pockets on `main_holder` and leaves it there, then
pops the next screw in at the presenter. **No screw-driving rotation** —
deliberately out of scope. A screw is always joint-fixed to something
(presenter → wrist → hole), never free-falling, mirroring
`park_tool_at_rack()`'s invariant for tools. The bit-tip offset and all ten
hole poses are CAD-derived, not hand-jogged — see `docs/tool-changer.md`.
Both welds use the *nominal* pose, not where the arm settled: a picked screw
goes on the bit axis, a placed screw into `compute_screw_hole_pose(hole)`.
Deliberate tradeoff — a screw ends up where a real pocket would constrain
it, so a clean-looking placement is **no longer evidence the arm arrived**
(the bit visibly separates from the screw by cuRobo's residual on release).

Pressing a grasp key also stages that object's yaml-specified finger widths
onto `GripperKeyboardControl` and opens the gripper to pregrasp width — C/O
ramp toward whichever object was grasped last, not a fixed global width.
C/O drives the **docked gripper tool's own** `panda_finger_joint1/2`
DriveAPI directly (`robot.set_gripper_tool_finger_target()`), every frame,
no `SingleArticulation` involved — those joints aren't part of the arm's
articulation at all.
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
- `_TELEOP_VELOCITY_SCALE = 0.6`, `_TELEOP_ACCELERATION_SCALE = 0.1`,
  `GRIPPER_CLOSE_SPEED = 0.02` m/s, `GRIPPER_DRIVE_STIFFNESS = 10000.0`.
  `GRIPPER_OPEN_POSITION`/`GRIPPER_CLOSED_POSITION` are only the *default*
  widths before any grasp key is pressed — each grasp key overrides them.
  `FRANKA_DRIVE_DAMPING = 210.0` is ~4x the bare-wrist value, for the rigid
  tool now bolted on (see gotchas).
- `OBSTACLE_PRIM_PATHS`: `main_holder_jig` + `tool_rack_gripper`.
  Deliberately excludes the conveyor/container prims — see open issues.
- `SCREW_HOLES`: the ten real mounting pockets, hand-measured off
  `main_holder`'s CAD, in metres in its scale-free frame. A **list**, not a
  name-keyed dict — order is the fill sequence 5/6 walks. Entries 2/3/7/8 keep
  the tighter values from its `tn__CutExtrude51..54` colliders; entries 1 and 4
  are marked `CHECK` in place (they land 1.0mm apart, so one is a misread).
  `SCREW_HOLE_INSERTION_DEPTH` is currently `0.00` — a placed screw sits at the
  pocket mouth, not down it. `SCREWDRIVER_TIP_LOCAL_POSITION` is likewise
  mesh-derived (274.854mm along the docked tool's local +Z).
  `/World/screw_presenter` is a real CAD asset baked into `mefron.usd`, so its
  live pose wins and `SCREW_PRESENTER_FALLBACK_*` is unused on this scene;
  `SCREW_PRESENTER_SEAT_LOCAL_POSITION` = `(6.66, -86.00, 72.00)`mm is where that
  presenter holds the screw, since the prim origin is its base plate, with a
  180°-about-X seat orientation so the head faces up.
- `TOOL_CHANGE_TARGETS`: dict keyed by tool name (`gripper`/`suction`/
  `screwdriver`), each holding its numpad `key`, `baked_tool_prim_path`,
  `rack_prim_path`, and `female_coupler_local_*`. **All 3 tools are now
  hand-placed and baked into `mefron.usd`** (the gripper switched over last,
  after a live-referenced `GRIPPER_TOOL_VISUAL_ONLY_USD` kept landing a gapped
  dock) — so there are no `dock_position` constants any more: dock poses are
  read back off the baked prims' live poses each run, and `rack_prim_path` is
  a lightweight non-physics anchor Xform re-synced to wherever the baked tool
  currently sits. Move a tool in the GUI, no code changes. Only
  `female_coupler_local_*` are still unmeasured placeholders.
  `TOOL_RACK_PRIM_PATH` (`/World/tool_rack`) is likewise the real baked rack.

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
- **The suction cup's release (L key) doesn't actually let go.** The real
  `SurfaceGripperManager` processes attach/detach as queued PhysX/USD
  actions on its own `onPhysicsStep`, so scripting the joint-enabled
  toggle directly from Python races its internal state (three variants
  tried, all reverted). Manual Stage-panel workaround still required.
- **The ee settles ~3mm short of `/World/target`, along the approach axis.**
  Measured live: cuRobo is exact (`FK(commanded joints)` hits the target to
  0.00mm/0.001°) — the whole error is joint tracking lag, ~0.18° on joints
  2/3/6. `_STATIC_JOINT_VELOCITY_THRESHOLD` (0.5 rad/s) is 5x looser than the
  residual velocity still present at "arrival", so every `on_arrival` side
  effect (dock, undock, screw weld) fires early. Full numbers, the live probe
  script, and the fix options: `docs/ee-arrival-accuracy.md`.
- **ATC: cuRobo has no collision awareness of whichever tool is currently
  docked** (nor of a carried screw), and `female_coupler_local_*` plus
  `SURFACE_GRIPPER_LOCAL_POSITION` are still placeholders pending hand-jog
  derivation — see `docs/tool-changer.md`'s open issues for the full list.
  The docked **gripper** tool also carries zero collision at all by
  construction (`vendor_gripper_tool_visual_only.py` strips it), so it can't
  collide with the part it grips either.
- **Screws: reach is tight and placed screws don't follow the jig.** The
  27.5cm screwdriver puts the worst hole 0.771m from the mount against the
  Panda's ~0.855m envelope, so `SCREW_APPROACH_CLEARANCE` is 0.02 (not the
  rack's 0.15) and an unreachable hole is a scene-layout fix, not a code one.
  A placed screw is welded to a static world anchor, so it stays behind if the
  conveyor moves `main_holder` afterward — same limitation
  `park_tool_at_rack()` has. Both in `docs/tool-changer.md`.
- **Screws: `SCREW_HOLES` entries 1 and 4 land 1.0mm apart**, and their x reads
  disagree across the mirror pair 4/6 (81.38 vs 81.83mm), so one of the two is
  a misread of the measurement sheet. Both are marked `CHECK` in `config.py`;
  every other pocket is 52mm+ from its nearest neighbour.

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
- **`DeletePrims` silently no-ops on an articulation-internal joint
  prim.** Confirmed for both the Franka's own finger joints
  (`remove_parallel_jaw_gripper()` uses `SetActive(False)` instead) and a
  URDF-importer-synthesized `root_joint` fixing a free-floating
  `base_link` to the world (`robot.spawn_dockable_tool()`'s hand-only
  branch) — `SetActive(False)` is what actually removes the constraint.
- **A `UsdPhysics.FixedJoint`'s target must resolve to a real
  `RigidBodyAPI` prim**, not just any descendant — a plain organizing
  Xform over an articulation's real links (or a CAD asset with no
  baked-in `RigidBodyAPI` at all, confirmed for `electric_screwdriver.usd`)
  silently fails to be pulled by the joint. See `docs/tool-changer.md`'s
  gotchas 3–4.
- **A jointed body's own enabled collision can fight the joint.** If both
  ends of a `FixedJoint` have real colliders that overlap once pulled
  together, contact-separation force reaches an equilibrium short of the
  joint's target instead of converging — disable collision on whichever
  side doesn't need it once joined. See `docs/tool-changer.md`'s gotcha 2.
- **Live-importing two robots via the URDF importer into the same
  file-backed stage isn't safe, no matter how they're named.** Its
  disk-persisted "Robot Description" cache is shared across every robot
  imported into that stage — confirmed to corrupt not just visual
  references but the *link structure itself* (a second import made
  `panda_link0`–`8` vanish from the first robot's own children). The only
  real fix is to not share the import: pre-bake the second robot as a
  standalone asset in its own anonymous stage and reference it instead
  (`scripts/vendor_gripper_tool.py`). See `docs/tool-changer.md`'s
  gotcha 6.

## Pinned versions

Isaac Sim `5.1.0`, cuRobo commit `ebb71702f3f70e767f40fd8e050674af0288abe8`,
torch `2.11.0+cu128` (CUDA 12.8). Dev GPU: RTX PRO 4000 Blackwell (sm_120)
— `TORCH_CUDA_ARCH_LIST` must be `12.0+PTX` for this GPU.
