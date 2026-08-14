# FR5 + PGC-140 migration (`atc-fairino`)

Moving the `atc` cell off its Franka Panda stand-in onto the real target
hardware: a **FAIRINO FR5** arm with a **DH Robotics PGC-140** parallel gripper.

Branched from `atc` @ `6374624` ("Record the part feeders as GUI-confirmed"),
i.e. deliberately *without* `835abb5`'s in-progress jig-base change.

Done procedurally, one GUI-verified step at a time, rather than as one swap.

| Step | Scope | Gate | State |
|---|---|---|---|
| **1** | Vendor the FR5, mount it on the pedestal in place of the Franka | GUI: scene opens, arm looks right | **GUI-confirmed 2026-08-13** |
| 2 | cuRobo config + collision spheres from the Lula/XRDF editor | teleop plans and moves | **GUI-confirmed 2026-08-13** |
| 3 | PGC-140 as the ATC's dockable gripper tool | Y docks it, C/O drives the fingers | **in progress** — placed, C/O wired, docks and moves; target origin off the mating face |

The gripper stays a **dockable ATC tool** — it is not bolted into a combined
arm+gripper URDF. The male coupler moves to the FR5's wrist, the PGC-140 replaces
`/World/gripper_tool_visual_only` in the rack, and suction/screwdriver are
untouched. cuRobo therefore plans the bare 6-DOF arm and never sees the fingers,
exactly as it does today.

## Prior art: mine the `dobot` branch, don't restart

That branch already did this class of swap once, for a Dobot CR5 + PGC-140, and
paid for several findings the hard way. Steps 2–3 should start from it:

- `a15a021` — "Replace Franka with CR5+PGC-140 in mefron.py". The template for
  this whole exercise, including the mount-yaw bug below.
- `robots/pgc140/` + its `SOURCE.md` — the PGC-140 already vendored from
  `DH-Robotics/dh_gripper_ros` @ `f59f9c2`, with the link/joint renames and the
  live-confirmed removal of `finger2_joint`'s `<mimic>` tag (this Isaac Sim
  version imports a prismatic `<mimic>` as a **rotational** `PhysxMimicJointAPI`
  with mangled limits and no drive, leaving finger2 stuck). Re-vendoring from
  upstream would walk straight back into that.
- `configs/curobo/cr5.yml` + `cr5_collision_spheres.yml` — the shape of a
  non-Franka cuRobo config in this repo, and the repo-relative→absolute path
  patching every loader must do (cuRobo joins `urdf_path`/`asset_root_path`
  against its *own* bundled dirs otherwise).
- `assets/Grasp_Editor/.../cr5_pgc140_gripper.usd` and
  `assets/Grasp_Editor/pgc_finger_print_scanner.yaml` — a gripper-only USD for
  the Grasp Editor, and one real PGC-140 grasp already exported against it.

Two of its live-confirmed findings apply directly here:

- **The CR5 needed a 180° yaw** in `MOUNT_ORIENTATION_WXYZ` where the Franka
  wanted identity. **The FR5 needs it too** — GUI-confirmed 2026-08-13, it faced
  backwards out of the cell on identity. `MOUNT_ORIENTATION_WXYZ` is now
  `[0, 0, 0, 1]`; headless readback puts wrist3 at `(+0.497, +0.102, +0.576)`
  relative to `base_link`, the exact mirror of the un-yawed pose.
- **The PGC-140's finger convention is inverted vs the Franka's.** `q=0.0` is
  **open** (~30mm from centerline) and `q=0.025` (each joint's upper limit) is
  **closed** (~13mm). The Franka is the other way round. An earlier draft on
  that branch locked the gripper closed during planning by getting this
  backwards.

## Step 1 — what landed

- **`robots/fr5/`** — `urdf/fairino5_v6.urdf` + seven STL meshes from
  `FAIR-INNOVATION/frcobot_ros2` @ `867cb32`. Why that file and not the
  sibling `FR5WM.urdf` (wrong kinematics, and it does not parse), plus the
  no-upstream-license finding: [`robots/fr5/SOURCE.md`](../robots/fr5/SOURCE.md).
- **`config.ROBOT_PRIM_PATH`** → `/World/FR5`. Every consumer already read the
  constant, so this was a one-line change; `/World/Franka` moved into
  `robot._STRAY_HISTORICAL_ARM_PATHS` so a Franka baked into `mefron.usd` by a
  past stray Save is deleted on open.
- **`robot.mount_franka()` → `robot.mount_arm()`**, importing `FR5_URDF_PATH`
  through the existing `usd_util.import_urdf()`. Note the shape change: the
  Franka came from cuRobo's bundled assets, the FR5 is repo-local.
- **`robot.print_arm_inventory()`** — prints the link/joint names the importer
  actually produced. Step 2's `fr5.yml` must be written against *those*, not
  against the URDF's names as read.
- **`robot.apply_home_pose()`** — see "The all-zero pose is a singularity" below.
- **`robot.apply_accent_color()`** — see "Accent color" below.
- **`mefron.py --arm-only`** — mounts the arm, poses it, paints it, and hands the
  GUI over. No ATC, no cuRobo, no teleop. It *plays* the timeline (unlike the
  teleop loop, which waits for the user) because only PhysX can actually move the
  arm to the staged home pose; safe here since there is no `motion_gen.warmup()`
  to corrupt. Delete it once the FR5 is wired all the way through.

### The all-zero pose is a singularity

The FR5's URDF zero configuration is **fully outstretched horizontally** — wrist3
lands 0.820m out and only 0.050m above the base. That is what a first GUI run
shows: an arm lying flat across the cell.

It is not merely ugly. Measured offline from the vendored URDF (numerical
Jacobian at the pose): **condition number `inf`** — genuinely rank-deficient, the
same class of failure the `dobot` branch hit when it left the CR5's
`retract_config` at all-zeros and every `plan_single()` failed IK.

`config.FR5_HOME_JOINT_POSITIONS = [0, -π/2, π/2, -π/2, -π/2, 0]` replaces it —
the classic UR-style ready pose, chosen for conditioning, not aesthetics:

| | all-zero | home pose |
|---|---|---|
| wrist3 rel. base | (-0.820, -0.102, +0.050) | (-0.497, -0.102, +0.577) |
| tool Z | (0, -1, 0) — sideways | (0, 0, -1) — straight down |
| Jacobian cond | **inf** | 8.2 |
| within joint limits | yes | yes |

**Confirmed live, headless 2026-08-13:** with the pose staged and the timeline
played, the simulated arm settles at wrist3 `(-0.497, -0.102, +0.576)` relative
to `base_link` — the offline FK prediction to within 1mm. That also confirms the
unit conversion: `UsdPhysics` angular drive targets and `JointStateAPI` positions
are in **degrees**, while the URDF, cuRobo and this constant are all radians.

**Step 2 should seed `fr5.yml`'s `cspace.retract_config` from this**, not from
zeros, for exactly the reason above.

### Generating the collision spheres (step 2's GUI half)

Run `mefron.py --arm-only`, then **Tools → Robotics → Lula Robot Description
Editor**, and select the articulation at `/World/FR5`.

- **Instanceable meshes cannot be auto-fitted.** The editor refuses them and only
  offers hand-authoring ("Found instanceable mesh at path … cannot be used to
  generate spheres automatically"). Everything the URDF importer produces is
  instanceable, so `--arm-only` calls `robot.un_instance_link_meshes()` up front
  to clear all 14 visuals/collisions scopes. Without it, auto-generate is dead on
  every link.
- **Leave `base_link` out** of the sphere set, matching cuRobo's own `ur5e.yml`,
  which has no `base_link` entry either. It is bolted to the pedestal, and a
  sphere there overlaps the mount — exactly what made every CR5 plan fail with
  `INVALID_START_STATE_WORLD_COLLISION` on the `dobot` branch.
- **Per-link budget worth copying**, from `ur5e.yml` — whose two long links are
  within 3mm of the FR5's (upper arm 0.425 identical, forearm 0.3922 vs 0.39501),
  so its proven distribution transfers: shoulder 1, upper arm 8, forearm 9,
  wrist1 4, wrist2 4, wrist3 1. 28 spheres total.

The editor's own "generate" button is a thin wrapper over
`lula.create_collision_sphere_generator(points, faces).generate_spheres(n, radius_offset)`,
with the mesh points transformed into the link's frame first — so if the GUI ever
becomes the bottleneck, the identical fit is scriptable headlessly.

**This cuRobo build reads XRDF natively** (`curobo/util/xrdf_utils.py`'s
`convert_xrdf_to_curobo()`, plus a bundled `ur10e.xrdf` reference), so the
editor's export can feed `fr5.yml` directly with no format conversion.

## Step 2 — what landed

`configs/curobo/fr5.xrdf`, exported from the editor, loaded through cuRobo's own
`convert_xrdf_to_curobo()` by `motion.load_robot_cfg()`. No `fr5.yml` exists and
none is needed — XRDF *is* the cuRobo config here.

`FRANKA_MOTION_GEN_ROBOT_CFG` is gone, replaced by `FR5_XRDF_PATH` +
`FR5_URDF_ASSET_ROOT` (absolute, because cuRobo resolves relative paths against
its own bundled dirs). `setup_motion_gen()` lost its `has_parallel_jaw_gripper`
argument and `_robot_cfg_without_gripper_joints()` went with it — both existed
only to strip `panda_finger_joint1/2` from the Franka's cspace, and the FR5
never had them.

The ATC's male coupler, the surface-gripper joint, and the screw wrist weld all
moved from `panda_hand` to `wrist3_link`.

### Three things the editor's export needed by hand

Re-exporting over `fr5.xrdf` **loses all three** — re-apply them:

1. **`tool_frames` was absent, and cuRobo hard-fails without it.** The editor has
   no ee-link concept, but `convert_xrdf_to_curobo()` does
   `output_dict["ee_link"] = tool_frames[0]` behind a `raise_error=True` lookup.
   Added `tool_frames: ["wrist3_link"]`.
2. **`collision.buffer_distance` was absent, which silently means zero padding**
   (`if buffer_distance is None: buffer_distance = 0.0`). Since the spheres were
   fitted with the editor's radius offset at 0, they'd have had no margin at all.
   Set to 0.01 per link, matching cuRobo's own `ur10e.xrdf`.
3. `wrist3_link` carried the **same sphere twice**, identical center and radius —
   the count field resets to 0 after a commit, which makes a double-click easy.

`modifiers: set_base_frame` is *not* needed: cuRobo takes `base_link` from
`kinematics_parser.root_link`, and the FR5's URDF root is already `base_link`
(unlike the CR5's combined URDF, which had a `dummy_link` root).

### Verified headless 2026-08-13

- **`convert_xrdf_to_curobo()` + `CudaRobotModel` build clean.** ee_link
  `wrist3_link`, joints `j1`…`j6`, `retract_config` carried from the editor's
  `default_joint_positions`, accel 10 / jerk 500, 6 collision links (no
  `base_link`).
- **Three independent methods agree on FK.** Offline URDF math
  `(-0.497, -0.102, +0.577)`, PhysX readback `(-0.497, -0.102, +0.576)`, cuRobo's
  own model `(-0.4971, -0.1021, +0.5766)`. cuRobo's FR5 matches the simulated one.
- **No sphere overlaps at `retract_config`.** All-pairwise clearance over
  non-ignored pairs, tightest `wrist1_link ↔ wrist3_link` at **+0.035m**. So the
  editor's adjacent-pairs-only `self_collision.ignore` is sufficient as exported —
  the CR5 needed three extra skip-one entries, the FR5 needs none. One
  configuration only, not a sweep.
- **`plan_single()` succeeds** to ±10cm X, −15cm Z, +20cm Y off the home pose
  (57–69 waypoints, 1.0–1.2s).
- **`test_mefron_teleop_headless.py` PASSES** — `plan_single success=True`, arm
  followed the simulated drag both directions, 0.165 rad max joint delta.
- Full `mefron.py --headless` runs start to finish, no tracebacks.

### Teleop jerked: the URDF importer left the joints undamped

**Root cause, confirmed by live trace + headless A/B.** Upstream's
`fairino5_v6.urdf` declares `<dynamics damping="0"/>` on all six joints, and
Isaac's URDF importer honours that over `import_urdf`'s own arguments. Read back
straight after import:

```
config asks : stiffness=1047.2  damping=210.0
j1..j6 got  : stiffness=625.0   damping=0.0      <- undamped springs
```

Undamped drives ring, which measured as **6.04x velocity overshoot with only
0.029 rad position error** — the arm buzzing along its commanded path at roughly
25Hz rather than lagging it.

**Fixed in the vendored URDF** (`damping="0"` -> `"10.0"`, matching what cuRobo's
own `franka_panda.urdf` declares), not in code: `6.04x -> 1.18x` through
`mount_arm()` itself. Full rationale and the re-vendor warning:
[`robots/fr5/SOURCE.md`](../robots/fr5/SOURCE.md).

**Damping is the whole story — stiffness is a red herring.** Stiffness 1047 with
damping 0 still measures 6.05x, identical to 625. `FR5_DRIVE_STRENGTH` /
`FR5_DRIVE_DAMPING` are passed to `import_urdf` and *ignored*; they are kept only
because the helper's signature requires them.

**This is also exactly why the Franka never did it:** its URDF declares
`<dynamics damping="10.0"/>`. Same bug the CR5 hit (`9f08fb5`, "the fully
undamped spring rang hardest right where a time-optimal trajectory's jerk peaks")
— same SolidWorks exporter, same `damping="0"`.

**Always read drive gains back after import; never assume the importer applied
them.**

#### How it was found, and what was wrongly blamed first

Four theories were tried and discarded before this, each reverted afterwards.
Recording them so nobody re-treads the path:

| blamed | verdict |
|---|---|
| render frame time | **wrong** — live trace: mean 19.3ms, p99 29.0ms |
| trajectory playback pinned to frame rate | real but not the symptom; reverted |
| teleop velocity/acceleration scales | not the cause; reverted to 0.6 / 0.1 |
| replans firing while still moving (`_STATIC_JOINT_VELOCITY_THRESHOLD`) | not the cause; reverted to 0.5 |
| 820ms obstacle rescan every 1000 frames | **real**, but a *freeze*, not jerk; reverted |

The obstacle-rescan stall is genuine and still there: `get_obstacles()` costs
**820ms** re-extracting `ConveyorBelt_A06_01`'s CAD mesh, robot-independently
(`update_world()` is 7.5ms on the FR5, 8.8ms on the Franka), and it runs every
`_TELEOP_OBSTACLE_RESCAN_INTERVAL` frames. Worth fixing on its own merits later —
caching on the obstacles' world transforms took it to 86ms — but it was reverted
here to keep this change set to the one proven fix.

Two headless measurements disagreed about overshoot (6.05x vs 1.03x) purely
because the "clean" one wrote drive gains explicitly before measuring, silently
applying the very fix that was missing. What settled it was instrumenting the
live GUI session — headless runs cannot see render cost — with a per-frame CSV of
commanded vs actual velocity. That tracer was removed with the other reverts; it
is worth rebuilding if motion quality is ever in question again.

### Accent color

The vendored URDF paints **every** link the same light grey
(`rgba 0.89804 0.91765 0.92941`) and contains no orange anywhere — so there was
no existing color to honour instead. `robot.apply_accent_color()` authors one
material and binds it to `FR5_ACCENT_LINK_NAMES` (`shoulder_link`,
`wrist2_link`); change `FR5_ACCENT_COLOR_RGB` to retune it.

Three traps, all confirmed on the imported stage — the first two each produced a
**silently white arm with a binding that read as correctly authored**:

1. **The importer binds its grey with `strongerThanDescendants`, on an
   intermediate Xform** (`{link}/visuals/{link}`), not on the Mesh. That beats
   any binding authored on the Mesh below it. `apply_accent_color()` therefore
   re-binds every prim in the subtree that already carries a binding, at that
   same strength — not just the meshes.
2. **The importer's materials are OmniPBR MDL**
   (`info:mdl:sourceAsset = @OmniPBR.mdl@`, `inputs:diffuse_color_constant`),
   not `UsdPreviewSurface`. A `UsdPreviewSurface` authored here rendered as
   untouched white. The accent material mirrors the importer's structure exactly.
3. Each link's `visuals` is an **instanceable prototype**; authoring through one
   silently no-ops. It goes through the existing `usd_util.un_instance_ancestor()`.

**Check bindings with `ComputeBoundMaterial()`, never `GetDirectBinding()`.**
The latter reports what is authored *on that prim* and happily returned the
accent material while the grey was still what actually rendered — it does not
account for ancestor binding strength. Confirmed working 2026-08-13: the mesh
resolves to `/World/FR5/Looks/FR5Accent`.

Runtime-only by necessity: the arm is re-imported from URDF every run, so nothing
about it can be baked into `mefron.usd` the way the scenery is. The material is
authored under the arm's own `Looks` scope so the re-import disposes of it too.
- `remove_parallel_jaw_gripper()` / `hide_hand_housing()` are no longer called:
  the FR5 ships a bare ISO flange with no hand to strip. Left in place, marked
  FRANKA-ONLY, pending step 3.

### FR5 facts worth having on hand

- **Import confirmed headless 2026-08-13** (`--arm-only --headless`): the
  importer produced 7 link Xforms and 6 drive-carrying joints, named exactly as
  the URDF names them — no importer mangling, so step 2's `fr5.yml` can use them
  verbatim. **Not** a confirmation that the arm is correctly *placed*; that is
  the GUI check below.
- Links: `base_link`, `shoulder_link`, `upperarm_link`, `forearm_link`,
  `wrist1_link`, `wrist2_link`, `wrist3_link`. Joints: `j1`…`j6`.
- `ee_link` is `wrist3_link` — there is no `tool0`/flange link. It carries
  visual geometry, which `motion.build_teleop_target()` requires.
- 922mm reach (425mm upper arm, 395mm forearm) against the Panda's ~855mm. That
  eases CLAUDE.md's "screws: reach is tight" open issue, and it means every
  reachability judgement made under the Franka is now conservative, not stale.
- Real `effort`/`velocity` limits on every joint, so cuRobo's
  `ValueError: Joint velocity limits is zero` (which forced a URDF patch on the
  CR5) cannot happen here.
- One STL serves both `<visual>` and `<collision>` — the arm renders flat grey,
  and every collision mesh is full-resolution CAD.

## Step 3 — the PGC-140 gripper (in progress)

The gripper tool is now the PGC-140, placed by hand in `mefron.usd` at
`/World/cr5_pgc140_gripper` from
[`robots/grippers/cr5_pgc140_gripper/`](../robots/grippers/cr5_pgc140_gripper/) —
which is the Grasp-Editor asset plus the female coupler bolted to the back of
`pgc140_base_link` (flush at z=-0.0084, body reaching z=-0.0184, mating face
outermost; same `orient` + `scale:unitsResolve` convention the Franka tool uses).

### C / O are wired

| | was (Franka hand) | now (PGC-140) |
|---|---|---|
| `GRIPPER_JOINT_NAMES` | `panda_finger_joint1/2` | `pgc140_finger1/2_joint` |
| `GRIPPER_FINGER_LINK_NAMES` | `panda_left/rightfinger` | `pgc140_finger1/2_link` |
| `GRIPPER_OPEN_POSITION` | 0.010 | **0.000** |
| `GRIPPER_CLOSED_POSITION` | 0.000 | **0.025** |
| `GRIPPER_DRIVE_DAMPING` | 200 | 1000 (the asset's own value) |

Open/closed **invert** — on the PGC-140 the joint measures inward travel. Verified
headless: `O` writes 0.0 and `C` writes 0.025 to both finger drives.

Three code changes were needed beyond renaming, each for a real reason:

1. **`type=acceleration` -> `force`.** The asset ships `acceleration`, which
   mass-normalises stiffness: on a 14g finger, stiffness 10000 becomes ~143 N/m.
   Exactly the trap `GRIPPER_DRIVE_TYPE` already documents for the Franka.
   `stiffen_gripper_drive()` fixes it; confirmed `type=force` after the run.
2. **Dropped the xform-stack reset** in `enable_gripper_tool_fingers()`. Baking
   each finger's world transform and setting `resetXformStack` was right for the
   Franka's hand-authored fingers and *wrong* for real URDF articulation links —
   it detaches them from the tool root, so they would stay put while the docked
   tool moved.
3. **`rootJoint` vs `root_joint`.** `spawn_dockable_tool()` only looked for the
   importer's `root_joint`, so this asset's `rootJoint` was silently skipped. Now
   both are checked, but only a `FixedJoint` that genuinely welds to the world is
   deactivated — the PGC-140's is a limit-free generic joint constraining nothing,
   and killing it would leave its articulation rootless.

### RESOLVED: the docked tool was a second articulation

**Symptom (live GUI):** `Y` docked, then the wrist link section vibrated, then the
arm went stuck, and a Stop/Play afterwards sent it haywire.

**Cause.** The PGC-140 asset carries `ArticulationRootAPI` — it was built for the
Grasp Editor, which needs a free-floating articulation. The ATC then welds it to
the arm with an `excludeFromArticulation` FixedJoint, so PhysX had **two
reduced-coordinate articulations rigidly coupled through a maximal-coordinate
constraint**. That chatters, then deadlocks. The Franka tool never hit it: its
asset has *no* `ArticulationRootAPI`, no rigid bodies and no joints at all — plain
geometry that the runtime gives physics to.

**Fix:** `toolchanger._demote_tool_articulation()`, called from
`spawn_dockable_tool()` for any tool with a `female_coupler_parent_link_name`.
It removes `ArticulationRootAPI` and deactivates the tool's root joint, leaving
three rigid bodies and two prismatic joints — the shape the ATC has always docked.
Nothing is lost: `set_gripper_tool_finger_target()` writes the finger `DriveAPI`
targets directly and never went through an articulation controller.
**Confirmed live 2026-08-13: docks, and the arm moves with the tool on.**

Runtime rather than an edit to `mefron.usd`, so it survives re-placing or
re-referencing the asset, and `assets/grasp_editor/`'s copy keeps its articulation
root where the Grasp Editor genuinely needs it.

Ruled out by measurement along the way, so nobody re-chases them: the tool's own
collision (`pgc140_base_link`'s collider is disabled; only the fingers collide,
93mm out) and arm drive damping (steady-state arm velocity 0.0000 both bare and
docked — a sweep suggesting otherwise was the probe re-authoring one wrist joint
path over a live one, this repo's own stale-`body1` gotcha).

### RESOLVED: the ee frame is the tool flange, not wrist3_link's origin

The draggable target's origin sat nowhere near where tools dock. Measuring the
imported geometry showed why, and turned up a second bug alongside it:

```
wrist3_link geometry   z = [+0.0532, +0.0990]   in its own frame
link origin  z = 0     -> 53mm short of any of its own metal, in empty space
outboard (tool) face   z = +0.0990
male coupler was at    z = [0, +0.020]          -> floating clear of the wrist
mate plane was at      z = 0                    -> 99mm inboard of the flange
```

Which face is outboard is settled two ways: it is 142mm from `wrist2_link`'s
origin versus 115mm for the inboard face, and **0.820 + 0.099 = 0.919m matches the
FR5's published 922mm reach**, which is quoted to the tool flange.

`FR5_TOOL_FLANGE_OFFSET = 0.0990` now drives three things:

1. **`fr5.xrdf` gains a `modifiers: add_frame`** for `tool_flange`, parented to
   `wrist3_link` at `[0, 0, 0.0990]`, and `tool_frames` points at it. cuRobo
   therefore plans the *flange*. Verified: `ee_link: tool_flange`, and FK moves
   99.0mm off the old wrist3 position.
2. **`robot.attach_tool_flange_frame()`** authors the same frame as a live child
   Xform. Necessary because the XRDF frame is synthetic and has no prim, while
   grasp/screw code reads the ee's *world pose* off a real one —
   `teleop` now takes `_ee_link_prim_path` from `config.FR5_EE_FRAME_PRIM_PATH`
   rather than `{robot}/{ee_link}`.
3. **`TOOL_CHANGER_MALE_LOCAL_POSITION`** puts the coupler on the flange, so its
   inner face — the mate plane — lands there too. Verified 10.0mm from the flange,
   i.e. exactly half the cylinder height.

`build_teleop_target()` needed care: `ee_link` is now a frame with no geometry, and
the `dobot` branch already recorded that a synthetic ee silently yields an
empty-bbox target. It now references **`wrist3_link`'s** visuals into a child
`ee_visual` Xform offset by `-FR5_TOOL_FLANGE_OFFSET`, so the mesh still draws
around the wrist while the target's origin is the dock face. Verified non-empty.

Cross-check that it is right: the old target sat at world z=1.386 at the home
pose; the new one is at 1.287, exactly 99mm lower, with the tool pointing down.
`plan_single` still succeeds and `test_mefron_teleop_headless.py` still passes.

**Every grasp/dock/screw offset is now flange-relative.** Doing this before any of
them were re-derived was deliberate — they were all still `[0,0,0]` placeholders,
so it cost nothing; after a re-derivation it would have invalidated all of them.

### The suction attach (`V`) pinned the arm to the world

**Symptom:** `V` froze the arm completely — it stopped following the target, and
only `L` freed it. Worked on the Franka.

`robot.attach_surface_gripper_physics()` authors a D6 with **`body0 = wrist3_link`
and `body1` never set**. In UsdPhysics an unset `body1` means the **world**, and
the joint locks `transX`/`transY` (`low > high`). Isaac's SurfaceGripper manager
binds `body1` on a successful grab; on a **failed** one it engages the constraint
anyway. Measured: `close_gripper()` took arm travel from **0.3000 rad to
0.0000**, with `status = GripperStatus.Open` — nothing grabbed, wrist welded to
the world.

**Nothing about the joint changed from `atc`.** The diff is two lines, and
`body1`, the locked axes and the drives are identical. What changed is where the
attach point lands:

| | frame | `SURFACE_GRIPPER_LOCAL_POSITION = 0.1` lands |
|---|---|---|
| Franka | `panda_hand` (spans z −0.026…0.066, fingertips ≈0.10) | at the TCP — grabs succeeded |
| FR5 | `wrist3_link` | **1mm past the flange**, 109mm short of the cup |

So on the FR5 nothing was ever within `SURFACE_GRIPPER_MAX_GRIP_DISTANCE` (0.03)
and every `V` was a failed grab. Fixed by putting the attach point on the cup:
`FR5_TOOL_FLANGE_OFFSET + SUCTION_TOOL_REACH` = 0.099 + 0.110 = **0.209**, the
110mm measured off `suction_gripper_with_tool_female`'s own geometry.

**Moving the attach point alone was not enough — `body1` had to be bound too.**
Isaac's manager rebinds `body1` on a successful grab, so an unset one is only
harmless while grabs succeed; that is the whole reason the Franka never showed
this. NVIDIA's own reference authoring settles the intended shape — in
`SurfaceGripper_gantry.usda` every `IsaacAttachmentPointAPI` joint is authored
`body0 = Gripper_Cones`, `body1 = Gantry_x` (the body the gripper is *mounted*
on), with **coincident frames** (`localPos1 = (…, ±0.15007496)`). Never unset.

The equivalent pair here is `wrist3_link` ↔ the docked suction tool, which does
carry `RigidBodyAPI` (applied at runtime by `spawn_dockable_tool()`; the FR5 has
no body at the flange at all — `tool_flange` is a bare `Xform`). Both frames must
land on the same point or the constraint carries a permanent violation and
saturates the instant it engages:

```
localPos0 = (0, 0, 0.209)   on wrist3_link   -- FR5_TOOL_FLANGE_OFFSET + SUCTION_TOOL_REACH
localPos1 = (0, 0, 0.110)   on the tool      -- its origin IS the flange, so cup = SUCTION_TOOL_REACH
```

Setting the body while leaving `localPos1` at identity was measured live and
froze the arm exactly as before — a 110mm standing violation. Confirmed working
live 2026-08-14 with both.

**The approach poses.** They place the *ee frame* relative to the part and were
jogged against `panda_hand`, so they were expected to be badly off. Re-jogged
against the FR5 with the cup on the screen, `suction_gripper_approach_on_screen`
came out **within ~2mm** of the Franka's value:

```
old (panda_hand) : [ 0.00028, -0.00024, -0.11558]
new (tool_flange): [ 0.00044,  0.00063, -0.11746]
```

They nearly coincide because `panda_hand` and `tool_flange` are both the **tool
mate plane** — the ATC's coupler sat on `panda_hand` and now sits on the flange.

But near-coincident is wrong here, and `N` still seats the cup too deep (open
issue). The attach point is 110mm past `tool_flange` where the Franka's was 100mm
past `panda_hand`, so the same flange pose buries the cup ~8mm further in:

```
Franka: -0.11558 + 0.100 = -0.0156   <- cup standoff that worked
FR5:    -0.11746 + 0.110 = -0.0075   <- ~8mm deeper
```

Reproducing the Franka's proven standoff wants local z ≈ **-0.1256**. Not
applied — poses in this repo are hand-jogged in the GUI, not computed.
`SURFACE_GRIPPER_APPROACH_CLEARANCE` (0.01) has **no code references** at all; it
is documentation for a value baked into that pose by hand.
`suction_gripper_approach_on_pcb_assembly` has the same 10mm issue and has not
been re-jogged.

**Guard added regardless.** `teleop._step_arm()` now checks
`_SUCTION_GRIP_SETTLE_FRAMES` (30) after a `V`, and if the manager still reports
Open it reopens the gripper and logs why. A failed grab becomes a no-op instead of
an unrecoverable-looking freeze.

Two dead ends, both reverted — **do not repeat**:

- *Invalidating the articulation handles on the open/close transition.* The
  handles were never stale; the arm was physically pinned. (The transition check
  was kept anyway — the manager does author a joint mid-Play, and `L` needs it.)
- *Rejecting `body1 = suction tool` as over-constraining.* It looks like a
  duplicate of the ATC's own dock constraint between the same two bodies, and a
  probe appeared to confirm it — but that probe docked from the rack, which
  freezes the arm on its own (see below) and did so in the control run too. The
  binding is in fact required, and is what NVIDIA's reference does.

**Headless probes repeatedly misled here.** They cannot dock the way the GUI does:
creating the dock joint while the tool is still at its rack makes PhysX snap
disjointed bodies together, which freezes the arm by itself and masks whatever is
being tested. For anything involving docking or the surface gripper, the GUI is
the source of truth.

### Still Franka-shaped

The four `GRASP_TARGETS` yamls remain keyed to `panda_hand` /
`panda_finger_joint1`, so `G/J/B/K` raise `KeyError` until re-exported against the
PGC-140. `grasp.compute_grasp_finger_widths_from_file()` now defaults its joint
name from `config.GRIPPER_JOINT_NAMES[0]`, so it will read the new names once the
yamls carry them.

## Verify step 1

```
${ISAACSIM_ROOT_PATH}/python.sh scripts/mefron.py --arm-only
```

1. **Base flush on `/World/ur10_mount`** — not sunk into the pedestal, not
   floating. `MOUNT_POSITION` is the pedestal prim's own translate and was never
   confirmed to be its top mounting flange.
2. ~~**Arm faces into the cell**~~ — settled: it needed the 180° yaw, applied.
3. **All seven links have visible geometry**, with `shoulder_link` and
   `wrist2_link` orange and the rest flat grey (the URDF ships no textures).
4. **The arm stands in its home pose, not flat.** If it is lying outstretched,
   `apply_home_pose()` did not take — check the joint-name warnings.
5. **No self-intersection at the home pose**, and the arm does not fold into
   the packing table.

Expected, not regressions:

- `mefron.usd` **and all four `configuration/*.usd`** show a diff afterwards —
  the URDF importer rewrites them unconditionally on every run (CLAUDE.md
  gotcha). Never blindly `git checkout` them unless, as after the step-1 run,
  the tree was known-clean beforehand.
- `open_stage()` logs `Unresolved reference prim path .../World/Franka/panda_hand/visuals`
  for the baked `/World/target` prim (and the same for the retired
  `target2`/`target3` → `Franka2`/`Franka3`, which already dangled before this
  branch). `motion.build_teleop_target()` re-authors `/World/target` against the
  live `ee_link` every run, so this clears itself in step 2 — the stale
  reference in the baked layer is cosmetic until then.
- **All six `test_mefron_*_headless.py` harnesses fail**, and plain `mefron.py`
  without `--arm-only` fails past the arm mount. They build cuRobo from
  `franka.yml` and reach for `panda_hand` paths on an arm that is neither. Step 2.

## Re-derivation checklist

Every one of these was hand-jogged against the Franka and its hand. **Nothing
here gets a number until it is re-derived in the GUI** — no numeric conversion,
no "close enough" carry-over. All are marked `STALE` or `UNVERIFIED` in
`config.py`.

| What | Where | Blocked on |
|---|---|---|
| ~~`MOUNT_ORIENTATION_WXYZ`~~ | `config.py` | **done** — 180° about Z, GUI-confirmed 2026-08-13 |
| `FR5_DRIVE_STRENGTH` / `FR5_DRIVE_DAMPING` | `config.py` | step 2 (carried over from the Franka's tuning) |
| ~~`FRANKA_MOTION_GEN_ROBOT_CFG`~~ | `config.py`, `motion.py` | **done** — `FR5_XRDF_PATH` |
| ~~FR5 collision spheres~~ | `configs/curobo/fr5.xrdf` | **done** — Lula editor, 2026-08-13 |
| The four `GRASP_TARGETS` yamls | `assets/*.yaml` | step 3 — re-export in the Grasp Editor against the PGC-140 |
| ~~`GRIPPER_JOINT_NAMES`, `GRIPPER_FINGER_LINK_NAMES`~~ | `config.py` | **done** — `pgc140_finger{1,2}_joint`/`_link` |
| ~~`GRIPPER_OPEN_POSITION` / `GRIPPER_CLOSED_POSITION`~~ | `config.py` | **done** — 0.000 / 0.025, inverted |
| `TOOL_CHANGER_GRIPPER_HAND_JOINT_LOCAL_ORIENTATION_WXYZ` | `config.py` | step 3 |
| `SURFACE_GRIPPER_LOCAL_POSITION` | `config.py` | step 3 (already pre-ATC stale) |
| `female_coupler_local_*` for the gripper tool | `config.TOOL_CHANGE_TARGETS` | step 3 (already placeholders) |
| Screw pick/place offsets | `config.py`, `screws.py` | step 3 — currently relative to `panda_hand` |
