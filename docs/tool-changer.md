# Automatic tool changer (ATC): design, alternatives considered, open issues

The `atc` branch replaces the 3-arm cell (arm 1 gripper, arm 2 suction, arm
3 screwdriver) with **one Franka** fitted with an automatic tool changer:
a permanent "male" coupler on the wrist, and three detachable "female"
tools (gripper, suction, screwdriver) parked in their own rack until a
numpad key docks one.

## Isaac Sim prior art considered, and why none fit as-is

No bundled "automatic tool changer" asset/extension exists in Isaac Sim
5.1. Two adjacent mechanisms exist, both ruled out for the same reason —
neither is built for a live, repeated, keyboard-triggered swap during Play:

- **`isaacsim.robot_setup.assembler` ("Robot Assembler")** — a real
  extension (Tools > Robotics > Asset Editors > Robot Assembler), already
  enabled in this repo's extension list, already researched in
  `docs/robot-assembler.md` for the unrelated part-assembly-snap problem.
  Same verdict applies here: authoring-time only (Stop mode, manual
  nudging), not meant to be re-triggered automatically at runtime.
- **USD variant-set end-effector swapping** (Isaac Sim's own "Setup a
  Manipulator" tutorial) — bakes the tool choice into the saved USD at
  authoring time via a variant selector, not a physically simulated
  docking motion during Play.
- **The `SurfaceGripper` schema** (already used for the old arm 2's
  suction cup, `robot.attach_surface_gripper_physics()`) — deliberately
  **not** reused for docking: CLAUDE.md already documents an unresolved
  runtime detach race in that schema (arm 2's L key "doesn't actually let
  go"). A plain `UsdPhysics.FixedJoint` sidesteps that class of bug
  entirely.

The mechanism that *does* fit is the one `docs/robot-assembler.md` already
recommended for its own (unrelated) problem: **a scripted
`UsdPhysics.FixedJoint` created/destroyed at runtime**, generalized from
the joint-authoring mechanics `attach_surface_gripper_physics()` already
exercises in this repo (there as an intentionally-soft D6; here as a
plain rigid weld).

## Architecture

- **Male coupler** (`robot.attach_tool_changer_male_coupler()`): a plain
  `UsdGeom.Cylinder` (Ø63mm — this repo's own documented Franka ISO
  9409-1-50 flange OD, the interface `robots/franka_panda/Props/suction
  gripper.usd` was custom-designed to match — × 20mm), riding kinematically
  on `panda_hand`, permanent for the life of the process.
- **Three dockable tools** (`config.TOOL_CHANGE_TARGETS`,
  `robot.spawn_dockable_tool()`), all referenced USD assets: suction/
  screwdriver reuse the existing `SUCTION_GRIPPER_USD`/`SCREWDRIVER_USD`;
  the gripper is `scripts/vendor_gripper_tool.py`'s pre-baked export of
  `robot.mount_franka_hand_only()`'s hand-only URDF (a small articulation
  of its own — actuated fingers — not a static prop, so it's a
  multi-link reference rather than a flat one; see gotcha 6 for why it's
  pre-baked rather than live-imported).
- **Parking vs. docking** (`robot.park_tool_at_rack()` /
  `dock_tool_to_wrist()` / `undock_tool_to_rack()`): a tool is **always**
  joint-fixed to something — its own rack when parked, the wrist's male
  coupler when docked — never free-falling. Docking/undocking deletes
  whichever joint currently exists and authors the other one via
  `_create_tool_fixed_joint()`.
- **Keyboard/teleop wiring** (`teleop.ToolChangerControl`,
  `_build_tool_change_queue()`): numpad 1/2/3 request a swap; the request
  is idle-gated like P (only starts once the arm has no in-flight plan and
  no waypoints queued from a previous swap). The queue is a generic
  ordered list of `(position, orientation, on_arrival)` waypoints — return
  the current tool to its rack (hover → descend → undock → retract) if one
  is docked, then approach the requested tool the same way (hover →
  descend → dock → retract). Hover clearance
  (`config.TOOL_RACK_APPROACH_CLEARANCE`) is relative to each dock pose,
  not a fixed world-Z constant like `ASSEMBLY_LIFT_HEIGHT` — see
  CLAUDE.md's open issue about that exact mistake.
- **Tool-gating**: J/B (grasp), C/O (gripper open/close), N/M
  (suction-approach), and suction attach/detach are only honored when the
  matching tool is currently docked; otherwise they print a warning and
  no-op. P stays tool-agnostic (places whichever object was last
  grasped/approached, gated on which tool is actually docked so the two
  code paths can't cross-fire).

## Gotchas confirmed live while building this (headless, real GPU)

Five real bugs surfaced only by actually running the mechanism under
PhysX, not by reading the USD Physics schema docs — recorded here so the
next person extending this doesn't have to rediscover them:

1. **`DeletePrims` silently no-ops on an articulation-internal joint
   prim.** `mount_franka_hand_only()`'s imported hand has a
   URDF-importer-synthesized `root_joint` (a `PhysicsFixedJoint`) welding
   `base_link` to the world — harmless for the main arm (whose base
   really should stay on its pedestal) but fatal here, since it fights
   the wrist/rack joint for control of the same body. `DeletePrims` on it
   does nothing (same gotcha `remove_parallel_jaw_gripper()`'s docstring
   already documents for the finger joints); `prim.SetActive(False)` is
   what actually removes the constraint.
2. **A docked tool's own enabled collision fights the wrist joint.**
   `spawn_dockable_tool()` leaves the tool as a real rigid body (needed
   while parked). Left enabled while docked too, its collider
   interpenetrates `panda_hand`'s own collider and the contact-separation
   force reaches an equilibrium tens of cm short of the joint's target
   instead of converging to it. Fixed by toggling `CollisionEnabledAttr`
   off on dock, back on on undock (`_set_tool_collision_enabled()`) — not
   `RigidBodyAPI`, which must stay on for the joint's solver to move the
   body at all.
3. **A `FixedJoint`'s target must resolve to an actual `RigidBodyAPI`
   prim, not just any descendant.** The gripper tool's `tool_prim_path` is
   a plain organizing Xform over its real links (`base_link`,
   `panda_hand`, ...) and is never itself simulated — a joint targeting a
   child of *that* prim never moves. `female_coupler` must be parented
   under whichever prim actually carries `RigidBodyAPI`
   (`_female_coupler_parent_prim_path()`: `base_link` for the gripper,
   `tool_prim_path` itself for the static suction/screwdriver props).
4. **Not every referenced CAD asset carries baked-in `RigidBodyAPI`.**
   `electric_screwdriver.usd` has none anywhere in its subtree (confirmed
   by direct inspection) — `attach_screwdriver_gripper()`'s old docstring
   even flagged this as "not separately confirmed" before the ATC needed
   it to actually be a joinable body. `spawn_dockable_tool()` now applies
   `UsdPhysics.RigidBodyAPI.Apply()` explicitly rather than trusting the
   source asset.
5. **Per-tool wrist joint paths, not one shared `wrist_joint` name.**
   Even after fixing 1–4, redefining a `FixedJoint` prim at the same path
   with a different `body1` target left PhysX solving against the
   previous tool's target. Not fully root-caused (fix 1–4 may have been
   the real cause all along; this was applied defensively and never
   reverted to check), but costs nothing and removes an entire class of
   same-path-reuse risk regardless.
6. **Live-importing a second robot via the URDF importer into `mefron.usd`'s
   own file-backed stage isn't safe at all, regardless of naming** — the
   importer's disk-persisted "Robot Description" cache
   (`MEFRON_CONFIGURATION_DIR`) is shared across every robot imported into
   that stage, not scoped per-robot. First surfaced as: the hand-only
   gripper tool's URDF reused `base_link`/`ee_link`, the exact same names
   the main arm's own `franka_panda.urdf` uses for its root/tip links,
   letting the tool's entry silently overwrite the arm's own *visual*
   reference in that shared cache — arm stayed a valid, correctly-posed
   prim (why headless pose/joint checks never caught it) but rendered
   invisible in the GUI. Renaming the tool's links to
   `"atc_gripper_tool_*"` fixed *that* symptom — but a follow-up direct
   inspection (`Franka.GetChildren()` before/after spawning the tool)
   found the collision goes deeper than visuals: spawning the tool made
   `panda_link0`–`panda_link8` and `right_gripper` disappear from
   `/World/Franka`'s own children entirely, replaced by the tool's link
   names — the *entire link structure*, not just visual references, gets
   corrupted, no naming scheme can fix that from the importing side. Also
   produced a compounding, self-perpetuating failure mode along the way: a
   stray Save while this was happening baked a stray top-level
   `/panda_gripper_only` prim directly into `mefron.usd` (the importer's
   own pre-`MovePrim` staging path, same mechanism as the already-
   documented `/panda` gotcha), which kept re-triggering the corruption on
   *every subsequent run* even after the immediate cause was fixed, since
   the stray prim itself was never cleaned up (now guarded in
   `clear_stray_robot_prims()`).
   - **Real fix**: stop live-importing the gripper tool into `mefron.usd`
     at all. `scripts/vendor_gripper_tool.py` bakes
     `mount_franka_hand_only()`'s hand-only URDF into a standalone,
     referenceable asset (`robots/franka_panda/Props/
     gripper_tool_hand_only.usd`) inside its own fresh anonymous stage —
     never opened from `mefron.usd`, so there's no shared configuration
     directory to collide with. `robot.spawn_dockable_tool()` references
     this asset the same way it references the suction/screwdriver
     assets (`_reference_tool_asset()`), uniformly for all 3 tools; no
     special-casing or link renaming needed once nothing is live-imported
     into the shared stage. `mount_franka_hand_only()` itself is
     unchanged — `mefron_gripper_probe.py`'s own single-robot-per-session
     use of it never hit this bug in the first place.

7. **Whether an mm-modeled CAD asset needs a manual 0.001 `local_scale` isn't predictable from
   the file alone -- `add_reference_to_stage()` (under `_reference_tool_asset()`, so every tool
   in `TOOL_CHANGE_TARGETS`) sometimes applies its own automatic correction via the Metrics
   Assembler (`get_metrics_assembler_interface().check_layers()` -- same mechanism the GUI's
   content-browser drag-drop uses, confirmed by reading `isaacsim.core.utils.stage`'s source),
   and a manual `local_scale=[0.001]*3` on top of that double-scales to 1e-6, not the intended
   0.001.** `electric_screwdriver.usd`/the original suction-only `suction gripper.usd` (what
   `SUCTION_GRIPPER_USD` named before both tools were swapped to their "_with_tool_female" CAD
   variants) needed `local_scale=[1,1,1]` because their own mesh data is pre-scaled at export time
   (no correction needed at all, automatic or manual). `electric_screwdriver_with_tool_female.usd`
   (`SCREWDRIVER_USD`, mesh data raw mm like `robots/accessories/tool rack.usd`) *also* needs
   `local_scale=[1,1,1]` -- confirmed live, reproducibly across repeat runs -- but for the
   opposite reason: the automatic correction reliably fires for this specific file and handles it
   without help. Trying `local_scale=[0.001]*3` on it first gave a real, measured 1e-6 scale
   factor (screwdriver bbox a fraction of a millimeter), not the expected 0.32m-scale success
   `tool rack.usd` got from the identical explicit-0.001 approach when it was still script-spawned
   (both assets have empty/no-scale default-prim xformOps, so the file's own structure doesn't
   predict which path fires). `suction_gripper_with_tool_female.usd` (`SUCTION_GRIPPER_USD`'s
   current asset) also measured correctly at `local_scale=[1,1,1]` (~9x13x11cm world bbox), not
   separately re-tested with an explicit 0.001 to identify which of the two reasons applies here.
   **The only reliable check is empirical: reference it via the actual code path and measure the
   resulting world bbox, never assume from metersPerUnit or a sibling asset's own working value.**

8. **`grasp.py`'s pose-composition helpers (`compute_dependent_world_pose()` et al.) assume a
   scale-free rigid pose, and silently give a wrong-but-plausible answer when that's not true.**
   They read the reference pose via `SingleXFormPrim.get_world_pose()`, which returns only
   translation + rotation — any scale on the reference prim or its ancestors is dropped, not
   flagged. `/World/tool_rack` carries a real `xformOp:scale:unitsResolve` of 0.001 (same Metrics-
   Assembler correction as gotcha 7), so an early version of the rack-relative tool placement that
   composed a local offset through `compute_dependent_world_pose()` gave a real, measured
   ~1000x-too-large result (confirmed live — no error, no NaN, just a wrong number). The eventual
   fix (see the open-issues entry below) sidesteps this class of bug entirely for rack-relative
   tools by making them real children of `TOOL_RACK_PRIM_PATH` — USD's own scene graph folds in
   the parent's FULL transform (translate/rotate/scale), no manual re-derivation needed.
   **`compute_dependent_world_pose()` is still correct for every remaining caller**
   (`ASSEMBLY_RELATIONSHIPS`, tool-changer coupler mates) — none of their reference prims carry
   scale — but it's the wrong tool for anything composed against a prim that might, like a rack or
   fixture referenced from an mm-modeled CAD file.

9. **A waypoint queue can deadlock on waypoint 0 when the new first waypoint
   numerically coincides with the previous sequence's last one.** A
   tool-change queue's first waypoint (hover above the *currently* docked
   tool's rack) is derived from the same rack snapshot as that tool's own
   dock sequence's final retract waypoint, so they can be bit-identical.
   `_step_arm()`'s "only re-plan if the target actually moved" check then
   (correctly) sees no change and never calls `plan_single` — but the queue
   only pops a waypoint once a plan *completes*, so it sat on waypoint 0
   forever, silently blocking every future request. Fixed in
   `teleop._start_motion_queue()` by offsetting the stored
   `target_pose`/`target_orientation` by a large finite `+1.0e6`, forcing the
   delta check to read as "changed". **NaN would not work**: IEEE 754 makes
   every NaN comparison `False`, so `norm(...) > threshold` would never fire
   and re-planning would be blocked permanently instead of forced.

All nine confirmed by actually running the mechanism, not by reading docs
— the first five via `scripts/test_mefron_tool_changer_headless.py`
(which docks/undocks all 3 tools across two full swap cycles and asserts
both the expected joint topology and that the tool's `female_coupler`
frame actually converges to the wrist or rack once PhysX settles); **the
sixth was not caught by that test at all** — it only checks poses/joints,
not cross-robot visual reference integrity, and only became visible when
the scene was opened in the real GUI. Worth remembering next time a
headless test reports all-green: it verifies what it checks, not
"nothing is wrong."

## Open issues (deliberately deferred, matching this repo's existing style
of tracked-but-incomplete features — e.g. the screwdriver's own "mounted
but no screw-driving control wired up yet")

- **C/O (gripper open/close) isn't wired to the docked gripper tool's own
  finger joints.** The gripper tool is a *separate* mini-articulation from
  the main arm's own (`mount_franka_hand_only()`, only rigidly welded via
  the coupler joint) — the old C/O mechanism drove
  `config.GRIPPER_JOINT_NAMES` on the **main arm's own** articulation,
  which no longer has live finger joints at all
  (`remove_parallel_jaw_gripper()` now applies to arm 1 too). Driving the
  docked gripper's actual fingers needs its own `SingleArticulation`
  handle scoped to whichever tool prim is currently docked, rebuilt on
  the same Stop/Play cadence as the main arm's — not implemented this
  pass. `arm["drive_builtin_gripper_joints"]` stays `False`; C/O is a
  harmless no-op until this lands.
- **`SURFACE_GRIPPER_LOCAL_POSITION` needs re-deriving.** It assumes the
  suction cup sits directly on `panda_hand`; now it docks beyond the
  coupler's 20mm standoff. Same hand-jog-then-read-back methodology as
  `docs/grasp-and-assembly-offsets.md`.
- **cuRobo has no collision awareness of whichever tool is currently
  docked.** Only the male coupler's own constant geometry (Ø63×20mm)
  would need adding to `franka.yml`'s spare `attached_object` link for a
  first pass; each tool's own additional collision volume beyond the
  coupler is a follow-up, consistent with the existing (separately
  tracked) `attach_objects_to_robot()` open issue in CLAUDE.md.
- **Suction and screwdriver are now baked directly into `mefron.usd` via
  the GUI** (`/World/suction_gripper_with_tool_female`,
  `/World/electric_screwdriver_with_tool_female` — direct children of
  `/World`, not of `tool_rack`, confirmed live — see
  `feedback_static_scenery_baked_into_scene` memory), the same way
  `tool_rack`/`main_holder` are: hand-placed and saved, no Python position
  constants at all.
  `config.TOOL_CHANGE_TARGETS[tool]["baked_tool_prim_path"]` points at the
  baked prim; `robot.spawn_dockable_tool()` never references or
  repositions it, only reads its live pose. `rack_prim_path` for these two
  is now a separate, lightweight, non-physics anchor Xform (still spawned
  fresh each run) that `spawn_dockable_tool()` syncs to the baked tool's
  *current* `ComputeLocalToWorldTransform()` every launch — it exists only
  because `park_tool_at_rack()`'s joint needs a static reference point, and
  a real rigid body can't be jointed to its own descendant (`female_coupler`
  lives under the baked tool itself). `dock_position`/`dock_orientation_wxyz`
  are that read-back, kept only because `grasp.py`'s docking math needs a
  world-space target — never a source of truth themselves.

  This replaces two earlier approaches, both abandoned for the same root
  cause (Kit's Property-panel Orient widget decomposes a stored quaternion
  using a different — and itself non-round-tripping — convention than
  `UsdGeom`'s `RotateXYZ` op, confirmed by reproducing the exact mismatch
  offline): (1) computing a world pose via a throwaway scratch prim and
  applying it with `SingleXFormPrim.set_world_pose()` on a *sibling* of
  `tool_rack`, where the typed `rack_local_*` value never matched what the
  panel showed; (2) parenting the tool anchor as a **real child** of
  `TOOL_RACK_PRIM_PATH` and authoring `rack_local_*` directly as its own
  `translate`/`rotateXYZ` ops — this fixed the Property-panel mismatch (no
  quaternion round-trip involved), but still required the user to type
  numbers into `config.py` and trust the sim to place them correctly,
  which is exactly the class of problem hand-placing in the GUI sidesteps
  entirely. Gripper hasn't had the same treatment yet (still script-spawned
  from `GRIPPER_TOOL_HAND_ONLY_USD` at a fixed placeholder `dock_position`/
  `dock_orientation_wxyz`) — baking it in later just means adding its own
  `baked_tool_prim_path`, no further code changes needed.
  `female_coupler_local_*` for all 3 remain unmeasured.
## Screw pick-and-place (the screwdriver's own action keys)

The screwdriver is no longer inert: with it docked, `config.SCREW_PICK_KEY`
(number-row **5**) picks the presented screw and `SCREW_PLACE_KEY` (**6**)
carries it to the next `main_holder` hole and leaves it there. **No
screw-driving rotation** — deliberately out of scope, the fastener just gets
deposited. Two keys rather than one cycle so the pick can be inspected before
committing to the placement.

### A screw is always joint-fixed to something

Exactly the invariant `robot.park_tool_at_rack()` states for tools, which
makes this whole feature a direct analogue of dock/undock — and means a screw
can never free-fall, so it never depends on `main_holder`'s collider (whose
convex-decomposition tuning is still an open issue):

| Stage | Joint path | body0 | body1 |
|---|---|---|---|
| presented | `screw_<i>/presenter_joint` | `presenter_anchor_<i>` (non-physics anchor) | screw |
| carried | `screw_<i>/tip_joint` | `panda_hand` | screw |
| placed | `screw_<i>/hole_joint` | `hole_anchor_<i>` (non-physics anchor) | screw |

Three rules, each inherited from a gotcha above:

- **A distinct joint path per stage**, never redefined in place (gotcha 5).
- **Zero-snap welds.** `_create_tool_fixed_joint()` defaults both local frames
  to the bodies' own origins, so a joint only welds cleanly when the two
  frames already coincide. The presenter and hole anchors are therefore
  *placed at the screw's own live pose* — the same trick
  `park_tool_at_rack()` uses. The tip joint can't use an anchor (a joint
  whose body0 isn't a rigid body anchors to the **world**, frozen, so it
  wouldn't follow the arm), so it instead passes a live-measured
  `body0_local_*` from `compute_relative_pose(panda_hand, screw)`.
- **body0 for the tip joint is `panda_hand`, not the docked tool prim.** The
  tool prim carries a 0.001 `unitsResolve` scale, and whether PhysX reads a
  joint's `localPos` in scaled or unscaled units is exactly the ambiguity
  gotcha 8 warns about. `panda_hand` is a real rigid body at unit scale, and
  the tool is rigidly welded to it, so the two are physically equivalent —
  this just avoids the trap. `attach_surface_gripper_physics()` already
  targets `panda_hand` the same way.

The screw itself gets `RigidBodyAPI` plus an explicit `MassAPI` (2 g, small
non-zero diagonal inertia — it has no colliders for PhysX to derive either
from) and **no colliders at all**: nothing to fight the holder's mesh
collider (gotcha 2), and nothing needs contact since every stage is
joint-driven.

### Two constants that are CAD-derived, not hand-jogged

- **`SCREWDRIVER_TIP_LOCAL_POSITION` = 274.854 mm along the docked tool
  root's local +Z.** `SCREWDRIVER_USD`'s own mesh points reach exactly that
  z, and the geometry there is axisymmetric (x/y centroid 0.00, symmetric
  ±6.99 mm 5 mm back from the tip), so the bit is on-axis. The asset's
  female coupler head occupies local z 0→10 mm, confirming +Z runs
  coupler→tip. Supersedes the `assembly` branch's guessed 265 mm
  `panda_hand`-frame value.
- **`SCREW_HOLES` = the four real mounting pockets.** `main_holder`'s
  `tn__CutExtrude51..54` collider sub-meshes are 6.65 mm square, 20 mm deep,
  entering at its top face (local z=0): centres `(±85.675, +57.955)` and
  `(±85.675, −56.045)` mm. Identity `local_orientation_wxyz` on all four,
  because `main_holder`'s own world rotation is already 180° about X, so a
  screw's local +Z (its tip) comes out pointing down into the pocket.

Both are stored in **metres**, matching `ASSEMBLY_RELATIONSHIPS`: these
offsets are composed against `SingleXFormPrim.get_world_pose()`, which drops
`main_holder`'s 0.001 scale (gotcha 8 again).

`SCREW_CARRY_LOCAL_POSITION` is one screw-length past the tip with **identity**
orientation — the placeholder screw asset's origin is its tip with the body
running back along local −Z, so its +Z already agrees with the tool's and the
head's outer face lands exactly on the bit tip with no twist.

### Reach is the binding constraint

The tool hangs 275 mm below the wrist, so a hole at world z≈0.99 needs the
wrist at z≈1.277 — about 0.78 m from `MOUNT_POSITION` for the far pair
(x≈3.229) against the Panda's ~0.855 m envelope. A 0.15 m hover (what the
tool rack uses) would push those hover waypoints to ~0.874 m, **past reach**;
hence `SCREW_APPROACH_CLEARANCE = 0.05` (worst hover ≈0.807 m).
`test_mefron_screw_headless.py` prints each leg's distance from the mount and
warns past the envelope. If `plan_single success=False` still appears for the
far holes, the fix is scene layout — move `main_holder`/the jig closer via the
GUI or the conveyor — or a smaller clearance, not code.

### Presenter: the same baked-or-fallback duality as the tools

`config.SCREW_PRESENTER_PRIM_PATH` (`/World/screw_presenter`) is a stand-in
for the real screw presenter. If that prim already exists in `mefron.usd`
(hand-placed in the GUI, per the `feedback_static_scenery_baked_into_scene`
memory), its live pose wins and `SCREW_PRESENTER_FALLBACK_*` is ignored —
identical to `TOOL_CHANGE_TARGETS`' `baked_tool_prim_path` vs
`dock_position`. Its pose composed with `SCREW_PRESENTER_SEAT_LOCAL_*` *is* the
presented screw's pose (the ground truth a real presenter would define), which
is why `grasp.compute_reference_world_pose()` had to be revived: the tool's pick
pose is derived *from* it by inverse composition, not the other way round. The
presenter prim is never deleted by the script, unlike everything under
`SCREW_SCOPE_PRIM_PATH` (including the per-screw `presenter_anchor_<i>`), which
`robot.clear_screws()` wipes every run.

As of the CAD presenter baked in by commit 621bad4, the prim origin is **not**
the seat: `robots/accessories/screw presenter.usd` is 126 × 216 × 153 mm with
its origin at the base-plate centre, so `SCREW_PRESENTER_SEAT_LOCAL_POSITION`
carries the seat offset — `(6.66, -86.00, 72.00)` mm in the presenter's own
scale-free frame, stored in metres like `SCREW_HOLES`. That prim's baked
`xformOp:orient` is identity, which would present the screw head-down, so
`SCREW_PRESENTER_SEAT_LOCAL_ORIENTATION_WXYZ` carries the 180°-about-X flip
(GUI-verified) that puts the tip in the seat and the head up. Without it the
pick pose put the tool root ~0.29 m *below* the seat, inside the table; with it
the wrist sits at z≈1.253, 0.772 m from `MOUNT_POSITION` — inside the ~0.855 m
envelope, ~0.80 m once `SCREW_APPROACH_CLEARANCE` hovers above it.

### Open issues specific to screws

- **A placed screw is welded to a static world anchor, so it does not follow
  `main_holder` if the jig moves afterward.** Accepted for now, and exactly
  what `park_tool_at_rack()` already does for tools. Jointing directly to
  `main_holder` (a real rigid body) would fix it but reintroduces the
  scaled-body `localPos` ambiguity above — worth doing once that convention
  is confirmed live.
- **cuRobo has no collision awareness of the carried screw or the
  presenter**, consistent with the tool-collision and
  `attach_objects_to_robot()` open issues.
- **`screw_m3.usd` is not real CAD** — two plain cylinders at M3 nominal
  dimensions, standing in until a real fastener drawing exists.
