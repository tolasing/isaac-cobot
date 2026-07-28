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
   0.001.** `electric_screwdriver.usd`/`SUCTION_GRIPPER_USD` need `local_scale=[1,1,1]` because
   their own mesh data is pre-scaled at export time (no correction needed at all, automatic or
   manual). `electric_screwdriver_with_tool_female.usd` (the screwdriver's own current
   `SCREWDRIVER_USD`, mesh data raw mm like `robots/accessories/tool rack.usd`) *also* needs
   `local_scale=[1,1,1]` -- confirmed live, reproducibly across repeat runs -- but for the
   opposite reason: the automatic correction reliably fires for this specific file and handles it
   without help. Trying `local_scale=[0.001]*3` on it first gave a real, measured 1e-6 scale
   factor (screwdriver bbox a fraction of a millimeter), not the expected 0.32m-scale success
   `tool rack.usd` got from the identical explicit-0.001 approach when it was still script-spawned
   (both assets have empty/no-scale default-prim xformOps, so the file's own structure doesn't
   predict which path fires). **The only reliable check is empirical: reference it via the actual
   code path and measure the resulting world bbox, never assume from metersPerUnit or a sibling
   asset's own working value.**

All six confirmed by actually running the mechanism, not by reading docs
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
- **Rack dock/approach positions and female-coupler local offsets are
  still mostly placeholder constants** pending the user hand-jogging them
  in the GUI and reading back the transform — exactly how `MOUNT_POSITION`
  and every `ASSEMBLY_RELATIONSHIPS` entry were derived. The screwdriver's
  `dock_position`/`dock_orientation_wxyz` are a first step past that: composed
  from `/World/tool_rack`'s own transform (now baked into `mefron.usd`, not
  script-spawned) plus a local offset on it, so it rests on the physical
  rack instead of floating at a guessed world point — still not hand-jog-
  confirmed against the actual tool, and gripper/suction haven't had the
  same treatment yet. `female_coupler_local_*` for all 3 remain unmeasured.
- Screwdriver gets no new action key (matches its pre-ATC "mounted but
  inert" state) — only dockability via numpad 3.
