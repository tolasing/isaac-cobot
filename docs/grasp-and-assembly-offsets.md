# Deriving grasp and assembly offsets

> **2026-08-08 cleanup.** The grasp-editor and probe scripts this document
> refers to (`mefron_grasp_editor_scene.py`, `franka_grasp_editor_scene.py`,
> `panda_hand_grasp_editor_scene.py`, `mefron_gripper_probe.py`,
> `mefron_screen_approach_probe.py`) were deleted; the derivations they produced
> are what this document records, and are still in `config.py`. The release-weld
> code moved from `robot.py` to `mefron_lib/assembly.py`, unchanged. See
> `docs/mefron-history.md`'s header for the full list.

How the fixed relative transforms used by `scripts/mefron.py`'s **G**/**P**
snap-to-pose keys were derived: `ASSEMBLY_RELATIONSHIPS["finger_print_scanner_on_main_holder"]`
(nicknamed **T_H_S** — `finger_print_scanner`'s pose relative to
`main_holder` at the correctly assembled position) and
`GRASP_OFFSET_POSITION`/`GRASP_OFFSET_ORIENTATION_WXYZ` (nicknamed
**T_S_G** — the gripper's grasp pose relative to `finger_print_scanner`).
Moved out of `CLAUDE.md` to keep that file focused on current state; see
`CLAUDE.md` for the current values in use.

## Method: `compute_relative_pose()`, not hand Euler-angle conversion

Both transforms were derived **live, by script, not by hand**: manually
jog/align the two relevant prims to a visually-confirmed good pose in the
Isaac Sim GUI, then read back both prims' resulting **world poses** and
compute the relative transform between them via a `compute_relative_pose()`
helper (uses
`isaacsim.core.utils.numpy.rotations.quats_to_rot_matrices`/
`rot_matrices_to_quats` — confirmed via direct source read to be
**scalar-first, wxyz**). Hand Euler-angle conversion was tried first and
produced a confirmed-wrong rotation earlier in this investigation, for an
unrelated pose — don't fall back to it.

Deriving T_H_S requires temporarily reparenting `finger_print_scanner`
under `main_holder` in the Stage tree to dial in exact visual alignment.
This only works with `mefron.usd` opened directly
(`omni.usd.get_context().open_stage()`, what `scripts/mefron.py` does) —
it hits a "Cannot move/rename ancestral prim" restriction in
`scripts/build_scene_mefron.py`'s referenced-stage session (which brings
`mefron.usd` in via `add_reference_to_stage()` instead). This is the reason
active grasp/assembly tuning work happens in `scripts/mefron.py`, not
`build_scene_mefron.py`, even though the latter is architecturally
preferred for everything else.

## T_H_S: `finger_print_scanner` relative to `main_holder`

**First derivation**: `local_position=[-0.05765023, 0.02069006, 0.01875005]`,
`local_orientation_wxyz=[0.999973595, -0.00618904850, 0.000842160478,
-0.00371422408]`.

**Re-derived a second time in a later session** — the value above visibly
placed the scanner wrong on the mount ("rederiving it is off from the pos
it is suppose to be at"). Same technique as before (manually re-aligned
`finger_print_scanner` under `main_holder` in the GUI, then
`compute_relative_pose()` on the two prims' resulting world poses), not a
hand-tweak of the old numbers. **Current value**, what `mefron.py`'s
`ASSEMBLY_RELATIONSHIPS` actually holds: `local_position=
[-0.05765001316747483, 0.02068996147910942, 0.01500000425999065]`,
`local_orientation_wxyz=[1.0, 0.0, 0.0, 0.0]`. X/Y moved by well under a
millimeter, but Z dropped from `0.01875` to `0.01500` (the part had been
sitting ~3.75mm too high) and the orientation simplified from a small
residual rotation to a clean identity quaternion — consistent with a more
carefully-aligned re-measurement rather than measurement noise.

## 2026-08-08: `main_holder` relative to `main_holder_jig` (the `G` key)

`main_holder` was until now only ever a *mount*, never a *part* — it sat
wherever the scene author parked it on the packing table. `assets/main_holder.yaml`
(a Grasp-Editor export, `grasp_0`) plus a seat pose supplied directly by hand
made it pickable, so the sequence now starts by dropping the base part into the
jig rather than assuming it is already there. It is wired as an ordinary
`GRASP_TARGETS` entry on **`G`**, and the flow is the same `G` → C → P → O as
every other part; the only structural novelty is that its
`ASSEMBLY_RELATIONSHIPS` entry mounts onto **`main_holder_jig`**, making it the
first relationship whose mount is the belt-driven jig instead of `main_holder`.

### The supplied offset was in millimetres, not metres

Unlike every other entry in this document, this one was **not** derived by
`compute_relative_pose()` on two live world poses — it was given directly as
`x 0, y 0, z -24.0` in the jig's own frame. That frame is millimetre-scale
(`main_holder_jig` carries `xformOp:scale:unitsResolve = (0.001, 0.001, 0.001)`),
while `ASSEMBLY_RELATIONSHIPS` offsets are scale-free **metres** — they are
composed onto `get_world_pose()` results, which drop scale. So the stored value
is `-0.024`, and the conversion was confirmed against scene geometry rather than
assumed:

| | value |
|---|---|
| jig origin (world) | `(3.14315, -4.78217, 0.98658)`, rotated 180° about **Y** |
| jig bbox | 250 × 250 × 88mm, **origin sits on its top face** |
| jig local `+Z` | points world **−Z**, so local `z = -24mm` is 24mm **up** |
| `main_holder` bbox | 182 × 238 × 22mm, **origin sits on its top face** |
| holder origin once seated | world Z `0.98658 + 0.024 = 1.01058` |
| holder underside | `1.01058 − 0.022 = 0.98858` → **2mm above the jig's top face** |

That seats it. Read as centimetres the same number would float the holder 240mm
in the air, so the unit is not in doubt. Same convention as
`main_holder_back_cover_on_main_holder`'s `-0.015`.

### Orientation: 180° about Z, specified not measured

Only a position was supplied, so the relative orientation was a separate call:
**`[0.0, 0.0, 0.0, 1.0]`, 180° about Z**. Identity was written first (the natural
reading of a CAD mate offset) and corrected by hand before anyone ran it.

Why identity was wrong here: the jig is rotated 180° about **Y** in world while
the holder parks at 180° about **X**, and those differ by exactly a 180° yaw
(`R_y(180) = R_z(180)·R_x(180)`). Under identity the holder would have landed
yawed 180° from how it sits on the table. The 180°-about-Z offset cancels that,
so the holder seats in the jig with the world orientation it already has — same
end facing the robot. Geometry does not disambiguate the two: both are flat and
centred (holder and jig are each centred on their own origin in X/Y, and 182×238
clears the 250×250 jig either way), which is exactly why it had to be specified.

No `ASSEMBLY_WELD_POSES` entry was added: `assembly_weld_local_pose()` falls back
to the `ASSEMBLY_RELATIONSHIPS` offset, so P and O agree by construction. Add one
only if the seated holder needs a visual nudge off where P drove, the way the
other four parts did.

### Knock-on effects (and non-effects)

- **Nothing else needed re-deriving.** Every other relationship is expressed in
  `main_holder`'s own frame and composed onto its *live* pose, so the whole
  sub-assembly follows the holder into the jig for free.
- **The weld turns `main_holder`'s collision off**, the default. Left that way
  deliberately: its convex-decomposition collider is still untuned (see
  `CLAUDE.md`'s open issues), and snapping an untuned collider into the jig is
  exactly the interpenetration-shake case the default guards against.
  `ASSEMBLY_WELD_KEEP_COLLISION_PART_PRIM_PATHS` is the opt-out if needed.
- **The anchor chain gains a link**: jig → `anchor_main_holder_jig` (kinematic) →
  `main_holder` → `anchor_main_holder` (kinematic) → sub-parts. Each anchor is
  pose-driven once per frame, so during a conveyor run sub-parts may trail by
  roughly two frames and re-seat when it stops. The anchor being kinematic means
  the holder adds **no** load to the friction-driven jig.
- **Grasp reach is tight**: the yaml puts the ee target at ≈
  `(2.055, -5.232, 1.029)`, 0.809m from `MOUNT_POSITION` against the Panda's
  ~0.855m envelope. A `plan_single` failure on `G` is a scene-layout fix, not a
  code one. The placement pose is comfortable by contrast: ≈
  `(3.056, -4.801, 1.111)`, 0.535m.
- **Unresolved**: the headless ride check fails for this relationship — see
  `CLAUDE.md`'s open issues.

## The official Grasp Editor tool was tried and abandoned for T_S_G

The official `isaacsim.robot_setup.grasp_editor` tool (`GraspSpec`) was
tried first for T_S_G and found **fundamentally unusable for this exact
Franka+`mefron.usd` combination** — abandoned, not worked around. Its
"Select Frames of Reference" dropdown came back permanently empty, and its
separate Joint Settings panel crashed outright with `AttributeError:
'NoneType' object has no attribute 'is_active'`.

Two wrong hypotheses were ruled out live first: not a UI refresh-timing
issue (retyping the filter field didn't help), and not a
dual-`SingleArticulation` ownership conflict with a running teleop loop
(built a separate scene with no teleop loop or cuRobo running at all,
`scripts/mefron_grasp_editor_scene.py` — dropdown was still empty).

**Actual confirmed root cause**, found via a direct diagnostic script that
bypassed the Grasp Editor UI entirely: the Franka's own articulation/DOF
resolution works fine (`dof_names` populates correctly, matching that the
SEKTION cabinet's identical-mechanism articulation also works) but
`Usd.PrimRange(art.prim)` — which the Grasp Editor's own
dropdown-population code uses — finds **zero** Xformable descendants under
the Franka. Traced to the URDF importer's file-backed-stage "layered Robot
Description" mechanism (see `docs/mefron-history.md`'s `assets/mefron/`
entry) producing genuinely broken internal cross-references for this
specific Franka in this specific file, confirmed via persistent "Could not
open asset"/"Unresolved reference prim path" warnings on every fresh,
freshly-cleared import — not just after a crash.

`scripts/franka_grasp_editor_scene.py`/`scripts/mefron_grasp_editor_scene.py`
remain in the repo as working diagnostic artifacts for future parts, in
case the Grasp Editor is worth retrying against a from-scratch stage for a
robot/asset combination that doesn't hit this same layered-import bug.

`scripts/panda_hand_grasp_editor_scene.py` combines that anonymous-stage
workaround with `mefron_gripper_probe.py`'s trimmed hand-only URDF (no
arm), for testing Grasp Editor against just the hand — but the full-arm
URDF's `panda_leftfinger`/`panda_rightfinger` meshes fail to resolve at
all on an anonymous stage (confirmed empty via a direct `Usd.PrimRange`
check, not just a visibility flag), a separate issue from the
layered-import bug above.

## T_S_G: gripper grasp pose relative to `finger_print_scanner`

**First derivation** — via `compute_relative_pose()` on the Franka's
`ee_link` and `finger_print_scanner`'s live world poses at a
manually-jogged, visually-confirmed good grasp, not via the Grasp Editor.
Confirmed the result is a real, physically-sensible transform, not a
derivation error: its near-1 component landed in the *last* slot
(`w≈0.99999`) rather than the first, initially looking suspicious next to
T_H_S's own result — double-checked directly against
`isaacsim/core/utils/numpy/rotations.py`'s own source (not assumed) and
confirmed `rot_matrices_to_quats` really is scalar-first, confirming this
is a legitimate ~180-degree rotation about the scanner's own local Z axis
(the gripper approaches from above; the scanner's CAD-authored local frame
has its own flipped axis convention relative to that approach direction),
not a bug. Value: `GRASP_OFFSET_POSITION=[0.01277519, -0.02169126,
-0.02863107]`, `GRASP_OFFSET_ORIENTATION_WXYZ=[-0.000518294608,
-0.00348700255, 0.000751325308, 0.999993504]`.

**Re-derived in a later session**, same technique (manually jog the
gripper to a fresh visually-confirmed good grasp, then
`compute_relative_pose()` on the live poses), not a hand-tweak. **Current
value**, what `mefron.py` actually holds: `GRASP_OFFSET_POSITION=
[0.00027002069774515104, -0.021693730387954874, -0.1271989186209571]`,
`GRASP_OFFSET_ORIENTATION_WXYZ=[-2.1523912431273915e-05,
-8.089888886539503e-06, 5.762411090611313e-06, 0.9999999997190347]` — a
near-identity rotation (`w≈1`) rather than the earlier
~180-degree-about-Z one, reflecting a different jog approach angle this
time, not a convention change.

## Wiring: `compute_grasp_approach_pose()` / `compute_assembly_grasp_target()`

Both T_H_S and T_S_G are wired into two pose functions and two keybindings,
table-position-independent by construction: `compute_grasp_approach_pose()`/
`compute_assembly_grasp_target()` each re-read the live world pose of
`finger_print_scanner`/`main_holder` on every call and compose it with the
fixed relative transforms above via a `compute_dependent_world_pose()`
helper (the forward direction of `compute_relative_pose()`), so neither
function depends on where the parts happened to be sitting when T_H_S/T_S_G
were derived. `GripperKeyboardControl` has two one-shot request/consume
method pairs (`request_grasp_approach()`/`consume_grasp_approach_request()`,
and the `_assembly_target` equivalents) wired to **G**/**P** keys in
`build_gripper_keyboard_control()`.

**Real bug found and fixed, caught by a headless regression test rather
than assumed working**: the first version placed the G/P snap-consumption
block *before* `run_teleop_loop()`'s own `past_pose`/`target_pose is None`
bootstrap block. On the very first eligible frame of a call where a request
was already pending, the snap fired first, so `target.get_world_pose()`
read back the *already-snapped* pose, and `target_pose` got bootstrapped
from that same post-snap value — making the debounce's
`norm(cube_position - target_pose)` distance check exactly zero, forever,
for that entire call. The snap itself worked (the target prim really did
move), but `motion_gen.plan_single()` was never even called — confirmed via
a headless test (`scripts/test_mefron_assembly_headless.py`) whose
pose-sanity checks passed (grasp-approach/assembly-target poses both landed
a plausible ~4-5cm from their reference objects) while its full run
produced **zero** occurrences of the `"plan_single"` log line across ~280
frames per phase. Three independent agents adversarially re-derived this
exact root cause from the live code before the fix was applied, and all
three converged on the same diagnosis and fix. Fixed by moving the
bootstrap block to run first (seeding the baseline from the true pre-snap
pose), then applying the snap and reassigning the local
`cube_position`/`cube_orientation` to the post-snap values so the rest of
that frame's logic sees the fresh pose. **Verified live** after the fix:
`plan_single success=True` for both phases, with real joint-position
deltas (`1.8159` rad for the grasp-approach move, `0.6809` rad more for
the subsequent assembly-placement move).

## The release weld: O/L pins an assembled part at its nominal pose

Added 2026-08-06, and the reason the placement-accuracy problems below are no
longer blocking. Two symptoms were being chased at once — parts slipping off
`main_holder` under gravity once the gripper opened, and parts landing visibly
off their assembled pose (cuRobo's ~3mm residual plus the still-open
grasp-centering problem below). **Neither is worth fixing for this scene**: it
exists for visual understanding of the automation pipeline, not as a source of
VLA training data, so the accepted answer is to make the assembled result
*look* right rather than to make the arm *place* it right.

So on release — **O** with the gripper docked, **L** with the suction cup —
`robot.weld_part_at_assembly_pose()` snaps the part onto the exact nominal pose
`compute_part_target_pose()` derives from `ASSEMBLY_RELATIONSHIPS`, and
joint-fixes it there. Same `UsdPhysics.FixedJoint` mechanism as the tool
changer's dock and `weld_screw_into_hole()`'s pocket, and the same invariant:
an assembled part is always jointed to something.

Three properties worth knowing:

- **Proximity-gated** (`config.ASSEMBLY_WELD_MAX_DISTANCE`, 5cm). Past that the
  release is ordinary — so aborting a grasp mid-air with O doesn't teleport the
  part onto the jig from across the cell. The distance is printed either way.
- **Reversible.** Pressing a grasp/approach key (J/B/K, N/M) for an object
  calls `release_assembly_weld()` first, deleting the joint and re-enabling its
  collision, so an assembled part can be picked back up. State is the joint
  prim's presence on the stage, not a Python variable, so it survives a
  Stop/Play.
- **The welded part's colliders are disabled** while welded, matching the
  docked-tool precedent (`docs/tool-changer.md` gotcha 2) and sidestepping
  `main_holder`'s untuned convex-decomposition collider, which already makes
  parts sink.

### Why a per-mount kinematic anchor, not a direct joint to `main_holder`

A part must ride `main_holder` when the conveyor moves the jig — a placed
screw's static world anchor was explicitly rejected here for that reason. But
jointing straight to `main_holder` means handing PhysX a `localPos` on a body
carrying a 0.001 `unitsResolve` scale, which is exactly the unresolved
ambiguity `docs/tool-changer.md`'s gotcha 8 warns about (scaled or unscaled
units? unconfirmed, and a wrong guess is a silent 1000x error).

`_ensure_assembly_anchor()` sidesteps the question instead of answering it. Per
mount prim, once, it creates a body at the mount's `get_world_pose()` —
scale-free by construction, so every part then welds onto an *unscaled* anchor
where metres mean metres. The part's offset on it needs no measurement: it is
`ASSEMBLY_RELATIONSHIPS[name]["local_position"/"local_orientation_wxyz"]`
verbatim, since that constant already expresses the part's pose in the mount's
own frame. `pcb_assembly_on_backpanel_support` falls out for free —
`backpanel_support` gets its own anchor once welded, giving the chain
`main_holder → anchor → backpanel_support → anchor → pcb_assembly`.

**That anchor is kinematic, and `sync_assembly_anchors()` drives it onto the
mount's live pose every teleop frame.** The first version instead made it a
dynamic 2 g body (copying `present_screw()`'s mass treatment) and joined it to
the mount with a second `FixedJoint` at identity local frames — elegant, since
a zero offset is scale-invariant and carries no ambiguity either. **It failed
live**: a `FixedJoint` is only as rigid as the mass ratio across it, and a 2 g
anchor holding a real CAD part off a ~5.7 cm lever sagged and rotated until the
part hung through `main_holder` (which its own disabled colliders no longer
stopped). Raising the anchor's mass would fix the ratio but put phantom
kilograms on a jig the conveyor moves by surface friction. Kinematic is
infinite mass to the solver with no dynamics of its own, so the weld is rigid
regardless of what the part weighs, and it tracks the mount exactly rather than
through a constraint that can lag.

One gotcha worth remembering from building it: a `stage.DefinePrim(path,
"Xform")` prim has **no** `xformOp`s at all, and `SingleXFormPrim(...,
reset_xform_properties=False).set_world_pose()` only writes existing ops — on a
freshly defined prim it raises `Empty typeName for ...xformOp:translate`
(confirmed live, crashed the teleop loop). The ops have to be authored once by
the default `reset_xform_properties=True` path first, which is safe on a prim
this module created — the reason every *scene* prim here passes `False` is to
preserve a `unitsResolve` scale op that a script-made anchor never has.

### The tradeoff, stated plainly

Same one the screws already carry: **a clean-looking assembly is no longer
evidence the arm arrived accurately.** The part is snapped from wherever it was
left, so a wrong `ASSEMBLY_RELATIONSHIPS` constant now shows up as a
confidently-wrong assembly rather than a near-miss, and the real placement error
is only visible in the printed correction distance.

`scripts/test_mefron_assembly_weld_headless.py` covers the mechanics (weld,
gate, gravity hold, riding a moved `main_holder`, un-weld).

## 2026-08-06/07: release weld, split poses, and both bugs closed

Started as a mid-investigation snapshot (the `d50b6ff` commit carrying it was
marked TEMPORARY, with two open bugs). **Both are now closed** — bug 1 was
disproved outright and bug 2 was root-caused live; see their sections below.
What landed:

### Landed

- **Two pose sets, on purpose.** `ASSEMBLY_RELATIONSHIPS` is what P *drives*
  to (motion-validated, unchanged); `config.ASSEMBLY_WELD_POSES` is where the
  O/L weld *seats* the part. `grasp.assembly_weld_local_pose()` /
  `compute_part_weld_pose()` resolve the weld's own value, falling back to the
  relationship when there's no override. The weld's snap pose and its joint's
  `body0_local_*` must both come from that resolver or the joint drags the part
  off what it just snapped to.
- **`ASSEMBLY_WELD_POSES` values** were measured in one pass from a single
  hand-placed assembly (all parts on the jig at once, then read back relative
  to their own mounts), so they're mutually consistent. `main_holder_back_cover`
  has no entry — it wasn't on the jig for that measurement. Gap vs P's pose,
  i.e. how far a part visibly jumps on release: `backpanel_support` ~12mm,
  `screen` ~7mm, `pcb_assembly` ~4mm, `finger_print_scanner` ~3mm.
- **The weld disables the part's colliders again** — dropped on 2026-08-06 for
  breaking re-grasping (fingers pass through) and the SurfaceGripper's attach
  (nothing to detect), then reinstated on 2026-08-07 with the symmetric
  re-enable those failures were actually missing: `release_assembly_weld()`
  restores collision, and `clear_assembly_welds()` still repairs it on load
  (needed because the URDF importer rewrites `mefron.usd` every run and would
  otherwise persist the disable into a session where nothing is welded). Why it
  had to come back: see the sighting section below.

### CLOSED bug 1 (2026-08-07): sleeping bodies, not a stuck suction joint

The original diagnosis — "the SurfaceGripper never really lets go, PhysX keeps
the constraint after the USD side is cleared" — is **wrong in both halves**.

**`body1=[]` was never evidence.** `SurfaceGripperComponent.h`'s `UsdActionType`
has exactly three members: `WriteStatus`, `WriteGrippedObjectsAndFilters`,
`WriteAttachmentPointBatch`. The manager **never writes `physics:body1`** — it
swaps PhysX actors directly — so that rel reads empty before, during and after
every grip, on a healthy gripper.

**The release works.** A minimal headless 1-DOF-gantry scene reusing
`attach_surface_gripper_physics()`'s exact authoring (same limits, drives,
`IsaacAttachmentPointAPI`, `excludeFromArticulation`) reproduces the symptom:
after `open_gripper()` the gripped box hangs in mid-air with `status=Open` and
`gripped=[]`. But with the joint untouched and still enabled, one velocity write
through the tensors API sends it into free fall (`box_z` +0.0557 → −21.73,
`vz` −20.62) and it never snaps back. Nothing was holding it — the body was
**asleep**, and neither enabling gravity nor cutting a joint wakes a sleeping
PhysX actor. A free control box falling to −207m in the same run is what keeps
that harness honest; an earlier version unsupported the box by toggling the
ground's `collisionEnabled`, which doesn't propagate at runtime, and every
conclusion drawn from it was void.

Ruled out — don't retry:

- **Authoring `physics:body1`** to a parking body the way the shipped
  `SurfaceGripper_gantry.usda` does. No effect on release.
- **Deleting the attachment joint prim, or `jointEnabled=False`,** after the
  open. Neither drops the object — the sleeping body is why, not a constraint.
- **Re-authoring the attachment joint mid-run** to force a release. Actively
  harmful: it re-creates prims under `panda_hand`, a live articulation link,
  rebuilding the articulation and making the arm go haywire (confirmed live).

### What the "welded part follows the wrist" sighting was

The same sleeping-body mechanism on the assembly side.
`weld_part_at_assembly_pose()` snaps the part onto its nominal pose — a real
correction, ~6mm for the screen — which leaves it **interpenetrating its mount**,
and pins it to a **kinematic** anchor, i.e. infinite mass to the solver. That
overlap has no way to resolve gently. It sits dormant while the scene is
quiescent, then arm motion (numpad 1's traverse to the tool rack is the one that
does it) wakes the bodies and the stored penetration discharges at once as a
violent shake, which reads as the assembly being dragged along.

Two wrong turns on the way there, both from stating inference as measurement:
that the arm was colliding with the jig, and that `main_holder` was being
"shoved". Neither was measured; both were rejected from live observation. What
did settle it was the user's own reading — the shake only appears once the arm
moves, i.e. once the sleeping bodies wake.

**Fix, and why this shape.** The weld now disables the part's collision
(`_set_prim_collision_enabled(part_prim_path, False)`, after the joint is
created), restored by `release_assembly_weld()` and by `clear_assembly_welds()`
on load. A placed part is final in this workflow and the joint alone holds it —
collision contributes nothing to holding a body welded to a kinematic anchor.
`d50b6ff` had tried the same disable and reverted it for breaking re-grasping and
the SurfaceGripper's attach; what makes it correct now is the **symmetric
re-enable** on un-weld, which teleop already triggers before every grasp
(J/B/K) and every suction approach (N/M). Known cost: a part placed later won't
rest on an already-welded one — it passes through until its own weld fires.

`UsdPhysics.FilteredPairsAPI` between part and mount was the alternative, and
would have kept colliders live; dropped as unnecessary once "placed is final"
was confirmed.

### CLOSED bug 2: `plan_single` failed because of `main_holder_jig`

Found live by the user: cuRobo's refusal came from **`main_holder_jig` being in
`config.OBSTACLE_PRIM_PATHS`**, not from the weld. Everything the earlier probe
ruled out stays ruled out — the arm was never mechanically pinned, and the weld
joint and kinematic anchor were both correct. cuRobo was refusing because of
that obstacle, so no `status`/`valid_query` instrumentation was needed after all.

**Still open as a consequence:** `config.OBSTACLE_PRIM_PATHS` remains the debug
value `["/World/ConveyorBelt_A06_01"]`, so `main_holder_jig` and
`tool_rack_gripper` are out of cuRobo's collision world and the arm will plan
straight through them. Restoring `main_holder_jig` naively re-breaks planning;
it needs the cuboid-approximation treatment `docs/mefron-history.md` already
prescribes for the conveyor CAD.

## Open problem: grasp-centering (not a joint asymmetry)

**Still open, not yet fixed, confirmed to NOT be a per-finger joint/drive
asymmetry.** Reviewing a screen recording of a grasp-close, the object
visibly shifted sideways as the fingers closed. A per-finger drive/mimic-
joint asymmetry between `panda_finger_joint1`/`panda_finger_joint2` looked
plausible and was about to be investigated as the cause. **The user
corrected this diagnosis directly**: "i wouldnt say fixed since the central
mount is closer to the right finger joint it reached first and then the
left joint comes" — i.e. `finger_print_scanner` isn't equidistant from both
fingertips at the moment closing begins (a grasp-*pose centering* issue),
so one finger contacts and starts pushing the object before the other one
arrives, rather than both sides closing onto it symmetrically.

This remains unresolved — no fix has been attempted. Two directions were
discussed but neither started: re-derive `GRASP_OFFSET_POSITION` checking
explicitly that it's equidistant from both fingertips at grasp time, or
derive it from the gripper's own finger-midpoint frame instead of `ee_link`
directly. **The per-finger joint-asymmetry hypothesis was explicitly
rejected by the user — don't re-investigate it without new evidence.**

Placement after the most recent T_H_S/T_S_G re-derivation still lands
close on X but visibly off on Y — likely this same grasp-centering problem
rather than a T_S_G derivation error, but not explicitly confirmed as the
same root cause versus a second, independent issue.

## Alternative method (not used): SolidWorks-side extraction

`solidworks_transform_extraction.md` (repo root) documents an alternative,
SolidWorks-side method (Coordinate Systems + Measure, or direct mate
values) for extracting the `finger_print_scanner`→`main_holder` relative
transform (`T_part_target`) at the CAD-authoring stage, instead of deriving
it live in Isaac Sim. **Superseded in practice** by the live
`compute_relative_pose()` approach documented above — kept as a reference
for a CAD-side alternative, not part of the executed pipeline.

## 2026-08-08: rationale migrated out of code comments

The 2026-08-08 cleanup capped every comment and docstring in
`scripts/mefron_lib/` at two lines. Assembly-weld rationale that lived
inline and had no home here yet was moved into this section rather than
dropped. The tool-changer and screw equivalents are in
`docs/tool-changer.md`'s section of the same name.

### `ASSEMBLY_WELD_POSES`' provenance

Measured together on 2026-08-06 from **one** hand-placed assembly — every
part sitting on the jig at once, then read back — so the entries are
mutually consistent rather than each measured in its own session. Float
noise below 1e-16 was cleaned to exact zeros/identity.

A relationship with no entry here welds at its `ASSEMBLY_RELATIONSHIPS`
pose instead; `main_holder_back_cover` is the only such case, because it
wasn't on the jig for that measurement.

The two dicts differing **is** the point — `P` drives to the
motion-validated `ASSEMBLY_RELATIONSHIPS` pose while the release weld seats
at `ASSEMBLY_WELD_POSES` — but they must stay within
`ASSEMBLY_WELD_MAX_DISTANCE` of each other: the gap is exactly how far the
part visibly jumps on release. Worst case here is `backpanel_support` at
12 mm.

### `weld_part_at_assembly_pose()`: collision must go off *after* the joint

The snap leaves the part interpenetrating its mount, an overlap a kinematic
anchor can never resolve. It sits quiet until the arm's motion wakes the
bodies, then discharges as a violent shake — the same sleeping-body effect
documented in the closed suction-release bug above. Disabling collision
before authoring the joint doesn't help; the ordering is what matters. See
`docs/tool-changer.md`'s gotcha 2 for the general form.

`body0_local_*` on the weld joint **is** the offset just snapped to: it
already expresses the part's pose in the mount's frame, and the anchor is
unscaled and kept coincident with that frame. It must come from the same
source as the snap target, or the joint pulls the part straight back off it.

### `clear_assembly_welds()` re-enables collision unconditionally

The URDF importer rewrites `mefron.usd` on every run, so a disabled
`collisionEnabled` left behind by a release weld would persist into a fresh
run — where nothing is welded and every part must be grippable. Nothing
else in the codebase turns part collision off, so re-enabling
unconditionally on load is safe.

### `_ensure_assembly_anchor()`'s xform-op handling

This is the one place in `mefron_lib` that uses `SingleXFormPrim`'s default
`reset_xform_properties=True`. A `DefinePrim`'d Xform has no `xformOps` at
all and only this path authors them — `sync_assembly_anchors()` merely
writes to them afterward. Safe on a prim this module created, which can't
carry a `unitsResolve` op to strip. Everywhere else must pass
`reset_xform_properties=False`, since the CAD prims do carry one.
