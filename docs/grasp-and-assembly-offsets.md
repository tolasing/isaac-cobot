# Deriving grasp and assembly offsets

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
