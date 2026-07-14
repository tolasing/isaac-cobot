---
description: Pre-flight checklist for importing a new (or modified) robot/gripper URDF into cuRobo + Isaac Sim in this repo. Systematizes the CR5+PGC140 bring-up bug list so the same failure modes get caught before a long debug session, instead of rediscovered one at a time.
---

# New robot/gripper URDF import checklist

Every item below is a bug that actually shipped once in this repo and cost
real debugging time. None of these fail loudly at import time — they all
surface later as a confusing, seemingly-unrelated error. Check them
up front, in this order:

1. **Joint velocity limits in the URDF aren't zero.** A `velocity="0"` on
   any joint (common SolidWorks-exporter artifact) loads fine and only
   fails the *first time* `MotionGenConfig.load_from_robot_config()`
   actually builds a full `MotionGen` —
   `curobo/rollout/cost/bound_cost.py`'s `set_bounds()` raises `ValueError:
   Joint velocity limits is zero`. cuRobo reads velocity straight from the
   URDF with **no config-level override** (jerk/acceleration *do* have
   overrides via `cspace.max_jerk`/`max_acceleration`, velocity doesn't).
   Don't trust "loads without error" as proof — it may not have gotten far
   enough to check.

2. **`retract_config` isn't a kinematic singularity.** A "reasonable-
   looking" pose (e.g. all zeros) can still be singular. Cheap to check
   with no cuRobo API needed — finite-difference the FK yourself and look
   at the singular values / condition number:
   ```python
   # perturb each joint by epsilon, recompute FK, build the Jacobian,
   # np.linalg.svd(J) — condition number should be a normal double-digit/
   # low-hundreds number, not >1e4 with near-zero trailing singular values.
   ```
   A singular retract pose makes IK fail for almost any nearby target,
   which looks like a reachability/collision bug, not a pose-choice bug.

3. **Collision spheres clear the mount pedestal/base, accounting for
   `collision_sphere_buffer`.** That buffer *adds* to every sphere's
   declared radius — a sphere that looks tangent to a plane using its raw
   radius will still poke through once the buffer is applied. Check the
   math explicitly, don't eyeball it.

4. **Self-collision-ignore list covers non-adjacent links that overlap at
   retract, not just adjacent joint pairs.** A robot's own joint geometry
   can fold non-adjacent links into genuine overlap even at a sane retract
   pose — this is normal (cuRobo's bundled `ur5e.yml` does the same), but
   it's masked until collision-sphere/world-collision bugs are fixed
   first, so don't assume a clean self-collision-ignore list means no
   phantom self-collision failures are coming.

5. **Read back `UsdPhysics.DriveAPI` after import — don't trust
   `ImportConfig` fields took effect.** `default_drive_strength`/
   `default_position_drive_damping` (and `override_joint_dynamics`) can
   silently not reach the authored joints on this Isaac Sim version —
   confirmed live: requesting `1e5`/`1e4` still produced
   `stiffness=625, damping=0` on every joint. If wrong, re-author
   `UsdPhysics.DriveAPI` directly post-import; don't just adjust the
   `ImportConfig` values and assume it worked.

6. **`self_collision=False` on the importer does not author any USD-level
   exclusion.** Confirmed live: no `PhysxArticulationAPI.
   enabledSelfCollisions` attribute or `FilteredPairsAPI` relationship
   gets created. If you need self-collision filtering, author
   `FilteredPairsAPI` explicitly yourself.

7. **`/World` (or whatever the target's parent is) must exist before
   importing.** `MovePrim` inside an importer wrapper silently no-ops if
   the destination's parent doesn't exist — the real content stays behind
   at a stage-root sibling path matching the URDF's `<robot name=...>`,
   and the intended path is a valid-but-permanently-empty prim. Symptom:
   `AttributeError: 'NoneType' object has no attribute 'is_homogeneous'`/
   `'link_names'` on the next `SingleArticulation(...)` — looks exactly
   like the async-import race in the next item, but more settle frames
   won't fix it. Trace the raw `omni.kit.commands.execute(
   "URDFParseAndImportFile", ...)` return value if unsure.

8. **Pump `simulation_app.update()` after import before building anything
   that reads the stage.** `URDFParseAndImportFile`'s asset population is
   asynchronous — the command returns a prim path immediately, but the
   stage isn't populated yet. A pipeline with a long call in between
   (`motion_gen.warmup()`, ~30s) never notices; a short script that
   imports and immediately builds a `SingleArticulation` does.

9. **Mimic joints on prismatic joints import wrong.** A `<mimic>` tag on a
   linear joint has been observed importing as a *rotational*
   `PhysxMimicJointAPI:rotX` with mangled limits and no `DriveAPI`
   attached at all. If a joint has no drive after import and you expected
   one, check for a stray mimic tag — removing it and driving both
   joints independently is the working pattern already used for both the
   Franka's fingers and the PGC-140's.

10. **If a Fixed-base import crashes the Grasp Editor extension on
    selection**, it's very likely `fix_base=True` authoring
    `ArticulationRootAPI` on a `PhysicsFixedJoint` rather than a plain
    Xform ancestor. Import with a Moveable Base (`fix_base=False`) instead,
    then explicitly move `ArticulationRootAPI` onto a real ancestor Xform
    of every link (mirroring `panda_hand`'s pattern) — see the
    `grasp-editor-import` skill for the full recipe.

After all of the above, run the repo's headless regression test (see the
`headless-test` skill) before touching the GUI — it catches most of 1-8
in under a minute.
