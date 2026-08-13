# Grasp Editor assets

## `cr5_pgc140_gripper/`

A **gripper-only** PGC-140 USD for NVIDIA's Grasp Editor
(`isaacsim.robot_setup.grasp_editor`). Despite the name it contains no CR5
geometry at all — only `pgc140_base_link` and the two finger links.

- **Origin:** the `dobot` branch, `d425280` ("Add standalone CR5+PGC-140
  gripper-only USD for the Grasp Editor tutorial"), copied here unchanged.
  Filenames are kept verbatim because `configuration/*_physics.usd` sublayers
  `configuration/*_base.usd` by relative name; renaming breaks composition.
  Only the parent directory changed (`assets/Grasp_Editor/Isaac/Robots/CR5/` →
  here), which is safe: every internal path is relative. Verified after the move
  — 3 meshes, 34410 points, 2 prismatic joints, nothing unresolved.
- **Built from** `robots/pgc140/urdf/pgc140_robot.urdf`, itself vendored from
  DH-Robotics — see [`../../robots/pgc140/SOURCE.md`](../../robots/pgc140/SOURCE.md).
- **Shape matters.** `PhysicsArticulationRootAPI` sits on a plain Xform with a
  *non-constraining* `rootJoint`, i.e. a genuinely free body. That is deliberate:
  a `fix_base=True` import instead crashed the Grasp Editor's
  `initialize_objects()` with `AttributeError: 'NoneType' object has no attribute
  'link_names'`. Do not "fix" the missing fixed joint.
- Finger joints are already drive-tuned in the asset (linear, stiffness 10000,
  damping 1000), so the undamped-import trap that bit `robots/fr5/` does not
  apply here.

```
/cr5_pgc140_robot                        ArticulationRoot
  pgc140_base_link                       RigidBody
  pgc140_finger1_link                    RigidBody
  pgc140_finger2_link                    RigidBody
  joints/pgc140_finger{1,2}_joint        Prismatic, axis X, limits (0, 0.025)
  rootJoint                              plain PhysicsJoint, constrains nothing
  FingerFrictionPhysicsMaterial
```

**Open is q=0.0, closed is q=0.025** — inverted versus the Franka hand, and
confirmed live on the `dobot` branch by computing each finger link's world
position at both extremes. Getting this backwards silently locks the gripper shut
during planning. When exporting a grasp, `cspace_position` (closed) is therefore
the **larger** number and `pregrasp_cspace_position` (open) the smaller.

## `pgc_finger_print_scanner.yaml`

One real PGC-140 grasp for `finger_print_scanner`, exported against the USD above
on the `dobot` branch (`a15a021`). Kept as a format reference and a starting
point — note it lists **both** finger joints, where the Franka-era yamls in
`assets/*.yaml` list only `panda_finger_joint1`.
