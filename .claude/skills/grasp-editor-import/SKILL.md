---
description: Manually importing a robot/gripper into the NVIDIA Grasp Editor tutorial scene (assets/Grasp_Editor/) via the GUI URDF importer. Use when adding a new gripper/end-effector variant to the Grasp Editor, or when Select-Frames-of-Reference doesn't show real geometry after an import.
---

# Getting a new gripper working in the Grasp Editor

The Grasp Editor extension is picky about articulation structure in ways
that aren't obvious from the URDF importer's own options. This is the
recipe that got the CR5+PGC-140 gripper working, after a scripted
(`import_cr5()`-based) equivalent crashed the extension outright.

## Why the scripted import path doesn't work here

`import_cr5()`'s default `fix_base=True` authors a real `PhysicsFixedJoint`
anchoring the robot to the world, with `ArticulationRootAPI` on *that
joint*. The Grasp Editor's `ui_builder.py` calls `populate_subframes()`,
which walks `Usd.PrimRange(self._articulation.prim)` — **descendants
only**. If `ArticulationRootAPI` sits on a joint or on a link that isn't a
genuine ancestor of every other link (the URDF importer places every link
as a flat sibling under the robot's root Xform regardless of kinematic
parent/child), the real geometry links are invisible to Select-Frames-of-
Reference, and selecting the articulation in the Grasp Editor can crash
with `AttributeError: 'NoneType' object has no attribute 'link_names'`.

The working Franka avoids this because `ArticulationRootAPI` sits on
`/World/panda_hand`, a plain Xform ancestor of the hand *and* both
fingers, with **no `RigidBodyAPI`** on that same prim.

## The recipe

1. Hand-edit the combined URDF: prepend a `dummy_link`/`dummy_joint` pair
   as the new root (only needed as an importer entry point, not the final
   structure).
2. Import via the Isaac Sim GUI's own URDF Importer, not
   `URDFParseAndImportFile`/`import_cr5()` directly:
   - **Links → Moveable Base** (`fix_base=False`) — this alone fixes the
     crash-on-selection.
   - Output as a "Referenced Model" into `assets/Grasp_Editor/Isaac/
     Robots/<Vendor>/<name>/`. This produces a layered `configuration/`
     folder (a `_base.usd`/`_physics.usd`/`_robot.usd` split) — this is how
     the importer behaves for *any* file-backed target stage, GUI or
     scripted, not specific to this asset.
3. Move `ArticulationRootAPI`/`PhysxArticulationAPI` off `dummy_link` and
   onto the real root container prim (e.g. `/cr5_pgc140_robot`), mirroring
   `panda_hand`'s pattern exactly: root container prim, no `RigidBodyAPI`
   on it.
4. Add a plain `UsdPhysics.Joint` (`rootJoint`, `body1` empty, identity
   local frames) — without this the gripper falls under gravity, since
   Moveable Base means nothing else anchors it.
5. Re-apply everything `import_cr5()` would normally have handled for a
   scripted import, since none of it carries over for a manual GUI import:
   - `disableGravity=True` on any link (e.g. fingers) that shouldn't fall
   - Drive `stiffness`/`damping`/`maxForce` on every actuated joint (read
     back after setting — see the `robot-import-checklist` skill, item 5)
   - Self-collision `FilteredPairsAPI` between links that legitimately
     overlap at some commanded position (e.g. two fingers fully closed) —
     check *this specific rig's* geometry for pairs beyond whatever a
     shared cuRobo config's `self_collision_ignore` already covers.
   - `solverPositionIterationCount`/`solverVelocityIterationCount` on the
     articulation *and* on each rigid body — velocity iterations default
     to `1`, which is a real bottleneck for simultaneous multi-contact
     grasps (e.g. two fingers closing on one part at once).
6. If you're adding an object for the gripper to grasp (e.g. a test
   part), make sure its `CollisionAPI` is on the actual mesh geometry, not
   a parent Xform — see the `audit-colliders` skill for the full pattern,
   including the "GUI edit lands in whichever file is open" gotcha, which
   applies here too (the Grasp Editor scene and the gripper's own
   `configuration/` layer are separate files).

## Verify

Open the scene, select the articulation, confirm Select-Frames-of-
Reference now lists the real links (not just `dummy_link`'s own empty
`visuals`/`collisions`). Then author a grasp and confirm both actuated
joints (e.g. both gripper fingers) actually reach their commanded targets
symmetrically — a stall on one side with the object's collision well
within its visual bounds usually means an ghost/duplicate collider on the
*grasped object*, not the gripper (see `audit-colliders`).
