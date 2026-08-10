"""The O/L release weld: snapping a released part onto its nominal assembly pose and joint-fixing
it to a mount-tracking kinematic anchor. Full design: docs/grasp-and-assembly-offsets.md."""

from __future__ import annotations

import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.usd
from isaacsim.core.prims import SingleXFormPrim
from pxr import Gf, Sdf, UsdPhysics

from . import config, feeder
from .usd_util import collision_enabled_flags, create_fixed_joint, set_prim_collision_enabled

# Which mount each anchor tracks, stored on the anchor itself so sync_assembly_anchors() needs no
# Python-side bookkeeping and stays correct across a Stop/Play.
_ASSEMBLY_ANCHOR_MOUNT_ATTR = "mefron:assemblyWeldMountPath"


def _assembly_anchor_path(mount_prim_path: str) -> str:
    return f"{config.ASSEMBLY_WELD_SCOPE_PRIM_PATH}/anchor_{Sdf.Path(mount_prim_path).name}"


# One joint path per part, never redefined in place -- docs/tool-changer.md's gotcha 5.
def _assembly_weld_joint_path(part_prim_path: str) -> str:
    return f"{config.ASSEMBLY_WELD_SCOPE_PRIM_PATH}/weld_{Sdf.Path(part_prim_path).name}"


def clear_assembly_welds() -> None:
    """Deletes the script-owned weld scope and re-enables part collision, so a run never inherits a
    previous one's anchors/joints. Why unconditional: docs/grasp-and-assembly-offsets.md."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(config.ASSEMBLY_WELD_SCOPE_PRIM_PATH).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[config.ASSEMBLY_WELD_SCOPE_PRIM_PATH])
        omni.kit.app.get_app().update()

    # Every feeder COPY too, not just config's base paths -- an older run's weld can have left
    # collision off on a copy, and nothing else would ever turn it back on.
    for part_prim_path in {
        *(relationship["part_prim_path"] for relationship in config.ASSEMBLY_RELATIONSHIPS.values()),
        *feeder.all_part_instance_prim_paths(),
    }:
        if any(not enabled for enabled in collision_enabled_flags(part_prim_path)):
            print(
                f"[mefron_lib] {part_prim_path} had collision disabled on load -- re-enabling "
                "(left behind by an older release weld; see this function's docstring).",
                flush=True,
            )
            set_prim_collision_enabled(part_prim_path, True)


def ensure_assembly_anchor(mount_prim_path: str) -> str:
    """The per-mount body every welded part (and placed screw) is jointed to. KINEMATIC and
    pose-driven, not jointed -- a dynamic anchor sagged. See docs/grasp-and-assembly-offsets.md."""
    stage = omni.usd.get_context().get_stage()
    anchor_path = _assembly_anchor_path(mount_prim_path)
    if stage.GetPrimAtPath(anchor_path).IsValid():
        return anchor_path

    # A Scope, not an Xform -- it can't carry a transform, so an anchor's local pose is its world pose.
    stage.DefinePrim(config.ASSEMBLY_WELD_SCOPE_PRIM_PATH, "Scope")
    stage.DefinePrim(anchor_path, "Xform")
    mount_trans, mount_quat = SingleXFormPrim(
        prim_path=mount_prim_path, reset_xform_properties=False
    ).get_world_pose()
    # The one place defaulting reset_xform_properties=True is correct -- this prim was just
    # DefinePrim'd and has no xformOps to strip. See docs/grasp-and-assembly-offsets.md.
    SingleXFormPrim(prim_path=anchor_path).set_world_pose(position=mount_trans, orientation=mount_quat)

    anchor_prim = stage.GetPrimAtPath(anchor_path)
    # Kinematic: infinite mass to the solver, so the weld is rigid whatever the part weighs. Explicit
    # mass anyway -- the anchor carries no colliders to derive one from.
    rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(anchor_prim)
    rigid_body_api.CreateKinematicEnabledAttr().Set(True)
    mass_api = UsdPhysics.MassAPI.Apply(anchor_prim)
    mass_api.CreateMassAttr().Set(config.ASSEMBLY_WELD_ANCHOR_MASS)
    mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*config.ASSEMBLY_WELD_ANCHOR_DIAGONAL_INERTIA))
    anchor_prim.CreateAttribute(_ASSEMBLY_ANCHOR_MOUNT_ATTR, Sdf.ValueTypeNames.String).Set(mount_prim_path)
    return anchor_path


def sync_assembly_anchors() -> None:
    """Drives every anchor onto its mount's live pose each frame -- what makes a welded part ride
    main_holder down the conveyor. get_world_pose() drops the mount's scale, which is the point."""
    stage = omni.usd.get_context().get_stage()
    scope_prim = stage.GetPrimAtPath(config.ASSEMBLY_WELD_SCOPE_PRIM_PATH)
    if not scope_prim.IsValid():
        return
    for anchor_prim in scope_prim.GetChildren():
        mount_attr = anchor_prim.GetAttribute(_ASSEMBLY_ANCHOR_MOUNT_ATTR)
        if not mount_attr.IsValid():
            continue
        # reset_xform_properties=False on both -- a mount carries an xformOp:scale:unitsResolve op
        # the default would silently strip (see grasp.py).
        mount_trans, mount_quat = SingleXFormPrim(
            prim_path=mount_attr.Get(), reset_xform_properties=False
        ).get_world_pose()
        SingleXFormPrim(prim_path=str(anchor_prim.GetPath()), reset_xform_properties=False).set_world_pose(
            position=mount_trans, orientation=mount_quat
        )


def weld_part_at_assembly_pose(relationship_name: str) -> bool:
    """Snaps a released part onto its nominal ASSEMBLY_WELD_POSES pose (NOT the one P drove to) and
    welds it there. Returns False past ASSEMBLY_WELD_MAX_DISTANCE, leaving a far release ordinary."""
    from .grasp import assembly_weld_local_pose, compute_part_weld_pose, relationship_pose_prim_paths

    relationship = config.ASSEMBLY_RELATIONSHIPS[relationship_name]
    # The live copies, not config's base paths -- one source of truth with the pose math above.
    part_prim_path, mount_prim_path = relationship_pose_prim_paths(relationship_name)
    part_xform = SingleXFormPrim(prim_path=part_prim_path, reset_xform_properties=False)
    live_trans, _ = part_xform.get_world_pose()
    target_trans, target_quat = compute_part_weld_pose(relationship_name)
    distance = float(np.linalg.norm(np.array(live_trans) - np.array(target_trans)))
    if distance > config.ASSEMBLY_WELD_MAX_DISTANCE:
        print(
            f"[mefron_lib] {part_prim_path} released {distance:.3f}m from its assembly pose "
            f"(> {config.ASSEMBLY_WELD_MAX_DISTANCE}m) -- not welding.",
            flush=True,
        )
        return False

    anchor_path = ensure_assembly_anchor(mount_prim_path)
    # Re-author the part's own USD xform too, so a Stop restores it ASSEMBLED rather than snapping it
    # back to where it started -- same reason weld_screw_into_hole() does it.
    part_xform.set_world_pose(position=target_trans, orientation=target_quat)
    # body0_local_* IS the offset just snapped to, and must come from the same source, or the joint
    # pulls the part straight back off it. See docs/grasp-and-assembly-offsets.md.
    weld_local_position, weld_local_orientation = assembly_weld_local_pose(relationship_name)
    create_fixed_joint(
        _assembly_weld_joint_path(part_prim_path),
        anchor_path,
        part_prim_path,
        body0_local_position=tuple(weld_local_position),
        body0_local_orientation_wxyz=tuple(weld_local_orientation),
    )
    # From now on this copy IS the assembled one -- mount poses and screw holes must read it, while a
    # grasp key moves on to the next copy on the belt.
    feeder.record_assembled(relationship["part_prim_path"], part_prim_path)
    # Must come AFTER the joint -- the snap leaves the part interpenetrating its mount, which wakes
    # as a violent shake once the arm moves. See docs/grasp-and-assembly-offsets.md.
    # Keyed on the BASE path: a copy must inherit its original's opt-out, not silently lose it.
    keep_collision = (
        feeder.base_part_prim_path_of(part_prim_path) in config.ASSEMBLY_WELD_KEEP_COLLISION_PART_PRIM_PATHS
    )
    if not keep_collision:
        set_prim_collision_enabled(part_prim_path, False)
    print(
        f"[mefron_lib] welded {part_prim_path} at its {relationship_name} pose ({distance:.3f}m "
        f"correction), collision {'KEPT (opted in, see config)' if keep_collision else 'off'}.",
        flush=True,
    )
    return True


def release_assembly_weld(part_prim_path: str) -> bool:
    """Inverse of weld_part_at_assembly_pose(): deletes the weld joint AND restores the collision it
    turned off. Stateless -- the joint prim's presence IS the state, so a Stop/Play can't desync it."""
    stage = omni.usd.get_context().get_stage()
    joint_path = _assembly_weld_joint_path(part_prim_path)
    if not stage.GetPrimAtPath(joint_path).IsValid():
        return False
    omni.kit.commands.execute("DeletePrims", paths=[joint_path])
    omni.kit.app.get_app().update()
    set_prim_collision_enabled(part_prim_path, True)
    # This copy is no longer assembled, so put it back in the running for belt_queue()/mount poses.
    feeder.forget_assembled(part_prim_path)
    print(f"[mefron_lib] released the assembly weld on {part_prim_path}.", flush=True)
    return True
