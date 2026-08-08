"""The Franka arm itself: mounting it on its UR10 pedestal, stripping its own hand for the ATC,
and gripper physics tuning (friction material, drive stiffness). See docs/mefron-history.md."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.usd
from isaacsim.core.prims import SingleXFormPrim
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from . import config
from .usd_util import import_urdf, un_instance_ancestor

# Pre-MovePrim intermediate paths a mount_franka() import can land at, plus the retired pre-ATC
# arm2/arm3 paths -- a stray Save bakes any of these into the shared mefron.usd as an orphan.
_STRAY_HISTORICAL_PANDA_PATH = "/panda"
_STRAY_HISTORICAL_GRIPPER_TOOL_PATH = "/panda_gripper_only"
_STRAY_HISTORICAL_ARM_PATHS = ["/World/Franka2", "/World/Franka3"]


def clear_stray_robot_prims() -> None:
    """Deletes robot prims a past session's stray Save baked into mefron.usd. Must run right after
    open_stage(), before the settle pump -- mount_franka()'s own cleanup is too late."""
    stage = omni.usd.get_context().get_stage()
    stray_paths = [
        path
        for path in (
            config.ROBOT_PRIM_PATH,
            _STRAY_HISTORICAL_PANDA_PATH,
            _STRAY_HISTORICAL_GRIPPER_TOOL_PATH,
            *_STRAY_HISTORICAL_ARM_PATHS,
        )
        if stage.GetPrimAtPath(path).IsValid()
    ]
    if not stray_paths:
        return
    print(
        f"[mefron_lib] clearing stray robot prim(s) left over from a past session's stray Save: {stray_paths}",
        flush=True,
    )
    omni.kit.commands.execute("DeletePrims", paths=stray_paths)
    omni.kit.app.get_app().update()


def mount_franka(
    prim_path: str = config.ROBOT_PRIM_PATH,
    mount_position=config.MOUNT_POSITION,
    mount_orientation_wxyz=config.MOUNT_ORIENTATION_WXYZ,
) -> None:
    """Mounts cuRobo's bundled Franka Panda at prim_path/mount_position. Must run BEFORE
    kit_experience.enable_full_experience_extensions() -- the URDF importer crashes otherwise."""
    from curobo.util_file import get_assets_path, join_path

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(prim_path).IsValid():
        # Without this, import_urdf's MovePrim silently uniquifies to e.g. /World/Franka_01. The
        # frame pump lets the delete commit before the importer's own uniqueness check runs.
        omni.kit.commands.execute("DeletePrims", paths=[prim_path])
        omni.kit.app.get_app().update()

    urdf_path = Path(join_path(get_assets_path(), config.FRANKA_URDF_RELATIVE_PATH))
    import_urdf(
        urdf_path=urdf_path,
        prim_path=prim_path,
        default_drive_strength=config.FRANKA_DRIVE_STRENGTH,
        default_position_drive_damping=config.FRANKA_DRIVE_DAMPING,
    )
    xform = SingleXFormPrim(prim_path=prim_path)
    xform.set_world_pose(
        position=np.array(mount_position),
        orientation=np.array(mount_orientation_wxyz),
    )


def remove_parallel_jaw_gripper(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Deactivates (not deletes) the Franka's own finger links + drive joints, so the ATC's tools
    can replace them. DeletePrims silently no-ops for these -- see docs/mefron-history.md."""
    stage = omni.usd.get_context().get_stage()
    paths = [f"{prim_path}/{name}" for name in config.GRIPPER_FINGER_LINK_NAMES] + [
        f"{prim_path}/joints/{name}" for name in config.GRIPPER_JOINT_NAMES
    ]
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"[mefron_lib] WARNING: {path} not found -- skipping deactivation.", flush=True)
            continue
        prim.SetActive(False)


def hide_hand_housing(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Hides panda_hand/visuals and disables panda_hand/collisions -- the ATC's hidden housing under
    the male coupler. Both sub-scopes are separately instanceable; see docs/mefron-history.md."""
    stage = omni.usd.get_context().get_stage()
    visuals_path = f"{prim_path}/panda_hand/visuals"
    prim = stage.GetPrimAtPath(visuals_path)
    if not prim.IsValid():
        print(f"[mefron_lib] WARNING: {visuals_path} not found -- skipping hide.", flush=True)
        return

    un_instance_ancestor(prim, visuals_path)
    UsdGeom.Imageable(prim).MakeInvisible()

    collisions_path = f"{prim_path}/panda_hand/collisions"
    collisions_prim = stage.GetPrimAtPath(collisions_path)
    if not collisions_prim.IsValid():
        print(f"[mefron_lib] WARNING: {collisions_path} not found -- skipping collision disable.", flush=True)
        return
    un_instance_ancestor(collisions_prim, collisions_path)
    for p in Usd.PrimRange(collisions_prim):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)


def attach_surface_gripper_physics(prim_path: str = config.ROBOT_PRIM_PATH) -> str:
    """Authors the surface_gripper attach mechanism on panda_hand: one UsdPhysics.Joint with
    compliance tuning -- without it lifting leaves the object behind. See docs/mefron-history.md."""
    from usd.schema.isaac import robot_schema

    stage = omni.usd.get_context().get_stage()
    hand_path = f"{prim_path}/panda_hand"
    joint_path = f"{hand_path}/{config.SURFACE_GRIPPER_JOINT_PRIM_NAME}"
    gripper_path = f"{hand_path}/{config.SURFACE_GRIPPER_PRIM_NAME}"

    for path in (joint_path, gripper_path):
        if stage.GetPrimAtPath(path).IsValid():
            omni.kit.commands.execute("DeletePrims", paths=[path])
    omni.kit.app.get_app().update()

    joint = UsdPhysics.Joint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([hand_path])
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*config.SURFACE_GRIPPER_LOCAL_POSITION))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(*config.SURFACE_GRIPPER_LOCAL_ORIENTATION_WXYZ))

    joint_prim = joint.GetPrim()
    # Not robot_schema.ApplyAttachmentPointAPI(): it passes the Enum's Python identifier rather than
    # .value (the real schema name), unlike every sibling Apply*() in that module.
    joint_prim.AddAppliedSchema("IsaacAttachmentPointAPI")
    joint_prim.CreateAttribute("isaac:forwardAxis", Sdf.ValueTypeNames.Token, custom=False).Set("Z")
    joint_prim.CreateAttribute("isaac:clearanceOffset", Sdf.ValueTypeNames.Float, custom=False).Set(0.008)

    for axis in ("transX", "transY"):
        limit = UsdPhysics.LimitAPI.Apply(joint_prim, axis)
        limit.CreateLowAttr().Set(1.0)
        limit.CreateHighAttr().Set(-1.0)  # low > high == locked, per PhysicsLimitAPI's own schema doc

    limit_z = UsdPhysics.LimitAPI.Apply(joint_prim, "transZ")
    limit_z.CreateLowAttr().Set(0.0)
    limit_z.CreateHighAttr().Set(0.01)
    drive_z = UsdPhysics.DriveAPI.Apply(joint_prim, "transZ")
    drive_z.CreateStiffnessAttr().Set(5000.0)
    drive_z.CreateDampingAttr().Set(100.0)

    for axis, stiffness in (("rotX", 100.0), ("rotY", 100.0), ("rotZ", 10000.0)):
        limit = UsdPhysics.LimitAPI.Apply(joint_prim, axis)
        limit.CreateLowAttr().Set(-3.0)
        limit.CreateHighAttr().Set(3.0)
        UsdPhysics.DriveAPI.Apply(joint_prim, axis).CreateStiffnessAttr().Set(stiffness)

    gripper_prim = robot_schema.CreateSurfaceGripper(stage, gripper_path)
    gripper_prim.GetAttribute(robot_schema.Attributes.MAX_GRIP_DISTANCE.name).Set(config.SURFACE_GRIPPER_MAX_GRIP_DISTANCE)
    gripper_prim.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name).SetTargets([joint_path])
    return gripper_path


def apply_gripper_friction(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Authors one high-friction physics material and binds it to the fingertip links and
    HIGH_FRICTION_PRIM_PATHS. Runtime-only, re-authored fresh every run; never persisted."""
    from omni.physx.scripts import utils as physx_utils
    from omni.physx.scripts.physicsUtils import add_physics_material_to_prim

    stage = omni.usd.get_context().get_stage()
    physx_utils.addRigidBodyMaterial(
        stage,
        config.GRIPPER_FRICTION_MATERIAL_PATH,
        staticFriction=config.GRIPPER_STATIC_FRICTION,
        dynamicFriction=config.GRIPPER_DYNAMIC_FRICTION,
        restitution=0.0,
    )

    target_paths = [f"{prim_path}/{name}" for name in config.GRIPPER_FINGER_LINK_NAMES]
    target_paths += config.HIGH_FRICTION_PRIM_PATHS
    for target_path in target_paths:
        prim = stage.GetPrimAtPath(target_path)
        if not prim.IsValid():
            print(f"[mefron_lib] WARNING: {target_path} not found -- skipping friction bind.", flush=True)
            continue
        add_physics_material_to_prim(stage, prim, config.GRIPPER_FRICTION_MATERIAL_PATH)


def stiffen_gripper_drive(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Raises the finger joints' position-drive stiffness/damping above the whole-robot import-time
    default, so their maxForce budget isn't left mostly unused."""
    stage = omni.usd.get_context().get_stage()
    for joint_name in config.GRIPPER_JOINT_NAMES:
        joint_prim = stage.GetPrimAtPath(f"{prim_path}/joints/{joint_name}")
        if not joint_prim.IsValid():
            print(f"[mefron_lib] WARNING: {joint_prim.GetPath()} not found -- skipping stiffen.", flush=True)
            continue
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "linear")
        drive.CreateTypeAttr().Set(config.GRIPPER_DRIVE_TYPE)
        drive.CreateStiffnessAttr().Set(config.GRIPPER_DRIVE_STIFFNESS)
        drive.CreateDampingAttr().Set(config.GRIPPER_DRIVE_DAMPING)
