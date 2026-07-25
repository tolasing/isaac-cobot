"""Mounting the Franka onto its UR10 pedestal, plus gripper physics tuning (friction material,
drive stiffness) for a stable grasp."""

from __future__ import annotations

import tempfile
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.usd
from isaacsim.core.prims import SingleXFormPrim
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from import_cr5 import import_cr5

from . import config

# Pre-MovePrim intermediate path every mount_franka() import lands at -- if a stray Save catches a
# session mid-import, /panda gets baked into mefron.usd as an orphaned leftover.
_STRAY_HISTORICAL_PANDA_PATH = "/panda"


def clear_stray_robot_prims() -> None:
    """Deletes any pre-existing robot prims (config.ROBOT_*_PRIM_PATH, plus the historical stray
    /panda path) sitting in the stage the moment open_stage() returns -- leftovers baked into
    mefron.usd by a past session's stray Save. Must run right after open_stage(), before the
    settle pump -- see docs/mefron-history.md for why mount_franka()'s own cleanup is too late."""
    stage = omni.usd.get_context().get_stage()
    stray_paths = [
        path
        for path in (
            config.ROBOT_PRIM_PATH,
            config.ROBOT_2_PRIM_PATH,
            config.ROBOT_3_PRIM_PATH,
            _STRAY_HISTORICAL_PANDA_PATH,
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
    """Mounts cuRobo's bundled Franka Panda at prim_path/mount_position (arm 1's constants by
    default; arms 2/3 pass their own ROBOT_N_PRIM_PATH/MOUNT_N_POSITION/ORIENTATION). Mounting a
    second/third Franka crashes Kit's URDF importer if the full-experience extensions are already
    loaded -- enable those only after all arms are mounted. See docs/mefron-history.md."""
    from curobo.util_file import get_assets_path, join_path

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(prim_path).IsValid():
        # A stray Save can persist a prior robot to disk -- without this, import_cr5's MovePrim
        # silently uniquifies to e.g. /World/Franka_01 instead of landing on prim_path.
        omni.kit.commands.execute("DeletePrims", paths=[prim_path])
        # Frame pump so the deletion commits before import_cr5()'s own uniqueness check runs --
        # otherwise a still-in-flight delete leaves both the old and new robot behind.
        omni.kit.app.get_app().update()

    urdf_path = Path(join_path(get_assets_path(), config.FRANKA_URDF_RELATIVE_PATH))
    import_cr5(
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


# Same hand/panda_leftfinger/panda_rightfinger/ee_link subtree as franka_panda.urdf's, rooted at a
# free-floating base_link instead of panda_link8. Mesh filenames are baked in as absolute paths at
# generation time (see write_hand_only_urdf()), so this template doesn't need to live next to the
# original's meshes/.
_HAND_ONLY_URDF_TEMPLATE = """<?xml version="1.0" ?>
<robot name="panda_gripper_only">
  <link name="base_link"/>
  <joint name="panda_hand_joint" type="fixed">
    <parent link="base_link"/>
    <child link="panda_hand"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <link name="panda_hand">
    <visual><geometry><mesh filename="{hand_visual}"/></geometry></visual>
    <collision><geometry><mesh filename="{hand_collision}"/></geometry></collision>
  </link>
  <link name="panda_leftfinger">
    <visual><geometry><mesh filename="{finger_visual}"/></geometry></visual>
    <collision><geometry><mesh filename="{finger_collision}"/></geometry></collision>
  </link>
  <link name="panda_rightfinger">
    <visual>
      <origin rpy="0 0 3.14159265359" xyz="0 0 0"/>
      <geometry><mesh filename="{finger_visual}"/></geometry>
    </visual>
    <collision>
      <origin rpy="0 0 3.14159265359" xyz="0 0 0"/>
      <geometry><mesh filename="{finger_collision}"/></geometry>
    </collision>
  </link>
  <joint name="panda_finger_joint1" type="prismatic">
    <parent link="panda_hand"/>
    <child link="panda_leftfinger"/>
    <origin rpy="0 0 0" xyz="0 0 0.0584"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="10.0"/>
    <limit effort="20" lower="0.0" upper="0.04" velocity="0.2"/>
  </joint>
  <joint name="panda_finger_joint2" type="prismatic">
    <parent link="panda_hand"/>
    <child link="panda_rightfinger"/>
    <origin rpy="0 0 0" xyz="0 0 0.0584"/>
    <axis xyz="0 -1 0"/>
    <dynamics damping="10.0"/>
    <limit effort="20" lower="0.0" upper="0.04" velocity="0.2"/>
  </joint>
  <link name="ee_link"/>
  <joint name="ee_fixed_joint" type="fixed">
    <parent link="panda_hand"/>
    <child link="ee_link"/>
    <origin rpy="0 0 0" xyz="0 0 0.1"/>
  </joint>
</robot>
"""


def write_hand_only_urdf() -> Path:
    from curobo.util_file import get_assets_path, join_path

    meshes_root = Path(join_path(get_assets_path(), "robot/franka_description/meshes"))
    urdf_text = _HAND_ONLY_URDF_TEMPLATE.format(
        hand_visual=meshes_root / "visual" / "hand.dae",
        hand_collision=meshes_root / "collision" / "hand.obj",
        finger_visual=meshes_root / "visual" / "finger.dae",
        finger_collision=meshes_root / "collision" / "finger.obj",
    )
    urdf_path = Path(tempfile.gettempdir()) / "mefron_hand_only.urdf"
    urdf_path.write_text(urdf_text)
    return urdf_path


def mount_franka_hand_only(prim_path: str) -> str:
    """Imports just panda_hand/panda_leftfinger/panda_rightfinger/ee_link (no arm) from the same
    cuRobo mesh files mount_franka() uses, rooted at a free-floating base_link. Does not touch stage
    selection or delete a stale prim at prim_path first -- callers needing that (e.g. re-running into an
    already-open session) should do it themselves, same as mefron_gripper_probe.py's spawn_gripper_probe()."""
    urdf_path = write_hand_only_urdf()
    return import_cr5(
        urdf_path=urdf_path,
        prim_path=prim_path,
        default_drive_strength=config.FRANKA_DRIVE_STRENGTH,
        default_position_drive_damping=config.FRANKA_DRIVE_DAMPING,
    )


def remove_parallel_jaw_gripper(prim_path: str = config.ROBOT_2_PRIM_PATH) -> None:
    """Deactivates (not deletes) the Franka's parallel-jaw finger links + drive joints on
    prim_path, for an arm converted to a suction end-effector (see attach_suction_gripper()).
    DeletePrims silently no-ops for these specific prims instead -- see docs/mefron-history.md."""
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


def hide_hand_housing(prim_path: str = config.ROBOT_2_PRIM_PATH) -> None:
    """Makes prim_path's panda_hand/visuals invisible for an arm converted to suction-only.
    Visibility only -- panda_hand and its collisions stay active (cuRobo's ee_link; dropping
    collision geometry would change planning, not just looks). See docs/mefron-history.md for why
    the nearest instance root gets un-shared first."""
    stage = omni.usd.get_context().get_stage()
    visuals_path = f"{prim_path}/panda_hand/visuals"
    prim = stage.GetPrimAtPath(visuals_path)
    if not prim.IsValid():
        print(f"[mefron_lib] WARNING: {visuals_path} not found -- skipping hide.", flush=True)
        return

    ancestor = prim
    while ancestor.IsValid():
        if ancestor.IsInstance():
            print(
                f"[mefron_lib] {visuals_path}: un-instancing shared prototype at {ancestor.GetPath()} before hiding.",
                flush=True,
            )
            ancestor.SetInstanceable(False)
            break
        ancestor = ancestor.GetParent()

    UsdGeom.Imageable(prim).MakeInvisible()


def attach_suction_gripper(prim_path: str = config.ROBOT_2_PRIM_PATH) -> None:
    """References config.SUCTION_GRIPPER_USD as a child of panda_hand (cuRobo's ee_link) so it
    rides along rigidly. Does not remove/hide the Franka's hand -- call
    remove_parallel_jaw_gripper()/hide_hand_housing() first. Disables the asset's own baked-in
    collider/RigidBodyAPI below -- see docs/mefron-history.md for why."""
    from isaacsim.core.utils.stage import add_reference_to_stage

    gripper_prim_path = f"{prim_path}/panda_hand/{config.SUCTION_GRIPPER_PRIM_NAME}"
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(gripper_prim_path).IsValid():
        # Same re-run safety as mount_franka() above -- avoid a uniquified duplicate on a second run
        # in the same session.
        omni.kit.commands.execute("DeletePrims", paths=[gripper_prim_path])
        omni.kit.app.get_app().update()

    add_reference_to_stage(usd_path=str(config.SUCTION_GRIPPER_USD), prim_path=gripper_prim_path)
    xform = SingleXFormPrim(prim_path=gripper_prim_path)
    xform.set_local_pose(
        translation=np.array(config.SUCTION_GRIPPER_LOCAL_POSITION),
        orientation=np.array(config.SUCTION_GRIPPER_LOCAL_ORIENTATION_WXYZ),
    )

    gripper_prim = stage.GetPrimAtPath(gripper_prim_path)
    for prim in Usd.PrimRange(gripper_prim):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)


def attach_screwdriver_gripper(prim_path: str = config.ROBOT_3_PRIM_PATH) -> None:
    """References config.SCREWDRIVER_USD as a child of panda_hand (cuRobo's ee_link), scaled to
    config.SCREWDRIVER_LOCAL_SCALE. Mounts the tool visually/for collision only -- no
    screw-driving control wired up yet. Disables baked-in CollisionAPI/RigidBodyAPI the same
    defensive way attach_suction_gripper() does (not separately confirmed needed here)."""
    from isaacsim.core.utils.stage import add_reference_to_stage

    tool_prim_path = f"{prim_path}/panda_hand/{config.SCREWDRIVER_PRIM_NAME}"
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(tool_prim_path).IsValid():
        # Same re-run safety as mount_franka() above -- avoid a uniquified duplicate on a second run
        # in the same session.
        omni.kit.commands.execute("DeletePrims", paths=[tool_prim_path])
        omni.kit.app.get_app().update()

    add_reference_to_stage(usd_path=str(config.SCREWDRIVER_USD), prim_path=tool_prim_path)
    xform = SingleXFormPrim(prim_path=tool_prim_path)
    xform.set_local_scale(np.array(config.SCREWDRIVER_LOCAL_SCALE))
    xform.set_local_pose(
        translation=np.array(config.SCREWDRIVER_LOCAL_POSITION),
        orientation=np.array(config.SCREWDRIVER_LOCAL_ORIENTATION_WXYZ),
    )

    tool_prim = stage.GetPrimAtPath(tool_prim_path)
    for prim in Usd.PrimRange(tool_prim):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)


def attach_surface_gripper_physics(prim_path: str = config.ROBOT_2_PRIM_PATH) -> str:
    """Authors the real isaacsim.robot.schema/surface_gripper attach mechanism on panda_hand: one
    UsdPhysics.Joint (IsaacAttachmentPointAPI) with PhysicsLimitAPI/DriveAPI compliance tuning --
    without it the joint is a free D6 and lifting leaves the object behind. excludeFromArticulation
    is required since panda_hand is a real link. Values/rationale: docs/mefron-history.md."""
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
    # Not robot_schema.ApplyAttachmentPointAPI(): that helper calls Classes.ATTACHMENT_POINT_API.name --
    # the plain Enum's Python identifier, "ATTACHMENT_POINT_API" -- instead of .value (the real schema
    # name, "IsaacAttachmentPointAPI"); every sibling Apply*() in that module correctly uses .value,
    # only this one doesn't. Authoring the real token directly instead.
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
    """Authors one high-friction physics material and binds it to the Franka's fingertip links and
    HIGH_FRICTION_PRIM_PATHS. Runtime-only (never persisted via stage.Save()); re-authored fresh every run.
    The friction material itself is shared/re-authored regardless of prim_path (it's idempotent), only
    the fingertip link paths bound to it are arm-specific."""
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
        drive.CreateStiffnessAttr().Set(config.GRIPPER_DRIVE_STIFFNESS)
        drive.CreateDampingAttr().Set(config.GRIPPER_DRIVE_DAMPING)
