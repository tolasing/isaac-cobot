"""Mounting the Franka onto mefron.usd's SEKTION cabinet plate, and the gripper physics tuning
(friction material, drive stiffness) needed for a stable grasp.
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import numpy as np
import omni.kit.commands
import omni.usd
from isaacsim.core.prims import SingleXFormPrim
from pxr import UsdPhysics

from import_cr5 import import_cr5

from . import config


def mount_franka() -> None:
    from curobo.util_file import get_assets_path, join_path

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(config.ROBOT_PRIM_PATH).IsValid():
        # mefron.usd is meant to stay Franka-free -- the robot only ever exists in this runtime
        # session -- but a stray Save can persist it to disk anyway. Without this, import_cr5's
        # MovePrim silently uniquifies to e.g. /World/Franka_01 instead of landing on ROBOT_PRIM_PATH,
        # leaving a duplicate robot behind on every subsequent run. Same pattern as
        # mefron_gripper_probe.py's spawn_gripper_probe().
        omni.kit.commands.execute("DeletePrims", paths=[config.ROBOT_PRIM_PATH])

    urdf_path = Path(join_path(get_assets_path(), config.FRANKA_URDF_RELATIVE_PATH))
    import_cr5(
        urdf_path=urdf_path,
        prim_path=config.ROBOT_PRIM_PATH,
        default_drive_strength=config.FRANKA_DRIVE_STRENGTH,
        default_position_drive_damping=config.FRANKA_DRIVE_DAMPING,
    )
    xform = SingleXFormPrim(prim_path=config.ROBOT_PRIM_PATH)
    xform.set_world_pose(
        position=np.array(config.MOUNT_POSITION),
        orientation=np.array(config.MOUNT_ORIENTATION_WXYZ),
    )


# Same hand/panda_leftfinger/panda_rightfinger/ee_link subtree as franka_panda.urdf's, just rooted at a
# free-floating base_link instead of panda_link8 (dropping panda_hand_joint's -45 degree yaw so
# base_link == panda_hand's own frame, since there's no arm frame left to stay consistent with).
# Mesh filenames are baked in as absolute paths resolved from cuRobo's own assets at generation time
# (see write_hand_only_urdf()), so this template doesn't need to live next to the original's meshes/.
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


def apply_gripper_friction() -> None:
    """Authors one high-friction physics material and binds it to the Franka's fingertip links and
    HIGH_FRICTION_PRIM_PATHS. Runtime-only (never persisted via stage.Save()); re-authored fresh every run."""
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

    target_paths = [f"{config.ROBOT_PRIM_PATH}/{name}" for name in config.GRIPPER_FINGER_LINK_NAMES]
    target_paths += config.HIGH_FRICTION_PRIM_PATHS
    for prim_path in target_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            print(f"[mefron_lib] WARNING: {prim_path} not found -- skipping friction bind.", flush=True)
            continue
        add_physics_material_to_prim(stage, prim, config.GRIPPER_FRICTION_MATERIAL_PATH)


def stiffen_gripper_drive() -> None:
    """Raises the finger joints' position-drive stiffness/damping above the whole-robot import-time
    default, so their maxForce budget isn't left mostly unused."""
    stage = omni.usd.get_context().get_stage()
    for joint_name in config.GRIPPER_JOINT_NAMES:
        joint_prim = stage.GetPrimAtPath(f"{config.ROBOT_PRIM_PATH}/joints/{joint_name}")
        if not joint_prim.IsValid():
            print(f"[mefron_lib] WARNING: {joint_prim.GetPath()} not found -- skipping stiffen.", flush=True)
            continue
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "linear")
        drive.CreateStiffnessAttr().Set(config.GRIPPER_DRIVE_STIFFNESS)
        drive.CreateDampingAttr().Set(config.GRIPPER_DRIVE_DAMPING)
