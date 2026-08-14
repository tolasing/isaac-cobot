"""The automatic tool changer: the wrist's permanent male coupler, spawning/parking the three
dockable tools, and docking/undocking them. Full design + gotchas: docs/tool-changer.md."""

from __future__ import annotations

import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.usd
from isaacsim.core.prims import SingleXFormPrim
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from . import config
from .robot import stiffen_gripper_drive
from .usd_util import create_fixed_joint, reference_asset, un_instance_ancestor


def attach_tool_changer_male_coupler(prim_path: str = config.ROBOT_PRIM_PATH) -> str:
    """Authors the ATC's permanent male half as a plain UsdGeom.Cylinder under the FR5's flange.
    Welded on by construction and never detaches -- only the tools docked to it do."""
    stage = omni.usd.get_context().get_stage()
    coupler_prim_path = _male_coupler_prim_path(prim_path)
    if stage.GetPrimAtPath(coupler_prim_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[coupler_prim_path])
        omni.kit.app.get_app().update()

    cylinder = UsdGeom.Cylinder.Define(stage, coupler_prim_path)
    cylinder.CreateRadiusAttr().Set(config.TOOL_CHANGER_CYLINDER_RADIUS)
    cylinder.CreateHeightAttr().Set(config.TOOL_CHANGER_CYLINDER_HEIGHT)
    cylinder.CreateAxisAttr().Set("Z")

    xform = SingleXFormPrim(prim_path=coupler_prim_path)
    xform.set_local_pose(
        translation=np.array(config.TOOL_CHANGER_MALE_LOCAL_POSITION),
        orientation=np.array(config.TOOL_CHANGER_MALE_LOCAL_ORIENTATION_WXYZ),
    )
    return coupler_prim_path


def _tool_prim_path(tool_name: str) -> str:
    target = config.TOOL_CHANGE_TARGETS[tool_name]
    if "baked_tool_prim_path" in target:
        return target["baked_tool_prim_path"]
    return f"{target['rack_prim_path']}/tool"


def _female_coupler_parent_prim_path(tool_name: str) -> str:
    """The prim female_coupler must be a child of -- it has to resolve to a real PhysX rigid body,
    which for a multi-link tool is a named link, not the tool root. docs/tool-changer.md gotchas 3-4."""
    parent_link_name = config.TOOL_CHANGE_TARGETS[tool_name].get("female_coupler_parent_link_name")
    if parent_link_name:
        return f"{_tool_prim_path(tool_name)}/{parent_link_name}"
    return _tool_prim_path(tool_name)


def _female_coupler_prim_path(tool_name: str) -> str:
    return f"{_female_coupler_parent_prim_path(tool_name)}/female_coupler"


def _male_coupler_prim_path(robot_prim_path: str = config.ROBOT_PRIM_PATH) -> str:
    return f"{robot_prim_path}/{config.FR5_EE_LINK}/{config.TOOL_CHANGER_MALE_PRIM_NAME}"


def _demote_tool_articulation(tool_prim_path: str) -> None:
    """Turns a multi-link tool from its own articulation into plain jointed rigid bodies, and frees
    its root. The ATC owns a tool's pose entirely -- it is always FixedJoint'd to a rack or the
    wrist -- and a second articulation welded on chatters then deadlocks. docs/fr5-migration.md."""
    stage = omni.usd.get_context().get_stage()
    tool_prim = stage.GetPrimAtPath(tool_prim_path)
    if tool_prim.IsValid() and tool_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        tool_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        print(f"[mefron_lib] {tool_prim_path}: removed ArticulationRootAPI -- docked as rigid bodies.", flush=True)

    # Both spellings: the URDF importer writes root_joint, the PGC-140's Grasp-Editor asset
    # rootJoint. DeletePrims no-ops on either; SetActive(False) is what works.
    for root_joint_name in ("root_joint", "rootJoint"):
        root_joint_prim = stage.GetPrimAtPath(f"{tool_prim_path}/{root_joint_name}")
        if root_joint_prim.IsValid() and root_joint_prim.IsActive():
            root_joint_prim.SetActive(False)
            print(f"[mefron_lib] {tool_prim_path}/{root_joint_name}: deactivated.", flush=True)


def spawn_dockable_tool(tool_name: str) -> str:
    """Places one of config.TOOL_CHANGE_TARGETS's tools at its rack as a real rigid body, ready to
    be jointed there. All 3 are baked into mefron.usd and only read. See docs/tool-changer.md."""
    target = config.TOOL_CHANGE_TARGETS[tool_name]
    rack_prim_path = target["rack_prim_path"]
    tool_prim_path = _tool_prim_path(tool_name)

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(rack_prim_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[rack_prim_path])
        omni.kit.app.get_app().update()

    if "baked_tool_prim_path" in target:
        # rack_prim_path is a lightweight non-physics anchor, re-synced to the baked tool's CURRENT
        # live pose every run -- so moving a tool in the GUI needs no code change.
        tool_world = Gf.Transform(
            UsdGeom.Xformable(stage.GetPrimAtPath(tool_prim_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        )
        tool_quat = tool_world.GetRotation().GetQuat()
        target["dock_position"] = list(tool_world.GetTranslation())
        target["dock_orientation_wxyz"] = [tool_quat.GetReal(), *tool_quat.GetImaginary()]
    stage.DefinePrim(rack_prim_path, "Xform")
    rack_xform = SingleXFormPrim(prim_path=rack_prim_path)
    rack_xform.set_world_pose(
        position=np.array(target["dock_position"]),
        orientation=np.array(target["dock_orientation_wxyz"]),
    )

    if "baked_tool_prim_path" not in target:
        reference_asset(
            target["asset"],
            tool_prim_path,
            local_position=[0.0, 0.0, 0.0],
            local_orientation_wxyz=[1.0, 0.0, 0.0, 0.0],
            local_scale=target.get("local_scale"),
        )
    if target.get("female_coupler_parent_link_name"):
        _demote_tool_articulation(tool_prim_path)
    else:
        # Flat single-prim asset -- electric_screwdriver.usd carries no baked-in RigidBodyAPI at
        # all, so apply it explicitly rather than trusting the source asset.
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(tool_prim_path))

    if "baked_tool_prim_path" not in target:
        # Local identity -- tool_prim_path's world pose is the rack's. Skipped for a baked tool,
        # whose prim IS the user's hand-placed one.
        tool_xform = SingleXFormPrim(prim_path=tool_prim_path)
        tool_xform.set_local_pose(
            translation=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )

    female_coupler_path = _female_coupler_prim_path(tool_name)
    stage.DefinePrim(female_coupler_path, "Xform")
    female_xform = SingleXFormPrim(prim_path=female_coupler_path)
    female_xform.set_local_pose(
        translation=np.array(target["female_coupler_local_position"]),
        orientation=np.array(target["female_coupler_local_orientation_wxyz"]),
    )

    # Un-share every instance root in the tool's subtree -- it derives from the same panda_hand mesh
    # source as the arm. Materialized first: un-instancing re-composes the stage mid-iteration.
    instance_prims = [p for p in Usd.PrimRange(stage.GetPrimAtPath(tool_prim_path)) if p.IsInstance()]
    for prim in instance_prims:
        prim.SetInstanceable(False)

    return tool_prim_path


def _rack_joint_path(tool_name: str) -> str:
    return f"{config.TOOL_CHANGE_TARGETS[tool_name]['rack_prim_path']}/rack_joint"


def _wrist_joint_path(tool_name: str, robot_prim_path: str = config.ROBOT_PRIM_PATH) -> str:
    # Per-tool, not one shared name -- redefining a Joint prim in place leaves PhysX solving against
    # the stale body1. See docs/tool-changer.md.
    return f"{_male_coupler_prim_path(robot_prim_path)}/wrist_joint_{tool_name}"


def park_tool_at_rack(tool_name: str) -> None:
    """Joints a freshly-spawned tool to its own rack, so it can't drift or fall before ever being
    docked -- a tool is ALWAYS joint-fixed to something (rack or wrist), never free-falling."""
    create_fixed_joint(
        _rack_joint_path(tool_name),
        config.TOOL_CHANGE_TARGETS[tool_name]["rack_prim_path"],
        _female_coupler_prim_path(tool_name),
    )


def enable_gripper_tool_fingers() -> None:
    """One-time-per-run fixup for the gripper tool's fingers: un-instancing, joint reactivation and
    drive stiffening. Rationales: docs/tool-changer.md and docs/fr5-migration.md."""
    stage = omni.usd.get_context().get_stage()
    tool_prim_path = _tool_prim_path("gripper")

    for finger_name in config.GRIPPER_FINGER_LINK_NAMES:
        finger_path = f"{tool_prim_path}/{finger_name}"
        finger_prim = stage.GetPrimAtPath(finger_path)
        if not finger_prim.IsValid():
            print(f"[mefron_lib] WARNING: {finger_path} not found -- skipping finger fixup.", flush=True)
            continue
        un_instance_ancestor(finger_prim, finger_path)
        # NO xform-stack reset here. That was for the Franka hand's hand-authored fingers; the
        # PGC-140's are real URDF articulation links, and baking their world transform would
        # detach them from the tool root -- they would stay put while the docked tool moved.

    for joint_name in config.GRIPPER_JOINT_NAMES:
        joint_path = f"{tool_prim_path}/joints/{joint_name}"
        joint_prim = stage.GetPrimAtPath(joint_path)
        if not joint_prim.IsValid():
            print(f"[mefron_lib] WARNING: {joint_path} not found -- skipping joint activation.", flush=True)
            continue
        joint_prim.SetActive(True)

    stiffen_gripper_drive(prim_path=tool_prim_path)


def set_gripper_tool_finger_target(target_position: float) -> None:
    """Writes the docked gripper tool's own finger DriveAPI target directly -- PhysX servos them
    natively, and these joints aren't part of the arm's articulation at all."""
    stage = omni.usd.get_context().get_stage()
    tool_prim_path = _tool_prim_path("gripper")
    for joint_name in config.GRIPPER_JOINT_NAMES:
        joint_prim = stage.GetPrimAtPath(f"{tool_prim_path}/joints/{joint_name}")
        if not joint_prim.IsValid():
            continue
        UsdPhysics.DriveAPI(joint_prim, "linear").GetTargetPositionAttr().Set(target_position)


def dock_tool_to_wrist(tool_name: str, robot_prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Swaps a tool's FixedJoint from its rack onto the wrist -- the "grab" half of a tool change.
    Caller must have settled the arm at the dock pose AND undocked any previous tool first."""
    stage = omni.usd.get_context().get_stage()
    rack_joint_path = _rack_joint_path(tool_name)
    if stage.GetPrimAtPath(rack_joint_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[rack_joint_path])
        omni.kit.app.get_app().update()

    # The gripper tool is still the Franka hand, mated panda_link8-to-panda_hand via the real URDF
    # offset. That link is gone on the FR5, so this whole branch waits on step 3's PGC-140.
    panda_link8_path = f"{robot_prim_path}/panda_link8"
    if tool_name == "gripper" and stage.GetPrimAtPath(panda_link8_path).IsValid():
        create_fixed_joint(
            _wrist_joint_path(tool_name, robot_prim_path),
            panda_link8_path,
            f"{_tool_prim_path(tool_name)}/panda_hand",
            body1_local_orientation_wxyz=config.TOOL_CHANGER_GRIPPER_HAND_JOINT_LOCAL_ORIENTATION_WXYZ,
        )
        return
    if tool_name == "gripper":
        print(
            f"[mefron_lib] WARNING: {panda_link8_path} not found -- docking the Franka gripper tool "
            "through the coupler with placeholder offsets. Expect a gapped dock until step 3.",
            flush=True,
        )

    create_fixed_joint(
        _wrist_joint_path(tool_name, robot_prim_path),
        _male_coupler_prim_path(robot_prim_path),
        _female_coupler_prim_path(tool_name),
        # The cylinder's origin is its middle, so mate at the inner face, not the center.
        body0_local_position=(0.0, 0.0, -config.TOOL_CHANGER_CYLINDER_HEIGHT / 2),
        body1_local_position=config.TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_POSITION,
        body1_local_orientation_wxyz=config.TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_ORIENTATION_WXYZ,
    )


def undock_tool_to_rack(tool_name: str, robot_prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Inverse of dock_tool_to_wrist() -- swaps the FixedJoint back onto the rack. Caller must have
    already planned/settled the arm back at the tool's dock pose first."""
    stage = omni.usd.get_context().get_stage()
    wrist_joint_path = _wrist_joint_path(tool_name, robot_prim_path)
    if stage.GetPrimAtPath(wrist_joint_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[wrist_joint_path])
        omni.kit.app.get_app().update()

    create_fixed_joint(
        _rack_joint_path(tool_name),
        config.TOOL_CHANGE_TARGETS[tool_name]["rack_prim_path"],
        _female_coupler_prim_path(tool_name),
    )
