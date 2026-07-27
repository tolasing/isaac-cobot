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
# session mid-import, a top-level prim named after the URDF's own <robot name="..."> gets baked
# into mefron.usd as an orphaned leftover. Also guards "/panda_gripper_only" -- an early ATC
# prototype briefly live-imported the gripper tool this same way (since replaced by
# scripts/vendor_gripper_tool.py's pre-baked asset, see docs/tool-changer.md's gotcha 6), and a
# stray Save from that prototype baked this path into mefron.usd; kept here for old checkouts.
_STRAY_HISTORICAL_PANDA_PATH = "/panda"
_STRAY_HISTORICAL_GRIPPER_TOOL_PATH = "/panda_gripper_only"
# Literal (not config.*) -- these arms were retired when the ATC branch moved to a single Franka,
# but mefron.usd is a shared asset file: a pre-ATC checkout's stray Save can still have baked
# /World/Franka2, /World/Franka3 into it, same failure mode _STRAY_HISTORICAL_PANDA_PATH guards.
_STRAY_HISTORICAL_ARM_PATHS = ["/World/Franka2", "/World/Franka3"]


def clear_stray_robot_prims() -> None:
    """Deletes any pre-existing robot prims (config.ROBOT_PRIM_PATH, the historical stray /panda
    and /panda_gripper_only paths, plus pre-ATC arm2/arm3 leftovers) sitting in the stage the
    moment open_stage() returns -- leftovers baked into mefron.usd by a past session's stray Save.
    Must run right after open_stage(), before the settle pump -- see docs/mefron-history.md for why
    mount_franka()'s own cleanup is too late."""
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
    """Mounts cuRobo's bundled Franka Panda at prim_path/mount_position (arm 1's constants by
    default -- the ATC's single arm). Historically also mounted a second/third Franka before the
    ATC branch retired them (see docs/mefron-history.md); the URDF importer crashing if the
    full-experience extensions are already loaded is why mount_franka() must still run before
    kit_experience.enable_full_experience_extensions()."""
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
    already-open session) should do it themselves, same as mefron_gripper_probe.py's spawn_gripper_probe().
    NOT used by spawn_dockable_tool() -- the URDF importer's disk-persisted "Robot Description" cache
    keys visuals by bare link name, and this template's base_link/ee_link collide with the main
    arm's own identically-named links if both are imported into the same session (confirmed live,
    broke the main arm's rendering) -- see docs/tool-changer.md's gotcha 6. The ATC gripper tool is
    a pre-vendored standalone asset instead (scripts/vendor_gripper_tool.py), referenced the same
    way as the suction/screwdriver tools, never live-imported into mefron.usd's own stage."""
    urdf_path = write_hand_only_urdf()
    return import_cr5(
        urdf_path=urdf_path,
        prim_path=prim_path,
        default_drive_strength=config.FRANKA_DRIVE_STRENGTH,
        default_position_drive_damping=config.FRANKA_DRIVE_DAMPING,
    )


def remove_parallel_jaw_gripper(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
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


def hide_hand_housing(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
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


def _reference_tool_asset(
    usd_path,
    prim_path: str,
    local_position,
    local_orientation_wxyz,
    local_scale=None,
    disable_physics: bool = True,
) -> None:
    """Shared by attach_suction_gripper()/attach_screwdriver_gripper() (permanently riding on
    panda_hand -- baked-in physics disabled since they move purely kinematically) and
    spawn_dockable_tool() (parked in an ATC rack as a real independent rigid body -- baked-in
    physics left enabled so PhysX/a FixedJoint can treat it as one). See docs/mefron-history.md for
    why the baked-in collider/RigidBodyAPI gets disabled in the permanent-mount case."""
    from isaacsim.core.utils.stage import add_reference_to_stage

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(prim_path).IsValid():
        # Same re-run safety as mount_franka() above -- avoid a uniquified duplicate on a second run
        # in the same session.
        omni.kit.commands.execute("DeletePrims", paths=[prim_path])
        omni.kit.app.get_app().update()

    add_reference_to_stage(usd_path=str(usd_path), prim_path=prim_path)
    xform = SingleXFormPrim(prim_path=prim_path)
    if local_scale is not None:
        xform.set_local_scale(np.array(local_scale))
    xform.set_local_pose(
        translation=np.array(local_position),
        orientation=np.array(local_orientation_wxyz),
    )

    if disable_physics:
        prim = stage.GetPrimAtPath(prim_path)
        for p in Usd.PrimRange(prim):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Set(False)


def attach_suction_gripper(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """References config.SUCTION_GRIPPER_USD as a child of panda_hand (cuRobo's ee_link) so it
    rides along rigidly. Does not remove/hide the Franka's hand -- call
    remove_parallel_jaw_gripper()/hide_hand_housing() first."""
    gripper_prim_path = f"{prim_path}/panda_hand/{config.SUCTION_GRIPPER_PRIM_NAME}"
    _reference_tool_asset(
        config.SUCTION_GRIPPER_USD,
        gripper_prim_path,
        config.SUCTION_GRIPPER_LOCAL_POSITION,
        config.SUCTION_GRIPPER_LOCAL_ORIENTATION_WXYZ,
    )


def attach_screwdriver_gripper(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """References config.SCREWDRIVER_USD as a child of panda_hand (cuRobo's ee_link), scaled to
    config.SCREWDRIVER_LOCAL_SCALE. Mounts the tool visually/for collision only -- no
    screw-driving control wired up yet."""
    tool_prim_path = f"{prim_path}/panda_hand/{config.SCREWDRIVER_PRIM_NAME}"
    _reference_tool_asset(
        config.SCREWDRIVER_USD,
        tool_prim_path,
        config.SCREWDRIVER_LOCAL_POSITION,
        config.SCREWDRIVER_LOCAL_ORIENTATION_WXYZ,
        local_scale=config.SCREWDRIVER_LOCAL_SCALE,
    )


def attach_tool_changer_male_coupler(prim_path: str = config.ROBOT_PRIM_PATH) -> str:
    """Authors the ATC's permanent male half: a plain UsdGeom.Cylinder (no CAD asset needed, unlike
    the other tools) as a child of panda_hand, sized to config.TOOL_CHANGER_CYLINDER_RADIUS/HEIGHT.
    Rides kinematically like the tools above -- it's welded on by construction and never detaches
    itself, only the tools docked to it do (see dock_tool_to_wrist()/undock_tool_to_rack())."""
    stage = omni.usd.get_context().get_stage()
    coupler_prim_path = f"{prim_path}/panda_hand/{config.TOOL_CHANGER_MALE_PRIM_NAME}"
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
    return f"{config.TOOL_CHANGE_TARGETS[tool_name]['rack_prim_path']}/tool"


def _female_coupler_parent_prim_path(tool_name: str) -> str:
    """The prim female_coupler should be a child of -- must resolve to a real PhysX rigid body, or
    a FixedJoint targeting it can't find one to attach to. For a flat, single-prim asset
    (suction/screwdriver) that's tool_prim_path itself (see spawn_dockable_tool()'s explicit
    RigidBodyAPI.Apply() there). For the gripper's own multi-link mini-articulation, tool_prim_path
    is just an organizing Xform -- the real rigid bodies are its links (base_link/panda_hand/...),
    confirmed live via direct inspection -- so config.TOOL_CHANGE_TARGETS marks which link
    female_coupler must live under instead ("female_coupler_parent_link_name")."""
    parent_link_name = config.TOOL_CHANGE_TARGETS[tool_name].get("female_coupler_parent_link_name")
    if parent_link_name:
        return f"{_tool_prim_path(tool_name)}/{parent_link_name}"
    return _tool_prim_path(tool_name)


def _female_coupler_prim_path(tool_name: str) -> str:
    return f"{_female_coupler_parent_prim_path(tool_name)}/female_coupler"


def _male_coupler_prim_path(robot_prim_path: str = config.ROBOT_PRIM_PATH) -> str:
    return f"{robot_prim_path}/panda_hand/{config.TOOL_CHANGER_MALE_PRIM_NAME}"


def spawn_dockable_tool(tool_name: str) -> str:
    """Places one of config.TOOL_CHANGE_TARGETS's tools at its rack position, as a real
    independent rigid body (physics left enabled -- the opposite of attach_suction_gripper()'s/
    attach_screwdriver_gripper()'s permanently-kinematic use of the same assets above) so it can
    sit jointed to the rack until docked. All 3 tools are referenced USD assets -- the gripper
    tool is scripts/vendor_gripper_tool.py's pre-baked export of
    mount_franka_hand_only()'s hand-only URDF, NOT a live import: confirmed live that live-
    importing a second robot into mefron.usd's own stage lets the URDF importer's shared
    "Robot Description" cache silently corrupt the main arm's own link structure, even at a fully
    distinct prim path -- see docs/tool-changer.md's gotcha 6. Returns the tool's own prim path."""
    target = config.TOOL_CHANGE_TARGETS[tool_name]
    rack_prim_path = target["rack_prim_path"]
    tool_prim_path = _tool_prim_path(tool_name)

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(rack_prim_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[rack_prim_path])
        omni.kit.app.get_app().update()
    stage.DefinePrim(rack_prim_path, "Xform")
    rack_xform = SingleXFormPrim(prim_path=rack_prim_path)
    rack_xform.set_world_pose(
        position=np.array(target["dock_position"]),
        orientation=np.array(target["dock_orientation_wxyz"]),
    )

    _reference_tool_asset(
        target["asset"],
        tool_prim_path,
        local_position=[0.0, 0.0, 0.0],
        local_orientation_wxyz=[1.0, 0.0, 0.0, 0.0],
        disable_physics=False,
    )
    if target.get("female_coupler_parent_link_name"):
        # Multi-link articulation (currently just the gripper tool) -- confirmed live the URDF
        # importer synthesizes a "root_joint" PhysicsFixedJoint welding its free-floating base_link
        # to the world, which must be removed for the rack/wrist FixedJoint to be the only thing
        # constraining it. DeletePrims silently no-ops on it (same gotcha as
        # remove_parallel_jaw_gripper()'s finger joints, see docs/mefron-history.md);
        # SetActive(False) is what actually removes the constraint.
        root_joint_prim = stage.GetPrimAtPath(f"{tool_prim_path}/root_joint")
        if root_joint_prim.IsValid():
            root_joint_prim.SetActive(False)
    else:
        # A flat, single-prim asset (suction/screwdriver) -- confirmed electric_screwdriver.usd
        # carries no baked-in RigidBodyAPI at all (unlike the suction gripper asset), so a
        # FixedJoint targeting a child of tool_prim_path can't resolve any rigid body to pull;
        # apply explicitly rather than trusting the source asset.
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(tool_prim_path))
    # Local identity either way -- tool_prim_path's world pose is rack_prim_path's (the dock pose).
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
    return tool_prim_path


def _create_tool_fixed_joint(
    joint_path: str,
    body0_path: str,
    body1_path: str,
    body1_local_position=(0.0, 0.0, 0.0),
    body1_local_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
) -> None:
    """Plain rigid UsdPhysics.FixedJoint between two prims -- no DriveAPI/LimitAPI compliance,
    unlike attach_surface_gripper_physics()'s intentionally-soft D6 (a tool-changer coupling should
    be rigid). body1's local frame defaults to identity (rack parking: body0/body1 origins already
    coincide) but dock_tool_to_wrist() passes the mate offset (body1 = female coupler expressed in
    body0/male-coupler-i.e.-ee_link's frame is what PhysX pulls into alignment). Both bodies get
    excludeFromArticulation since panda_hand is a real articulation link."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(joint_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[joint_path])
        omni.kit.app.get_app().update()

    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([body0_path])
    joint.CreateBody1Rel().SetTargets([body1_path])
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*body1_local_position))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(*body1_local_orientation_wxyz))
    joint.CreateExcludeFromArticulationAttr().Set(True)


def _rack_joint_path(tool_name: str) -> str:
    return f"{config.TOOL_CHANGE_TARGETS[tool_name]['rack_prim_path']}/rack_joint"


def _wrist_joint_path(tool_name: str, robot_prim_path: str = config.ROBOT_PRIM_PATH) -> str:
    # Per-tool, not one shared "wrist_joint" name -- confirmed live that redefining a Joint prim at
    # the same path with a different body1 target leaves PhysX solving against the stale target
    # (the previous tool's female_coupler), even though body0/localPos/localRot look correct from
    # the USD side. A fresh path per tool sidesteps that redefinition entirely.
    return f"{_male_coupler_prim_path(robot_prim_path)}/wrist_joint_{tool_name}"


def park_tool_at_rack(tool_name: str) -> None:
    """Creates the initial FixedJoint anchoring a freshly-spawned tool to its own rack, so it
    doesn't drift/fall under gravity before ever being docked -- a parked tool is always
    joint-fixed to something (rack or wrist), never free-falling."""
    _create_tool_fixed_joint(
        _rack_joint_path(tool_name),
        config.TOOL_CHANGE_TARGETS[tool_name]["rack_prim_path"],
        _female_coupler_prim_path(tool_name),
    )


def _set_tool_collision_enabled(tool_name: str, enabled: bool) -> None:
    """Toggles CollisionEnabledAttr (not RigidBodyAPI -- the body must stay dynamic for the joint's
    solver to actually pull it into place) across the tool's own subtree. Confirmed live: without
    this, a docked tool's real collider fights the wrist joint's pull against panda_hand's own
    collider, settling tens of cm short of the joint's target instead of converging to it -- the
    same reasoning attach_suction_gripper()/attach_screwdriver_gripper() disable collision for
    permanently-mounted tools, just toggled dynamically here since a parked tool DOES need real
    collision (sitting in its rack) while a docked one doesn't."""
    stage = omni.usd.get_context().get_stage()
    tool_prim = stage.GetPrimAtPath(_tool_prim_path(tool_name))
    for prim in Usd.PrimRange(tool_prim):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(enabled)


def dock_tool_to_wrist(tool_name: str, robot_prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Swaps a tool's FixedJoint from its rack onto the wrist's male coupler -- the "grab" half of
    a tool change. Caller (see teleop.py's tool-change waypoint queue) is responsible for having
    already planned/settled the arm at the tool's dock pose first, or this snaps the tool a
    noticeable distance instead of a clean small correction. Also responsible for having already
    undocked any PREVIOUSLY docked tool first (undock_tool_to_rack()) -- only one tool should ever
    be wrist-jointed at a time; docking a second one without releasing the first leaves both
    simultaneously welded to the same male coupler instead."""
    stage = omni.usd.get_context().get_stage()
    rack_joint_path = _rack_joint_path(tool_name)
    if stage.GetPrimAtPath(rack_joint_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[rack_joint_path])
        omni.kit.app.get_app().update()

    _create_tool_fixed_joint(
        _wrist_joint_path(tool_name, robot_prim_path),
        _male_coupler_prim_path(robot_prim_path),
        _female_coupler_prim_path(tool_name),
        body1_local_position=config.TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_POSITION,
        body1_local_orientation_wxyz=config.TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_ORIENTATION_WXYZ,
    )
    _set_tool_collision_enabled(tool_name, enabled=False)


def undock_tool_to_rack(tool_name: str, robot_prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Inverse of dock_tool_to_wrist() -- swaps the FixedJoint back onto the rack. Caller must have
    already planned/settled the arm back at the tool's dock pose first."""
    stage = omni.usd.get_context().get_stage()
    wrist_joint_path = _wrist_joint_path(tool_name, robot_prim_path)
    if stage.GetPrimAtPath(wrist_joint_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[wrist_joint_path])
        omni.kit.app.get_app().update()

    _create_tool_fixed_joint(
        _rack_joint_path(tool_name),
        config.TOOL_CHANGE_TARGETS[tool_name]["rack_prim_path"],
        _female_coupler_prim_path(tool_name),
    )
    _set_tool_collision_enabled(tool_name, enabled=True)


def attach_surface_gripper_physics(prim_path: str = config.ROBOT_PRIM_PATH) -> str:
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
