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


def _un_instance_ancestor(prim, context_path: str) -> None:
    """Walks up from prim to the nearest instanceable ancestor and un-shares it -- the URDF
    importer makes imported mesh geometry instanceable by default, so authoring directly on an
    instance-proxy prim (Set() on an attribute, MakeInvisible(), ...) silently no-ops. Shared by
    hide_hand_housing()'s visuals/collisions handling below."""
    ancestor = prim
    while ancestor.IsValid():
        if ancestor.IsInstance():
            print(
                f"[mefron_lib] {context_path}: un-instancing shared prototype at {ancestor.GetPath()} first.",
                flush=True,
            )
            ancestor.SetInstanceable(False)
            return
        ancestor = ancestor.GetParent()


def hide_hand_housing(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Makes prim_path's panda_hand/visuals invisible and disables panda_hand/collisions, for the
    ATC's permanently-hidden housing under the male coupler + whichever tool is docked. Old
    reasoning (docs/mefron-history.md) kept collisions active so the other two arms' planners in
    the pre-ATC 3-Franka cell would still see this hand as an obstacle -- moot now there's only one
    arm; leaving it active just lets PhysX contact with the (cuRobo-invisible, see
    CLAUDE.md's open issues) rack/tool fight the commanded trajectory instead. See
    docs/mefron-history.md for why the nearest instance root gets un-shared first -- collisions
    gets the same treatment as visuals since it's confirmed to be its own separately-instanceable
    sub-scope, not covered by un-instancing visuals alone."""
    stage = omni.usd.get_context().get_stage()
    visuals_path = f"{prim_path}/panda_hand/visuals"
    prim = stage.GetPrimAtPath(visuals_path)
    if not prim.IsValid():
        print(f"[mefron_lib] WARNING: {visuals_path} not found -- skipping hide.", flush=True)
        return

    _un_instance_ancestor(prim, visuals_path)
    UsdGeom.Imageable(prim).MakeInvisible()

    collisions_path = f"{prim_path}/panda_hand/collisions"
    collisions_prim = stage.GetPrimAtPath(collisions_path)
    if not collisions_prim.IsValid():
        print(f"[mefron_lib] WARNING: {collisions_path} not found -- skipping collision disable.", flush=True)
        return
    _un_instance_ancestor(collisions_prim, collisions_path)
    for p in Usd.PrimRange(collisions_prim):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)


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
    target = config.TOOL_CHANGE_TARGETS[tool_name]
    if "baked_tool_prim_path" in target:
        return target["baked_tool_prim_path"]
    return f"{target['rack_prim_path']}/tool"


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
    sit jointed to the rack until docked. The gripper tool is a referenced USD asset --
    scripts/vendor_gripper_tool.py's pre-baked export of mount_franka_hand_only()'s hand-only URDF,
    NOT a live import: confirmed live that live-importing a second robot into mefron.usd's own
    stage lets the URDF importer's shared "Robot Description" cache silently corrupt the main arm's
    own link structure, even at a fully distinct prim path -- see docs/tool-changer.md's gotcha 6.
    Suction/screwdriver are instead baked directly into mefron.usd via the GUI (see
    feedback_static_scenery_baked_into_scene memory) -- never referenced/repositioned here, only
    read. Returns the tool's own prim path."""
    target = config.TOOL_CHANGE_TARGETS[tool_name]
    rack_prim_path = target["rack_prim_path"]
    tool_prim_path = _tool_prim_path(tool_name)

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(rack_prim_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[rack_prim_path])
        omni.kit.app.get_app().update()

    if "baked_tool_prim_path" in target:
        # rack_prim_path is a lightweight, non-physics anchor -- park_tool_at_rack()'s joint needs a
        # static reference point, and a real rigid body can't be jointed to its own descendant
        # (female_coupler lives under the baked tool itself). Sync it to the baked tool's CURRENT
        # live world pose every run, so it always matches wherever the tool was placed in the GUI.
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
        _reference_tool_asset(
            target["asset"],
            tool_prim_path,
            local_position=[0.0, 0.0, 0.0],
            local_orientation_wxyz=[1.0, 0.0, 0.0, 0.0],
            local_scale=target.get("local_scale"),
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

    if "baked_tool_prim_path" not in target:
        # Local identity -- tool_prim_path's world pose is rack_prim_path's (the dock pose). Skipped
        # for baked tools: tool_prim_path IS the user's hand-placed prim, never touched here.
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

    # Un-instance the tool's own subtree unconditionally -- confirmed live the gripper tool
    # disappears the moment the main Franka arm appears, the same "shared instanceable prototype"
    # gotcha hide_hand_housing() already works around for the arm's own panda_hand/visuals, just
    # hitting a different pairing here: this tool's geometry was derived from the identical
    # panda_hand URDF mesh source, so it risks sharing one prototype with the live-imported arm.
    # SetInstanceable(False) directly on every instance prim found (not just walking up from one
    # leaf, since there may be several independent instance roots in this subtree) un-shares all of
    # them up front, before any visibility-toggling code elsewhere in this module ever runs.
    # Materialized first -- un-instancing an ancestor re-composes the stage, which would
    # invalidate an in-progress PrimRange iterator.
    instance_prims = [p for p in Usd.PrimRange(stage.GetPrimAtPath(tool_prim_path)) if p.IsInstance()]
    for prim in instance_prims:
        prim.SetInstanceable(False)

    return tool_prim_path


def _create_tool_fixed_joint(
    joint_path: str,
    body0_path: str,
    body1_path: str,
    body0_local_position=(0.0, 0.0, 0.0),
    body0_local_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
    body1_local_position=(0.0, 0.0, 0.0),
    body1_local_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
) -> None:
    """Plain rigid UsdPhysics.FixedJoint between two prims -- no DriveAPI/LimitAPI compliance,
    unlike attach_surface_gripper_physics()'s intentionally-soft D6. Both local frames default to
    each body's own prim origin; dock_tool_to_wrist() offsets body0 to the male coupler's outward
    face (not its center) and body1 to the mate pose. excludeFromArticulation since panda_hand is a
    real articulation link."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(joint_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[joint_path])
        omni.kit.app.get_app().update()

    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([body0_path])
    joint.CreateBody1Rel().SetTargets([body1_path])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*body0_local_position))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(*body0_local_orientation_wxyz))
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


def enable_gripper_tool_fingers() -> None:
    """One-time-per-run structural fixup for the gripper tool's two fingers, called once from
    mefron.py right after the dockable-tool spawn loop. The fingers/their RigidBodyAPI are hand-
    authored directly in mefron.usd (see feedback_static_scenery_baked_into_scene memory), but two
    things PhysX needs are easy to get wrong by hand and are restored here instead: (1) each finger
    needs UsdGeom.Xformable.SetResetXformStack(True) since it's a RigidBodyAPI'd child of another
    enabled rigid body (panda_hand/tool root) -- without it PhysX logs "missing xformstack reset
    when child of another enabled rigid body" and the finger can fly off on the first physics step
    (confirmed live earlier this session); ClearXformOpOrder()/AddTransformOp() must run BEFORE
    SetResetXformStack() since they author the same xformOpOrder attribute. (2) the two
    panda_finger_joint1/2 prismatic joints ship deactivated from the vendor bake (see
    vendor_gripper_tool_visual_only.py) and must be explicitly reactivated for their DriveAPI to
    have anything to act on. (3) that same bake left their drive at import_cr5()'s whole-robot
    default (stiffness=625/damping=10) -- stiffen_gripper_drive() raises it, same as it always did
    for the pre-ATC arm-mounted gripper, just pointed at the tool's own path now."""
    stage = omni.usd.get_context().get_stage()
    tool_prim_path = _tool_prim_path("gripper")

    for finger_name in config.GRIPPER_FINGER_LINK_NAMES:
        finger_path = f"{tool_prim_path}/{finger_name}"
        finger_prim = stage.GetPrimAtPath(finger_path)
        if not finger_prim.IsValid():
            print(f"[mefron_lib] WARNING: {finger_path} not found -- skipping finger fixup.", flush=True)
            continue
        _un_instance_ancestor(finger_prim, finger_path)
        finger_xformable = UsdGeom.Xformable(finger_prim)
        if not finger_xformable.GetResetXformStack():
            world_transform = finger_xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            finger_xformable.ClearXformOpOrder()
            finger_xformable.AddTransformOp().Set(world_transform)
            finger_xformable.SetResetXformStack(True)

    for joint_name in config.GRIPPER_JOINT_NAMES:
        joint_path = f"{tool_prim_path}/joints/{joint_name}"
        joint_prim = stage.GetPrimAtPath(joint_path)
        if not joint_prim.IsValid():
            print(f"[mefron_lib] WARNING: {joint_path} not found -- skipping joint activation.", flush=True)
            continue
        joint_prim.SetActive(True)

    stiffen_gripper_drive(prim_path=tool_prim_path)


def set_gripper_tool_finger_target(target_position: float) -> None:
    """Sets the docked gripper tool's own two finger joints' DriveAPI target position directly --
    vendor_gripper_tool_visual_only.py bakes a linear DriveAPI onto each panda_finger_joint1/2, so
    PhysX servos them natively; no SingleArticulation needed, since this tool's joints aren't part
    of the arm's own articulation (see teleop._step_arm()'s C/O block). Path is fixed since the
    gripper tool is a baked prim, not dynamically spawned -- see config.TOOL_CHANGE_TARGETS["gripper"]."""
    stage = omni.usd.get_context().get_stage()
    tool_prim_path = _tool_prim_path("gripper")
    for joint_name in config.GRIPPER_JOINT_NAMES:
        joint_prim = stage.GetPrimAtPath(f"{tool_prim_path}/joints/{joint_name}")
        if not joint_prim.IsValid():
            continue
        UsdPhysics.DriveAPI(joint_prim, "linear").GetTargetPositionAttr().Set(target_position)


def dock_tool_to_wrist(tool_name: str, robot_prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Swaps a tool's FixedJoint from its rack onto the wrist -- the "grab" half of a tool change.
    Caller (see teleop.py's tool-change waypoint queue) is responsible for having already
    planned/settled the arm at the tool's dock pose first, or this snaps the tool a noticeable
    distance instead of a clean small correction. Also responsible for having already undocked any
    PREVIOUSLY docked tool first (undock_tool_to_rack()) -- only one tool should ever be
    wrist-jointed at a time."""
    stage = omni.usd.get_context().get_stage()
    rack_joint_path = _rack_joint_path(tool_name)
    if stage.GetPrimAtPath(rack_joint_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[rack_joint_path])
        omni.kit.app.get_app().update()

    if tool_name == "gripper":
        # Same stock panda_hand mesh as the arm's own -- mate panda_link8 directly to it via the
        # real URDF offset instead of approximating through the male/female coupler geometry (see
        # config.TOOL_CHANGER_GRIPPER_HAND_JOINT_LOCAL_ORIENTATION_WXYZ).
        _create_tool_fixed_joint(
            _wrist_joint_path(tool_name, robot_prim_path),
            f"{robot_prim_path}/panda_link8",
            f"{_tool_prim_path(tool_name)}/panda_hand",
            body1_local_orientation_wxyz=config.TOOL_CHANGER_GRIPPER_HAND_JOINT_LOCAL_ORIENTATION_WXYZ,
        )
        return

    _create_tool_fixed_joint(
        _wrist_joint_path(tool_name, robot_prim_path),
        _male_coupler_prim_path(robot_prim_path),
        _female_coupler_prim_path(tool_name),
        # UsdGeom.Cylinder is centered on its own prim origin, and TOOL_CHANGER_MALE_LOCAL_POSITION
        # places that origin at the cylinder's middle (see config.py) -- mate at the inner face
        # instead (confirmed live for the gripper before it moved to the panda_link8 scheme above).
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

    _create_tool_fixed_joint(
        _rack_joint_path(tool_name),
        config.TOOL_CHANGE_TARGETS[tool_name]["rack_prim_path"],
        _female_coupler_prim_path(tool_name),
    )


def _screw_prim_path(index: int) -> str:
    return f"{config.SCREW_SCOPE_PRIM_PATH}/screw_{index}"


# One joint path per lifecycle stage, never redefined in place -- docs/tool-changer.md's gotcha 5
# (PhysX kept solving against a stale body1 when a joint prim was redefined at the same path).
def _screw_presenter_joint_path(index: int) -> str:
    return f"{_screw_prim_path(index)}/presenter_joint"


def _screw_tip_joint_path(index: int) -> str:
    return f"{_screw_prim_path(index)}/tip_joint"


def _screw_hole_joint_path(index: int) -> str:
    return f"{_screw_prim_path(index)}/hole_joint"


def _screw_hole_anchor_path(index: int) -> str:
    return f"{config.SCREW_SCOPE_PRIM_PATH}/hole_anchor_{index}"


def _screw_presenter_anchor_path(index: int) -> str:
    return f"{config.SCREW_SCOPE_PRIM_PATH}/presenter_anchor_{index}"


def clear_screws() -> None:
    """Deletes the whole script-owned screw scope, so a run never inherits screws from a previous
    one. Needed for the same reason clear_stray_robot_prims() is: the URDF importer rewrites
    mefron.usd on every run (see CLAUDE.md), so anything spawned here can get baked in. Deliberately
    leaves config.SCREW_PRESENTER_PRIM_PATH alone -- that one may be hand-placed."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(config.SCREW_SCOPE_PRIM_PATH).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[config.SCREW_SCOPE_PRIM_PATH])
        omni.kit.app.get_app().update()


def ensure_screw_presenter() -> str:
    """Stand-in for the real screw presenter: returns config.SCREW_PRESENTER_PRIM_PATH, creating it
    at the SCREW_PRESENTER_FALLBACK_* pose only if it doesn't already exist. Same baked-vs-fallback
    duality as TOOL_CHANGE_TARGETS' baked_tool_prim_path -- hand-place this prim in the GUI and its
    own pose wins, with no config change."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(config.SCREW_PRESENTER_PRIM_PATH).IsValid():
        print(f"[mefron_lib] using the scene's own {config.SCREW_PRESENTER_PRIM_PATH} pose.", flush=True)
        return config.SCREW_PRESENTER_PRIM_PATH

    stage.DefinePrim(config.SCREW_PRESENTER_PRIM_PATH, "Xform")
    SingleXFormPrim(prim_path=config.SCREW_PRESENTER_PRIM_PATH).set_world_pose(
        position=np.array(config.SCREW_PRESENTER_FALLBACK_POSITION),
        orientation=np.array(config.SCREW_PRESENTER_FALLBACK_ORIENTATION_WXYZ),
    )
    print(
        f"[mefron_lib] spawned placeholder {config.SCREW_PRESENTER_PRIM_PATH} at "
        f"{config.SCREW_PRESENTER_FALLBACK_POSITION} -- hand-place it in the GUI and save to override.",
        flush=True,
    )
    return config.SCREW_PRESENTER_PRIM_PATH


def present_screw(index: int) -> str:
    """Pops a screw in at the presenter, jointed to it so it can't fall -- a screw is ALWAYS
    joint-fixed to something (presenter, wrist, or hole), the same invariant park_tool_at_rack()
    states for tools. Explicit mass/inertia because the placeholder asset carries no colliders for
    PhysX to derive either from."""
    from .grasp import compute_screw_presenter_pose

    stage = omni.usd.get_context().get_stage()
    screw_prim_path = _screw_prim_path(index)
    presenter_trans, presenter_quat = compute_screw_presenter_pose()

    # A Scope, not an Xform -- it can't carry a transform at all, so a screw's own local pose below
    # is guaranteed to be its world pose.
    stage.DefinePrim(config.SCREW_SCOPE_PRIM_PATH, "Scope")
    # disable_physics=False -- unlike a permanently-kinematic mounted tool, this has to be a real
    # rigid body for a FixedJoint's solver to move it at all (docs/tool-changer.md's gotcha 2).
    _reference_tool_asset(
        config.SCREW_USD,
        screw_prim_path,
        local_position=presenter_trans,
        local_orientation_wxyz=presenter_quat,
        disable_physics=False,
    )
    screw_prim = stage.GetPrimAtPath(screw_prim_path)
    UsdPhysics.RigidBodyAPI.Apply(screw_prim)
    mass_api = UsdPhysics.MassAPI.Apply(screw_prim)
    mass_api.CreateMassAttr().Set(config.SCREW_MASS)
    mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*config.SCREW_DIAGONAL_INERTIA))

    # Anchor at the seat pose, not the presenter prim itself: identity local frames on both sides
    # then weld with zero snap (as weld_screw_into_hole() does), and body0 stays clear of the
    # presenter's 0.001 unitsResolve scale -- docs/tool-changer.md's gotcha 8. Either way body0 isn't
    # a rigid body, so the screw is anchored to the world.
    anchor_path = _screw_presenter_anchor_path(index)
    stage.DefinePrim(anchor_path, "Xform")
    SingleXFormPrim(prim_path=anchor_path).set_world_pose(position=presenter_trans, orientation=presenter_quat)
    _create_tool_fixed_joint(_screw_presenter_joint_path(index), anchor_path, screw_prim_path)
    return screw_prim_path


def attach_screw_to_wrist(index: int, robot_prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Swaps a presented screw's joint onto the wrist -- the "pick" half of a screw cycle, and the
    direct analogue of dock_tool_to_wrist(). body0 is panda_hand itself: it's a real rigid body at
    unit scale, so localPos0 is unambiguous -- unlike the docked tool prim, whose 0.001 scale makes
    a joint's own frame exactly the kind of guess docs/tool-changer.md's gotcha 8 warns about.
    Caller must have settled the arm at the pick pose first. Welds at the nominal carry pose, see below."""
    stage = omni.usd.get_context().get_stage()
    from .grasp import compute_dependent_world_pose, compute_relative_pose

    presenter_joint_path = _screw_presenter_joint_path(index)
    if stage.GetPrimAtPath(presenter_joint_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[presenter_joint_path])
        omni.kit.app.get_app().update()

    # The NOMINAL carry pose (docked tool's live pose + SCREW_CARRY_LOCAL_*), not the arm's settled
    # pose: welding the live hand->screw offset froze cuRobo's ~2mm residual approach error into the
    # joint, leaving the screw permanently off the bit axis for the rest of the cycle.
    tool_trans, tool_quat = SingleXFormPrim(
        prim_path=_tool_prim_path("screwdriver"), reset_xform_properties=False
    ).get_world_pose()
    screw_trans, screw_quat = compute_dependent_world_pose(
        tool_trans, tool_quat, config.SCREW_CARRY_LOCAL_POSITION, config.SCREW_CARRY_LOCAL_ORIENTATION_WXYZ
    )
    # Move the screw onto that pose before jointing, so the correction isn't a visible snap -- same
    # re-authoring weld_screw_into_hole() does, and safe here since a screw carries no colliders.
    SingleXFormPrim(prim_path=_screw_prim_path(index), reset_xform_properties=False).set_world_pose(
        position=screw_trans, orientation=screw_quat
    )

    hand_path = f"{robot_prim_path}/panda_hand"
    hand_trans, hand_quat = SingleXFormPrim(prim_path=hand_path, reset_xform_properties=False).get_world_pose()
    local_trans, local_quat = compute_relative_pose(hand_trans, hand_quat, screw_trans, screw_quat)

    _create_tool_fixed_joint(
        _screw_tip_joint_path(index),
        hand_path,
        _screw_prim_path(index),
        body0_local_position=tuple(local_trans),
        body0_local_orientation_wxyz=tuple(local_quat),
    )


def weld_screw_into_hole(index: int, hole_index: int) -> None:
    """Releases a carried screw into its hole -- the "drop" half, and the analogue of
    undock_tool_to_rack(): the screw leaves the wrist for a static anchor, never free-falling. Seats
    it at the hole's own nominal pose, not the arm's -- see below. Also re-authors the screw's own USD
    xform, so a Stop restores it AT the hole instead of snapping back to where it was first spawned."""
    stage = omni.usd.get_context().get_stage()
    from .grasp import compute_screw_hole_pose

    screw_prim_path = _screw_prim_path(index)
    tip_joint_path = _screw_tip_joint_path(index)
    if stage.GetPrimAtPath(tip_joint_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[tip_joint_path])
        omni.kit.app.get_app().update()

    # The hole's nominal pose (main_holder's LIVE pose + SCREW_HOLES[hole_index] + insertion depth),
    # not wherever the arm settled: welding the live pose left placed screws 2-3mm out in x/y and
    # ~5mm too deep, measured by reparenting them under main_holder. A real screw is constrained by
    # the pocket, not by the arm's accuracy -- same reasoning as attach_screw_to_wrist()'s weld.
    screw_trans, screw_quat = compute_screw_hole_pose(hole_index)
    screw_xform = SingleXFormPrim(prim_path=screw_prim_path, reset_xform_properties=False)
    screw_xform.set_world_pose(position=screw_trans, orientation=screw_quat)

    anchor_path = _screw_hole_anchor_path(hole_index)
    stage.DefinePrim(anchor_path, "Xform")
    SingleXFormPrim(prim_path=anchor_path).set_world_pose(position=screw_trans, orientation=screw_quat)
    # Anchor placed at that same pose, so identity local frames on both sides weld with zero snap. A
    # static world anchor, not main_holder itself: see docs/tool-changer.md for why.
    _create_tool_fixed_joint(_screw_hole_joint_path(index), anchor_path, screw_prim_path)


def _assembly_anchor_path(mount_prim_path: str) -> str:
    return f"{config.ASSEMBLY_WELD_SCOPE_PRIM_PATH}/anchor_{Sdf.Path(mount_prim_path).name}"


# Which mount each anchor tracks, stored on the anchor itself so sync_assembly_anchors() needs no
# Python-side bookkeeping and stays correct across a Stop/Play.
_ASSEMBLY_ANCHOR_MOUNT_ATTR = "mefron:assemblyWeldMountPath"


# One joint path per part, never redefined in place -- docs/tool-changer.md's gotcha 5.
def _assembly_weld_joint_path(part_prim_path: str) -> str:
    return f"{config.ASSEMBLY_WELD_SCOPE_PRIM_PATH}/weld_{Sdf.Path(part_prim_path).name}"


def clear_assembly_welds() -> None:
    """Deletes the whole script-owned assembly-weld scope, so a run never inherits a previous one's
    anchors/joints. Same reasoning as clear_screws(): the URDF importer rewrites mefron.usd on every
    run (see CLAUDE.md), so anything spawned here can get baked in. Also re-enables collision on the
    assembly parts, which weld_part_at_assembly_pose() turns off -- that same importer rewrite would
    otherwise persist it into a fresh run, where nothing is welded and every part must be grippable.
    Nothing else here turns collision off, so re-enabling unconditionally is safe."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(config.ASSEMBLY_WELD_SCOPE_PRIM_PATH).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[config.ASSEMBLY_WELD_SCOPE_PRIM_PATH])
        omni.kit.app.get_app().update()

    for relationship in config.ASSEMBLY_RELATIONSHIPS.values():
        part_prim_path = relationship["part_prim_path"]
        if any(not enabled for enabled in _collision_enabled_flags(part_prim_path)):
            print(
                f"[mefron_lib] {part_prim_path} had collision disabled on load -- re-enabling "
                "(left behind by an older release weld; see this function's docstring).",
                flush=True,
            )
            _set_prim_collision_enabled(part_prim_path, True)


def _collision_enabled_flags(prim_path: str) -> list:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return []
    return [
        bool(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get())
        for p in Usd.PrimRange(prim)
        if p.HasAPI(UsdPhysics.CollisionAPI)
    ]


def _set_prim_collision_enabled(prim_path: str, enabled: bool) -> None:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(enabled)


def _ensure_assembly_anchor(mount_prim_path: str) -> str:
    """The per-mount body every welded part is jointed to, created on first use. KINEMATIC, and
    driven from the mount's live pose by sync_assembly_anchors() rather than jointed to it: a
    dynamic anchor is only as stiff as its own mass allows, and a light one visibly sagged and
    rotated under a real part's weight (confirmed live). See docs/grasp-and-assembly-offsets.md."""
    stage = omni.usd.get_context().get_stage()
    anchor_path = _assembly_anchor_path(mount_prim_path)
    if stage.GetPrimAtPath(anchor_path).IsValid():
        return anchor_path

    # A Scope, not an Xform -- it can't carry a transform, so an anchor's local pose is its world pose.
    stage.DefinePrim(config.ASSEMBLY_WELD_SCOPE_PRIM_PATH, "Scope")
    stage.DefinePrim(anchor_path, "Xform")
    # Default reset_xform_properties=True (unlike everywhere else here): a DefinePrim'd Xform has NO
    # xformOps at all, and only this path authors them -- sync_assembly_anchors() below just writes
    # to them. Safe on a prim this module created, which can't carry a unitsResolve op to strip.
    mount_trans, mount_quat = SingleXFormPrim(
        prim_path=mount_prim_path, reset_xform_properties=False
    ).get_world_pose()
    SingleXFormPrim(prim_path=anchor_path).set_world_pose(position=mount_trans, orientation=mount_quat)

    anchor_prim = stage.GetPrimAtPath(anchor_path)
    # Kinematic: infinite mass to the solver, so the part's weld joint is genuinely rigid no matter
    # what the part weighs. Explicit mass anyway -- the anchor carries no colliders to derive one from.
    rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(anchor_prim)
    rigid_body_api.CreateKinematicEnabledAttr().Set(True)
    mass_api = UsdPhysics.MassAPI.Apply(anchor_prim)
    mass_api.CreateMassAttr().Set(config.ASSEMBLY_WELD_ANCHOR_MASS)
    mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*config.ASSEMBLY_WELD_ANCHOR_DIAGONAL_INERTIA))
    anchor_prim.CreateAttribute(_ASSEMBLY_ANCHOR_MOUNT_ATTR, Sdf.ValueTypeNames.String).Set(mount_prim_path)
    return anchor_path


def sync_assembly_anchors() -> None:
    """Drives every assembly anchor onto its mount's current world pose -- called every teleop frame,
    and what makes a welded part ride main_holder down the conveyor. get_world_pose() drops the
    mount's 0.001 unitsResolve scale, which is the point: the anchor stays unscaled, so a weld's
    localPos on it is unambiguous metres (docs/tool-changer.md's gotcha 8)."""
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
    """Snaps a released part onto its exact nominal assembly pose and welds it to its mount's anchor
    -- the assembly analogue of weld_screw_into_hole(), and the reason a placed part no longer slips
    under gravity. That pose is config.ASSEMBLY_WELD_POSES', NOT the (separate) one P drove to.
    No-ops (returns False) past config.ASSEMBLY_WELD_MAX_DISTANCE, so a far release stays ordinary."""
    from .grasp import assembly_weld_local_pose, compute_part_weld_pose

    relationship = config.ASSEMBLY_RELATIONSHIPS[relationship_name]
    part_prim_path = relationship["part_prim_path"]
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

    anchor_path = _ensure_assembly_anchor(relationship["mount_prim_path"])
    # Re-author the part's own USD xform too, so a Stop restores it ASSEMBLED rather than snapping it
    # back to where it started -- same reason weld_screw_into_hole() does it.
    part_xform.set_world_pose(position=target_trans, orientation=target_quat)
    # body0_local_* IS the weld offset just snapped to -- it already expresses the part's pose in the
    # mount's frame, and the anchor is unscaled and kept coincident with that frame. Must come from
    # the same source as target_trans/quat above, or the joint would pull the part back off it.
    weld_local_position, weld_local_orientation = assembly_weld_local_pose(relationship_name)
    _create_tool_fixed_joint(
        _assembly_weld_joint_path(part_prim_path),
        anchor_path,
        part_prim_path,
        body0_local_position=tuple(weld_local_position),
        body0_local_orientation_wxyz=tuple(weld_local_orientation),
    )
    # Collision off once welded -- a placed part is final, and the joint alone holds it (kinematic
    # anchor == infinite mass). Must come AFTER the joint: the snap leaves the part interpenetrating
    # its mount, an overlap a kinematic anchor can never resolve, so it sits quiet until the arm's
    # motion wakes the bodies and then discharges as a violent shake. See docs/tool-changer.md's gotcha 2.
    _set_prim_collision_enabled(part_prim_path, False)
    print(
        f"[mefron_lib] welded {part_prim_path} at its {relationship_name} pose ({distance:.3f}m "
        "correction), collision off.",
        flush=True,
    )
    return True


def release_assembly_weld(part_prim_path: str) -> bool:
    """Inverse of weld_part_at_assembly_pose(): deletes that part's weld joint AND restores the
    collision that weld turned off, so an un-welded part is grippable again. Stateless: the joint
    prim's presence on the stage IS the state, so this stays correct across a Stop/Play."""
    stage = omni.usd.get_context().get_stage()
    joint_path = _assembly_weld_joint_path(part_prim_path)
    if not stage.GetPrimAtPath(joint_path).IsValid():
        return False
    omni.kit.commands.execute("DeletePrims", paths=[joint_path])
    omni.kit.app.get_app().update()
    _set_prim_collision_enabled(part_prim_path, True)
    print(f"[mefron_lib] released the assembly weld on {part_prim_path}.", flush=True)
    return True


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
        drive.CreateTypeAttr().Set(config.GRIPPER_DRIVE_TYPE)
        drive.CreateStiffnessAttr().Set(config.GRIPPER_DRIVE_STIFFNESS)
        drive.CreateDampingAttr().Set(config.GRIPPER_DRIVE_DAMPING)
