"""Mounting the Franka onto mefron.usd's SEKTION cabinet plate, and the gripper physics tuning
(friction material, drive stiffness) needed for a stable grasp.
"""

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


def mount_franka(
    prim_path: str = config.ROBOT_PRIM_PATH,
    mount_position=config.MOUNT_POSITION,
    mount_orientation_wxyz=config.MOUNT_ORIENTATION_WXYZ,
) -> None:
    """Mounts cuRobo's bundled Franka Panda at prim_path/mount_position. Defaults to the first arm's
    own constants; the second arm calls this with config.ROBOT_2_PRIM_PATH/MOUNT_2_POSITION/
    MOUNT_2_ORIENTATION_WXYZ instead -- same URDF, same drive tuning, only the destination differs.

    A second real call to this function (i.e. a second native URDF import in the same process)
    reliably crashes Kit's isaacsim.asset.importer.urdf plugin -- but ONLY when the full experience's
    ~120 extra extensions (isaacsim.exp.full.kit, used for the Physics debug-viz menu) are already
    loaded at the time of the second import; confirmed live it's specifically about them being
    *present during* that second import call, not merely present in the process at all -- mounting
    both Frankas first under the plain base experience, then enabling those same extra extensions
    afterward, reproduces the identical final feature set with zero crash. See
    kit_experience.enable_full_experience_extensions(), which mefron.py calls right after this
    function's second (arm 2) call returns, and docs/mefron-history.md for the full diagnosis
    (an AddInternalReference()-based workaround was tried and rejected first: the imported URDF's
    joints all use absolute-path body0/body1 relationships, so a referenced copy's joints still
    target the original robot's rigid bodies instead of its own -- confirmed live, not just theorized)."""
    from curobo.util_file import get_assets_path, join_path

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(prim_path).IsValid():
        # mefron.usd is meant to stay Franka-free -- the robot only ever exists in this runtime
        # session -- but a stray Save can persist it to disk anyway. Without this, import_cr5's
        # MovePrim silently uniquifies to e.g. /World/Franka_01 instead of landing on prim_path,
        # leaving a duplicate robot behind on every subsequent run. Same pattern as
        # mefron_gripper_probe.py's spawn_gripper_probe().
        omni.kit.commands.execute("DeletePrims", paths=[prim_path])
        # A frame pump so the deletion is fully committed to the stage before import_cr5() runs its
        # own uniqueness check below -- without this, a still-in-flight delete can make the importer
        # think prim_path is still occupied and uniquify to prim_path + "_01" instead, leaving both
        # the just-deleted and the freshly-imported robot behind. Confirmed live this happens for a
        # stray Save left over from interactive testing (a real, not hypothetical, failure mode).
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


def remove_parallel_jaw_gripper(prim_path: str = config.ROBOT_2_PRIM_PATH) -> None:
    """Deactivates the Franka's parallel-jaw finger links + their drive joints on prim_path, for an
    arm that's being converted to a suction end-effector instead (see attach_suction_gripper()).
    Leaves panda_hand/ee_link/right_gripper alone -- only the two finger links + their prismatic
    joints go.

    Deactivates rather than deletes -- same choice docs/mefron-history.md already made for the
    duplicate /PhysicsScene issue, and for the same reason: reversible, in-memory only, never
    touches the file on disk. Not just a style preference here: confirmed live that
    `omni.kit.commands.execute("DeletePrims", ...)` silently no-ops for these specific prims
    (returns success, no error, but the prims stay valid/active) since their specs live across
    the URDF importer's disk-persisted, multi-layer `configuration/` stack (see CLAUDE.md's
    importer-side-effect gotcha) rather than purely on the current edit target. `Usd.Prim.SetActive
    (False)` authors directly on the stage's current edit target regardless, and works."""
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
    """Makes prim_path's panda_hand/visuals invisible -- for an arm converted to suction-only, where
    the bare Franka parallel-jaw housing (minus fingers, see remove_parallel_jaw_gripper()) still
    reads as "a gripper is still there" even with the fingers gone.

    Visibility only, deliberately NOT deactivation, and deliberately leaves panda_hand/collisions
    alone: panda_hand itself must stay active (it's cuRobo's franka.yml ee_link, and
    ee_link/suction_gripper are both parented under it) -- only its own visuals sub-scope is hidden.
    Its collisions sub-scope is left active on purpose: config.OBSTACLE_PRIM_PATHS includes each arm's
    own root path so cuRobo treats the OTHER arm as a real collision obstacle -- dropping panda_hand's
    collision geometry would make arm 1's planner stop seeing it there at all, which is a physics-
    behavior change, not the purely visual fix this function is for."""
    stage = omni.usd.get_context().get_stage()
    visuals_path = f"{prim_path}/panda_hand/visuals"
    prim = stage.GetPrimAtPath(visuals_path)
    if not prim.IsValid():
        print(f"[mefron_lib] WARNING: {visuals_path} not found -- skipping hide.", flush=True)
        return
    UsdGeom.Imageable(prim).MakeInvisible()


def attach_suction_gripper(prim_path: str = config.ROBOT_2_PRIM_PATH) -> None:
    """References config.SUCTION_GRIPPER_USD (see robots/ur10_suction/SOURCE.md) as a child of
    prim_path's panda_hand -- cuRobo's franka.yml ee_link, the same frame grasp.py's poses are
    already expressed in, so the gripper rides along rigidly with the hand. Does not remove/hide
    the Franka's own hand -- callers converting an arm to suction-only should also call
    remove_parallel_jaw_gripper()/hide_hand_housing() first (see mefron.py). No suction control
    (attach/detach, keyboard key) is wired up yet. Positioned at config.SUCTION_GRIPPER_LOCAL_POSITION/ORIENTATION_WXYZ --
    see that constant's own comment for how it was derived from the asset's own geometry.

    Unlike the borrowed UR10 asset, this custom SolidWorks-exported flange comes in with a baked-in
    collider on its own mesh (confirmed live 2026-07-17: a PhysX raycast from the cup tip along
    panda_hand's own +Z self-hit this asset's Revolve1/Mesh at distance 0.0, before ever reaching
    outward -- silently breaking attach_surface_gripper_physics()'s grab raycast, since the ray
    starts sitting right on this mesh's own surface) plus a baked-in enabled RigidBodyAPI (confirmed
    live 2026-07-17: PhysX logs "missing xformstack reset when child of another enabled rigid body"
    for this prim under panda_hand once mounted -- a nested-rigid-body hierarchy, not merely a stray
    collider). Both disabled below via CollisionEnabled/RigidBodyEnabled=False rather than removing
    the APIs outright: a 2026-07-18 attempt at prim.RemoveAPI(...) instead (to also silence the
    xformstack warning, which disabling alone doesn't -- that check is structural, on
    HasAPI(RigidBodyAPI), not on whether the instance is enabled) coincided with the suction gripper
    mesh going invisible in the user's own GUI run; reverted back to this disable-only version to
    isolate whether RemoveAPI was actually the cause before trying again. So: the xformstack warning
    is expected to still appear with this version -- that's the known tradeoff pending a fix that
    gets both."""
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


def attach_surface_gripper_physics(prim_path: str = config.ROBOT_2_PRIM_PATH) -> str:
    """Authors the real isaacsim.robot.schema/isaacsim.robot.surface_gripper attach mechanism on
    prim_path's panda_hand -- the actual rigid body (the visual suction_gripper child referenced by
    attach_suction_gripper() has zero physics of its own -- that function strips any collider/rigid-
    body its USD source brings in, see its own docstring for why that stripping is load-bearing here).
    One plain UsdPhysics.Joint tagged with IsaacAttachmentPointAPI, plus PhysicsLimitAPI/
    PhysicsDriveAPI compliance tuning (values taken directly from NVIDIA's own bundled reference,
    isaacsim.robot.surface_gripper's data/SurfaceGripper_gantry.usda) so the joint actually
    constrains the grabbed object once attached -- confirmed live 2026-07-17 that without this, the
    joint is a fully-free D6 (the schema's own documented default): the manager reports
    Closed/gripped correctly, but nothing physically holds the object, so lifting leaves it behind.
    transX/transY are locked (schema semantics: low > high == locked, per PhysicsLimitAPI's own
    doc); transZ gets a small compliant range + spring (give along the suction axis, like a real cup
    flexing slightly); rotX/rotY get a looser spring (tilt compliance); rotZ is much stiffer
    (resists the object spinning about the suction axis itself). This authoring happens before
    Play/attach, so the low>high locked-axis convention is honored by the initial parse -- hot-
    patching an already-live joint mid-session needs a tiny valid range instead, not an inverted one.
    plus the IsaacSurfaceGripper bookkeeping prim pointing at it. body1 left unbound; the
    SurfaceGripperManager rebinds it live to whatever it finds within max_grip_distance at close
    time -- see robots/ur10_suction/short_gripper.usd's own Suction_Joint for the equivalent pattern
    on the borrowed asset this one supersedes.
    excludeFromArticulation is the one physics attribute that ISN'T optional here: panda_hand is a
    real articulation link, and without it PhysX tries to fold this joint into the Franka's own
    reduced-coordinate solve instead of treating it as an auxiliary maximal-coordinate constraint."""
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
