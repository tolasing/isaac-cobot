"""The arm itself: mounting it on its UR10 pedestal, and gripper physics tuning (friction
material, drive stiffness). See docs/mefron-history.md and docs/fr5-migration.md."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.usd
from isaacsim.core.prims import SingleXFormPrim
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from . import config, feeder
from .usd_util import import_urdf, un_instance_ancestor

# Pre-MovePrim intermediate paths a mount_arm() import can land at, plus the retired pre-ATC
# arm2/arm3 paths -- a stray Save bakes any of these into the shared mefron.usd as an orphan.
_STRAY_HISTORICAL_PANDA_PATH = "/panda"
_STRAY_HISTORICAL_GRIPPER_TOOL_PATH = "/panda_gripper_only"
# /World/Franka joins the list now the live arm is an FR5 at its own path -- see docs/fr5-migration.md.
_STRAY_HISTORICAL_ARM_PATHS = ["/World/Franka", "/World/Franka2", "/World/Franka3"]


def clear_stray_robot_prims() -> None:
    """Deletes robot prims a past session's stray Save baked into mefron.usd. Must run right after
    open_stage(), before the settle pump -- mount_arm()'s own cleanup is too late."""
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


def mount_arm(
    prim_path: str = config.ROBOT_PRIM_PATH,
    mount_position=config.MOUNT_POSITION,
    mount_orientation_wxyz=config.MOUNT_ORIENTATION_WXYZ,
) -> None:
    """Mounts the vendored FAIRINO FR5 at prim_path/mount_position. Must run BEFORE
    kit_experience.enable_full_experience_extensions() -- the URDF importer crashes otherwise."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(prim_path).IsValid():
        # Without this, import_urdf's MovePrim silently uniquifies to e.g. /World/FR5_01. The
        # frame pump lets the delete commit before the importer's own uniqueness check runs.
        omni.kit.commands.execute("DeletePrims", paths=[prim_path])
        omni.kit.app.get_app().update()

    import_urdf(
        urdf_path=Path(config.FR5_URDF_PATH),
        prim_path=prim_path,
        default_drive_strength=config.FR5_DRIVE_STRENGTH,
        default_position_drive_damping=config.FR5_DRIVE_DAMPING,
    )
    xform = SingleXFormPrim(prim_path=prim_path)
    xform.set_world_pose(
        position=np.array(mount_position),
        orientation=np.array(mount_orientation_wxyz),
    )
    attach_tool_flange_frame(prim_path)


def attach_tool_flange_frame(prim_path: str = config.ROBOT_PRIM_PATH) -> str:
    """Authors the ee frame as a live child of wrist3_link. cuRobo gets the same frame from the
    XRDF's add_frame; this is the prim grasp/screw code reads the ee's world pose off."""
    stage = omni.usd.get_context().get_stage()
    flange_path = f"{prim_path}/{config.FR5_EE_LINK}/{config.FR5_EE_FRAME_NAME}"
    stage.DefinePrim(flange_path, "Xform")
    SingleXFormPrim(prim_path=flange_path).set_local_pose(
        translation=np.array([0.0, 0.0, config.FR5_TOOL_FLANGE_OFFSET]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )
    return flange_path


def apply_home_pose(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Stages FR5_HOME_JOINT_POSITIONS on every joint's angular drive + JointStateAPI, so PhysX
    settles there on Play instead of the URDF's outstretched, singular all-zero pose."""
    from pxr import PhysxSchema

    stage = omni.usd.get_context().get_stage()
    for joint_name, radians in zip(config.FR5_JOINT_NAMES, config.FR5_HOME_JOINT_POSITIONS):
        joint_prim = stage.GetPrimAtPath(f"{prim_path}/joints/{joint_name}")
        if not joint_prim.IsValid():
            print(f"[mefron_lib] WARNING: {prim_path}/joints/{joint_name} not found -- skipping home pose.", flush=True)
            continue
        # UsdPhysics angular quantities are DEGREES, unlike the URDF's radians and cuRobo's.
        degrees = float(np.degrees(radians))
        UsdPhysics.DriveAPI.Apply(joint_prim, "angular").CreateTargetPositionAttr().Set(degrees)
        PhysxSchema.JointStateAPI.Apply(joint_prim, "angular").CreatePositionAttr().Set(degrees)


def apply_accent_color(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Paints FR5_ACCENT_LINK_NAMES with FR5_ACCENT_COLOR_RGB. Runtime-only: the vendored URDF has
    one flat grey for every link and no accent color to honour instead."""
    from pxr import UsdShade

    stage = omni.usd.get_context().get_stage()
    material_path = f"{prim_path}/Looks/{config.FR5_ACCENT_MATERIAL_NAME}"
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    # OmniPBR MDL, matching the URDF importer's own material convention exactly -- a
    # UsdPreviewSurface authored here rendered as untouched white. See docs/fr5-migration.md.
    shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*config.FR5_ACCENT_COLOR_RGB)
    )
    material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")

    for link_name in config.FR5_ACCENT_LINK_NAMES:
        visuals_path = f"{prim_path}/{link_name}/visuals"
        visuals_prim = stage.GetPrimAtPath(visuals_path)
        if not visuals_prim.IsValid():
            print(f"[mefron_lib] WARNING: {visuals_path} not found -- skipping accent color.", flush=True)
            continue
        # Every link's visuals is a shared instance prototype; binding through one silently no-ops.
        un_instance_ancestor(visuals_prim, visuals_path)
        # The importer binds the grey on an intermediate Xform as strongerThanDescendants, which
        # beats any binding on the Mesh below it -- so re-bind wherever a binding is already
        # authored, at that same strength, not just on the Mesh.
        rebound = [
            prim
            for prim in Usd.PrimRange(visuals_prim)
            if prim.IsA(UsdGeom.Imageable)
            and (
                prim.IsA(UsdGeom.Mesh)
                or bool(UsdShade.MaterialBindingAPI(prim).GetDirectBinding().GetMaterialPath())
            )
        ]
        if not rebound:
            print(f"[mefron_lib] WARNING: {visuals_path} has nothing bindable -- skipping accent color.", flush=True)
            continue
        for prim in rebound:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants)


def un_instance_link_meshes(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Un-shares every link's visuals/collisions. The Lula Robot Description Editor refuses to
    auto-generate collision spheres from instanceable meshes, and the importer makes them all so."""
    stage = omni.usd.get_context().get_stage()
    for link_name in [config.FR5_BASE_LINK, *config.FR5_MOVING_LINK_NAMES]:
        for scope in ("visuals", "collisions"):
            scope_path = f"{prim_path}/{link_name}/{scope}"
            scope_prim = stage.GetPrimAtPath(scope_path)
            if scope_prim.IsValid():
                un_instance_ancestor(scope_prim, scope_path)


def print_arm_inventory(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Prints the link/joint names the importer actually produced. Step 2's fr5.yml must be written
    against these, not against the URDF's own names -- see docs/fr5-migration.md."""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(prim_path)
    if not root.IsValid():
        print(f"[mefron_lib] {prim_path}: MISSING -- nothing to inventory.", flush=True)
        return

    links = [p.GetName() for p in root.GetChildren() if p.GetTypeName() == "Xform"]
    joints = [p.GetName() for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.DriveAPI)]
    print(f"[mefron_lib] {prim_path} links ({len(links)}): {links}", flush=True)
    print(f"[mefron_lib] {prim_path} driven joints ({len(joints)}): {joints}", flush=True)


def remove_parallel_jaw_gripper(prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """FRANKA-ONLY, UNCALLED since the FR5 swap -- it ships a bare flange with no hand to strip.
    Deactivates (not deletes) the Franka's own finger links + drive joints. docs/fr5-migration.md."""
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
    """FRANKA-ONLY, UNCALLED since the FR5 swap. Hides panda_hand/visuals and disables its
    collisions -- the ATC's hidden housing under the male coupler. docs/fr5-migration.md."""
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
    """Authors the surface_gripper attach mechanism on the FR5's flange: one UsdPhysics.Joint with
    compliance tuning -- without it lifting leaves the object behind. See docs/mefron-history.md."""
    from usd.schema.isaac import robot_schema

    stage = omni.usd.get_context().get_stage()
    hand_path = f"{prim_path}/{config.FR5_EE_LINK}"
    joint_path = f"{hand_path}/{config.SURFACE_GRIPPER_JOINT_PRIM_NAME}"
    gripper_path = f"{hand_path}/{config.SURFACE_GRIPPER_PRIM_NAME}"

    for path in (joint_path, gripper_path):
        if stage.GetPrimAtPath(path).IsValid():
            omni.kit.commands.execute("DeletePrims", paths=[path])
    omni.kit.app.get_app().update()

    joint = UsdPhysics.Joint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([hand_path])
    # body1 MUST NOT be left unset: that means the WORLD, and when the manager engages the
    # constraint on V the wrist is pinned there (measured 0.30 rad of travel -> 0.0000). The
    # suction tool co-moves with body0 -- V/L are gated on it being docked. docs/fr5-migration.md.
    joint.CreateBody1Rel().SetTargets([config.TOOL_CHANGE_TARGETS["suction"]["baked_tool_prim_path"]])
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*config.SURFACE_GRIPPER_LOCAL_POSITION))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(*config.SURFACE_GRIPPER_LOCAL_ORIENTATION_WXYZ))
    # Body1's frame must land on the SAME point, or the constraint carries a permanent violation and
    # saturates the moment it engages. The docked tool's origin is the flange, so the cup is
    # SUCTION_TOOL_REACH out along its Z -- exactly where localPos0 puts body0's frame.
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, config.SUCTION_TOOL_REACH))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(*config.SURFACE_GRIPPER_LOCAL_ORIENTATION_WXYZ))

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
    # Every GUI-placed copy of a high-friction part too, or only the original grips properly.
    for high_friction_prim_path in config.HIGH_FRICTION_PRIM_PATHS:
        target_paths += feeder.part_instance_prim_paths(high_friction_prim_path) or [high_friction_prim_path]
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
