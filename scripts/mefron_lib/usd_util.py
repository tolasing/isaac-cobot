"""USD/PhysX primitives shared by robot.py, toolchanger.py, assembly.py and screws.py --
URDF import, asset referencing, fixed joints, instancing and collision toggles."""

from __future__ import annotations

import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.usd
from isaacsim.core.prims import SingleXFormPrim
from pxr import Gf, Usd, UsdPhysics


def import_urdf(
    urdf_path,
    prim_path: str,
    default_drive_strength: float,
    default_position_drive_damping: float,
) -> str:
    """Imports a URDF via URDFParseAndImportFile and moves it to prim_path. Drive defaults are
    caller-supplied -- they are robot-specific, not a sane global."""
    # isaacsim.asset.importer.urdf exports no directly-constructible config class -- the
    # URDFCreateImportConfig command is the only way to get a properly-initialized ImportConfig.
    import_config = omni.kit.commands.execute("URDFCreateImportConfig")[1]
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.import_inertia_tensor = True
    import_config.self_collision = False
    import_config.distance_scale = 1.0
    import_config.default_drive_strength = default_drive_strength
    import_config.default_position_drive_damping = default_position_drive_damping

    status, imported_prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
    )
    if not status:
        raise RuntimeError(f"URDF import failed for {urdf_path}")

    if imported_prim_path != prim_path:
        omni.kit.commands.execute("MovePrim", path_from=imported_prim_path, path_to=prim_path)
    return prim_path


def un_instance_ancestor(prim, context_path: str) -> None:
    """Walks up from prim to the nearest instanceable ancestor and un-shares it -- authoring on an
    instance-proxy prim silently no-ops. See docs/mefron-history.md."""
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


def reference_asset(
    usd_path,
    prim_path: str,
    local_position,
    local_orientation_wxyz,
    local_scale=None,
) -> None:
    """References a USD asset at prim_path with a local pose, leaving its baked-in physics enabled
    -- every caller here needs a real rigid body a FixedJoint can move (screws, dockable tools)."""
    from isaacsim.core.utils.stage import add_reference_to_stage

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(prim_path).IsValid():
        # Re-run safety -- otherwise a second run in the same session lands a uniquified duplicate.
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


def create_fixed_joint(
    joint_path: str,
    body0_path: str,
    body1_path: str,
    body0_local_position=(0.0, 0.0, 0.0),
    body0_local_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
    body1_local_position=(0.0, 0.0, 0.0),
    body1_local_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
) -> None:
    """Plain rigid UsdPhysics.FixedJoint, local frames defaulting to each body's own origin. No
    compliance, unlike the surface gripper's soft D6; excludeFromArticulation for panda_hand."""
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


def collision_enabled_flags(prim_path: str) -> list:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return []
    return [
        bool(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get())
        for p in Usd.PrimRange(prim)
        if p.HasAPI(UsdPhysics.CollisionAPI)
    ]


def set_prim_collision_enabled(prim_path: str, enabled: bool) -> None:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(enabled)
