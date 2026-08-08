"""Screw pick-and-place for the screwdriver tool: presenting, picking onto the wrist, and seating
into main_holder_back_cover's clearance holes. No driving rotation. See docs/tool-changer.md."""

from __future__ import annotations

import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.usd
from isaacsim.core.prims import SingleXFormPrim
from pxr import Gf, UsdPhysics

from . import config
from .assembly import ensure_assembly_anchor
from .toolchanger import _tool_prim_path
from .usd_util import create_fixed_joint, reference_asset


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


def _screw_presenter_anchor_path(index: int) -> str:
    return f"{config.SCREW_SCOPE_PRIM_PATH}/presenter_anchor_{index}"


def clear_screws() -> None:
    """Deletes the script-owned screw scope, so a run never inherits a previous one's screws -- the
    URDF importer rewrites mefron.usd every run. Leaves the (possibly hand-placed) presenter alone."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(config.SCREW_SCOPE_PRIM_PATH).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[config.SCREW_SCOPE_PRIM_PATH])
        omni.kit.app.get_app().update()


def ensure_screw_presenter() -> str:
    """Returns config.SCREW_PRESENTER_PRIM_PATH, creating it at the fallback pose only if absent.
    Same baked-or-fallback duality as TOOL_CHANGE_TARGETS: hand-place it and its own pose wins."""
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
    """Pops a screw in at the presenter, jointed so it can't fall -- a screw is ALWAYS joint-fixed
    to something. Explicit mass/inertia: the placeholder asset has no colliders to derive them."""
    from .grasp import compute_screw_presenter_pose

    stage = omni.usd.get_context().get_stage()
    screw_prim_path = _screw_prim_path(index)
    presenter_trans, presenter_quat = compute_screw_presenter_pose()

    # A Scope, not an Xform -- it can't carry a transform at all, so a screw's own local pose below
    # is guaranteed to be its world pose.
    stage.DefinePrim(config.SCREW_SCOPE_PRIM_PATH, "Scope")
    # Physics stays enabled -- this has to be a real rigid body for a FixedJoint's solver to move it
    # at all (docs/tool-changer.md's gotcha 2).
    reference_asset(
        config.SCREW_USD,
        screw_prim_path,
        local_position=presenter_trans,
        local_orientation_wxyz=presenter_quat,
    )
    screw_prim = stage.GetPrimAtPath(screw_prim_path)
    UsdPhysics.RigidBodyAPI.Apply(screw_prim)
    mass_api = UsdPhysics.MassAPI.Apply(screw_prim)
    mass_api.CreateMassAttr().Set(config.SCREW_MASS)
    mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*config.SCREW_DIAGONAL_INERTIA))

    # body0 is a script-created anchor at the seat pose, not the presenter prim -- identity frames
    # weld with zero snap, and it stays clear of the presenter's 0.001 scale. See docs/tool-changer.md.
    anchor_path = _screw_presenter_anchor_path(index)
    stage.DefinePrim(anchor_path, "Xform")
    SingleXFormPrim(prim_path=anchor_path).set_world_pose(position=presenter_trans, orientation=presenter_quat)
    create_fixed_joint(_screw_presenter_joint_path(index), anchor_path, screw_prim_path)
    return screw_prim_path


def attach_screw_to_wrist(index: int, robot_prim_path: str = config.ROBOT_PRIM_PATH) -> None:
    """Swaps a presented screw's joint onto the wrist -- the "pick" half, and the analogue of
    toolchanger.dock_tool_to_wrist(). Caller must have settled the arm at the pick pose first."""
    stage = omni.usd.get_context().get_stage()
    from .grasp import compute_dependent_world_pose, compute_relative_pose

    presenter_joint_path = _screw_presenter_joint_path(index)
    if stage.GetPrimAtPath(presenter_joint_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[presenter_joint_path])
        omni.kit.app.get_app().update()

    # The NOMINAL carry pose, not the arm's settled one -- welding the live offset froze cuRobo's
    # ~2mm residual into the joint, off the bit axis for the whole cycle. See docs/tool-changer.md.
    tool_trans, tool_quat = SingleXFormPrim(
        prim_path=_tool_prim_path("screwdriver"), reset_xform_properties=False
    ).get_world_pose()
    screw_trans, screw_quat = compute_dependent_world_pose(
        tool_trans, tool_quat, config.SCREW_CARRY_LOCAL_POSITION, config.SCREW_CARRY_LOCAL_ORIENTATION_WXYZ
    )
    # Move the screw onto that pose before jointing, so the correction isn't a visible snap -- safe
    # here since a screw carries no colliders.
    SingleXFormPrim(prim_path=_screw_prim_path(index), reset_xform_properties=False).set_world_pose(
        position=screw_trans, orientation=screw_quat
    )

    # body0 is panda_hand itself: a real rigid body at unit scale, so localPos0 is unambiguous --
    # unlike the docked tool prim, whose 0.001 scale makes a joint frame a guess (gotcha 8).
    hand_path = f"{robot_prim_path}/panda_hand"
    hand_trans, hand_quat = SingleXFormPrim(prim_path=hand_path, reset_xform_properties=False).get_world_pose()
    local_trans, local_quat = compute_relative_pose(hand_trans, hand_quat, screw_trans, screw_quat)

    create_fixed_joint(
        _screw_tip_joint_path(index),
        hand_path,
        _screw_prim_path(index),
        body0_local_position=tuple(local_trans),
        body0_local_orientation_wxyz=tuple(local_quat),
    )


def weld_screw_into_hole(index: int, hole_index: int) -> None:
    """Releases a carried screw into its hole, seated at the hole's own nominal pose rather than
    the arm's. Re-authors the screw's USD xform too, so a Stop restores it AT the hole."""
    stage = omni.usd.get_context().get_stage()
    from .grasp import compute_screw_hole_pose, screw_hole_local_pose

    screw_prim_path = _screw_prim_path(index)
    tip_joint_path = _screw_tip_joint_path(index)
    if stage.GetPrimAtPath(tip_joint_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[tip_joint_path])
        omni.kit.app.get_app().update()

    # The hole's nominal pose (cover's LIVE pose + SCREW_HOLES + depth), not where the arm settled --
    # that left screws 2-3mm out in x/y and ~5mm deep. See docs/tool-changer.md.
    screw_trans, screw_quat = compute_screw_hole_pose(hole_index)
    screw_xform = SingleXFormPrim(prim_path=screw_prim_path, reset_xform_properties=False)
    screw_xform.set_world_pose(position=screw_trans, orientation=screw_quat)

    # The same kinematic anchor a welded part rides, so a placed screw follows the cover. It sits ON
    # the mount's frame, so the joint frame is the hole's own local pose -- from the same helper.
    anchor_path = ensure_assembly_anchor(config.SCREW_HOLE_MOUNT_PRIM_PATH)
    hole_local_trans, hole_local_quat = screw_hole_local_pose(hole_index)
    create_fixed_joint(
        _screw_hole_joint_path(index),
        anchor_path,
        screw_prim_path,
        body0_local_position=tuple(hole_local_trans),
        body0_local_orientation_wxyz=tuple(hole_local_quat),
    )
