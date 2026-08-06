"""Headless regression test for the O/L release weld (robot.weld_part_at_assembly_pose() /
release_assembly_weld()). Drives no arm and no cuRobo -- it places the part directly and asserts the
weld snaps it onto its nominal ASSEMBLY_RELATIONSHIPS pose, holds it there under gravity, carries it
when main_holder moves, and lets go again.
Run: ${ISAACSIM_ROOT_PATH}/python.sh scripts/test_mefron_assembly_weld_headless.py --headless"""

from __future__ import annotations

import sys

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # Base experience, like test_mefron_screw_headless.py: core prims + UsdPhysics + pose math only.
    simulation_app = SimulationApp({"headless": _headless})

# Must run before any omni/curobo import -- see mefron_lib/kit_bootstrap.py's docstring.
from mefron_lib.kit_bootstrap import clear_stale_robot_configuration, preload_real_packaging  # noqa: E402

preload_real_packaging()

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleRigidPrim, SingleXFormPrim  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402
from mefron_lib import config, grasp, robot  # noqa: E402

_RELATIONSHIP = "finger_print_scanner_on_main_holder"
_SETTLE_FRAMES = 90
# Zero-snap weld (the part is re-authored onto the nominal pose before jointing), so this should
# converge tightly -- 5mm still absorbs PhysX settling jitter without masking a real frame mistake.
_WELD_TOLERANCE = 0.005
# How far off-nominal each case starts: inside / well outside config.ASSEMBLY_WELD_MAX_DISTANCE.
_NEAR_OFFSET = 0.005
_FAR_OFFSET = 0.30
# How far main_holder is teleported for the ride check, and the minimum that counts as having moved.
_MOUNT_NUDGE = 0.15
_MIN_MOUNT_MOVEMENT = 0.05

_failures: list[str] = []


def _check(label: str, condition: bool) -> None:
    print(f"[test_mefron_assembly_weld_headless] {'PASS' if condition else 'FAIL'}: {label}", flush=True)
    if not condition:
        _failures.append(label)


def _settle(simulation_app) -> None:
    for _ in range(_SETTLE_FRAMES):
        simulation_app.update()


def _part_xform() -> SingleXFormPrim:
    # reset_xform_properties=False -- the part carries an xformOp:scale:unitsResolve op the default
    # would silently strip (see grasp.py).
    return SingleXFormPrim(
        prim_path=config.ASSEMBLY_RELATIONSHIPS[_RELATIONSHIP]["part_prim_path"], reset_xform_properties=False
    )


def _collision_enabled_flags(prim_path: str) -> list[bool]:
    stage = omni.usd.get_context().get_stage()
    return [
        bool(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get())
        for p in Usd.PrimRange(stage.GetPrimAtPath(prim_path))
        if p.HasAPI(UsdPhysics.CollisionAPI)
    ]


def main() -> None:
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)
    carb.settings.get_settings().set_bool("/app/player/useFixedTimeStepping", True)

    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)
    stage_context = omni.usd.get_context()
    stage_context.open_stage(str(config.MEFRON_USD))
    for _ in range(120):
        simulation_app.update()

    stage = stage_context.get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")
    robot.clear_assembly_welds()

    relationship = config.ASSEMBLY_RELATIONSHIPS[_RELATIONSHIP]
    part_prim_path = relationship["part_prim_path"]
    mount_prim_path = relationship["mount_prim_path"]
    joint_path = robot._assembly_weld_joint_path(part_prim_path)
    anchor_path = robot._assembly_anchor_path(mount_prim_path)
    part_xform = _part_xform()

    # Far case: a release nowhere near the assembly pose must stay an ordinary release.
    nominal_trans, nominal_quat = grasp.compute_part_target_pose(_RELATIONSHIP)
    far_trans = np.array(nominal_trans) + np.array([_FAR_OFFSET, 0.0, 0.0])
    part_xform.set_world_pose(position=far_trans, orientation=nominal_quat)
    _check("a release far from the assembly pose does not weld", not robot.weld_part_at_assembly_pose(_RELATIONSHIP))
    _check("no weld joint authored for the far release", not stage.GetPrimAtPath(joint_path).IsValid())
    parked_trans, _ = part_xform.get_world_pose()
    _check(
        "the far-released part was left where it was, not snapped",
        float(np.linalg.norm(parked_trans - far_trans)) < _WELD_TOLERANCE,
    )

    # Near case: the arm's real placement error, exaggerated to _NEAR_OFFSET.
    part_xform.set_world_pose(
        position=np.array(nominal_trans) + np.array([_NEAR_OFFSET, 0.0, 0.0]), orientation=nominal_quat
    )
    collisions_before = _collision_enabled_flags(part_prim_path)
    _check("a release near the assembly pose welds", robot.weld_part_at_assembly_pose(_RELATIONSHIP))
    _check("weld joint authored", stage.GetPrimAtPath(joint_path).IsValid())

    anchor_prim = stage.GetPrimAtPath(anchor_path)
    # Kinematic is the load-bearing property: a dynamic anchor is only as stiff as its own mass, and
    # a light one visibly sagged under a real part until this changed.
    _check(
        "the mount's anchor is a KINEMATIC rigid body, so the weld can't sag under the part's weight",
        anchor_prim.IsValid()
        and anchor_prim.HasAPI(UsdPhysics.RigidBodyAPI)
        and bool(UsdPhysics.RigidBodyAPI(anchor_prim).GetKinematicEnabledAttr().Get()),
    )
    if collisions_before:
        _check(
            "the welded part's colliders are disabled, so none can fight the joint holding it",
            not any(_collision_enabled_flags(part_prim_path)),
        )
    else:
        print(
            f"[test_mefron_assembly_weld_headless] NOTE: {part_prim_path} carries no CollisionAPI at all "
            "-- skipping the collision-toggle checks.",
            flush=True,
        )

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    _settle(simulation_app)

    # The weld's whole job: a part that would otherwise slip off the jig sits exactly on its nominal
    # assembly pose, several seconds of simulated gravity later.
    welded_trans, _ = part_xform.get_world_pose()
    drift = float(np.linalg.norm(np.array(nominal_trans) - welded_trans))
    _check(
        f"the welded part sits on its nominal assembly pose under gravity (drift={drift:.4f}m)",
        drift < _WELD_TOLERANCE,
    )

    # The load-bearing check: does an assembled part ride main_holder down the conveyor? The anchor
    # follows the mount through sync_assembly_anchors(), which run_teleop_loop() calls every frame --
    # stood in for here, since this test drives no teleop loop.
    mount_prim = stage.GetPrimAtPath(mount_prim_path)
    # SingleRigidPrim for a simulated body -- teleporting one needs the PhysX-side write, since a
    # plain USD xform write gets overwritten by the body's own pose on the next step.
    mount_mover = (
        SingleRigidPrim(prim_path=mount_prim_path, name="mefron_weld_test_mount")
        if mount_prim.HasAPI(UsdPhysics.RigidBodyAPI)
        else SingleXFormPrim(prim_path=mount_prim_path, reset_xform_properties=False)
    )
    mount_trans_before, mount_quat_before = mount_mover.get_world_pose()
    mount_mover.set_world_pose(
        position=np.array(mount_trans_before) + np.array([0.0, _MOUNT_NUDGE, 0.0]),
        orientation=mount_quat_before,
    )
    robot.sync_assembly_anchors()
    _settle(simulation_app)
    mount_trans_after, _ = mount_mover.get_world_pose()
    mount_moved = float(np.linalg.norm(np.array(mount_trans_after) - np.array(mount_trans_before)))
    _check(
        f"main_holder actually moved before the ride check (moved {mount_moved:.4f}m)",
        mount_moved > _MIN_MOUNT_MOVEMENT,
    )

    # Recomputed off the mount's NEW live pose -- compute_part_target_pose() reads it live, so this
    # is the assembled pose the part should have been carried to.
    moved_nominal_trans, _ = grasp.compute_part_target_pose(_RELATIONSHIP)
    rode_trans, _ = part_xform.get_world_pose()
    ride_error = float(np.linalg.norm(np.array(moved_nominal_trans) - rode_trans))
    _check(
        f"the welded part rides main_holder to its new pose (err={ride_error:.4f}m)",
        ride_error < _WELD_TOLERANCE,
    )

    _check("release_assembly_weld() removes the weld", robot.release_assembly_weld(part_prim_path))
    _check("weld joint gone after release", not stage.GetPrimAtPath(joint_path).IsValid())
    if collisions_before:
        _check(
            "the part's colliders are re-enabled after release",
            _collision_enabled_flags(part_prim_path) == collisions_before,
        )
    _check("releasing an unwelded part is a no-op", not robot.release_assembly_weld(part_prim_path))

    if _failures:
        print(f"[test_mefron_assembly_weld_headless] {len(_failures)} CHECK(S) FAILED: {_failures}", flush=True)
    else:
        print("[test_mefron_assembly_weld_headless] ALL CHECKS PASSED", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
