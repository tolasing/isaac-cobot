"""Headless regression test for the screw pick-and-place mechanics (robot.present_screw()/
attach_screw_to_wrist()/weld_screw_into_hole() and grasp.compute_ee_target_for_screw_pose()).
Drives no cuRobo plan -- it moves the arm's joints directly and asserts the screw actually rides the
wrist while carried, and actually stays put in world space once welded into a hole.
Run: ${ISAACSIM_ROOT_PATH}/python.sh scripts/test_mefron_screw_headless.py --headless"""

from __future__ import annotations

import sys

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # Base experience, like test_mefron_tool_changer_headless.py and unlike the full-kit rule for
    # headless runs: nothing here needs a full-experience extension (core prims + UsdPhysics + pure
    # pose math, no grasp_editor/conveyor), and the full kit's startup dominates the runtime.
    simulation_app = SimulationApp({"headless": _headless})

# Must run before any omni/curobo import -- see mefron_lib/kit_bootstrap.py's docstring.
from mefron_lib.kit_bootstrap import clear_stale_robot_configuration, preload_real_packaging  # noqa: E402

preload_real_packaging()

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: E402
from pxr import UsdPhysics  # noqa: E402
from mefron_lib import config, grasp, robot  # noqa: E402

_SETTLE_FRAMES = 90
# Zero-snap welds (anchor placed at the body's own pose), so these should converge tightly -- 5mm
# still absorbs ordinary PhysX settling jitter without masking a real frame/scale mistake.
_WELD_TOLERANCE = 0.005
# The carried screw's offset from the bit is a nominal constant, not a settled measurement, so it
# should converge far tighter than a weld -- 1mm still catches the ~2mm error this guards against.
_CARRY_TOLERANCE = 0.001
# Pure pose composition, no physics -- anything above float noise here is a real math bug.
_MATH_TOLERANCE = 1.0e-6
# Panda's nominal max reach, for the reachability warning only (never a failure -- an out-of-envelope
# hole is a scene-layout issue, not a code defect). See docs/tool-changer.md's screw section.
_PANDA_REACH = 0.855

_failures: list[str] = []


def _check(label: str, condition: bool) -> None:
    print(f"[test_mefron_screw_headless] {'PASS' if condition else 'FAIL'}: {label}", flush=True)
    if not condition:
        _failures.append(label)


def _settle(simulation_app) -> None:
    for _ in range(_SETTLE_FRAMES):
        simulation_app.update()


def _screw_pose(index: int):
    return SingleXFormPrim(
        prim_path=robot._screw_prim_path(index), reset_xform_properties=False
    ).get_world_pose()


def _hand_pose() -> tuple:
    return SingleXFormPrim(
        prim_path=f"{config.ROBOT_PRIM_PATH}/panda_hand", reset_xform_properties=False
    ).get_world_pose()


_ARM_POSES = [(-1.30, -2.50), (-0.90, -2.00)]
# Below this, an arm "move" didn't really move and the ride/stay checks would pass vacuously.
_MIN_ARM_MOVEMENT = 0.05


def _move_arm(simulation_app, pose_index: int) -> float:
    """Drives panda_joint2/4 to one of two absolute in-limit configurations (absolute, not a delta --
    a delta off an unknown start can land outside a joint's limit and get silently clamped to no
    motion). Rebuilds the articulation handle first: authoring a joint prim live stales whatever
    handle existed before, same gotcha CLAUDE.md documents for Stop. Returns how far the hand moved."""
    before_trans, _ = _hand_pose()
    articulation = SingleArticulation(prim_path=config.ROBOT_PRIM_PATH, name=f"mefron_screw_test_robot_{pose_index}")
    articulation.initialize()
    positions = articulation.get_joint_positions()
    positions[1], positions[3] = _ARM_POSES[pose_index]
    articulation.set_joint_positions(positions)
    _settle(simulation_app)
    after_trans, _ = _hand_pose()
    return float(np.linalg.norm(after_trans - before_trans))


def _check_ee_target_round_trip(label: str, ee_link_prim_path: str, want_trans, want_quat) -> None:
    """compute_ee_target_for_screw_pose() inverts SCREW_CARRY_LOCAL_* and the live tool->ee offset;
    this re-derives the screw pose from its answer using the OPPOSITE (ee->tool) direction, so a
    dropped inverse or a flipped frame shows up as a real mismatch rather than a plausible number."""
    ee_trans, ee_quat = grasp.compute_ee_target_for_screw_pose(ee_link_prim_path, want_trans, want_quat)

    tool_trans, tool_quat = SingleXFormPrim(
        prim_path=robot._tool_prim_path("screwdriver"), reset_xform_properties=False
    ).get_world_pose()
    live_ee_trans, live_ee_quat = SingleXFormPrim(
        prim_path=ee_link_prim_path, reset_xform_properties=False
    ).get_world_pose()
    tool_wrt_ee = grasp.compute_relative_pose(live_ee_trans, live_ee_quat, tool_trans, tool_quat)
    placed_tool = grasp.compute_dependent_world_pose(ee_trans, ee_quat, *tool_wrt_ee)
    got_trans, got_quat = grasp.compute_dependent_world_pose(
        *placed_tool, config.SCREW_CARRY_LOCAL_POSITION, config.SCREW_CARRY_LOCAL_ORIENTATION_WXYZ
    )

    position_error = float(np.linalg.norm(np.array(want_trans) - got_trans))
    # abs() on the dot product -- q and -q are the same rotation.
    orientation_error = 1.0 - abs(float(np.dot(np.array(want_quat), got_quat)))
    _check(
        f"{label}: ee target round-trips to the wanted screw pose (pos err={position_error:.2e}m, "
        f"rot err={orientation_error:.2e})",
        position_error < _MATH_TOLERANCE and orientation_error < _MATH_TOLERANCE,
    )
    reach = float(np.linalg.norm(ee_trans - np.array(config.MOUNT_POSITION)))
    if reach > _PANDA_REACH:
        print(
            f"[test_mefron_screw_headless] WARNING: {label}'s ee target is {reach:.3f}m from the mount, "
            f"past the Panda's ~{_PANDA_REACH}m reach -- cuRobo will fail to plan it. Move main_holder "
            "closer (GUI or conveyor) or lower config.SCREW_APPROACH_CLEARANCE.",
            flush=True,
        )
    else:
        print(f"[test_mefron_screw_headless] {label}'s ee target is {reach:.3f}m from the mount.", flush=True)


def main() -> None:
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)
    carb.settings.get_settings().set_bool("/app/player/useFixedTimeStepping", True)

    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)
    stage_context = omni.usd.get_context()
    stage_context.open_stage(str(config.MEFRON_USD))
    robot.clear_stray_robot_prims()
    for _ in range(120):
        simulation_app.update()

    robot.mount_franka()
    robot.remove_parallel_jaw_gripper()
    robot.hide_hand_housing()
    robot.attach_tool_changer_male_coupler()

    for tool_name in config.TOOL_CHANGE_TARGETS:
        robot.spawn_dockable_tool(tool_name)
        robot.park_tool_at_rack(tool_name)

    robot.clear_screws()
    robot.ensure_screw_presenter()
    robot.present_screw(0)

    stage = stage_context.get_stage()
    ee_link_prim_path = f"{config.ROBOT_PRIM_PATH}/panda_hand"
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    screw_prim = stage.GetPrimAtPath(robot._screw_prim_path(0))
    _check("screw 0 spawned as a real rigid body", screw_prim.IsValid() and screw_prim.HasAPI(UsdPhysics.RigidBodyAPI))
    _check(
        "screw 0 has an explicit mass (it has no colliders for PhysX to derive one from)",
        screw_prim.HasAPI(UsdPhysics.MassAPI),
    )
    _check("screw 0 starts jointed to the presenter", stage.GetPrimAtPath(robot._screw_presenter_joint_path(0)).IsValid())

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    _settle(simulation_app)

    # The presenter weld's whole job: a screw that would otherwise free-fall stays exactly where it
    # was presented, several seconds of simulated gravity later.
    presenter_trans, _ = grasp.compute_screw_presenter_pose()
    parked_trans, _ = _screw_pose(0)
    drift = float(np.linalg.norm(presenter_trans - parked_trans))
    _check(f"screw 0 stays at the presenter under gravity (drift={drift:.4f}m)", drift < _WELD_TOLERANCE)

    robot.dock_tool_to_wrist("screwdriver")
    _settle(simulation_app)
    _check(
        "screwdriver docked for the screw legs",
        stage.GetPrimAtPath(robot._wrist_joint_path("screwdriver")).IsValid(),
    )

    # The CAD-derived bit tip should sit one tool-length out from the wrist once docked.
    tool_trans, tool_quat = SingleXFormPrim(
        prim_path=robot._tool_prim_path("screwdriver"), reset_xform_properties=False
    ).get_world_pose()
    tip_trans, _ = grasp.compute_dependent_world_pose(
        tool_trans, tool_quat, config.SCREWDRIVER_TIP_LOCAL_POSITION, [1.0, 0.0, 0.0, 0.0]
    )
    hand_trans, _ = _hand_pose()
    tip_distance = float(np.linalg.norm(tip_trans - hand_trans))
    expected_tip_distance = config.SCREWDRIVER_TIP_LOCAL_POSITION[2]
    _check(
        f"bit tip sits {expected_tip_distance:.4f}m out from panda_hand (measured {tip_distance:.4f}m)",
        abs(tip_distance - expected_tip_distance) < _WELD_TOLERANCE,
    )

    _check_ee_target_round_trip("presenter", ee_link_prim_path, *grasp.compute_screw_presenter_pose())
    for hole_index in range(len(config.SCREW_HOLES)):
        _check_ee_target_round_trip(f"hole {hole_index}", ee_link_prim_path, *grasp.compute_screw_hole_pose(hole_index))

    # Pick: the screw leaves the presenter for the wrist. Not driven to the pick pose first (no cuRobo
    # here), so the weld's own correction to the nominal carry pose is the whole distance -- which is
    # exactly what's asserted next.
    robot.attach_screw_to_wrist(0)
    _settle(simulation_app)
    _check("screw 0 jointed to the wrist after picking", stage.GetPrimAtPath(robot._screw_tip_joint_path(0)).IsValid())
    _check(
        "screw 0's presenter joint removed after picking",
        not stage.GetPrimAtPath(robot._screw_presenter_joint_path(0)).IsValid(),
    )

    # Regression: the weld used to freeze the arm's live settled pose into the joint, so cuRobo's
    # ~2mm residual approach error left the screw permanently off the bit axis (localPos0 x/y ~1mm
    # instead of 0). Tighter than _WELD_TOLERANCE on purpose -- 5mm wouldn't have caught it.
    tool_trans, tool_quat = SingleXFormPrim(
        prim_path=robot._tool_prim_path("screwdriver"), reset_xform_properties=False
    ).get_world_pose()
    carried_local_trans, _ = grasp.compute_relative_pose(tool_trans, tool_quat, *_screw_pose(0))
    carry_error = float(np.linalg.norm(carried_local_trans - np.array(config.SCREW_CARRY_LOCAL_POSITION)))
    _check(
        f"carried screw 0 sits on the bit's nominal carry pose, not wherever the arm settled "
        f"(err={carry_error:.5f}m)",
        carry_error < _CARRY_TOLERANCE,
    )

    carried_before = grasp.compute_relative_pose(*_hand_pose(), *_screw_pose(0))
    moved = _move_arm(simulation_app, 0)
    _check(f"the arm actually moved before the ride check (hand moved {moved:.4f}m)", moved > _MIN_ARM_MOVEMENT)
    carried_after = grasp.compute_relative_pose(*_hand_pose(), *_screw_pose(0))
    ride_error = float(np.linalg.norm(np.array(carried_before[0]) - np.array(carried_after[0])))
    _check(f"screw 0 rides the wrist through an arm move (offset drift={ride_error:.4f}m)", ride_error < _WELD_TOLERANCE)

    # Place: the screw leaves the wrist for a static anchor at its own current pose.
    placed_trans_before, _ = _screw_pose(0)
    robot.weld_screw_into_hole(0, 0)
    _settle(simulation_app)
    _check("screw 0 jointed into hole 0 after placing", stage.GetPrimAtPath(robot._screw_hole_joint_path(0)).IsValid())
    _check(
        "screw 0's tip joint removed after placing",
        not stage.GetPrimAtPath(robot._screw_tip_joint_path(0)).IsValid(),
    )

    # Regression, the place-side twin of the carry check above: welding the arm's settled pose left
    # screws ~2-3mm out in x/y and ~5mm too deep in main_holder's own frame. Adding the insertion
    # depth straight onto local_position is exact only while every hole's orientation is identity.
    mount_trans, mount_quat = SingleXFormPrim(
        prim_path=config.SCREW_HOLE_MOUNT_PRIM_PATH, reset_xform_properties=False
    ).get_world_pose()
    placed_local_trans, _ = grasp.compute_relative_pose(mount_trans, mount_quat, *_screw_pose(0))
    want_local = np.array(config.SCREW_HOLES[0]["local_position"]) + np.array(
        [0.0, 0.0, config.SCREW_HOLE_INSERTION_DEPTH]
    )
    hole_error = float(np.linalg.norm(placed_local_trans - want_local))
    _check(
        f"placed screw 0 seats at hole 0's nominal pose in main_holder's frame (err={hole_error:.5f}m)",
        hole_error < _CARRY_TOLERANCE,
    )

    moved = _move_arm(simulation_app, 1)
    _check(f"the arm actually moved away before the stay check (hand moved {moved:.4f}m)", moved > _MIN_ARM_MOVEMENT)
    placed_trans_after, _ = _screw_pose(0)
    stay_error = float(np.linalg.norm(placed_trans_before - placed_trans_after))
    _check(
        f"placed screw 0 stays put in world space while the arm moves away (drift={stay_error:.4f}m)",
        stay_error < _WELD_TOLERANCE,
    )

    # And the next screw pops in, so the presenter is never empty mid-sequence.
    robot.present_screw(1)
    _settle(simulation_app)
    presenter_trans, _ = grasp.compute_screw_presenter_pose()
    next_trans, _ = _screw_pose(1)
    drift = float(np.linalg.norm(presenter_trans - next_trans))
    _check(f"screw 1 pops in parked at the presenter (drift={drift:.4f}m)", drift < _WELD_TOLERANCE)

    if _failures:
        print(f"[test_mefron_screw_headless] {len(_failures)} CHECK(S) FAILED: {_failures}", flush=True)
    else:
        print("[test_mefron_screw_headless] ALL CHECKS PASSED", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
