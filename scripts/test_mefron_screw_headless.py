"""Headless regression test for the screw pick-and-place mechanics: moves the arm's joints directly
(no cuRobo) and asserts a screw rides the wrist, then the hole, then the cover. Run with --headless."""

from __future__ import annotations

import sys

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # Base experience: nothing here needs a full-experience extension (core prims + UsdPhysics +
    # pose math only), and the full kit's startup would dominate the runtime.
    simulation_app = SimulationApp({"headless": _headless})

# Must run before any omni/curobo import -- see mefron_lib/kit_bootstrap.py's docstring.
from mefron_lib.kit_bootstrap import clear_stale_robot_configuration, preload_real_packaging  # noqa: E402

preload_real_packaging()

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim, SingleXFormPrim  # noqa: E402
from pxr import UsdPhysics  # noqa: E402
from mefron_lib import assembly, config, grasp, robot, screws, toolchanger  # noqa: E402

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
        prim_path=screws._screw_prim_path(index), reset_xform_properties=False
    ).get_world_pose()


def _hand_pose() -> tuple:
    return SingleXFormPrim(
        prim_path=f"{config.ROBOT_PRIM_PATH}/panda_hand", reset_xform_properties=False
    ).get_world_pose()


_ARM_POSES = [(-1.30, -2.50), (-0.90, -2.00)]
# Below these, a "move" didn't really move and the ride/stay checks would pass vacuously.
_MIN_ARM_MOVEMENT = 0.05
_MIN_MOUNT_MOVEMENT = 0.05
# How far the back cover is nudged for the ride check -- well past _WELD_TOLERANCE, so a screw that
# stayed behind reads as an unmistakable failure rather than settling noise.
_MOUNT_NUDGE = 0.25


def _move_arm(simulation_app, pose_index: int) -> float:
    """Drives panda_joint2/4 to one of two ABSOLUTE in-limit configurations (a delta off an unknown
    start can get silently clamped). Rebuilds the articulation handle first; returns hand travel."""
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
    """Re-derives the screw pose from compute_ee_target_for_screw_pose()'s answer in the OPPOSITE
    direction, so a dropped inverse or flipped frame shows up as a mismatch, not a plausible number."""
    ee_trans, ee_quat = grasp.compute_ee_target_for_screw_pose(ee_link_prim_path, want_trans, want_quat)

    tool_trans, tool_quat = SingleXFormPrim(
        prim_path=toolchanger._tool_prim_path("screwdriver"), reset_xform_properties=False
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

    robot.mount_arm()
    robot.remove_parallel_jaw_gripper()
    robot.hide_hand_housing()
    toolchanger.attach_tool_changer_male_coupler()

    for tool_name in config.TOOL_CHANGE_TARGETS:
        toolchanger.spawn_dockable_tool(tool_name)
        toolchanger.park_tool_at_rack(tool_name)

    # clear_assembly_welds() first, same order as mefron.py: a placed screw's anchor now lives in the
    # assembly-weld scope, so a stale one baked into mefron.usd by an earlier run has to go too.
    assembly.clear_assembly_welds()
    screws.clear_screws()
    screws.ensure_screw_presenter()
    screws.present_screw(0)

    stage = stage_context.get_stage()
    ee_link_prim_path = f"{config.ROBOT_PRIM_PATH}/panda_hand"
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    screw_prim = stage.GetPrimAtPath(screws._screw_prim_path(0))
    _check("screw 0 spawned as a real rigid body", screw_prim.IsValid() and screw_prim.HasAPI(UsdPhysics.RigidBodyAPI))
    _check(
        "screw 0 has an explicit mass (it has no colliders for PhysX to derive one from)",
        screw_prim.HasAPI(UsdPhysics.MassAPI),
    )
    _check("screw 0 starts jointed to the presenter", stage.GetPrimAtPath(screws._screw_presenter_joint_path(0)).IsValid())

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    _settle(simulation_app)

    # The presenter weld's whole job: a screw that would otherwise free-fall stays exactly where it
    # was presented, several seconds of simulated gravity later.
    presenter_trans, _ = grasp.compute_screw_presenter_pose()
    parked_trans, _ = _screw_pose(0)
    drift = float(np.linalg.norm(presenter_trans - parked_trans))
    _check(f"screw 0 stays at the presenter under gravity (drift={drift:.4f}m)", drift < _WELD_TOLERANCE)

    toolchanger.dock_tool_to_wrist("screwdriver")
    _settle(simulation_app)
    _check(
        "screwdriver docked for the screw legs",
        stage.GetPrimAtPath(toolchanger._wrist_joint_path("screwdriver")).IsValid(),
    )

    # The CAD-derived bit tip should sit one tool-length out from the wrist once docked.
    tool_trans, tool_quat = SingleXFormPrim(
        prim_path=toolchanger._tool_prim_path("screwdriver"), reset_xform_properties=False
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

    # Pick: the screw leaves the presenter for the wrist. Not driven to the pick pose first, so the
    # weld's correction to the nominal carry pose is the whole distance -- asserted next.
    screws.attach_screw_to_wrist(0)
    _settle(simulation_app)
    _check("screw 0 jointed to the wrist after picking", stage.GetPrimAtPath(screws._screw_tip_joint_path(0)).IsValid())
    _check(
        "screw 0's presenter joint removed after picking",
        not stage.GetPrimAtPath(screws._screw_presenter_joint_path(0)).IsValid(),
    )

    # Regression: the weld used to freeze the arm's settled pose, leaving the screw ~1mm off the bit
    # axis. Tighter than _WELD_TOLERANCE on purpose -- 5mm wouldn't have caught it.
    tool_trans, tool_quat = SingleXFormPrim(
        prim_path=toolchanger._tool_prim_path("screwdriver"), reset_xform_properties=False
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

    # Place: the screw leaves the wrist for the back cover's own kinematic anchor.
    screws.weld_screw_into_hole(0, 0)
    _settle(simulation_app)
    _check("screw 0 jointed into hole 0 after placing", stage.GetPrimAtPath(screws._screw_hole_joint_path(0)).IsValid())
    _check(
        "screw 0's tip joint removed after placing",
        not stage.GetPrimAtPath(screws._screw_tip_joint_path(0)).IsValid(),
    )

    # Place-side twin of the carry check above. Compared against screw_hole_local_pose(), the same
    # helper the weld joint's own body0 frame comes from.
    mount_trans, mount_quat = SingleXFormPrim(
        prim_path=config.SCREW_HOLE_MOUNT_PRIM_PATH, reset_xform_properties=False
    ).get_world_pose()
    placed_local_trans, _ = grasp.compute_relative_pose(mount_trans, mount_quat, *_screw_pose(0))
    want_local, _ = grasp.screw_hole_local_pose(0)
    hole_error = float(np.linalg.norm(placed_local_trans - np.array(want_local)))
    _check(
        f"placed screw 0 seats at hole 0's nominal pose in the mount's frame (err={hole_error:.5f}m)",
        hole_error < _CARRY_TOLERANCE,
    )

    # Read AFTER the weld, not before it: the weld snaps the screw off the bit onto the hole, so a
    # pre-weld baseline measures that (intended, ~0.5m) teleport instead of the drift being checked.
    placed_trans_before, _ = _screw_pose(0)
    moved = _move_arm(simulation_app, 1)
    _check(f"the arm actually moved away before the stay check (hand moved {moved:.4f}m)", moved > _MIN_ARM_MOVEMENT)
    placed_trans_after, _ = _screw_pose(0)
    stay_error = float(np.linalg.norm(placed_trans_before - placed_trans_after))
    _check(
        f"placed screw 0 stays put in world space while the arm moves away (drift={stay_error:.4f}m)",
        stay_error < _WELD_TOLERANCE,
    )

    # The load-bearing check for welding to the cover rather than a static world anchor: does a
    # placed screw RIDE it? sync_assembly_anchors() is called by hand, as there's no teleop loop.
    mount_prim = stage.GetPrimAtPath(config.SCREW_HOLE_MOUNT_PRIM_PATH)
    # SingleRigidPrim for a simulated body -- teleporting one needs the PhysX-side write, since a
    # plain USD xform write gets overwritten by the body's own pose on the next step.
    mount_mover = (
        SingleRigidPrim(prim_path=config.SCREW_HOLE_MOUNT_PRIM_PATH, name="mefron_screw_test_mount")
        if mount_prim.HasAPI(UsdPhysics.RigidBodyAPI)
        else SingleXFormPrim(prim_path=config.SCREW_HOLE_MOUNT_PRIM_PATH, reset_xform_properties=False)
    )
    mount_trans_before, mount_quat_before = mount_mover.get_world_pose()
    mount_mover.set_world_pose(
        position=np.array(mount_trans_before) + np.array([0.0, _MOUNT_NUDGE, 0.0]),
        orientation=mount_quat_before,
    )
    # Every frame, not once: the cover is a real dynamic body, so it keeps settling after the nudge
    # and a single sync would leave the anchor (and the screw) behind wherever it was at frame 0.
    for _ in range(_SETTLE_FRAMES):
        assembly.sync_assembly_anchors()
        simulation_app.update()
    mount_trans_after, _ = mount_mover.get_world_pose()
    mount_moved = float(np.linalg.norm(np.array(mount_trans_after) - np.array(mount_trans_before)))
    _check(
        f"the back cover actually moved before the ride check (moved {mount_moved:.4f}m)",
        mount_moved > _MIN_MOUNT_MOVEMENT,
    )

    # Recomputed off the cover's NEW live pose -- compute_screw_hole_pose() reads it live, so this is
    # where the screw should have been carried to.
    moved_hole_trans, _ = grasp.compute_screw_hole_pose(0)
    rode_trans, _ = _screw_pose(0)
    ride_error = float(np.linalg.norm(np.array(moved_hole_trans) - rode_trans))
    _check(
        f"placed screw 0 rides the back cover to its new pose (err={ride_error:.4f}m)",
        ride_error < _WELD_TOLERANCE,
    )

    # And the next screw pops in, so the presenter is never empty mid-sequence.
    screws.present_screw(1)
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
