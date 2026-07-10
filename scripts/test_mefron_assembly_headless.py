"""Headless regression test for mefron_lib's G/P one-shot grasp-approach and
assembly-target snap requests, driven via run_teleop_loop() like test_mefron_teleop_headless.py.

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/test_mefron_assembly_headless.py --headless
"""

from __future__ import annotations

import sys

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": _headless})

# Must run before any omni/curobo import -- see mefron_lib/kit_bootstrap.py's docstring.
from mefron_lib.kit_bootstrap import clear_stale_robot_configuration, preload_real_packaging  # noqa: E402

preload_real_packaging()

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: E402
from pxr import UsdPhysics  # noqa: E402
from mefron_lib import config, grasp, robot, teleop  # noqa: E402

# 900, not 300: phase 1's discrete MotionGen trajectory needs more real time than 300 frames provides
# to actually finish (time-dilated playback + headless frames ticking faster than real time) -- the
# post-phase-1 assembly-target sanity check below needs phase 1 to have truly completed, not just
# started, or it measures a still-in-transit gripper pose. Same fix as
# test_mefron_reactive_placement_headless.py's own _MAX_ITERATIONS_PHASE1.
_MAX_ITERATIONS_PER_PHASE = 900


def main() -> None:
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)
    omni.usd.get_context().open_stage(str(config.MEFRON_USD))
    for _ in range(120):
        simulation_app.update()

    robot.mount_franka()
    robot.apply_gripper_friction()
    robot.stiffen_gripper_drive()

    print("[test_mefron_assembly_headless] warming up cuRobo motion_gen...", flush=True)
    motion_gen, robot_cfg = teleop.setup_motion_gen()
    target = teleop.build_teleop_target(robot_cfg)

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(5):
        simulation_app.update()

    # Sanity-check the pose math directly, independent of whether cuRobo can reach it.
    scanner_trans, scanner_quat = SingleXFormPrim(prim_path="/World/finger_print_scanner").get_world_pose()
    approach_trans, approach_quat = grasp.compute_grasp_approach_pose()
    print(
        f"[test_mefron_assembly_headless] scanner world pose: pos={scanner_trans} quat_wxyz={scanner_quat}",
        flush=True,
    )
    print(
        f"[test_mefron_assembly_headless] grasp-approach target pose: pos={approach_trans} quat_wxyz={approach_quat}",
        flush=True,
    )
    approach_distance = float(np.linalg.norm(np.array(approach_trans) - np.array(scanner_trans)))
    print(f"[test_mefron_assembly_headless] approach pose is {approach_distance:.4f} m from the scanner", flush=True)
    assert approach_distance < 0.2, "grasp-approach pose is implausibly far from the scanner"

    # compute_assembly_grasp_target() now needs a real live gripper-to-part relationship (it composes
    # main_holder's target pose with the CURRENT measured offset, same as compute_reactive_assembly_target()) --
    # meaningless before phase 1 has actually moved the gripper near the part, so that check moves to
    # after phase 1 below instead of running here against the retract-config starting pose.
    ee_link_prim_path = f"{config.ROBOT_PRIM_PATH}/{robot_cfg['kinematics']['ee_link']}"

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    start_positions = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])

    # Phase 1: simulate pressing G (grasp-approach) before run_teleop_loop() starts.
    gripper_control = teleop.GripperKeyboardControl()
    gripper_control.request_grasp_approach()
    teleop.run_teleop_loop(
        simulation_app, motion_gen, robot_cfg, target, max_iterations=_MAX_ITERATIONS_PER_PHASE, gripper_control=gripper_control
    )

    verify_robot = SingleArticulation(prim_path=config.ROBOT_PRIM_PATH, name="verify_robot_phase1")
    verify_robot.initialize()
    idx_list = [verify_robot.get_dof_index(x) for x in j_names]
    phase1_positions = verify_robot.get_joint_positions(idx_list)
    phase1_delta = float(np.max(np.abs(phase1_positions - start_positions)))
    print(f"[test_mefron_assembly_headless] phase 1 (grasp approach) max joint delta: {phase1_delta:.4f} rad", flush=True)
    del verify_robot  # must go out of scope before run_teleop_loop() builds its own again -- see test_mefron_teleop_headless.py

    # Sanity-check compute_assembly_grasp_target() against the post-phase-1 live gripper pose, not
    # retract config -- it now composes main_holder's target with the CURRENT measured gripper-to-part
    # offset (same underlying call as compute_reactive_assembly_target()), so it only makes sense once
    # phase 1 has actually moved the gripper near the part.
    holder_trans, holder_quat = SingleXFormPrim(prim_path="/World/main_holder").get_world_pose()
    assembly_trans, assembly_quat = grasp.compute_assembly_grasp_target(ee_link_prim_path)
    print(
        f"[test_mefron_assembly_headless] main_holder world pose: pos={holder_trans} quat_wxyz={holder_quat}",
        flush=True,
    )
    print(
        f"[test_mefron_assembly_headless] assembly-target pose: pos={assembly_trans} quat_wxyz={assembly_quat}",
        flush=True,
    )
    assembly_distance = float(np.linalg.norm(np.array(assembly_trans) - np.array(holder_trans)))
    print(f"[test_mefron_assembly_headless] assembly target is {assembly_distance:.4f} m from main_holder", flush=True)
    assert assembly_distance < 0.2, "assembly-target pose is implausibly far from main_holder"

    # Phase 2: simulate pressing P (assembly target), continuing from wherever
    # phase 1 left the robot.
    gripper_control.request_assembly_target()
    teleop.run_teleop_loop(
        simulation_app, motion_gen, robot_cfg, target, max_iterations=_MAX_ITERATIONS_PER_PHASE, gripper_control=gripper_control
    )

    verify_robot = SingleArticulation(prim_path=config.ROBOT_PRIM_PATH, name="verify_robot_phase2")
    verify_robot.initialize()
    idx_list = [verify_robot.get_dof_index(x) for x in j_names]
    phase2_positions = verify_robot.get_joint_positions(idx_list)
    phase2_delta = float(np.max(np.abs(phase2_positions - phase1_positions)))
    print(f"[test_mefron_assembly_headless] phase 2 (assembly target) max joint delta vs phase 1: {phase2_delta:.4f} rad", flush=True)

    if phase1_delta < 0.05 or phase2_delta < 0.05:
        print("[test_mefron_assembly_headless] FAIL: robot did not move meaningfully for one or both phases.", flush=True)
    else:
        print("[test_mefron_assembly_headless] PASS: both G (grasp approach) and P (assembly target) drove the robot.", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
