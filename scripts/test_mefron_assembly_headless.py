"""Headless regression test for mefron.py's G/P one-shot grasp-approach and
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

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

import mefron  # noqa: E402

_MAX_ITERATIONS_PER_PHASE = 300


def main() -> None:
    if __name__ == "__main__":
        mefron.simulation_app = simulation_app

    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    mefron.clear_stale_robot_configuration()
    omni.usd.get_context().open_stage(str(mefron.MEFRON_USD))
    for _ in range(120):
        simulation_app.update()

    mefron.mount_franka()
    mefron.apply_gripper_friction()
    mefron.stiffen_gripper_drive()

    print("[test_mefron_assembly_headless] warming up cuRobo motion_gen...", flush=True)
    motion_gen, robot_cfg = mefron.setup_motion_gen()
    target = mefron.build_teleop_target(robot_cfg)

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(5):
        simulation_app.update()

    # Sanity-check the pose math directly, independent of whether cuRobo can reach it.
    scanner_trans, scanner_quat = SingleXFormPrim(prim_path="/World/finger_print_scanner").get_world_pose()
    approach_trans, approach_quat = mefron.compute_grasp_approach_pose()
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

    holder_trans, holder_quat = SingleXFormPrim(prim_path="/World/main_holder").get_world_pose()
    assembly_trans, assembly_quat = mefron.compute_assembly_grasp_target()
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

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    start_positions = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])

    # Phase 1: simulate pressing G (grasp-approach) before run_teleop_loop() starts.
    gripper_control = mefron.GripperKeyboardControl()
    gripper_control.request_grasp_approach()
    mefron.run_teleop_loop(motion_gen, robot_cfg, target, max_iterations=_MAX_ITERATIONS_PER_PHASE, gripper_control=gripper_control)

    robot = SingleArticulation(prim_path=mefron.ROBOT_PRIM_PATH, name="verify_robot_phase1")
    robot.initialize()
    idx_list = [robot.get_dof_index(x) for x in j_names]
    phase1_positions = robot.get_joint_positions(idx_list)
    phase1_delta = float(np.max(np.abs(phase1_positions - start_positions)))
    print(f"[test_mefron_assembly_headless] phase 1 (grasp approach) max joint delta: {phase1_delta:.4f} rad", flush=True)
    del robot  # must go out of scope before run_teleop_loop() builds its own again -- see test_mefron_teleop_headless.py

    # Phase 2: simulate pressing P (assembly target), continuing from wherever
    # phase 1 left the robot.
    gripper_control.request_assembly_target()
    mefron.run_teleop_loop(motion_gen, robot_cfg, target, max_iterations=_MAX_ITERATIONS_PER_PHASE, gripper_control=gripper_control)

    robot = SingleArticulation(prim_path=mefron.ROBOT_PRIM_PATH, name="verify_robot_phase2")
    robot.initialize()
    idx_list = [robot.get_dof_index(x) for x in j_names]
    phase2_positions = robot.get_joint_positions(idx_list)
    phase2_delta = float(np.max(np.abs(phase2_positions - phase1_positions)))
    print(f"[test_mefron_assembly_headless] phase 2 (assembly target) max joint delta vs phase 1: {phase2_delta:.4f} rad", flush=True)

    if phase1_delta < 0.05 or phase2_delta < 0.05:
        print("[test_mefron_assembly_headless] FAIL: robot did not move meaningfully for one or both phases.", flush=True)
    else:
        print("[test_mefron_assembly_headless] PASS: both G (grasp approach) and P (assembly target) drove the robot.", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
