"""Headless regression test for mefron_lib's J/K/P one-shot grasp-approach and
assembly-target snap requests, driven via run_teleop_loop() like test_mefron_teleop_headless.py.

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/test_mefron_assembly_headless.py --headless
"""

from __future__ import annotations

import os
import sys

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # Unlike mefron.py's own headless path (which never touches J), this test exercises
    # grasp.compute_grasp_approach_pose_from_file() unconditionally -- that needs the
    # isaacsim.robot_setup.grasp_editor extension, which only the full experience loads.
    simulation_app = SimulationApp(
        {"headless": _headless}, experience=f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    )

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
# started, or it measures a still-in-transit gripper pose.
_MAX_ITERATIONS_PER_PHASE = 900


def _joint_positions(name: str, j_names: list[str]) -> np.ndarray:
    verify_robot = SingleArticulation(prim_path=config.ROBOT_PRIM_PATH, name=name)
    verify_robot.initialize()
    idx_list = [verify_robot.get_dof_index(x) for x in j_names]
    return verify_robot.get_joint_positions(idx_list)


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
    gripper_control = teleop.GripperKeyboardControl()
    # Arm 1 only, mirroring mefron.py's own arm-1 dict -- see teleop.run_teleop_loop()'s docstring
    # for the required per-arm dict shape.
    arms = [
        {
            "motion_gen": motion_gen,
            "robot_cfg": robot_cfg,
            "target": target,
            "gripper_control": gripper_control,
            "robot_prim_path": config.ROBOT_PRIM_PATH,
            "target_prim_path": config.TARGET_PRIM_PATH,
            "mount_position": config.MOUNT_POSITION,
            "mount_orientation_wxyz": config.MOUNT_ORIENTATION_WXYZ,
            "name": "arm1",
        }
    ]

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(5):
        simulation_app.update()

    # Sanity-check the pose math directly, independent of whether cuRobo can reach it.
    scanner_trans, scanner_quat = SingleXFormPrim(prim_path="/World/finger_print_scanner").get_world_pose()
    finger_print_scanner_target = config.GRASP_TARGETS["finger_print_scanner"]
    approach_trans, approach_quat = grasp.compute_grasp_approach_pose_from_file(
        finger_print_scanner_target["yaml_path"], finger_print_scanner_target["grasp_name"]
    )
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
    # main_holder's target pose with the CURRENT measured offset) -- meaningless before phase 1 has
    # actually moved the gripper near the part, so that check moves to after phase 1 below.
    ee_link_prim_path = f"{config.ROBOT_PRIM_PATH}/{robot_cfg['kinematics']['ee_link']}"

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    start_positions = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])

    # Phase 1: simulate pressing J (grasp-approach) before run_teleop_loop() starts.
    gripper_control.request_grasp_approach_from_file("finger_print_scanner")
    teleop.run_teleop_loop(simulation_app, arms, max_iterations=_MAX_ITERATIONS_PER_PHASE)

    phase1_positions = _joint_positions("verify_robot_phase1", j_names)
    phase1_delta = float(np.max(np.abs(phase1_positions - start_positions)))
    print(f"[test_mefron_assembly_headless] phase 1 (J: grasp approach) max joint delta: {phase1_delta:.4f} rad", flush=True)

    # Sanity-check compute_assembly_grasp_target() against the post-phase-1 live gripper pose, not
    # retract config -- it now composes main_holder's target with the CURRENT measured gripper-to-part
    # offset, so it only makes sense once phase 1 has actually moved the gripper near the part.
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
    teleop.run_teleop_loop(simulation_app, arms, max_iterations=_MAX_ITERATIONS_PER_PHASE)

    phase2_positions = _joint_positions("verify_robot_phase2", j_names)
    phase2_delta = float(np.max(np.abs(phase2_positions - phase1_positions)))
    print(f"[test_mefron_assembly_headless] phase 2 (P: assembly target) max joint delta vs phase 1: {phase2_delta:.4f} rad", flush=True)

    # Phase 3: simulate pressing K (grasp-approach for pcb_assembly), continuing from wherever
    # phase 2 left the robot. Sanity-check the pose math first, same shape as J's own check above.
    pcb_assembly_target = config.GRASP_TARGETS["pcb_assembly"]
    pcb_trans, pcb_quat = SingleXFormPrim(prim_path=pcb_assembly_target["part_prim_path"]).get_world_pose()
    pcb_approach_trans, pcb_approach_quat = grasp.compute_grasp_approach_pose_from_file(
        pcb_assembly_target["yaml_path"], pcb_assembly_target["grasp_name"]
    )
    print(
        f"[test_mefron_assembly_headless] pcb_assembly world pose: pos={pcb_trans} quat_wxyz={pcb_quat}",
        flush=True,
    )
    print(
        f"[test_mefron_assembly_headless] K grasp-approach target pose: pos={pcb_approach_trans} quat_wxyz={pcb_approach_quat}",
        flush=True,
    )
    pcb_approach_distance = float(np.linalg.norm(np.array(pcb_approach_trans) - np.array(pcb_trans)))
    print(f"[test_mefron_assembly_headless] K approach pose is {pcb_approach_distance:.4f} m from pcb_assembly", flush=True)
    assert pcb_approach_distance < 0.2, "K's grasp-approach pose is implausibly far from pcb_assembly"

    gripper_control.request_grasp_approach_from_file("pcb_assembly")
    teleop.run_teleop_loop(simulation_app, arms, max_iterations=_MAX_ITERATIONS_PER_PHASE)

    phase3_positions = _joint_positions("verify_robot_phase3", j_names)
    phase3_delta = float(np.max(np.abs(phase3_positions - phase2_positions)))
    print(f"[test_mefron_assembly_headless] phase 3 (K: grasp approach) max joint delta vs phase 2: {phase3_delta:.4f} rad", flush=True)

    if phase1_delta < 0.05 or phase2_delta < 0.05 or phase3_delta < 0.05:
        print("[test_mefron_assembly_headless] FAIL: robot did not move meaningfully for one or more phases.", flush=True)
    else:
        print("[test_mefron_assembly_headless] PASS: J (grasp approach), P (assembly target), and K (grasp approach) all drove the robot.", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
