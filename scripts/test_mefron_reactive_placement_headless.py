"""Headless regression test for mefron_lib's M one-shot reactive-assembly request: a discrete
MotionGen approach to the nominal assembly pose, handed off to mpc_solver for continuous
slip-corrected tracking via grasp.compute_reactive_assembly_target(). Driven via run_teleop_loop()
like test_mefron_assembly_headless.py, whose G-phase this test reuses verbatim as its phase 1.

Like test_mefron_assembly_headless.py's own G/P phases, this test never issues a gripper-close
request, so nothing is ever actually grasped and there's no real slip for the reactive target to
correct -- this test exercises the mechanism (discrete-to-MPC handoff, joint-state plumbing,
convergence/timeout bookkeeping), not the motivating slip-correction scenario itself. That needs
the manual GUI checklist in the MPC plan (drag/grasp/close, compare M vs P placement).

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/test_mefron_reactive_placement_headless.py --headless
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
from mefron_lib import config, grasp, mpc, robot, teleop  # noqa: E402

# Larger than test_mefron_assembly_headless.py's 300: that test only checks joint movement happened
# (any partial progress along the trajectory satisfies it), but phase 2 here needs phase 1's discrete
# MotionGen plan to have FULLY completed before the post-phase-1 sanity check measures the live
# gripper pose -- 300 wasn't enough real time for the whole (time-dilated) trajectory to play out.
_MAX_ITERATIONS_PHASE1 = 900
# Covers phase 2's settle overhead + discrete approach leg + up to config._MPC_MAX_TRACKING_STEPS
# MPC steps. MPC steps are gated on wall-clock config._MPC_STEP_DT, not frame count, and headless
# frames tick much faster than that gate, so this frame budget is generous rather than tight -- see
# the plan's "flagged as genuinely uncertain" section for why the exact ratio isn't derived here.
_MAX_ITERATIONS_PHASE2 = 2500


def main() -> None:
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)
    omni.usd.get_context().open_stage(str(config.MEFRON_USD))
    for _ in range(120):
        simulation_app.update()

    robot.mount_franka()
    robot.apply_gripper_friction()
    robot.stiffen_gripper_drive()

    print("[test_mefron_reactive_placement_headless] warming up cuRobo motion_gen...", flush=True)
    motion_gen, robot_cfg = teleop.setup_motion_gen()
    print("[test_mefron_reactive_placement_headless] warming up cuRobo mpc_solver...", flush=True)
    mpc_solver = mpc.setup_mpc_solver(robot_cfg)
    target = teleop.build_teleop_target(robot_cfg)
    ee_link_prim_path = f"{config.ROBOT_PRIM_PATH}/{robot_cfg['kinematics']['ee_link']}"

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(5):
        simulation_app.update()

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    start_positions = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])

    # Phase 1: simulate pressing G (grasp-approach), identical to test_mefron_assembly_headless.py --
    # phase 2 (M) needs a plausible post-grasp gripper pose to measure a reactive offset from, not the
    # arm sitting at its retract config.
    gripper_control = teleop.GripperKeyboardControl()
    gripper_control.request_grasp_approach()
    teleop.run_teleop_loop(
        simulation_app, motion_gen, robot_cfg, target, max_iterations=_MAX_ITERATIONS_PHASE1, gripper_control=gripper_control
    )

    verify_robot = SingleArticulation(prim_path=config.ROBOT_PRIM_PATH, name="verify_robot_phase1")
    verify_robot.initialize()
    idx_list = [verify_robot.get_dof_index(x) for x in j_names]
    phase1_positions = verify_robot.get_joint_positions(idx_list)
    phase1_delta = float(np.max(np.abs(phase1_positions - start_positions)))
    print(f"[test_mefron_reactive_placement_headless] phase 1 (grasp approach) max joint delta: {phase1_delta:.4f} rad", flush=True)
    del verify_robot  # must go out of scope before run_teleop_loop() builds its own again -- see test_mefron_teleop_headless.py

    # Sanity-check the reactive-target pose math directly (against the post-phase-1 live gripper
    # pose, not retract config -- that's the whole point of the reactive vs. fixed-nominal target).
    holder_trans, holder_quat = SingleXFormPrim(prim_path="/World/main_holder").get_world_pose()
    reactive_trans, reactive_quat = grasp.compute_reactive_assembly_target(ee_link_prim_path)
    print(
        f"[test_mefron_reactive_placement_headless] main_holder world pose: pos={holder_trans} quat_wxyz={holder_quat}",
        flush=True,
    )
    print(
        f"[test_mefron_reactive_placement_headless] reactive-assembly target pose: pos={reactive_trans} quat_wxyz={reactive_quat}",
        flush=True,
    )
    reactive_distance = float(np.linalg.norm(np.array(reactive_trans) - np.array(holder_trans)))
    print(f"[test_mefron_reactive_placement_headless] reactive target is {reactive_distance:.4f} m from main_holder", flush=True)
    assert reactive_distance < 0.2, "reactive-assembly target pose is implausibly far from main_holder"

    # Phase 2: simulate pressing M (reactive assembly placement), continuing from wherever phase 1
    # left the robot -- exercises the discrete-approach-then-MPC-handoff path end to end.
    gripper_control.request_reactive_assembly()
    teleop.run_teleop_loop(
        simulation_app,
        motion_gen,
        robot_cfg,
        target,
        max_iterations=_MAX_ITERATIONS_PHASE2,
        gripper_control=gripper_control,
        mpc_solver=mpc_solver,
    )

    verify_robot = SingleArticulation(prim_path=config.ROBOT_PRIM_PATH, name="verify_robot_phase2")
    verify_robot.initialize()
    idx_list = [verify_robot.get_dof_index(x) for x in j_names]
    phase2_positions = verify_robot.get_joint_positions(idx_list)
    phase2_delta = float(np.max(np.abs(phase2_positions - phase1_positions)))
    print(
        f"[test_mefron_reactive_placement_headless] phase 2 (reactive assembly) max joint delta vs phase 1: {phase2_delta:.4f} rad",
        flush=True,
    )
    del verify_robot

    # Post-loop residual check: how far the gripper's actual final pose is from a freshly-recomputed
    # reactive target (i.e. the MPC tracking error at loop-exit, cross-checked independently of
    # mpc_result.metrics). Printed, not asserted -- hard-asserting convergence against freshly-guessed,
    # untuned config._MPC_* thresholds would make this flaky for the wrong reason before those
    # constants get live-tuned against a real GPU run (see the plan's uncertainty list).
    final_gripper_trans, final_gripper_quat = SingleXFormPrim(prim_path=ee_link_prim_path).get_world_pose()
    residual_target_trans, residual_target_quat = grasp.compute_reactive_assembly_target(ee_link_prim_path)
    residual_distance = float(np.linalg.norm(np.array(residual_target_trans) - np.array(final_gripper_trans)))
    print(
        f"[test_mefron_reactive_placement_headless] post-loop residual: gripper pos={final_gripper_trans}, "
        f"recomputed target pos={residual_target_trans} quat_wxyz={residual_target_quat} "
        f"(residual distance={residual_distance:.4f} m)",
        flush=True,
    )

    if phase2_delta < 0.05:
        print(
            "[test_mefron_reactive_placement_headless] FAIL: robot did not move meaningfully during the reactive-assembly phase.",
            flush=True,
        )
    else:
        print(
            "[test_mefron_reactive_placement_headless] PASS: M (reactive assembly placement) drove the robot.",
            flush=True,
        )

    simulation_app.close()


if __name__ == "__main__":
    main()
