"""Headless regression test for mefron.py's run_teleop_loop(): fakes a mouse
drag by monkeypatching target.get_world_pose(), then asserts the robot moved.

Run: ${ISAACSIM_ROOT_PATH}/python.sh scripts/test_mefron_teleop_headless.py --headless
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
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

import mefron  # noqa: E402

# Small offset from the target's reachable starting pose.
_DRAG_OFFSET = np.array([0.0, 0.05, 0.05])
_MAX_ITERATIONS = 200
# Must clear run_teleop_loop's own init/settle frame counts first.
_SETTLE_CALLS = 40


def main() -> None:
    if __name__ == "__main__":
        mefron.simulation_app = simulation_app

    # Replicates mefron.main()'s fix; main() itself returns early in headless mode.
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    mefron.clear_stale_robot_configuration()
    omni.usd.get_context().open_stage(str(mefron.MEFRON_USD))
    for _ in range(120):
        simulation_app.update()

    mefron.mount_franka()
    mefron.apply_gripper_friction()
    mefron.stiffen_gripper_drive()

    print("[test_mefron_teleop_headless] warming up cuRobo motion_gen...", flush=True)
    motion_gen, robot_cfg = mefron.setup_motion_gen()

    target = mefron.build_teleop_target(robot_cfg)

    # Must exist before timeline.play(); run_teleop_loop() only defines it later.
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    # A few frames so is_playing()/the physics view are actually live before
    # run_teleop_loop's own init/settle phase starts counting.
    for _ in range(5):
        simulation_app.update()

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    start_positions = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])

    start_position, start_orientation = target.get_world_pose()
    dragged_position = start_position + _DRAG_OFFSET

    call_count = {"n": 0}

    def fake_get_world_pose():
        call_count["n"] += 1
        if call_count["n"] < _SETTLE_CALLS:
            return start_position, start_orientation
        return dragged_position, start_orientation

    target.get_world_pose = fake_get_world_pose

    mefron.run_teleop_loop(motion_gen, robot_cfg, target, max_iterations=_MAX_ITERATIONS)

    # Constructed only after run_teleop_loop's own SingleArticulation goes out of scope.
    robot = SingleArticulation(prim_path=mefron.ROBOT_PRIM_PATH, name="verify_robot")
    robot.initialize()
    idx_list = [robot.get_dof_index(x) for x in j_names]
    end_positions = robot.get_joint_positions(idx_list)
    max_delta = float(np.max(np.abs(end_positions - start_positions)))
    print(f"[test_mefron_teleop_headless] max joint-position delta: {max_delta:.4f} rad", flush=True)
    if max_delta < 0.05:
        print(
            "[test_mefron_teleop_headless] FAIL: robot did not move meaningfully in response to the simulated drag.",
            flush=True,
        )
    else:
        print("[test_mefron_teleop_headless] PASS: robot followed the simulated drag.", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
