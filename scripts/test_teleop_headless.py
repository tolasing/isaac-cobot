"""Headless verification that dragging the teleop target drives the robot via cuRobo's MotionGen
-- exercises build_scene.run_teleop_loop() directly, faking a drag by monkeypatching
target.get_world_pose(). Reuses build_scene.py as a library: must assign
build_scene.simulation_app explicitly, and define /physicsScene before timeline.play()."""

from __future__ import annotations

import sys

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": _headless})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

import build_scene  # noqa: E402

# Small enough to stay within reach of the target's own starting pose (see
# table_layout.yaml's teleop_target comment -- that starting pose is itself
# already a confirmed-reachable retract-config pose).
_DRAG_OFFSET = np.array([0.0, 0.05, 0.05])
_MAX_ITERATIONS = 200
# Must clear build_scene._TELEOP_INIT_FRAMES (10) + _TELEOP_SETTLE_FRAMES
# (20) before the simulated drag lands, so run_teleop_loop's debounce logic
# sees a genuinely static target first, same as a real pre-drag pause would.
_SETTLE_CALLS = 40


def main() -> None:
    if __name__ == "__main__":
        build_scene.simulation_app = simulation_app

    cfg = build_scene.load_config()
    build_scene.build_factory(cfg)
    for _ in range(120):
        simulation_app.update()
    build_scene.prune_factory_dressing(cfg)
    build_scene.build_ergo_tables(cfg)
    build_scene.build_assembly_parts(cfg)

    robot_prim_path = cfg["cr5_mount"]["prim_path"]
    build_scene.mount_cr5(cfg)
    build_scene.mount_cr5_pedestal(cfg)

    print("[test_teleop_headless] warming up cuRobo motion_gen...", flush=True)
    motion_gen, robot_cfg = build_scene.setup_curobo_motion_gen(cfg)
    if motion_gen is None:
        print("[test_teleop_headless] cuRobo not installed -- nothing to test.", flush=True)
        simulation_app.close()
        return

    target = build_scene.build_teleop_target(cfg, robot_prim_path=robot_prim_path, robot_cfg=robot_cfg)

    # Must exist before the timeline plays -- run_teleop_loop() (called
    # below) defines this itself too, but only at its own top, too late
    # here since we call timeline.play() before run_teleop_loop() even
    # starts.
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    # A few frames so is_playing()/the physics view are actually live before
    # run_teleop_loop's own init/settle phase starts counting -- mirrors
    # main()'s own factory-load frame pump, just for physics instead.
    for _ in range(5):
        simulation_app.update()

    # The guaranteed starting pose -- run_teleop_loop's own init phase drives the robot here, so
    # there's no need for a separate SingleArticulation just to observe it.
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

    build_scene.run_teleop_loop(
        cfg, motion_gen, robot_cfg, target, robot_prim_path=robot_prim_path, max_iterations=_MAX_ITERATIONS
    )

    # Only constructed now, after run_teleop_loop's own internal
    # SingleArticulation has gone out of scope -- see this module's own
    # docstring for why holding two at once breaks PhysX's tensor view.
    robot = SingleArticulation(prim_path=robot_prim_path, name="verify_robot")
    robot.initialize()
    idx_list = [robot.get_dof_index(x) for x in j_names]
    end_positions = robot.get_joint_positions(idx_list)
    max_delta = float(np.max(np.abs(end_positions - start_positions)))
    print(f"[test_teleop_headless] max joint-position delta: {max_delta:.4f} rad", flush=True)
    if max_delta < 0.05:
        print("[test_teleop_headless] FAIL: robot did not move meaningfully in response to the simulated drag.", flush=True)
    else:
        print("[test_teleop_headless] PASS: robot followed the simulated drag.", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
