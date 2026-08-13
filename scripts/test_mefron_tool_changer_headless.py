"""Headless regression test for the ATC's dock/undock mechanics: swaps all 3 tools across two
cycles and asserts the joint exists AND the tool's pose really tracks it. Run with --headless."""

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
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from pxr import UsdPhysics  # noqa: E402
from mefron_lib import config, robot, toolchanger  # noqa: E402

_SETTLE_FRAMES = 90
# The docked offset's magnitude is ~0.02m -- 0.05m leaves headroom for the rotational contribution
# without masking a real convergence failure. Those offsets are still placeholders.
_DOCKED_DISTANCE_TOLERANCE = 0.05
# 1cm was occasionally tripped by PhysX settling jitter on re-parking (~1.07cm observed) -- 2cm
# still catches a real "didn't return to the rack" failure without flaking.
_PARKED_POSITION_TOLERANCE = 0.02

_failures: list[str] = []


def _check(label: str, condition: bool) -> None:
    print(f"[test_mefron_tool_changer_headless] {'PASS' if condition else 'FAIL'}: {label}", flush=True)
    if not condition:
        _failures.append(label)


def _settle(simulation_app) -> None:
    for _ in range(_SETTLE_FRAMES):
        simulation_app.update()


def _assert_docked(stage, tool_name: str) -> None:
    _check(f"{tool_name}: wrist joint exists after docking", stage.GetPrimAtPath(toolchanger._wrist_joint_path(tool_name)).IsValid())
    _check(
        f"{tool_name}: rack joint removed after docking",
        not stage.GetPrimAtPath(toolchanger._rack_joint_path(tool_name)).IsValid(),
    )
    hand_trans, _ = SingleXFormPrim(
        prim_path=f"{config.ROBOT_PRIM_PATH}/panda_hand", reset_xform_properties=False
    ).get_world_pose()
    # female_coupler, not tool_prim_path -- the latter can be an organizing Xform that never moves.
    # female_coupler is one of the joint's own endpoints, correct for all 3 tool types.
    tool_trans, _ = SingleXFormPrim(
        prim_path=toolchanger._female_coupler_prim_path(tool_name), reset_xform_properties=False
    ).get_world_pose()
    distance = float(np.linalg.norm(hand_trans - tool_trans))
    _check(f"{tool_name}: tool tracks the wrist after docking (distance={distance:.4f}m)", distance < _DOCKED_DISTANCE_TOLERANCE)


def _assert_parked(stage, tool_name: str) -> None:
    _check(
        f"{tool_name}: rack joint restored after undocking",
        stage.GetPrimAtPath(toolchanger._rack_joint_path(tool_name)).IsValid(),
    )
    # female_coupler, not tool_prim_path -- the latter can be an organizing Xform that never moves.
    # female_coupler is one of the joint's own endpoints, correct for all 3 tool types.
    tool_trans, _ = SingleXFormPrim(
        prim_path=toolchanger._female_coupler_prim_path(tool_name), reset_xform_properties=False
    ).get_world_pose()
    expected_position = np.array(config.TOOL_CHANGE_TARGETS[tool_name]["dock_position"])
    distance = float(np.linalg.norm(expected_position - tool_trans))
    _check(
        f"{tool_name}: tool back at its rack position after undocking (distance={distance:.4f}m)",
        distance < _PARKED_POSITION_TOLERANCE,
    )


def main() -> None:
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)
    # Decouples physics stepping from wall-clock time -- without it a fixed frame-count settle can
    # cover noticeably less simulated time and end mid-convergence.
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

    stage = stage_context.get_stage()
    # Confirmed live regression (docs/tool-changer.md's gotcha 6): a second URDF import silently
    # erased panda_link0-8 from the MAIN arm. Only checking its own structure catches that.
    expected_franka_children = {f"panda_link{i}" for i in range(8)} | {"panda_hand", "ee_link"}
    actual_franka_children = {c.GetName() for c in stage.GetPrimAtPath(config.ROBOT_PRIM_PATH).GetChildren()}
    _check(
        "main arm's own link structure is untouched by spawning the gripper tool",
        expected_franka_children.issubset(actual_franka_children),
    )
    _check(
        "gripper tool's base_link lands at its own tool_prim_path",
        stage.GetPrimAtPath(toolchanger._female_coupler_parent_prim_path("gripper")).IsValid(),
    )
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    _settle(simulation_app)

    for tool_name in config.TOOL_CHANGE_TARGETS:
        _check(
            f"{tool_name}: starts parked at its own rack, nothing on the wrist",
            stage.GetPrimAtPath(toolchanger._rack_joint_path(tool_name)).IsValid()
            and not stage.GetPrimAtPath(toolchanger._wrist_joint_path(tool_name)).IsValid(),
        )

    # Cycle 1: dock suction, then swap to screwdriver -- undock before dock, the same ordering
    # motion.build_tool_change_queue() enforces.
    toolchanger.dock_tool_to_wrist("suction")
    _settle(simulation_app)
    _assert_docked(stage, "suction")

    toolchanger.undock_tool_to_rack("suction")
    toolchanger.dock_tool_to_wrist("screwdriver")
    _settle(simulation_app)
    _assert_parked(stage, "suction")
    _assert_docked(stage, "screwdriver")

    # Cycle 2: swap to gripper, then back to a fully bare wrist -- confirms this isn't a one-shot
    # mechanism and that undocking with nothing queued next leaves a clean parked state.
    toolchanger.undock_tool_to_rack("screwdriver")
    toolchanger.dock_tool_to_wrist("gripper")
    _settle(simulation_app)
    _assert_parked(stage, "screwdriver")
    _assert_docked(stage, "gripper")

    toolchanger.undock_tool_to_rack("gripper")
    _settle(simulation_app)
    _assert_parked(stage, "gripper")

    if _failures:
        print(f"[test_mefron_tool_changer_headless] {len(_failures)} CHECK(S) FAILED: {_failures}", flush=True)
    else:
        print("[test_mefron_tool_changer_headless] ALL CHECKS PASSED", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
