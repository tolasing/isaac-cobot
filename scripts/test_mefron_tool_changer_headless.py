"""Headless regression test for the ATC's dock/undock mechanics
(robot.spawn_dockable_tool()/park_tool_at_rack()/dock_tool_to_wrist()/undock_tool_to_rack()):
swaps between all 3 tools across two full cycles and asserts, after each step, that the right
FixedJoint exists and the tool's world pose actually tracks the wrist (or the rack) once PhysX
settles -- not just that the USD authoring calls didn't raise.
Run: ${ISAACSIM_ROOT_PATH}/python.sh scripts/test_mefron_tool_changer_headless.py --headless"""

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
from mefron_lib import config, robot  # noqa: E402

_SETTLE_FRAMES = 90
# TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_POSITION's magnitude is ~0.02m -- 0.05m gives headroom for the
# rotational contribution without masking a real convergence failure (see docs/tool-changer.md for
# why these particular offsets are still placeholders pending hand-jog derivation).
_DOCKED_DISTANCE_TOLERANCE = 0.05
# 1cm was occasionally tripped by ordinary PhysX settling jitter on re-parking (observed ~1.07cm on
# an otherwise-clean run) -- these positions are placeholders anyway, so 2cm is plenty to still
# catch a real "didn't return to the rack" failure without flaking on normal noise.
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
    _check(f"{tool_name}: wrist joint exists after docking", stage.GetPrimAtPath(robot._wrist_joint_path(tool_name)).IsValid())
    _check(
        f"{tool_name}: rack joint removed after docking",
        not stage.GetPrimAtPath(robot._rack_joint_path(tool_name)).IsValid(),
    )
    hand_trans, _ = SingleXFormPrim(
        prim_path=f"{config.ROBOT_PRIM_PATH}/panda_hand", reset_xform_properties=False
    ).get_world_pose()
    # female_coupler, not tool_prim_path -- for the gripper tool, tool_prim_path is just an
    # organizing Xform over its real rigid links (base_link/panda_hand/...) and never itself
    # moves; female_coupler is one of the joint's own two endpoints, correct for all 3 tool types.
    tool_trans, _ = SingleXFormPrim(
        prim_path=robot._female_coupler_prim_path(tool_name), reset_xform_properties=False
    ).get_world_pose()
    distance = float(np.linalg.norm(hand_trans - tool_trans))
    _check(f"{tool_name}: tool tracks the wrist after docking (distance={distance:.4f}m)", distance < _DOCKED_DISTANCE_TOLERANCE)


def _assert_parked(stage, tool_name: str) -> None:
    _check(
        f"{tool_name}: rack joint restored after undocking",
        stage.GetPrimAtPath(robot._rack_joint_path(tool_name)).IsValid(),
    )
    # female_coupler, not tool_prim_path -- for the gripper tool, tool_prim_path is just an
    # organizing Xform over its real rigid links (base_link/panda_hand/...) and never itself
    # moves; female_coupler is one of the joint's own two endpoints, correct for all 3 tool types.
    tool_trans, _ = SingleXFormPrim(
        prim_path=robot._female_coupler_prim_path(tool_name), reset_xform_properties=False
    ).get_world_pose()
    expected_position = np.array(config.TOOL_CHANGE_TARGETS[tool_name]["dock_position"])
    distance = float(np.linalg.norm(expected_position - tool_trans))
    _check(
        f"{tool_name}: tool back at its rack position after undocking (distance={distance:.4f}m)",
        distance < _PARKED_POSITION_TOLERANCE,
    )


def main() -> None:
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)
    # Decouples physics stepping from real wall-clock time -- with 3 tools' worth of physics per
    # frame instead of 1, without this a fixed frame-count settle can correspond to noticeably less
    # simulated time and end mid-convergence. See CLAUDE.md's mefron.py note on the same setting.
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

    stage = stage_context.get_stage()
    # Confirmed live regression (docs/tool-changer.md's gotcha 6): live-importing the gripper tool
    # via the URDF importer into mefron.usd's own stage silently corrupted the main arm's own link
    # structure through a shared "Robot Description" cache -- panda_link0-8 disappeared from
    # /World/Franka entirely, replaced by the tool's own link names, even though both imports used
    # fully distinct prim paths. Neither the joint/pose checks below nor a naive "does the tool land
    # at the right path" check would have caught this (the tool WAS also at its own correct path;
    # the main arm was the corrupted one) -- checking the main arm's own structure directly instead.
    expected_franka_children = {f"panda_link{i}" for i in range(8)} | {"panda_hand", "ee_link"}
    actual_franka_children = {c.GetName() for c in stage.GetPrimAtPath(config.ROBOT_PRIM_PATH).GetChildren()}
    _check(
        "main arm's own link structure is untouched by spawning the gripper tool",
        expected_franka_children.issubset(actual_franka_children),
    )
    _check(
        "gripper tool's base_link lands at its own tool_prim_path",
        stage.GetPrimAtPath(robot._female_coupler_parent_prim_path("gripper")).IsValid(),
    )
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    _settle(simulation_app)

    for tool_name in config.TOOL_CHANGE_TARGETS:
        _check(
            f"{tool_name}: starts parked at its own rack, nothing on the wrist",
            stage.GetPrimAtPath(robot._rack_joint_path(tool_name)).IsValid()
            and not stage.GetPrimAtPath(robot._wrist_joint_path(tool_name)).IsValid(),
        )

    # Cycle 1: dock suction, then swap directly to screwdriver (undock before dock, same ordering
    # teleop._build_tool_change_queue() enforces -- see dock_tool_to_wrist()'s own docstring for
    # why skipping the undock leaves the previous tool jointless).
    robot.dock_tool_to_wrist("suction")
    _settle(simulation_app)
    _assert_docked(stage, "suction")

    robot.undock_tool_to_rack("suction")
    robot.dock_tool_to_wrist("screwdriver")
    _settle(simulation_app)
    _assert_parked(stage, "suction")
    _assert_docked(stage, "screwdriver")

    # Cycle 2: swap to gripper, then back to a fully bare wrist -- confirms this isn't a one-shot
    # mechanism and that undocking with nothing queued next leaves a clean parked state.
    robot.undock_tool_to_rack("screwdriver")
    robot.dock_tool_to_wrist("gripper")
    _settle(simulation_app)
    _assert_parked(stage, "screwdriver")
    _assert_docked(stage, "gripper")

    robot.undock_tool_to_rack("gripper")
    _settle(simulation_app)
    _assert_parked(stage, "gripper")

    if _failures:
        print(f"[test_mefron_tool_changer_headless] {len(_failures)} CHECK(S) FAILED: {_failures}", flush=True)
    else:
        print("[test_mefron_tool_changer_headless] ALL CHECKS PASSED", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
