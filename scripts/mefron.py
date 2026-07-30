"""Interactive cuRobo teleop + pick-and-place for the mefron scanner-assembly scene, on a single
Franka fitted with an automatic tool changer (ATC): numpad 1/2/3 sends the arm to dock/undock the
gripper/suction/screwdriver tool at its own rack. Thin entry point -- the actual logic lives in
mefron_lib/ (config, robot, grasp, teleop). See docs/mefron-history.md for bug history,
docs/grasp-and-assembly-offsets.md for how the grasp/assembly poses were derived, and
docs/tool-changer.md for the ATC design and open issues.
"""

from __future__ import annotations

import sys

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
from mefron_lib import config, conveyor, kit_experience, robot, teleop  # noqa: E402


def main() -> None:
    # Ensures Play actually creates a PhysX simulation view -- otherwise this is a GUI toggle that's
    # easy to have off, in which case is_playing() lies and SingleArticulation.initialize() never gets a real view.
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)
    # Decouples physics stepping from real wall-clock time -- without it, heavier per-frame Python
    # cost destabilizes friction-coupled mechanisms like the conveyor. See docs/mefron-history.md.
    carb.settings.get_settings().set_bool("/app/player/useFixedTimeStepping", True)

    # Must run BEFORE open_stage(): mefron.usd has a persisted, broken /panda prim reference, and
    # resolving it against stale files caches an Sdf.Layer that later crashes mount_franka()'s import ("a layer already exists").
    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)

    omni.usd.get_context().open_stage(str(config.MEFRON_USD))
    # Must run before the settle pump below -- see robot.clear_stray_robot_prims()'s own docstring.
    robot.clear_stray_robot_prims()

    # mefron.usd's own content resolves asynchronously, same reasoning as
    # build_scene.py's own post-build_factory() frame pump.
    for _ in range(120):
        simulation_app.update()

    robot.mount_franka()
    # The ATC replaces this arm's own built-in parallel-jaw hand with a swappable tool -- same
    # deactivate-fingers-then-hide-housing pattern the old arm2/arm3 used, now applied here too, so
    # panda_hand terminates the wrist cleanly for the male coupler. See both functions' docstrings.
    robot.remove_parallel_jaw_gripper()
    robot.hide_hand_housing()
    robot.attach_tool_changer_male_coupler()
    # Real isaacsim.robot.schema/surface_gripper attach mechanism, permanently on panda_hand
    # regardless of which tool is currently docked -- harmless unless the suction tool is docked
    # and V/L is pressed (see teleop.build_surface_gripper_keyboard_control()'s tool-gating).
    # SURFACE_GRIPPER_LOCAL_POSITION needs re-deriving now the coupler adds a standoff -- see
    # docs/tool-changer.md.
    surface_gripper_path = robot.attach_surface_gripper_physics()

    # Three dockable tools, parked at their own rack until a numpad key docks one -- see
    # docs/tool-changer.md.
    for tool_name in config.TOOL_CHANGE_TARGETS:
        robot.spawn_dockable_tool(tool_name)
        robot.park_tool_at_rack(tool_name)
    robot.enable_gripper_tool_fingers()

    if not _headless:
        kit_experience.enable_full_experience_extensions()

    stage = omni.usd.get_context().get_stage()
    for status_path in [config.ROBOT_PRIM_PATH, *config.OBSTACLE_PRIM_PATHS]:
        prim = stage.GetPrimAtPath(status_path)
        print(f"[mefron] {status_path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

    print("[mefron] warming up cuRobo motion_gen (viewport will look frozen/black until this finishes)...", flush=True)
    # has_parallel_jaw_gripper=False -- panda_finger_joint1/2 are deactivated on this arm's own
    # articulation now (the gripper is a separate dockable tool module), same reason arm2/arm3 used
    # to need this before the ATC existed.
    motion_gen, robot_cfg = teleop.setup_motion_gen(config.ROBOT_PRIM_PATH, config.TARGET_PRIM_PATH, has_parallel_jaw_gripper=False)
    print("[mefron] curobo motion_gen: READY", flush=True)

    # Force a stop unconditionally: if physics was left playing across warmup()'s ~30s unpumped gap,
    # PhysX's simulation view gets corrupted; the loop rebuilds cleanly on the next fresh Play regardless.
    omni.timeline.get_timeline_interface().stop()

    target = teleop.build_teleop_target(robot_cfg, config.ROBOT_PRIM_PATH, config.TARGET_PRIM_PATH, config.MOUNT_POSITION, config.MOUNT_ORIENTATION_WXYZ)
    target_prim = stage.GetPrimAtPath(config.TARGET_PRIM_PATH)
    print(f"[mefron] {config.TARGET_PRIM_PATH}: {'OK' if target_prim.IsValid() else 'MISSING'}", flush=True)

    if _headless:
        simulation_app.close()
        return

    gripper_control = teleop.build_gripper_keyboard_control()
    suction_approach_control = teleop.build_suction_approach_keyboard_control()
    tool_changer_control = teleop.build_tool_changer_keyboard_control()
    surface_gripper_control = teleop.build_surface_gripper_keyboard_control(
        surface_gripper_path, tool_changer_control=tool_changer_control
    )
    print(
        "[mefron] Tool changer: press "
        + ", ".join(f"{target['key']} to dock {name}" for name, target in config.TOOL_CHANGE_TARGETS.items())
        + ".",
        flush=True,
    )
    print(
        "[mefron] Gripper tool (once docked): press C to close, O to open (finger drive not wired "
        "yet -- see docs/tool-changer.md). J/B to approach a grasp, P to place.",
        flush=True,
    )
    print(
        "[mefron] Suction tool (once docked): press "
        + ", ".join(f"{target['key']} to approach {name}" for name, target in config.SUCTION_TARGETS.items())
        + f", {config.SUCTION_ATTACH_KEY} to attach, {config.SUCTION_DETACH_KEY} to release, "
        "P to place on main_holder (whichever object was last approached).",
        flush=True,
    )
    conveyor.setup_conveyor_belt_graph()
    conveyor_control = conveyor.build_conveyor_control()
    print(
        f"[mefron] Conveyor: press {config.CONVEYOR_TOGGLE_KEY} to send main_holder_jig forward "
        f"{config.CONVEYOR_TRAVEL_DISTANCE}m, press again to send it back the same distance.",
        flush=True,
    )
    print("[mefron] click Play in the GUI to start teleop.", flush=True)
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
            "suction_control": suction_approach_control,
            "surface_gripper_control": surface_gripper_control,
            "tool_changer_control": tool_changer_control,
        },
    ]
    teleop.run_teleop_loop(simulation_app, arms, conveyor_control=conveyor_control)
    simulation_app.close()


if __name__ == "__main__":
    main()
