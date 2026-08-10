"""Interactive cuRobo teleop + pick-and-place on one Franka with an automatic tool changer.
Thin entry point -- the logic lives in mefron_lib/. See CLAUDE.md and docs/."""

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
from mefron_lib import (  # noqa: E402
    assembly,
    config,
    conveyor,
    feeder,
    keyboard,
    kit_experience,
    motion,
    robot,
    screws,
    teleop,
    toolchanger,
)


def main() -> None:
    # Ensures Play actually creates a PhysX simulation view -- otherwise is_playing() lies and
    # SingleArticulation.initialize() never gets a real view.
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)
    # Decouples physics stepping from wall-clock time -- without it, heavier per-frame Python cost
    # destabilizes friction-coupled mechanisms like the conveyor. See docs/mefron-history.md.
    carb.settings.get_settings().set_bool("/app/player/useFixedTimeStepping", True)

    # Must run BEFORE open_stage(): mefron.usd has a persisted, broken /panda prim reference, and
    # resolving it caches an Sdf.Layer that later crashes mount_franka()'s import.
    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)

    omni.usd.get_context().open_stage(str(config.MEFRON_USD))
    # Must run before the settle pump below -- see robot.clear_stray_robot_prims()'s own docstring.
    robot.clear_stray_robot_prims()

    # mefron.usd's own content resolves asynchronously.
    for _ in range(120):
        simulation_app.update()

    robot.mount_franka()
    # The ATC replaces this arm's own hand with a swappable tool, so panda_hand terminates the wrist
    # cleanly for the male coupler.
    robot.remove_parallel_jaw_gripper()
    robot.hide_hand_housing()
    toolchanger.attach_tool_changer_male_coupler()
    # Rides panda_hand permanently, whichever tool is docked -- V/L is gated on the suction tool.
    # SURFACE_GRIPPER_LOCAL_POSITION needs re-deriving now the coupler adds a standoff.
    surface_gripper_path = robot.attach_surface_gripper_physics()

    # Three dockable tools, parked at their own rack until a tool-change key docks one.
    for tool_name in config.TOOL_CHANGE_TARGETS:
        toolchanger.spawn_dockable_tool(tool_name)
        toolchanger.park_tool_at_rack(tool_name)
    toolchanger.enable_gripper_tool_fingers()

    # Anything a previous run's O/L release weld or screw cycle authored -- the URDF importer
    # rewrites mefron.usd every run, so a run must never inherit either.
    assembly.clear_assembly_welds()
    screws.clear_screws()
    screws.ensure_screw_presenter()
    screws.present_screw(0)

    if not _headless:
        kit_experience.enable_full_experience_extensions()

    stage = omni.usd.get_context().get_stage()
    for status_path in [config.ROBOT_PRIM_PATH, *config.OBSTACLE_PRIM_PATHS]:
        prim = stage.GetPrimAtPath(status_path)
        print(f"[mefron] {status_path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

    print("[mefron] warming up cuRobo motion_gen (viewport will look frozen/black until this finishes)...", flush=True)
    # has_parallel_jaw_gripper=False -- panda_finger_joint1/2 are deactivated on this arm's own
    # articulation now; the gripper is a separate dockable tool module.
    motion_gen, robot_cfg = motion.setup_motion_gen(
        config.ROBOT_PRIM_PATH, config.TARGET_PRIM_PATH, has_parallel_jaw_gripper=False
    )
    print("[mefron] curobo motion_gen: READY", flush=True)

    # Force a stop unconditionally: physics left playing across warmup()'s ~30s unpumped gap corrupts
    # PhysX's simulation view. The loop rebuilds cleanly on the next fresh Play regardless.
    omni.timeline.get_timeline_interface().stop()

    target = motion.build_teleop_target(
        robot_cfg, config.ROBOT_PRIM_PATH, config.TARGET_PRIM_PATH, config.MOUNT_POSITION, config.MOUNT_ORIENTATION_WXYZ
    )
    target_prim = stage.GetPrimAtPath(config.TARGET_PRIM_PATH)
    print(f"[mefron] {config.TARGET_PRIM_PATH}: {'OK' if target_prim.IsValid() else 'MISSING'}", flush=True)

    if _headless:
        simulation_app.close()
        return

    gripper_control = keyboard.build_gripper_keyboard_control()
    suction_approach_control = keyboard.build_suction_approach_keyboard_control()
    tool_changer_control = keyboard.build_tool_changer_keyboard_control()
    surface_gripper_control = keyboard.build_surface_gripper_keyboard_control(
        surface_gripper_path, tool_changer_control=tool_changer_control
    )
    print(
        "[mefron] Tool changer: press "
        + ", ".join(f"{target_cfg['key']} to dock {name}" for name, target_cfg in config.TOOL_CHANGE_TARGETS.items())
        + ".",
        flush=True,
    )
    print(
        "[mefron] Gripper tool (once docked): press C to close, O to open. "
        + "/".join(target_cfg["key"] for target_cfg in config.GRASP_TARGETS.values())
        + " to approach a "
        f"grasp, P to place. Releasing within {config.ASSEMBLY_WELD_MAX_DISTANCE}m of the assembly "
        "pose snaps the part there and welds it; a grasp key un-welds it again.",
        flush=True,
    )
    print(
        "[mefron] Suction tool (once docked): press "
        + ", ".join(f"{target_cfg['key']} to approach {name}" for name, target_cfg in config.SUCTION_TARGETS.items())
        + f", {config.SUCTION_ATTACH_KEY} to attach, {config.SUCTION_DETACH_KEY} to release (welds "
        "the part at its assembly pose, same as O), P to place on main_holder (whichever object was "
        "last approached).",
        flush=True,
    )
    screw_control = keyboard.build_screw_keyboard_control()
    print(
        f"[mefron] Screwdriver tool (once docked): press {config.SCREW_PICK_KEY} to pick the presented "
        f"screw, {config.SCREW_PLACE_KEY} to place it in the next of "
        f"{len(config.SCREW_HOLES)} main_holder holes. No screw-driving rotation.",
        flush=True,
    )
    conveyor.setup_conveyor_belt_graph()
    conveyor_control = conveyor.build_conveyor_control()
    print(
        f"[mefron] Conveyor: press {config.CONVEYOR_TOGGLE_KEY} to send main_holder_jig forward "
        f"{config.CONVEYOR_TRAVEL_DISTANCE}m, press again to send it back the same distance.",
        flush=True,
    )
    # Builds one graph + one photo-eye per config.PART_FEEDERS belt, then runs itself -- no key is
    # needed for the normal cycle.
    feeder_control = feeder.build_feeder_control()
    print(
        f"[mefron] Part feeders: {len(feeder_control.feeders)} of {len(config.PART_FEEDERS)} belts armed. "
        f"{config.FEEDER_ADVANCE_DELAY_SECONDS}s after a part leaves its pick spot, that belt brings the "
        f"next copy up and its photo-eye stops it there. Press {config.FEEDER_ADVANCE_KEY} to skip the wait.",
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
            "screw_control": screw_control,
        },
    ]
    teleop.run_teleop_loop(
        simulation_app, arms, conveyor_control=conveyor_control, feeder_control=feeder_control
    )
    simulation_app.close()


if __name__ == "__main__":
    main()
