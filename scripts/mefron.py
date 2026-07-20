"""Interactive cuRobo teleop + pick-and-place for the mefron scanner-assembly scene. Thin entry point --
the actual logic lives in mefron_lib/ (config, robot, grasp, teleop). See docs/mefron-history.md for bug
history and docs/grasp-and-assembly-offsets.md for how the grasp/assembly poses were derived.
"""

from __future__ import annotations

import sys

from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # ALWAYS the plain base experience at construction time, even for interactive runs -- mounting a
    # second Franka (a second native URDF import) crashes Kit's URDF importer plugin if the full
    # experience's extra extensions are already loaded when that happens. Loading them AFTER both
    # Frankas are mounted (kit_experience.enable_full_experience_extensions(), below) reproduces the
    # exact same feature set (Physics debug-viz menu included) with zero crash -- see
    # robot.mount_franka()'s own docstring for the full diagnosis.
    simulation_app = SimulationApp({"headless": _headless})

# Must run before any omni/curobo import -- see mefron_lib/kit_bootstrap.py's docstring.
from mefron_lib.kit_bootstrap import clear_stale_robot_configuration, preload_real_packaging  # noqa: E402

preload_real_packaging()

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from mefron_lib import config, kit_experience, robot, teleop  # noqa: E402


def main() -> None:
    # Ensures Play actually creates a PhysX simulation view -- otherwise this is a GUI toggle that's
    # easy to have off, in which case is_playing() lies and SingleArticulation.initialize() never gets a real view.
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    # Must run BEFORE open_stage(): mefron.usd has a persisted, broken /panda prim reference, and
    # resolving it against stale files caches an Sdf.Layer that later crashes mount_franka()'s import ("a layer already exists").
    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)

    omni.usd.get_context().open_stage(str(config.MEFRON_USD))
    # Must run before the settle pump below -- see robot.clear_stray_robot_prims()'s own docstring:
    # leftover Franka/Franka2/Franka3/panda prims baked into mefron.usd's saved root layer by a past
    # stray Save get partially registered by PhysX/Fabric/Hydra during that pump if left alive this
    # long, desyncing rendering from the CURRENT run's freshly re-mounted robots (physics/cuRobo
    # motion planning stays correct regardless -- it addresses each arm by its own exact prim_path).
    robot.clear_stray_robot_prims()

    # mefron.usd's own content resolves asynchronously, same reasoning as
    # build_scene.py's own post-build_factory() frame pump.
    for _ in range(120):
        simulation_app.update()

    robot.mount_franka()
    robot.apply_gripper_friction()
    robot.stiffen_gripper_drive()

    # Second arm: same URDF, same friction/drive tuning, only the destination differs -- safe to
    # mount here (before the full experience's extra extensions load) same as arm 1, see
    # robot.mount_franka()'s own docstring.
    robot.mount_franka(config.ROBOT_2_PRIM_PATH, config.MOUNT_2_POSITION, config.MOUNT_2_ORIENTATION_WXYZ)

    # Third arm: same URDF, same friction/drive tuning, only the destination differs -- safe to
    # mount here (before the full experience's extra extensions load) same as arm 1, see
    # robot.mount_franka()'s own docstring.
    robot.mount_franka(config.ROBOT_3_PRIM_PATH, config.MOUNT_3_POSITION, config.MOUNT_3_ORIENTATION_WXYZ)

    # Arm 2 is a suction-only arm -- no parallel-jaw fingers, so no friction/drive tuning for them
    # either. See robot.remove_parallel_jaw_gripper()/attach_suction_gripper()'s own docstrings.
    robot.remove_parallel_jaw_gripper(config.ROBOT_2_PRIM_PATH)
    robot.hide_hand_housing(config.ROBOT_2_PRIM_PATH)
    robot.attach_suction_gripper(config.ROBOT_2_PRIM_PATH)
    surface_gripper_path = robot.attach_surface_gripper_physics(config.ROBOT_2_PRIM_PATH)

    if not _headless:
        kit_experience.enable_full_experience_extensions()

    stage = omni.usd.get_context().get_stage()
    for status_path in [config.ROBOT_PRIM_PATH, config.ROBOT_2_PRIM_PATH, *config.OBSTACLE_PRIM_PATHS]:
        prim = stage.GetPrimAtPath(status_path)
        print(f"[mefron] {status_path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

    print("[mefron] warming up cuRobo motion_gen for arm 1 (viewport will look frozen/black until this finishes)...", flush=True)
    motion_gen, robot_cfg = teleop.setup_motion_gen(config.ROBOT_PRIM_PATH, config.TARGET_PRIM_PATH)
    print("[mefron] arm 1 curobo motion_gen: READY", flush=True)
    print("[mefron] warming up cuRobo motion_gen for arm 2...", flush=True)
    motion_gen_2, robot_cfg_2 = teleop.setup_motion_gen(
        config.ROBOT_2_PRIM_PATH, config.TARGET_2_PRIM_PATH, has_parallel_jaw_gripper=False
    )
    print("[mefron] arm 2 curobo motion_gen: READY", flush=True)

    print("[mefron] warming up cuRobo motion_gen for arm 3...", flush=True)
    motion_gen_3, robot_cfg_3 = teleop.setup_motion_gen(
        config.ROBOT_3_PRIM_PATH, config.TARGET_3_PRIM_PATH
    )
    print("[mefron] arm 3 curobo motion_gen: READY", flush=True)

    # Force a stop unconditionally: if physics was left playing across warmup()'s ~30s unpumped gap,
    # PhysX's simulation view gets corrupted; the loop rebuilds cleanly on the next fresh Play regardless.
    omni.timeline.get_timeline_interface().stop()

    target = teleop.build_teleop_target(robot_cfg, config.ROBOT_PRIM_PATH, config.TARGET_PRIM_PATH, config.MOUNT_POSITION, config.MOUNT_ORIENTATION_WXYZ)
    target_prim = stage.GetPrimAtPath(config.TARGET_PRIM_PATH)
    print(f"[mefron] {config.TARGET_PRIM_PATH}: {'OK' if target_prim.IsValid() else 'MISSING'}", flush=True)

    target_2 = teleop.build_teleop_target(
        robot_cfg_2, config.ROBOT_2_PRIM_PATH, config.TARGET_2_PRIM_PATH, config.MOUNT_2_POSITION, config.MOUNT_2_ORIENTATION_WXYZ
    )
    target_2_prim = stage.GetPrimAtPath(config.TARGET_2_PRIM_PATH)
    print(f"[mefron] {config.TARGET_2_PRIM_PATH}: {'OK' if target_2_prim.IsValid() else 'MISSING'}", flush=True)

    target_3 = teleop.build_teleop_target(
        robot_cfg_3, config.ROBOT_3_PRIM_PATH, config.TARGET_3_PRIM_PATH, config.MOUNT_3_POSITION, config.MOUNT_3_ORIENTATION_WXYZ
    )
    target_3_prim = stage.GetPrimAtPath(config.TARGET_3_PRIM_PATH)
    print(f"[mefron] {config.TARGET_3_PRIM_PATH}: {'OK' if target_3_prim.IsValid() else 'MISSING'}", flush=True)



    if _headless:
        simulation_app.close()
        return

    gripper_control = teleop.build_gripper_keyboard_control()
    print("[mefron] Arm 1 gripper: press C to close, O to open.", flush=True)
    # Arm 2's gripper_control must stay None -- its parallel-jaw finger joints are deactivated, and a
    # real GripperKeyboardControl would try to resolve config.GRIPPER_JOINT_NAMES via get_dof_index()
    # in _step_arm()'s init block, hitting its unresolved-joint-index RuntimeError. Its suction
    # controls are separate, independent keyboard subscriptions instead (see teleop.py).
    suction_approach_control = teleop.build_suction_approach_keyboard_control()
    surface_gripper_control = teleop.build_surface_gripper_keyboard_control(surface_gripper_path)
    # Second, independent P subscription -- arm 1's gripper_control above already owns its own; one
    # P press fires both (see AssemblyPlacementControl's docstring).
    assembly_placement_control = teleop.build_assembly_placement_keyboard_control()
    print(
        f"[mefron] Arm 2 suction: press {config.SUCTION_APPROACH_KEY} to approach {config.SCREEN_PRIM_PATH}, "
        f"{config.SUCTION_ATTACH_KEY} to attach, {config.SUCTION_DETACH_KEY} to release, "
        "P to place on main_holder.",
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
            },
        {
            "motion_gen": motion_gen_2,
            "robot_cfg": robot_cfg_2,
            "target": target_2,
            "gripper_control": None,
            "robot_prim_path": config.ROBOT_2_PRIM_PATH,
            "target_prim_path": config.TARGET_2_PRIM_PATH,
            "mount_position": config.MOUNT_2_POSITION,
            "mount_orientation_wxyz": config.MOUNT_2_ORIENTATION_WXYZ,
            "name": "arm2",
            "suction_control": suction_approach_control,
            "suction_approach_relationship": "suction_gripper_approach_on_screen",
            "assembly_control": assembly_placement_control,
            "assembly_relationship": "screen_on_main_holder",
            "surface_gripper_control": surface_gripper_control,
        },
        {
            "motion_gen": motion_gen_3,
            "robot_cfg": robot_cfg_3,
            "target": target_3,
            "gripper_control": None,
            "robot_prim_path": config.ROBOT_3_PRIM_PATH,
            "target_prim_path": config.TARGET_3_PRIM_PATH,
            "mount_position": config.MOUNT_3_POSITION,
            "mount_orientation_wxyz": config.MOUNT_3_ORIENTATION_WXYZ,
            "name": "arm3",
        }
    ]
    teleop.run_teleop_loop(simulation_app, arms)
    simulation_app.close()


if __name__ == "__main__":
    main()
