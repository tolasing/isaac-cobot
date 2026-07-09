"""Interactive cuRobo teleop + pick-and-place for the mefron scanner-assembly scene. Thin entry point --
the actual logic lives in mefron_lib/ (config, robot, grasp, teleop). See docs/mefron-history.md for bug
history and docs/grasp-and-assembly-offsets.md for how the grasp/assembly poses were derived.
"""

from __future__ import annotations

import os
import sys

from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # Use the full isaac-sim.sh experience (UI extensions like Physics debug viz) for
    # interactive runs; headless verification doesn't need it.
    experience = "" if _headless else f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    simulation_app = SimulationApp({"headless": _headless}, experience=experience)

# Must run before any omni/curobo import -- see mefron_lib/kit_bootstrap.py's docstring.
from mefron_lib.kit_bootstrap import clear_stale_robot_configuration, preload_real_packaging  # noqa: E402

preload_real_packaging()

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from mefron_lib import config, mpc, robot, teleop  # noqa: E402


def main() -> None:
    # Ensures Play actually creates a PhysX simulation view -- otherwise this is a GUI toggle that's
    # easy to have off, in which case is_playing() lies and SingleArticulation.initialize() never gets a real view.
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    # Must run BEFORE open_stage(): mefron.usd has a persisted, broken /panda prim reference, and
    # resolving it against stale files caches an Sdf.Layer that later crashes mount_franka()'s import ("a layer already exists").
    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)

    omni.usd.get_context().open_stage(str(config.MEFRON_USD))
    # mefron.usd's own content resolves asynchronously, same reasoning as
    # build_scene.py's own post-build_factory() frame pump.
    for _ in range(120):
        simulation_app.update()

    robot.mount_franka()
    robot.apply_gripper_friction()
    robot.stiffen_gripper_drive()

    stage = omni.usd.get_context().get_stage()
    for status_path in [config.ROBOT_PRIM_PATH, *config.OBSTACLE_PRIM_PATHS]:
        prim = stage.GetPrimAtPath(status_path)
        print(f"[mefron] {status_path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

    print("[mefron] warming up cuRobo motion_gen (viewport will look frozen/black until this finishes)...", flush=True)
    motion_gen, robot_cfg = teleop.setup_motion_gen()
    print("[mefron] curobo motion_gen: READY", flush=True)

    print("[mefron] warming up cuRobo mpc_solver...", flush=True)
    mpc_solver = mpc.setup_mpc_solver(robot_cfg)
    print("[mefron] curobo mpc_solver: READY", flush=True)

    # Force a stop unconditionally: if physics was left playing across either warmup's unpumped gap,
    # PhysX's simulation view gets corrupted; the loop rebuilds cleanly on the next fresh Play regardless.
    # Must come after BOTH warmups, not between them, or the second one runs across the same corrupting gap.
    omni.timeline.get_timeline_interface().stop()

    target = teleop.build_teleop_target(robot_cfg)
    target_prim = stage.GetPrimAtPath(config.TARGET_PRIM_PATH)
    print(f"[mefron] {config.TARGET_PRIM_PATH}: {'OK' if target_prim.IsValid() else 'MISSING'}", flush=True)

    if _headless:
        simulation_app.close()
        return

    gripper_control = teleop.build_gripper_keyboard_control()
    print("[mefron] Gripper: press C to close, O to open.", flush=True)
    print("[mefron] click Play in the GUI to start teleop.", flush=True)
    teleop.run_teleop_loop(
        simulation_app, motion_gen, robot_cfg, target, gripper_control=gripper_control, mpc_solver=mpc_solver
    )
    simulation_app.close()


if __name__ == "__main__":
    main()
