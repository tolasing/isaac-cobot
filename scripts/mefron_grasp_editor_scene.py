"""Diagnostic scene for the Grasp Editor SingleArticulation-ownership race (see
docs/grasp-and-assembly-offsets.md): no teleop loop, no cuRobo, robot left idle.

Run standalone: ${ISAACSIM_ROOT_PATH}/python.sh scripts/mefron_grasp_editor_scene.py
"""

from __future__ import annotations

import os
import sys

from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # Full experience for interactive runs -- Grasp Editor is full-experience-only.
    experience = "" if _headless else f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    simulation_app = SimulationApp({"headless": _headless}, experience=experience)

# Must run before any omni/curobo import -- see mefron_lib/kit_bootstrap.py's docstring.
from mefron_lib.kit_bootstrap import clear_stale_robot_configuration, preload_real_packaging  # noqa: E402

preload_real_packaging()

import carb.settings  # noqa: E402
import omni.usd  # noqa: E402
from mefron_lib import config, robot  # noqa: E402


def main() -> None:
    # Same fix mefron.main() applies -- see mefron.py's comment on this setting.
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)
    omni.usd.get_context().open_stage(str(config.MEFRON_USD))
    for _ in range(120):
        simulation_app.update()

    robot.mount_franka()
    robot.apply_gripper_friction()
    robot.stiffen_gripper_drive()

    stage = omni.usd.get_context().get_stage()
    for status_path in [config.ROBOT_PRIM_PATH, *config.OBSTACLE_PRIM_PATHS]:
        prim = stage.GetPrimAtPath(status_path)
        print(f"[mefron_grasp_editor_scene] {status_path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

    print(
        "[mefron_grasp_editor_scene] Scene ready -- no cuRobo, no teleop loop, robot idle and uncontested.",
        flush=True,
    )
    print(
        "[mefron_grasp_editor_scene] Open the Grasp Editor now -- it can freely claim /World/Franka.",
        flush=True,
    )

    if _headless:
        simulation_app.close()
        return

    while simulation_app.is_running():
        simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
