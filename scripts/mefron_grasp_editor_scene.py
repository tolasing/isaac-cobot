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

import carb.settings  # noqa: E402
import omni.usd  # noqa: E402

import mefron  # noqa: E402 -- also runs mefron.py's packaging-preload workaround


def main() -> None:
    mefron.simulation_app = simulation_app

    # Same fix mefron.main() applies -- see mefron.py's comment on this setting.
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    mefron.clear_stale_robot_configuration()
    omni.usd.get_context().open_stage(str(mefron.MEFRON_USD))
    for _ in range(120):
        simulation_app.update()

    mefron.mount_franka()
    mefron.apply_gripper_friction()
    mefron.stiffen_gripper_drive()

    stage = omni.usd.get_context().get_stage()
    for status_path in [mefron.ROBOT_PRIM_PATH, *mefron.OBSTACLE_PRIM_PATHS]:
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
