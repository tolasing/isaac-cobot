"""Diagnostic scene built to chase a Grasp Editor bug on a fresh anonymous
stage; kept as a working artifact. Full investigation: docs/grasp-and-assembly-offsets.md.

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/franka_grasp_editor_scene.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # Full experience for interactive runs -- Grasp Editor is full-experience-only.
    experience = "" if _headless else f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    simulation_app = SimulationApp({"headless": _headless}, experience=experience)

import carb.settings  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402

import mefron  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
SCANNER_USD = REPO_ROOT / "assets" / "mefron" / "scanner assembly" / "finger print scanner.usd"
SCANNER_PRIM_PATH = "/World/finger_print_scanner"


def main() -> None:
    mefron.simulation_app = simulation_app

    # Same fix mefron.main() applies -- see mefron.py's own comment on this setting.
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    # Deliberately no open_stage() -- SimulationApp's default anonymous stage
    # avoids the URDF importer's layered-import mechanism entirely (see docstring).
    mefron.mount_franka()
    for _ in range(60):
        simulation_app.update()

    if not SCANNER_USD.is_file():
        raise FileNotFoundError(f"{SCANNER_USD} not found.")
    add_reference_to_stage(usd_path=str(SCANNER_USD), prim_path=SCANNER_PRIM_PATH)
    for _ in range(60):
        simulation_app.update()

    # Needs finger_print_scanner resolved on the stage first (HIGH_FRICTION_PRIM_PATHS entry).
    mefron.apply_gripper_friction()
    mefron.stiffen_gripper_drive()

    stage = omni.usd.get_context().get_stage()
    for status_path in [mefron.ROBOT_PRIM_PATH, SCANNER_PRIM_PATH]:
        prim = stage.GetPrimAtPath(status_path)
        print(f"[franka_grasp_editor_scene] {status_path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

    print("[franka_grasp_editor_scene] Scene ready -- open the Grasp Editor now.", flush=True)

    if _headless:
        simulation_app.close()
        return

    while simulation_app.is_running():
        simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
