"""One-time (re-runnable) bake of robot.mount_franka_hand_only()'s hand-only URDF into a standalone
robots/franka_panda/Props/gripper_tool_hand_only.usd asset, for the ATC gripper tool to reference
(robot.spawn_dockable_tool()) instead of live-importing it into mefron.usd's own stage every run.

Imports into a brand-new anonymous stage (never mefron.usd) specifically so this never shares a
"Robot Description" configuration directory with the main arm's own import -- confirmed live that
sharing one lets a second URDF import silently overwrite parts of the first robot's own link
structure, even with fully distinct prim paths (see docs/tool-changer.md's gotcha 6).
Run: ${ISAACSIM_ROOT_PATH}/python.sh scripts/vendor_gripper_tool.py --headless"""

from __future__ import annotations

import sys

from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": _headless})

from mefron_lib.kit_bootstrap import preload_real_packaging  # noqa: E402

preload_real_packaging()

import omni.usd  # noqa: E402
from mefron_lib import config, robot  # noqa: E402

OUTPUT_PATH = config.REPO_ROOT / "robots" / "franka_panda" / "Props" / "gripper_tool_hand_only.usd"
EXPORT_PRIM_PATH = "/gripper_tool_hand_only"


def main() -> None:
    stage_context = omni.usd.get_context()
    stage_context.new_stage()
    for _ in range(10):
        simulation_app.update()

    robot.mount_franka_hand_only(EXPORT_PRIM_PATH)
    for _ in range(10):
        simulation_app.update()

    stage = stage_context.get_stage()
    stage.SetDefaultPrim(stage.GetPrimAtPath(EXPORT_PRIM_PATH))
    stage.GetRootLayer().Export(str(OUTPUT_PATH))
    print(f"[vendor_gripper_tool] exported to {OUTPUT_PATH}", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
