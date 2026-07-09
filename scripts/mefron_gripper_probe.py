"""Standalone Franka hand+fingers+ee_link probe, for measuring grasp offsets against mefron.usd's
parts without the full 7-DOF arm/IK in the way. Opens mefron.usd directly (same pattern as mefron.py)
but skips mount_franka()/motion_gen entirely -- this is a static-measurement tool, not a teleop script.

Drag /World/GripperProbe's base_link in the viewport (Stop mode -- a fixed-base articulation ignores
Xform edits once Play starts driving it) to position ee_link against the real part mesh, then read back
its world pose the same way docs/grasp-and-assembly-offsets.md's compute_relative_pose() does.

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/mefron_gripper_probe.py

Can also be imported from mefron.py's own Script Editor (see spawn_gripper_probe()) to add the probe
to an already-open session instead of opening a second stage.
"""

from __future__ import annotations

import os
import sys

from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    experience = "" if _headless else f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    simulation_app = SimulationApp({"headless": _headless}, experience=experience)

# Must run before any omni/curobo import -- see mefron_lib/kit_bootstrap.py's docstring.
from mefron_lib.kit_bootstrap import clear_stale_robot_configuration, preload_real_packaging  # noqa: E402

preload_real_packaging()

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from mefron_lib import config, robot  # noqa: E402

GRIPPER_PROBE_PRIM_PATH = "/World/GripperProbe"


def spawn_gripper_probe(prim_path: str = GRIPPER_PROBE_PRIM_PATH) -> str:
    """Imports the hand-only probe into whatever stage is currently open -- does not open/replace it,
    so this is also callable from mefron.py's own Script Editor while its scene is already loaded."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(prim_path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[prim_path])
    # URDFParseAndImportFile parents under the current Stage-tree selection if one exists -- clear it
    # so the probe always lands as a direct child of /World regardless of what's selected (confirmed
    # live: importing with finger_print_scanner selected nested the probe under it instead of /World,
    # which also broke PhysX -- a rigid-body articulation nested inside another one).
    omni.usd.get_context().get_selection().clear_selected_prim_paths()

    return robot.mount_franka_hand_only(prim_path)


def main() -> None:
    # Must run BEFORE open_stage(): mefron.usd has a persisted, broken /panda prim reference, and
    # resolving it against stale files caches an Sdf.Layer that later crashes the URDF import.
    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)

    omni.usd.get_context().open_stage(str(config.MEFRON_USD))
    # mefron.usd's own content resolves asynchronously, same reasoning as build_scene.py's post-build_factory() frame pump.
    for _ in range(120):
        simulation_app.update()

    spawn_gripper_probe()
    print(
        f"[mefron_gripper_probe] Imported at {GRIPPER_PROBE_PRIM_PATH} (no arm, no motion_gen -- Stop mode "
        "only). Drag its base_link in the viewport to position ee_link against the real part mesh, then "
        "read back ee_link/finger_print_scanner's world poses (Script Editor + get_world_pose(), as in "
        "docs/grasp-and-assembly-offsets.md) for a fresh GRASP_OFFSET_POSITION/ORIENTATION_WXYZ.",
        flush=True,
    )
    while simulation_app.is_running():
        simulation_app.update()


if __name__ == "__main__":
    main()
