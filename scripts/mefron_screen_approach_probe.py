"""One-time headless derivation: a first-pass suction-gripper approach pose for /World/screen, to
seed config.ASSEMBLY_RELATIONSHIPS["suction_gripper_approach_on_screen"]. Prints numbers to paste
by hand -- same manual workflow docs/grasp-and-assembly-offsets.md already establishes for the
other relative-pose constants. Treat this as a first pass to visually confirm/re-derive once seen,
not final -- same caveat that doc's own methodology carries.

Mounts arm 2 only, no timeline.play(), no motion_gen/cuRobo -- SingleXFormPrim.get_world_pose()
reads the USD xformCache directly, so pure USD math is enough (same "Stop mode only" reasoning as
mefron_gripper_probe.py).

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/mefron_screen_approach_probe.py --headless
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

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from mefron_lib import config, grasp, robot  # noqa: E402
from pxr import UsdGeom, UsdPhysics, Usd  # noqa: E402


def main() -> None:
    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)
    omni.usd.get_context().open_stage(str(config.MEFRON_USD))
    # mefron.usd's own content resolves asynchronously, same reasoning as build_scene.py's post-build_factory() frame pump.
    for _ in range(120):
        simulation_app.update()

    robot.mount_franka(config.ROBOT_2_PRIM_PATH, config.MOUNT_2_POSITION, config.MOUNT_2_ORIENTATION_WXYZ)
    robot.remove_parallel_jaw_gripper(config.ROBOT_2_PRIM_PATH)
    robot.attach_suction_gripper(config.ROBOT_2_PRIM_PATH)

    stage = omni.usd.get_context().get_stage()
    hand_trans, hand_quat = SingleXFormPrim(prim_path=f"{config.ROBOT_2_PRIM_PATH}/panda_hand").get_world_pose()
    screen_trans, screen_quat = SingleXFormPrim(prim_path=config.SCREEN_PRIM_PATH).get_world_pose()

    # (a) Literal answer to "pose of screen wrt the suction gripper" -- current live tip frame.
    tip_trans, tip_quat = grasp.compute_dependent_world_pose(
        hand_trans, hand_quat, config.SURFACE_GRIPPER_LOCAL_POSITION, config.SURFACE_GRIPPER_LOCAL_ORIENTATION_WXYZ
    )
    screen_wrt_tip_pos, screen_wrt_tip_quat = grasp.compute_relative_pose(tip_trans, tip_quat, screen_trans, screen_quat)
    print(f"[probe] gripper tip world pose: pos={tip_trans} quat_wxyz={tip_quat}", flush=True)
    print(f"[probe] screen wrt current gripper-tip frame: pos={screen_wrt_tip_pos} quat_wxyz={screen_wrt_tip_quat}", flush=True)
    print(f"[probe] straight-line distance, tip to screen: {np.linalg.norm(screen_trans - tip_trans):.4f} m", flush=True)

    # (b) First-pass approach-pose heuristic, bbox-derived. screen's rotation is pure-Z (flat, not
    # tilted -- confirmed live in an earlier session), so "approach from directly above, pointing
    # straight down" is geometrically justified, not a guess.
    screen_prim = stage.GetPrimAtPath(config.SCREEN_PRIM_PATH)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
    world_range = bbox_cache.ComputeWorldBound(screen_prim).ComputeAlignedRange()
    top_z = world_range.GetMax()[2]
    center_x = (world_range.GetMin()[0] + world_range.GetMax()[0]) / 2
    center_y = (world_range.GetMin()[1] + world_range.GetMax()[1]) / 2
    approach_position = np.array([center_x, center_y, top_z + config.SURFACE_GRIPPER_APPROACH_CLEARANCE])
    approach_orientation_wxyz = np.array([0.0, 1.0, 0.0, 0.0])  # local +Z -> world -Z ("point straight down")
    print(f"[probe] screen world bbox: min={world_range.GetMin()} max={world_range.GetMax()}", flush=True)
    print(f"[probe] approach_position (world): {list(approach_position)}", flush=True)

    rel_pos, rel_quat = grasp.compute_relative_pose(screen_trans, screen_quat, approach_position, approach_orientation_wxyz)
    print(f"[probe] local_position={list(rel_pos)}", flush=True)
    print(f"[probe] local_orientation_wxyz={list(rel_quat)}", flush=True)

    if screen_prim.HasAPI(UsdPhysics.MassAPI):
        print(f"[probe] screen physics:mass = {UsdPhysics.MassAPI(screen_prim).GetMassAttr().Get()}", flush=True)
    else:
        print(
            "[probe] no authored physics:mass -- sanity-check SURFACE_GRIPPER_COAXIAL/SHEAR_FORCE_LIMIT against a real estimate",
            flush=True,
        )

    simulation_app.close()


if __name__ == "__main__":
    main()
