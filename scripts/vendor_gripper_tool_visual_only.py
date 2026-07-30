"""One-time (re-runnable) bake of robot.mount_franka_hand_only()'s hand-only URDF into a
VISUALS-ONLY standalone robots/franka_panda/Props/gripper_tool_visual_only.usd asset -- same "flat,
single rigid body" shape as suction_gripper_with_tool_female.usd/
electric_screwdriver_with_tool_female.usd, for the ATC gripper tool to reference instead of
gripper_tool_hand_only.usd's real multi-link mini-articulation.

Confirmed live: rigidly bolting that real multi-link body (its own joints/mass/inertia) onto the
wrist via dock_tool_to_wrist()'s FixedJoint produces a genuine dynamic resonance -- joint velocity
pins at the Panda wrist joints' own hardware limit (2.61 rad/s) and never decays, independent of
pose accuracy, collision state, or joint-damping increases tried first. Suction/screwdriver never
hit this because they're simple visual props with one flat RigidBodyAPI, not a separately-jointed
mini-articulation -- stripping this asset down to the same shape sidesteps the problem instead of
continuing to chase drive-gain tuning. See docs/tool-changer.md.

Strips all joints (SetActive(False), same proven-reliable pattern as
robot.remove_parallel_jaw_gripper() for URDF-synthesized joints -- DeletePrims silently no-ops for
these) and fully RemoveAPI's RigidBodyAPI/CollisionAPI everywhere -- this asset carries zero
collision by construction now, so _set_tool_collision_enabled() naturally has nothing to toggle for
the gripper tool and its dock/undock calls become a no-op for it (the tool floats in its rack same
as suction/screwdriver already do, an already-accepted tradeoff). _enable_finger_joints() re-applies
RigidBodyAPI to just the two fingers afterward for the C/O drive; the user's own manually-authored
finger collider lives as a stronger opinion directly in mefron.usd, unaffected by this bake.

Run: ${ISAACSIM_ROOT_PATH}/python.sh scripts/vendor_gripper_tool_visual_only.py --headless
"""

from __future__ import annotations

import sys

from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": _headless})

from mefron_lib.kit_bootstrap import preload_real_packaging  # noqa: E402

preload_real_packaging()

import omni.usd  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402
from mefron_lib import config, robot  # noqa: E402

OUTPUT_PATH = config.REPO_ROOT / "robots" / "franka_panda" / "Props" / "gripper_tool_visual_only.usd"
EXPORT_PRIM_PATH = "/gripper_tool_visual_only"


def _strip_physics(stage, root_prim_path: str) -> None:
    """RemoveAPI (not disable) for both CollisionAPI and RigidBodyAPI -- the gripper tool is meant
    to carry zero collision at all now, so _set_tool_collision_enabled() has nothing left to find
    for this tool. RigidBodyAPI removal (vs. disable-only) was already confirmed necessary to avoid
    PhysX's "missing xformstack reset when child of another enabled rigid body" ERROR once
    spawn_dockable_tool() applies a fresh enabled RigidBodyAPI at the tool root."""
    root_prim = stage.GetPrimAtPath(root_prim_path)
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdPhysics.Joint):
            prim.SetActive(False)
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.RemoveAPI(UsdPhysics.CollisionAPI)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)


def main() -> None:
    stage_context = omni.usd.get_context()
    stage_context.new_stage()
    for _ in range(10):
        simulation_app.update()

    robot.mount_franka_hand_only(EXPORT_PRIM_PATH)
    for _ in range(10):
        simulation_app.update()

    stage = stage_context.get_stage()
    _strip_physics(stage, EXPORT_PRIM_PATH)
    for _ in range(5):
        simulation_app.update()

    stage.SetDefaultPrim(stage.GetPrimAtPath(EXPORT_PRIM_PATH))
    stage.GetRootLayer().Export(str(OUTPUT_PATH))
    print(f"[vendor_gripper_tool_visual_only] exported to {OUTPUT_PATH}", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
