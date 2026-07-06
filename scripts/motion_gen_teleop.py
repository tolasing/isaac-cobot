"""Interactive drag-target teleop demo for this repo's actual scene: builds
the factory backdrop + ergo tables + mounted robot (CR5, or currently the
Franka override -- see configs/scene/table_layout.yaml), spawns a draggable
target cuboid, and plans+executes a cuRobo motion whenever the target moves.

This is a from-scratch adaptation of cuRobo's own
examples/curobo_reference/motion_gen_reacher.py -- NOT a copy or edit of
that pristine file, which is deliberately left untouched (see its own
header comment and CLAUDE.md's "Needs verification"). Isaac Sim 6.0.1
removed the omni.isaac.* namespace that reference script hard-codes, with
no fallback (confirmed live: `ModuleNotFoundError: No module named
'omni.isaac'`) -- this script exists because of that break.

Differences from the pristine reference script, by design:
- Builds THIS repo's actual scene via build_scene.py's already-verified
  functions (factory + ergo tables + mounted robot + pedestal), instead of
  a synthetic world with a flat collision_table.yml.
- cuRobo's collision world is synced from the real stage (UsdHelper's
  get_obstacles_from_stage()) but deliberately scoped to just the two ergo
  tables (`only_paths=[ergo_tables instance prim paths]`), NOT the whole
  factory backdrop. Syncing the entire ~13,500-object factory scene works
  but is both far too slow for an interactive loop (confirmed live: cuRobo
  warmup alone took minutes) and pointless, since none of that geometry is
  anywhere near this cell's actual reachable workspace.
- The reused RobotPedestal (table_layout.yaml's cr5_mount.pedestal) is
  excluded from the obstacle sync, even though it already falls outside
  the ergo-tables-only scope above -- confirmed live that omitting this
  makes cuRobo see the robot colliding with its own mounting stand and
  refuse to plan at all (`MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION`).
- The robot is NOT at the world origin here (cr5_mount.position places it
  elsewhere in the factory scene) -- unlike the pristine script's
  origin-mounted Franka, where the robot's own frame and world frame
  coincide. The draggable target's pose must be transformed into the
  robot's base frame before being handed to cuRobo
  (Pose.compute_local_pose()); getting this wrong would aim every plan at
  the wrong location, offset by the robot's mount translation.
- Uses isaacsim.core.api / isaacsim.core.utils (Isaac Sim 6.0.1's current,
  deprecated-but-functional namespace -- see scripts/import_cr5.py's own
  module comment) instead of omni.isaac.*.
- Under Newton (this repo's default physics backend), SimulationManager
  forces articulation state onto a torch backend
  ("Changing backend from 'numpy' to 'torch' since NumPy cannot be used
  with GPU pipelines"). Two concrete consequences found by actually running
  this against a live 6.0.1/Newton install, not guessed from docs:
    - robot.get_joints_state().positions/.velocities come back as CUDA
      torch tensors, not numpy -- np.isnan/np.max etc. need an explicit
      .cpu().numpy() first, or they raise/silently misbehave.
    - robot.set_joint_positions()'s own wrapper
      (isaacsim.core.prims.impl.single_articulation.SingleArticulation)
      calls its own, stale self._backend_utils.expand_dims() on
      joint_indices before delegating to self._articulation_view, which
      resolves indices via a *different*, correctly-torch backend_utils --
      passing a torch tensor through the wrapper gets silently coerced back
      to a numpy array by the wrapper's numpy-based expand_dims (numpy
      implicitly calls a tensor's __array__), which then crashes deeper
      down (`'numpy.ndarray' object has no attribute 'to'`) since that
      inner call expected the tensor to survive intact. Fixed by calling
      robot._articulation_view.set_joint_positions() directly, bypassing
      the wrapper's buggy preprocessing -- the pristine reference script
      already reaches into `_articulation_view` for other calls
      (initialize(), set_max_efforts()), so this isn't an unprecedented
      pattern, just a new call site needing it.
  Not clear whether this is fixed in a newer Isaac Sim point release or is
  a standing Newton/deprecated-Core-API interaction -- worth rechecking on
  a future version bump rather than assuming this workaround stays needed
  forever.
- cuRobo's WorldMeshCollision calls wp.torch.device_from_torch(), an older
  nested-submodule warp API path that doesn't exist in the warp-lang 1.14.0
  actually installed (pip auto-resolved the newest release when
  Dockerfile.curobo installed cuRobo from source -- not necessarily what
  the pinned cuRobo commit was tested against; confirmed this is the only
  wp.torch.* call site in the whole installed cuRobo package). Shimmed a
  fake warp.torch namespace at module load time rather than re-pinning
  warp-lang in the Dockerfile or patching cuRobo's vendored source.
- Trims CLI flags not relevant to this project (--reactive,
  --constrain_grasp_approach, --reach_partial_pose, --hold_partial_pose,
  --external_asset_path/--external_robot_configs_path -- this repo already
  fixes robot config via table_layout.yaml/cr5_mount.robot_override) down
  to just --headless and --visualize-spheres.

Verified against a live Isaac Sim 6.0.1 install (real GPU, Newton physics
backend, Franka via cr5_mount.robot_override): --headless mode builds the
full scene, warms up cuRobo against 74 real obstacles synced from the two
ergo tables, moves the target programmatically, and successfully plans and
executes a reach (confirmed via a real MotionGenResult.success, not just
"no exception raised") -- joint positions end at a new, non-default,
finite pose matching the moved target. NOT verified in interactive GUI
mode with a human actually dragging the cube: this host currently has no
X11 forwarding configured for the docker/container.py container path (see
CLAUDE.md's devcontainer X11 entry) -- run this on a host that has one, or
finish that separately first. NOT verified against the CR5 itself (only
Franka, since cr5_mount.robot_override.enabled is still true).

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/motion_gen_teleop.py

--headless mode can't show you dragging the cube (there's no GUI) -- it
instead moves the target itself once, programmatically, so the whole
build+plan+execute pipeline can be verified by automation without a
display.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--headless",
    action="store_true",
    default=False,
    help="Run headless; also drives the target cuboid programmatically once, for automated smoke-testing.",
)
parser.add_argument(
    "--visualize-spheres",
    action="store_true",
    default=False,
    help="Visualize the robot's cuRobo collision spheres.",
)
args = parser.parse_args()

import os  # noqa: E402

from isaacsim import SimulationApp  # noqa: E402

if __name__ == "__main__":
    # SimulationApp defaults to the minimal isaacsim.exp.base.kit experience
    # when no `experience` is passed (confirmed by reading
    # isaacsim.simulation_app.SimulationApp's own resolution order) -- not
    # the full-featured isaacsim.exp.full.kit that isaac-sim.sh (and this
    # repo's own scripts/launch_isaac_sim.sh) launches, which is why panels
    # like Joint Inspector were missing. Only apply this for the interactive
    # GUI path -- the --headless automated smoke test doesn't need the full
    # UI experience and this keeps that existing path unchanged. Deliberately
    # NOT isaacsim.exp.full.newton.kit -- that variant disables PhysX
    # extensions entirely, which would break this project's own
    # physics_backend.enabled: false fallback; this script already calls
    # enable_newton_physics() itself when the config asks for it, and the
    # generic full.kit experience supports both backends.
    experience = "" if args.headless else f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    simulation_app = SimulationApp(
        {"headless": args.headless, "width": "1920", "height": "1080"}, experience=experience
    )

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.kit.commands  # noqa: E402
import torch  # noqa: E402
import warp as wp  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import cuboid, sphere  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Sdf, Usd, UsdPhysics, UsdShade  # noqa: E402

from curobo.geom.types import WorldConfig  # noqa: E402
from curobo.types.base import TensorDeviceType  # noqa: E402
from curobo.types.math import Pose  # noqa: E402
from curobo.types.state import JointState  # noqa: E402
from curobo.util.logger import log_error, setup_curobo_logger  # noqa: E402
from curobo.util.usd_helper import UsdHelper  # noqa: E402
from curobo.util_file import get_robot_configs_path, join_path, load_yaml  # noqa: E402
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig  # noqa: E402

# cuRobo's WorldMeshCollision (curobo/geom/sdf/world_mesh.py, the only
# wp.torch.* call site in the whole installed package) calls
# wp.torch.device_from_torch() -- an older nested-submodule API path that
# doesn't exist in warp-lang 1.14.0 (what pip actually resolved when
# Dockerfile.curobo installed cuRobo from source, not necessarily what the
# pinned cuRobo commit was tested against): that function now lives at the
# top level as wp.device_from_torch instead. Confirmed live
# (`ModuleNotFoundError: No module named 'warp.torch'`) before adding this.
# A fake "warp.torch" namespace shim here is cheaper than re-pinning an
# older warp-lang in the Dockerfile (which risks reopening the Newton
# physics backend's own warp-version questions) or patching cuRobo's
# vendored source directly.
if not hasattr(wp, "torch"):
    import types

    wp.torch = types.SimpleNamespace(device_from_torch=wp.device_from_torch)

from build_scene import (  # noqa: E402
    build_ergo_tables,
    build_factory,
    load_config,
    mount_cr5,
    mount_cr5_pedestal,
    setup_curobo_motion_gen,
)
from newton_backend import enable_newton_physics  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent


def resolve_robot_cfg_dict(cfg: dict) -> dict:
    """Loads the same robot config yml that build_scene.setup_curobo_motion_gen()
    builds its MotionGen from -- mirrors that function's CR5-vs-
    robot_override branching, but returns the raw dict (for joint names /
    retract config) instead of a constructed MotionGen. Kept as a small,
    separate helper here rather than changing setup_curobo_motion_gen()'s
    own return type, since build_scene.py's callers only need the
    MotionGen itself.
    """
    mount_cfg = cfg["cr5_mount"]
    override = mount_cfg.get("robot_override")
    if override and override.get("enabled"):
        robot_cfg_path = join_path(get_robot_configs_path(), override["motion_gen_robot_cfg"])
        return load_yaml(robot_cfg_path)["robot_cfg"]
    cr5_yml = REPO_ROOT / "configs" / "curobo" / "cr5.yml"
    return load_yaml(str(cr5_yml))["robot_cfg"]


def main() -> None:
    cfg = load_config()

    physics_backend_cfg = cfg.get("physics_backend", {})
    if physics_backend_cfg.get("enabled", False):
        if not enable_newton_physics():
            raise RuntimeError("physics_backend.enabled is true but Newton could not be enabled -- see logs above")

    setup_curobo_logger("warn")
    tensor_args = TensorDeviceType()

    world = World()
    build_factory(cfg)
    for _ in range(120):
        simulation_app.update()
    build_ergo_tables(cfg)
    mount_cr5(cfg)
    mount_cr5_pedestal(cfg)
    world.scene.add_default_ground_plane()

    mount_cfg = cfg["cr5_mount"]
    robot_prim_path = mount_cfg["prim_path"]
    robot = world.scene.add(Robot(prim_path=robot_prim_path, name="robot"))

    robot_base_pose = Pose(
        position=tensor_args.to_device(np.array(mount_cfg["position"], dtype=np.float32)),
        quaternion=tensor_args.to_device(np.array(mount_cfg["orientation_wxyz"], dtype=np.float32)),
    )

    robot_cfg = resolve_robot_cfg_dict(cfg)
    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = robot_cfg["kinematics"]["cspace"]["retract_config"]

    # Gripper open/close via keyboard -- panda_finger_joint1/2 aren't in
    # cuRobo's cspace joint_names above (gripper actuation is separate from
    # arm motion planning), so this is fully independent of the cmd_plan
    # loop below; can't conflict since it targets different joint indices.
    # carb.input + omni.appwindow, per-frame polling (this script already
    # has a while-loop stepping every frame, so no callback/subscription
    # needed) -- confirmed this is the current, non-deprecated pattern by
    # reading a real usage (isaacsim.replicator.experimental.mobility_gen's
    # KeyboardDriver) rather than assuming from general knowledge.
    import carb.input
    import omni.appwindow

    appwindow = omni.appwindow.get_default_app_window()
    input_iface = carb.input.acquire_input_interface()
    keyboard = appwindow.get_keyboard()

    # Confirmed from franka_panda.urdf: both joints are prismatic,
    # limit lower=0.0 upper=0.04 (meters). O opens (toward 0.04 each,
    # moving the two fingers apart -- each joint's own axis is flipped
    # relative to the other, so the same positive target opens both),
    # C closes (toward 0.0, fingers together).
    GRIPPER_OPEN_POS = 0.04
    GRIPPER_CLOSED_POS = 0.0
    gripper_finger_idx = None  # resolved once the robot articulation is ready, below

    # Target cuboid to drag, placed within reach in the ROBOT's own base
    # frame (this scene mounts the robot away from the world origin), then
    # converted to world coordinates for actually spawning it. Created
    # before the obstacle sync below so it can be excluded from it by path.
    target_local = Pose(
        position=tensor_args.to_device(np.array([0.4, 0.0, 0.4], dtype=np.float32)),
        quaternion=tensor_args.to_device(np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float32)),
    )
    target_world = robot_base_pose.multiply(target_local)
    target_world_pos = target_world.position.cpu().numpy().reshape(3)
    target_world_quat = target_world.quaternion.cpu().numpy().reshape(4)

    # The drag target IS the ghost gripper -- following main's own
    # build_teleop_target() (scripts/build_scene.py on main, from the
    # "interactive cuRobo teleop" commit): a *detached copy* of the robot's
    # own end-effector visual mesh, not a plain marker, so it shows exactly
    # what will arrive at that pose -- one unified prim serves as both the
    # visual and the functional drag target, no separate cube.
    #
    # main's own version copies a clean f"{ee_link}/visuals" sub-scope
    # (Isaac Sim 5.1.0's URDF importer apparently produced one) via
    # omni.kit.commands.execute("CopyPrim", ...). This branch's importer
    # (6.0.1, redesigned -- see CLAUDE.md's import_cr5.py entry) doesn't
    # produce a single equivalent "visuals" prim: confirmed live by walking
    # the actual imported hierarchy that panda_hand's visual content is
    # split across panda_hand/hand, panda_hand/hand_1 (clean, no APIs) and
    # panda_hand/panda_leftfinger/finger(+finger_1),
    # panda_hand/panda_rightfinger/finger(+finger_1) (also clean -- the
    # RigidBodyAPI actually lives one level up, on panda_leftfinger/
    # panda_rightfinger themselves). Rather than hand-assembling six
    # separate sub-copies at two different nesting depths (fragile, and
    # this script must also still work for the CR5 once
    # robot_override.enabled flips back to false), CopyPrim the *whole*
    # panda_hand (found by name search, not hardcoded -- same reasoning)
    # and strip RigidBodyAPI/CollisionAPI from the copy afterward --
    # CopyPrim's own doc comment on main notes it correctly preserves
    # instanceable mesh references end to end, and unlike
    # AddInternalReference (tried first, worked for physics-safety once
    # APIs were stripped, but left an unconfirmed doubt about whether a
    # live reference's own composed transform could stack with the pose
    # set afterward) a CopyPrim is a fully independent, flattened prim
    # spec -- no composition arc, no ambiguity about transform stacking.
    #
    # Known simplification (confirmed acceptable via AskUserQuestion): the
    # copy's fingers are frozen at whatever pose panda_hand's descendants
    # were in at copy time -- only the ghost's overall position/
    # orientation is draggable, matching the real gripper's live open/close
    # state isn't attempted here.
    #
    # Fallback: if panda_hand can't be found (e.g. testing the CR5 later,
    # once robot_override.enabled flips back to false and there's no
    # Franka-specific "panda_hand" link at all), fall back to a plain
    # VisualCuboid so there's always something to drag.
    GHOST_PRIM_PATH = "/World/GhostGripper"
    geometry_scope_prim = world.stage.GetPrimAtPath(f"{robot_prim_path}/Geometry")
    search_root = geometry_scope_prim if geometry_scope_prim.IsValid() else world.stage.GetPrimAtPath(robot_prim_path)
    panda_hand_path = None
    for p in Usd.PrimRange(search_root):
        if p.GetName() == "panda_hand":
            panda_hand_path = p.GetPath()
            break

    if panda_hand_path is not None:
        omni.kit.commands.execute("CopyPrim", path_from=str(panda_hand_path), path_to=GHOST_PRIM_PATH)
        ghost_prim = world.stage.GetPrimAtPath(GHOST_PRIM_PATH)
        # Confirmed live (diagnostic scan): panda_leftfinger/panda_rightfinger
        # (children of panda_hand) carry real RigidBodyAPI -- copied along
        # with everything else by CopyPrim, same as a reference would have.
        # Left in place, these would give Newton two phantom, unconstrained
        # "rigid bodies" with no joint attaching them to anything -- the
        # actual cause of a GUI-only NaN crash on Play. Must walk the WHOLE
        # copied subtree, not just its root.
        for p in Usd.PrimRange(ghost_prim):
            for api in (UsdPhysics.RigidBodyAPI, UsdPhysics.CollisionAPI, UsdPhysics.ArticulationRootAPI):
                if p.HasAPI(api):
                    p.RemoveAPI(api)

        ghost_material = UsdShade.Material.Define(world.stage, "/World/Looks/GhostGripperMaterial")
        ghost_shader = UsdShade.Shader.Define(world.stage, "/World/Looks/GhostGripperMaterial/Shader")
        ghost_shader.CreateIdAttr("UsdPreviewSurface")
        ghost_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.2, 0.6, 1.0))
        ghost_shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.25)
        ghost_material.CreateSurfaceOutput().ConnectToSource(ghost_shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(ghost_prim).Bind(
            ghost_material, bindingStrength=UsdShade.Tokens.strongerThanDescendants
        )

        target = SingleXFormPrim(GHOST_PRIM_PATH)
        target.set_world_pose(position=target_world_pos, orientation=target_world_quat)
        print(f"[motion_gen_teleop] ghost gripper cloned from {panda_hand_path}", flush=True)
        print("[motion_gen_teleop] this is now the drag target -- no separate cube", flush=True)
    else:
        print(
            f"[motion_gen_teleop] WARNING: no 'panda_hand' prim found under {robot_prim_path} "
            "-- falling back to a plain cube target (expected if robot_override is disabled / running the CR5)",
            flush=True,
        )
        target = cuboid.VisualCuboid(
            "/World/target",
            position=target_world_pos,
            orientation=target_world_quat,
            color=np.array([1.0, 0, 0]),
            size=0.05,
        )

    usd_help = UsdHelper()
    usd_help.load_stage(world.stage)

    # Scoped to just the two ergo tables near the robot, NOT the whole
    # factory: get_obstacles_from_stage() converts every matched prim into
    # a real cuRobo mesh/SDF obstacle, and the full factory backdrop (racks,
    # welding line, roof, etc. -- 13000+ prims) made this both far too slow
    # for an interactive loop (confirmed live: warmup alone took minutes)
    # and pointless, since none of it is anywhere near this cell's actual
    # reachable workspace. The reused RobotPedestal (table_layout.yaml's
    # cr5_mount.pedestal) is excluded too even though it's outside this
    # narrower scope already, defensively, since a wider-scoped only_paths
    # would otherwise let the robot see its own mounting stand as an
    # obstacle and refuse to plan at all
    # (MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION, confirmed live).
    only_paths = [instance["prim_path"] for instance in cfg["ergo_tables"]["instances"]]
    pedestal_prim_path = mount_cfg["pedestal"]["prim_path"]
    ignore_substring = [
        robot_prim_path,
        pedestal_prim_path,
        "/World/target",
        GHOST_PRIM_PATH,
        "/World/defaultGroundPlane",
        "/curobo",
    ]

    def sync_obstacles_from_stage():
        return usd_help.get_obstacles_from_stage(
            only_paths=only_paths,
            reference_prim_path=robot_prim_path,
            ignore_substring=ignore_substring,
        ).get_collision_check_world()

    print("[motion_gen_teleop] warming up cuRobo motion_gen (viewport will look frozen/black until this finishes)...", flush=True)
    # Real factory-scene geometry as the starting obstacle world (synced
    # from the stage that's already built above), unlike the pristine
    # reference script's synthetic collision_table.yml -- cuRobo's warmup
    # requires at least one obstacle to be present, so this can't be a
    # bare WorldConfig() and rely solely on the loop's periodic re-sync.
    # build_scene.py's own call to this function passes no world_cfg at
    # all (world_coll_checker stays unset -- it doesn't need collision
    # checking, just a warmed-up config), so this must be passed explicitly.
    initial_obstacles = sync_obstacles_from_stage()
    print(f"[motion_gen_teleop] {len(initial_obstacles.objects)} obstacle(s) synced from stage for warmup", flush=True)
    motion_gen = setup_curobo_motion_gen(cfg, world_cfg=initial_obstacles)
    if motion_gen is None:
        raise RuntimeError("cuRobo isn't installed -- motion_gen_teleop.py needs the curobo Docker profile")
    print("[motion_gen_teleop] cuRobo is ready", flush=True)

    # GUI mode waits for the user to click Play (matches the pristine
    # reference script's UX -- gives a look at the built scene first).
    # Headless has no GUI to click Play in, so start the timeline directly.
    if args.headless:
        world.play()

    plan_config = MotionGenPlanConfig(
        enable_graph=False,
        enable_graph_attempt=2,
        max_attempts=4,
        enable_finetune_trajopt=True,
        time_dilation_factor=0.5,
    )

    articulation_controller = None
    idx_list = None
    cmd_plan = None
    cmd_idx = 0
    past_pose = None
    target_pose = None
    past_orientation = None
    target_orientation = None
    spheres = None
    headless_move_done = False
    cmd_plan_ever_succeeded = False

    i = 0
    loop_count = 0
    # Safety net for --headless automated runs only -- GUI mode relies on
    # the user closing the window, not a step count.
    max_headless_loops = 2000
    while simulation_app.is_running():
        loop_count += 1
        if args.headless and loop_count > max_headless_loops:
            raise RuntimeError(f"FAIL: headless smoke test exceeded {max_headless_loops} loop iterations without finishing")
        world.step(render=not args.headless)
        if not world.is_playing():
            if i % 100 == 0:
                print("**** Click Play to start simulation *****", flush=True)
            i += 1
            continue

        step_index = world.current_time_step_index
        if articulation_controller is None:
            articulation_controller = robot.get_articulation_controller()
        if step_index < 10:
            robot.initialize()
            # torch tensors, not plain lists/numpy -- Newton forces
            # SimulationManager onto a "torch" backend ("Changing backend
            # from 'numpy' to 'torch' since NumPy cannot be used with GPU
            # pipelines"). Also: call _articulation_view.set_joint_positions()
            # directly rather than the robot.set_joint_positions() wrapper --
            # that wrapper's own self._backend_utils.expand_dims(joint_indices, 0)
            # silently coerces a torch tensor back into a numpy array (numpy's
            # expand_dims invokes __array__ on anything array-like), which
            # then blows up in the *view's* separate, correctly-torch
            # self._backend_utils.resolve_indices() a few calls later
            # (AttributeError: 'numpy.ndarray' object has no attribute 'to') --
            # a real mismatch between the wrapper's and the view's own backend
            # state under Newton, confirmed by reading both classes' source.
            idx_list = torch.tensor([robot.get_dof_index(x) for x in j_names], dtype=torch.long)
            robot._articulation_view.set_joint_positions(
                positions=torch.tensor(default_config, dtype=torch.float32).unsqueeze(0),
                joint_indices=idx_list,
            )
            gripper_finger_idx = torch.tensor(
                [robot.get_dof_index("panda_finger_joint1"), robot.get_dof_index("panda_finger_joint2")],
                dtype=torch.long,
            )
        if step_index < 20:
            continue

        # Gripper open/close -- independent of cuRobo's arm plan below
        # (different joint indices, can't conflict). Drive TARGET, not an
        # instant teleport, so it moves smoothly under the existing joint
        # stiffness/damping -- same physically-driven behavior already
        # confirmed working via the GUI's Joint Inspector, not a kinematic
        # snap. GUI-only: headless has no keyboard to poll.
        if not args.headless and gripper_finger_idx is not None:
            if input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.O) != 0:
                robot._articulation_view.set_joint_position_targets(
                    positions=torch.tensor([[GRIPPER_OPEN_POS, GRIPPER_OPEN_POS]], dtype=torch.float32),
                    joint_indices=gripper_finger_idx,
                )
            elif input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.C) != 0:
                robot._articulation_view.set_joint_position_targets(
                    positions=torch.tensor([[GRIPPER_CLOSED_POS, GRIPPER_CLOSED_POS]], dtype=torch.float32),
                    joint_indices=gripper_finger_idx,
                )

        if step_index == 50 or step_index % 1000 == 0:
            print(f"[motion_gen_teleop] syncing cuRobo world from stage w.r.t. {robot_prim_path}", flush=True)
            motion_gen.update_world(sync_obstacles_from_stage())
            carb.log_info("[motion_gen_teleop] synced cuRobo world from stage")

        # In headless smoke-test mode there's no GUI to drag the cube in --
        # move it ourselves once, to exercise the same plan+execute path.
        if args.headless and not headless_move_done and step_index > 60:
            new_pos = target_world_pos + np.array([0.1, 0.05, -0.05])
            target.set_world_pose(position=new_pos, orientation=target_world_quat)
            headless_move_done = True

        cube_position, cube_orientation = target.get_world_pose()

        if past_pose is None:
            past_pose = cube_position
        if target_pose is None:
            target_pose = cube_position
        if target_orientation is None:
            target_orientation = cube_orientation
        if past_orientation is None:
            past_orientation = cube_orientation

        sim_js = robot.get_joints_state()
        if sim_js is None:
            continue
        sim_js_names = robot.dof_names
        # Under Newton, get_joints_state() returns CUDA torch tensors, not
        # numpy arrays ("Changing backend from 'numpy' to 'torch' since
        # NumPy cannot be used with GPU pipelines") -- np.isnan/np.max etc.
        # below need numpy, and a CUDA tensor can't implicitly convert.
        sim_js_positions = sim_js.positions.cpu().numpy() if torch.is_tensor(sim_js.positions) else sim_js.positions
        sim_js_velocities = sim_js.velocities.cpu().numpy() if torch.is_tensor(sim_js.velocities) else sim_js.velocities
        if np.any(np.isnan(sim_js_positions)):
            log_error("isaac sim has returned NAN joint position values.")
        cu_js = JointState(
            position=tensor_args.to_device(sim_js_positions),
            velocity=tensor_args.to_device(sim_js_velocities) * 0.0,
            acceleration=tensor_args.to_device(sim_js_velocities) * 0.0,
            jerk=tensor_args.to_device(sim_js_velocities) * 0.0,
            joint_names=sim_js_names,
        )
        cu_js = cu_js.get_ordered_joint_state(motion_gen.kinematics.joint_names)

        if args.visualize_spheres and step_index % 2 == 0:
            sph_list = motion_gen.kinematics.get_robot_as_spheres(cu_js.position)
            if spheres is None:
                spheres = []
                for si, s in enumerate(sph_list[0]):
                    sp = sphere.VisualSphere(
                        prim_path=f"/curobo/robot_sphere_{si}",
                        position=np.ravel(s.position),
                        radius=float(s.radius),
                        color=np.array([0, 0.8, 0.2]),
                    )
                    spheres.append(sp)
            else:
                for si, s in enumerate(sph_list[0]):
                    if not np.isnan(s.position[0]):
                        spheres[si].set_world_pose(position=np.ravel(s.position))
                        spheres[si].set_radius(float(s.radius))

        robot_static = np.max(np.abs(sim_js_velocities)) < 0.5
        if (
            (np.linalg.norm(cube_position - target_pose) > 1e-3 or np.linalg.norm(cube_orientation - target_orientation) > 1e-3)
            and np.linalg.norm(past_pose - cube_position) == 0.0
            and np.linalg.norm(past_orientation - cube_orientation) == 0.0
            and robot_static
        ):
            target_pose_world = Pose(
                position=tensor_args.to_device(cube_position),
                quaternion=tensor_args.to_device(cube_orientation),
            )
            # Convert from world frame into the robot's own base frame --
            # this scene's robot isn't at the world origin, unlike the
            # pristine reference script's setup.
            ik_goal = robot_base_pose.compute_local_pose(target_pose_world)

            result = motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, plan_config)
            succ = result.success.item()
            if succ:
                cmd_plan_ever_succeeded = True
                cmd_plan = result.get_interpolated_plan()
                cmd_plan = motion_gen.get_full_js(cmd_plan)
                new_idx_list = []
                common_js_names = []
                for x in sim_js_names:
                    if x in cmd_plan.joint_names:
                        new_idx_list.append(robot.get_dof_index(x))
                        common_js_names.append(x)
                idx_list = torch.tensor(new_idx_list, dtype=torch.long)
                cmd_plan = cmd_plan.get_ordered_joint_state(common_js_names)
                cmd_idx = 0
                print(f"[motion_gen_teleop] plan succeeded, {len(cmd_plan.position)} steps", flush=True)
            else:
                carb.log_warn(f"[motion_gen_teleop] plan did not converge to a solution: {result.status}")
            target_pose = cube_position
            target_orientation = cube_orientation
        past_pose = cube_position
        past_orientation = cube_orientation

        if cmd_plan is not None:
            cmd_state = cmd_plan[cmd_idx]
            art_action = ArticulationAction(
                cmd_state.position.cpu().numpy(),
                cmd_state.velocity.cpu().numpy(),
                joint_indices=idx_list,
            )
            articulation_controller.apply_action(art_action)
            cmd_idx += 1
            for _ in range(2):
                world.step(render=False)
            if cmd_idx >= len(cmd_plan.position):
                cmd_idx = 0
                cmd_plan = None

        if args.headless and headless_move_done and cmd_plan is None and step_index > 400:
            break

    if args.headless:
        final_pos, _ = target.get_world_pose()
        joints_final = robot.get_joints_state()
        final_joint_positions = (
            joints_final.positions.cpu().numpy() if torch.is_tensor(joints_final.positions) else joints_final.positions
        )
        print(f"[motion_gen_teleop] final target world pos: {final_pos}", flush=True)
        print(f"[motion_gen_teleop] final joint positions: {final_joint_positions}", flush=True)
        if np.any(np.isnan(final_joint_positions)):
            raise RuntimeError("FAIL: robot joint positions are NaN after headless smoke test")
        if not cmd_plan_ever_succeeded:
            raise RuntimeError("FAIL: cuRobo never successfully planned a reach to the moved target")
        print("[motion_gen_teleop] PASS: headless smoke test completed, plan succeeded, no NaNs", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
