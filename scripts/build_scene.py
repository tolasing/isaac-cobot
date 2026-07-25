"""Builds /World/Factory, two ErgoTable desks, imports+mounts the CR5 cobot (or, temporarily, a
Franka Panda -- see cr5_mount.robot_override), and runs an interactive cuRobo teleop loop.
Superseded by build_scene_mefron.py -- see README.md. Run standalone:
${ISAACSIM_ROOT_PATH}/python.sh scripts/build_scene.py"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import yaml
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": _headless})

import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402

from import_cr5 import import_cr5  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = REPO_ROOT / "configs" / "scene" / "table_layout.yaml"


def load_config() -> dict:
    return yaml.safe_load(CONFIG_PATH.read_text())


def build_factory(cfg: dict) -> None:
    factory_cfg = cfg["factory"]
    backdrop_usd = REPO_ROOT / factory_cfg["backdrop_usd"]
    if not backdrop_usd.is_file():
        raise FileNotFoundError(f"{backdrop_usd} not found -- see assets/factory/SOURCE.md for how to fetch it.")
    add_reference_to_stage(usd_path=str(backdrop_usd), prim_path=factory_cfg["prim_path"])


def build_ergo_tables(cfg: dict) -> None:
    """Copies the vendored ErgoTable desk prop to two positions near the robot. CopyPrim (not
    MovePrim -- see mount_cr5_pedestal's docstring) duplicates the source's composition arcs
    cleanly, so each copy renders with full geometry independent of the original."""
    ergo_cfg = cfg["ergo_tables"]
    source_path = ergo_cfg["source_prim_path"]
    for instance in ergo_cfg["instances"]:
        prim_path = instance["prim_path"]
        omni.kit.commands.execute("CopyPrim", path_from=source_path, path_to=prim_path)
        x, y = instance["position_xy"]
        xform = SingleXFormPrim(prim_path=prim_path)
        xform.set_world_pose(position=np.array([x, y, 0.0]), orientation=np.array(instance["orientation_wxyz"]))
        xform.set_local_scale(np.array(ergo_cfg["scale"]))


def build_assembly_parts(cfg: dict) -> None:
    """References external assembly-part USD files onto the work surfaces, via
    add_reference_to_stage (not CopyPrim -- these are standalone files, not prims already on this
    stage). instance.rigid_body: true applies setRigidBody's convexHull approximation, NOT
    kinematic: it's meant to be picked up and moved, not a fixed prop like the table/pedestal."""
    assembly_cfg = cfg.get("assembly_parts")
    if not assembly_cfg:
        return

    stage = omni.usd.get_context().get_stage()
    for instance in assembly_cfg["instances"]:
        usd_path = REPO_ROOT / instance["usd_path"]
        if not usd_path.is_file():
            raise FileNotFoundError(f"{usd_path} not found (assembly_parts.instances[{instance['name']!r}]).")
        prim_path = instance["prim_path"]
        add_reference_to_stage(usd_path=str(usd_path), prim_path=prim_path)
        xform = SingleXFormPrim(prim_path=prim_path)
        xform.set_world_pose(
            position=np.array(instance["position"]),
            orientation=np.array(instance["orientation_wxyz"]),
        )
        xform.set_local_scale(np.array(instance["scale"]))

        if instance.get("rigid_body"):
            from omni.physx.scripts import utils as physx_utils

            physx_utils.setRigidBody(stage.GetPrimAtPath(prim_path), "convexHull", False)


def mount_cr5(cfg: dict) -> None:
    mount_cfg = cfg["cr5_mount"]
    override = mount_cfg.get("robot_override")
    if override and override.get("enabled"):
        # Lazy import: build_scene.py otherwise has no cuRobo dependency
        # and must keep working in the `base` profile (no cuRobo installed)
        # when this temporary override isn't enabled.
        from curobo.util_file import get_assets_path, join_path

        urdf_path = Path(join_path(get_assets_path(), override["urdf_relative_path"]))
        import_cr5(
            urdf_path=urdf_path,
            prim_path=mount_cfg["prim_path"],
            default_drive_strength=override["default_drive_strength"],
            default_position_drive_damping=override["default_position_drive_damping"],
        )
    else:
        import_cr5(prim_path=mount_cfg["prim_path"])
    xform = SingleXFormPrim(prim_path=mount_cfg["prim_path"])
    xform.set_world_pose(
        position=np.array(mount_cfg["position"]),
        orientation=np.array(mount_cfg["orientation_wxyz"]),
    )
    xform.set_local_scale(np.array(mount_cfg["scale"]))


def mount_cr5_pedestal(cfg: dict) -> None:
    """Repositions the reused RobotPedestal prim so the robot isn't left floating. Overrides pose
    in place rather than moving it: MovePrim on this deeply-referenced vendored prim leaves an
    empty shell behind. Uses set_local_pose(), not set_world_pose(): the config values are LOCAL,
    read directly from the GUI's Property panel, since the parent chain has a large baked-in offset."""
    pedestal_cfg = cfg["cr5_mount"]["pedestal"]
    xform = SingleXFormPrim(prim_path=pedestal_cfg["prim_path"])
    xform.set_local_pose(
        translation=np.array(pedestal_cfg["local_translation"]),
        orientation=np.array(pedestal_cfg["local_orientation_wxyz"]),
    )
    xform.set_local_scale(np.array(pedestal_cfg["scale"]))


def build_teleop_target(cfg: dict, robot_prim_path: str, robot_cfg: dict) -> SingleXFormPrim:
    """Creates a draggable target the operator moves to command the end-effector pose via cuRobo
    -- a detached copy of the robot's own end-effector visual mesh, not a plain marker. CopyPrim
    correctly preserves the instanceable mesh reference the URDF importer uses, so the copy
    renders with full geometry despite the source showing zero children under plain traversal."""
    target_cfg = cfg["teleop_target"]
    ee_link = robot_cfg["kinematics"]["ee_link"]
    source_path = f"{robot_prim_path}/{ee_link}/visuals"
    omni.kit.commands.execute("CopyPrim", path_from=source_path, path_to=target_cfg["prim_path"])
    xform = SingleXFormPrim(prim_path=target_cfg["prim_path"])
    xform.set_world_pose(
        position=np.array(target_cfg["position"]),
        orientation=np.array(target_cfg["orientation_wxyz"]),
    )
    return xform


def get_teleop_obstacles(cfg: dict, robot_prim_path: str):
    """Scans just the ergo tables and pedestal for cuRobo collision obstacles, not the whole
    /World/Factory backdrop (thousands of small meshes, none in the robot's actual reach). Derives
    the scan scope from ergo_tables/cr5_mount.pedestal's own config, not a separately-maintained
    path list, so there's nothing to keep in sync if those prims move."""
    from curobo.util.usd_helper import UsdHelper

    target_cfg = cfg["teleop_target"]
    only_paths = [instance["prim_path"] for instance in cfg["ergo_tables"]["instances"]]
    only_paths.append(cfg["cr5_mount"]["pedestal"]["prim_path"])

    usd_help = UsdHelper()
    usd_help.load_stage(omni.usd.get_context().get_stage())
    return usd_help.get_obstacles_from_stage(
        only_paths=only_paths,
        reference_prim_path=robot_prim_path,
        ignore_substring=[robot_prim_path, target_cfg["prim_path"], "/curobo"],
    ).get_collision_check_world()


def setup_curobo_motion_gen(cfg: dict):
    """Builds and warms up a cuRobo MotionGen for whichever robot is mounted at cr5_mount. Returns
    (motion_gen, robot_cfg) -- both None if cuRobo isn't installed (the `base` Docker profile).
    Passes a real, populated world up front, not the None default -- confirmed live that an
    empty/absent world leaves world_coll_checker as None and warmup() itself fails."""
    try:
        from curobo.types.base import TensorDeviceType
        from curobo.util_file import get_robot_configs_path, join_path, load_yaml
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
    except ImportError:
        print("[build_scene] cuRobo not installed -- skipping MotionGen setup.", flush=True)
        return None, None

    mount_cfg = cfg["cr5_mount"]
    override = mount_cfg.get("robot_override")
    if override and override.get("enabled"):
        robot_cfg = load_yaml(join_path(get_robot_configs_path(), override["motion_gen_robot_cfg"]))["robot_cfg"]
    else:
        # See configs/curobo/cr5.yml's module comment: urdf_path/
        # asset_root_path/collision_spheres are repo-root-relative for
        # readability, but cuRobo always resolves them against its own
        # bundled assets/config dirs unless patched to absolute paths here.
        cr5_yml = REPO_ROOT / "configs" / "curobo" / "cr5.yml"
        robot_cfg = load_yaml(str(cr5_yml))["robot_cfg"]
        k = robot_cfg["kinematics"]
        k["urdf_path"] = str(REPO_ROOT / k["urdf_path"])
        k["asset_root_path"] = str(REPO_ROOT / k["asset_root_path"])
        k["collision_spheres"] = str(cr5_yml.parent / k["collision_spheres"])

    world_cfg = get_teleop_obstacles(cfg, robot_prim_path=mount_cfg["prim_path"])
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        {"robot_cfg": robot_cfg}, world_cfg, tensor_args=TensorDeviceType()
    )
    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup()
    return motion_gen, robot_cfg


# Loop-timing constants (frame counts, not scene/physical facts -- kept as
# plain constants here rather than promoted to table_layout.yaml). Ported
# from examples/curobo_reference/motion_gen_reacher.py's own magic numbers.
_TELEOP_INIT_FRAMES = 10  # hold default pose this many frames while physics/drives settle
_TELEOP_SETTLE_FRAMES = 20  # then wait this many more before planning starts
_TELEOP_OBSTACLE_RESCAN_INTERVAL = 1000  # re-scan obstacles every N frames


def run_teleop_loop(
    cfg: dict,
    motion_gen,
    robot_cfg: dict,
    target: SingleXFormPrim,
    robot_prim_path: str,
    max_iterations: int | None = None,
) -> None:
    """Drag `target` in the GUI viewport; the robot follows via cuRobo's MotionGen. A from-scratch
    port of examples/curobo_reference/motion_gen_reacher.py's debounce/plan/apply pattern, adapted
    to this repo's SingleArticulation convention and robot_cfg-sourced joint names/retract pose.
    max_iterations: None for interactive use; finite for headless verification."""
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.state import JointState
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

    # import_cr5() imports with create_physics_scene=False, so nothing has created one yet --
    # SingleArticulation.initialize() needs a real PhysicsScene prim to produce a simulation view,
    # confirmed live it raises AttributeError deep in isaacsim.core.prims otherwise.
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    tensor_args = TensorDeviceType()
    plan_config = MotionGenPlanConfig()
    timeline = omni.timeline.get_timeline_interface()

    # motion_gen operates in the robot's own base-link frame, not USD world space -- confirmed
    # live that passing the target's raw world pose as the IK goal made every plan fail with
    # IK_FAIL, since this robot (unlike the reference example's) isn't mounted at world origin.
    mount_cfg = cfg["cr5_mount"]
    robot_base_pose = Pose(
        position=tensor_args.to_device(np.array(mount_cfg["position"])),
        quaternion=tensor_args.to_device(np.array(mount_cfg["orientation_wxyz"])),
    )

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])
    target_cfg = cfg["teleop_target"]
    pose_delta_threshold = target_cfg["pose_delta_threshold"]
    static_joint_velocity_threshold = target_cfg["static_joint_velocity_threshold"]

    # robot.initialize() needs a real PhysX simulation view, which only exists once physics has
    # actually stepped -- confirmed live that calling it before the loop checks is_playing() below
    # crashes, since Play hasn't started yet. Defer until the loop confirms physics is running.
    robot = SingleArticulation(prim_path=robot_prim_path, name="teleop_robot")
    idx_list = None
    articulation_controller = None

    past_pose = None
    past_orientation = None
    target_pose = None
    target_orientation = None
    cmd_plan = None
    cmd_idx = 0
    obstacles = None
    step_index = 0
    not_playing_frames = 0

    while simulation_app.is_running():
        simulation_app.update()

        if not timeline.is_playing():
            not_playing_frames += 1
            if not_playing_frames % 100 == 0:
                print("[build_scene] Click Play to start cuRobo teleop.", flush=True)
            continue

        # step_index only advances while playing -- otherwise a user who takes a while to click
        # Play would blow past _TELEOP_INIT_FRAMES/_TELEOP_SETTLE_FRAMES before physics even started.
        step_index += 1
        if max_iterations is not None and step_index > max_iterations:
            return

        if idx_list is None:
            robot.initialize()
            idx_list = [robot.get_dof_index(x) for x in j_names]
            articulation_controller = robot.get_articulation_controller()

        if step_index < _TELEOP_INIT_FRAMES:
            robot.set_joint_positions(default_config, idx_list)
            continue
        if step_index < _TELEOP_SETTLE_FRAMES:
            continue

        if obstacles is None or step_index % _TELEOP_OBSTACLE_RESCAN_INTERVAL == 0:
            obstacles = get_teleop_obstacles(cfg, robot_prim_path)
            motion_gen.update_world(obstacles)

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
        cu_js = JointState(
            position=tensor_args.to_device(sim_js.positions),
            velocity=tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=sim_js_names,
        )
        cu_js = cu_js.get_ordered_joint_state(motion_gen.kinematics.joint_names)

        robot_static = bool(np.max(np.abs(sim_js.velocities)) < static_joint_velocity_threshold)

        if (
            (
                np.linalg.norm(cube_position - target_pose) > pose_delta_threshold
                or np.linalg.norm(cube_orientation - target_orientation) > pose_delta_threshold
            )
            and np.linalg.norm(past_pose - cube_position) == 0.0
            and np.linalg.norm(past_orientation - cube_orientation) == 0.0
            and robot_static
            and cmd_plan is None
        ):
            world_target_pose = Pose(
                position=tensor_args.to_device(cube_position),
                quaternion=tensor_args.to_device(cube_orientation),
            )
            ik_goal = robot_base_pose.compute_local_pose(world_target_pose)
            result = motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, plan_config)
            print(f"[build_scene] teleop plan_single success={result.success.item()}", flush=True)
            if result.success.item():
                cmd_plan = motion_gen.get_full_js(result.get_interpolated_plan())
                cmd_plan = cmd_plan.get_ordered_joint_state(sim_js_names)
                cmd_idx = 0
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
            if cmd_idx >= len(cmd_plan.position):
                cmd_idx = 0
                cmd_plan = None


def prune_factory_dressing(cfg: dict) -> list[str]:
    """Deactivates the welding line's sliding rail and robot pedestals under /World/Factory,
    leaving every other prim untouched -- two matching modes against `factory` config:
    prune_name_startswith (prefix match) and prune_exact_paths (for over-generic names, e.g.
    "Link1"). Deactivation, not deletion: reversible, never touches Factory.usd on disk."""
    factory_cfg = cfg["factory"]
    prefixes = [p.lower() for p in factory_cfg.get("prune_name_startswith", [])]
    exact_paths = factory_cfg.get("prune_exact_paths", [])

    stage = omni.usd.get_context().get_stage()
    pruned = []

    if prefixes:
        root = stage.GetPrimAtPath(factory_cfg["prim_path"])
        it = iter(Usd.PrimRange(root))
        for prim in it:
            if any(prim.GetName().lower().startswith(p) for p in prefixes):
                prim.SetActive(False)
                pruned.append(str(prim.GetPath()))
                it.PruneChildren()

    for path in exact_paths:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            prim.SetActive(False)
            pruned.append(path)

    return pruned


# TEMPORARY -- testing GitHub isaac-sim/IsaacSim#191 (drag-and-drop breaks after a URDF import).
# True skips mount_cr5()/cuRobo/teleop-target entirely so the rest of the scene still builds and
# can confirm drag-drop works with no URDF import. Not meant to be a permanent mode.
_SKIP_ROBOT_FOR_DRAGDROP_TEST = True


def main() -> None:
    cfg = load_config()
    build_factory(cfg)

    # The factory backdrop is a large USD reference and resolves
    # asynchronously -- give it a bounded number of frames to load before
    # pruning/copying/mounting/printing below (build_ergo_tables() copies a
    # prim that lives inside this reference, so it must come after this).
    for _ in range(120):
        simulation_app.update()

    pruned = prune_factory_dressing(cfg)
    print(f"[build_scene] pruned {len(pruned)} factory prim(s): {pruned}", flush=True)

    # After pruning so both copies inherit the deactivated Monitor/Keyboard.
    build_ergo_tables(cfg)

    # After build_ergo_tables(): assembly_parts.instances are positioned
    # relative to a specific ergo table's already-built world pose (see
    # table_layout.yaml's own comment on how that position was derived).
    build_assembly_parts(cfg)

    robot_prim_path = cfg["cr5_mount"]["prim_path"]
    motion_gen, robot_cfg, target = None, None, None
    if _SKIP_ROBOT_FOR_DRAGDROP_TEST:
        print(
            "[build_scene] _SKIP_ROBOT_FOR_DRAGDROP_TEST is True -- skipping "
            "mount_cr5()/cuRobo/teleop-target entirely (temporary, see this flag's own comment).",
            flush=True,
        )
    else:
        mount_cr5(cfg)
        mount_cr5_pedestal(cfg)

        # motion_gen.warmup() blocks the main thread with real GPU work and calls no
        # simulation_app.update() of its own -- the viewport going black/frozen is expected, not a hang.
        print("[build_scene] warming up cuRobo motion_gen (viewport will look frozen/black until this finishes)...", flush=True)
        motion_gen, robot_cfg = setup_curobo_motion_gen(cfg)
        print(f"[build_scene] curobo motion_gen: {'READY' if motion_gen else 'SKIPPED'}", flush=True)

        if motion_gen is not None:
            # Only build the teleop target if there's actually a motion_gen to
            # drive it -- no cuRobo means no teleop, so no point creating a
            # ghost target nothing will ever move.
            target = build_teleop_target(cfg, robot_prim_path=robot_prim_path, robot_cfg=robot_cfg)

    stage = omni.usd.get_context().get_stage()
    pedestal_prim_path = cfg["cr5_mount"]["pedestal"]["prim_path"]
    ergo_table_paths = [instance["prim_path"] for instance in cfg["ergo_tables"]["instances"]]
    assembly_part_paths = [instance["prim_path"] for instance in cfg.get("assembly_parts", {}).get("instances", [])]
    status_paths = [
        cfg["factory"]["prim_path"],
        *ergo_table_paths,
        *assembly_part_paths,
        robot_prim_path,
        pedestal_prim_path,
    ]
    if target is not None:
        status_paths.append(cfg["teleop_target"]["prim_path"])
    for prim_path in status_paths:
        prim = stage.GetPrimAtPath(prim_path)
        num_children = len(prim.GetChildren()) if prim.IsValid() else 0
        status = "OK" if prim.IsValid() else "MISSING"
        print(f"[build_scene] {prim_path}: {status} ({num_children} children)", flush=True)

    if _headless:
        simulation_app.close()
        return

    if motion_gen is not None:
        run_teleop_loop(cfg, motion_gen, robot_cfg, target, robot_prim_path=robot_prim_path)
    else:
        print("[build_scene] cuRobo not installed -- skipping interactive teleop; falling back to a bare update loop.", flush=True)
        while simulation_app.is_running():
            simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
