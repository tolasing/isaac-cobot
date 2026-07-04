"""Builds /World/Factory (backdrop), two reused ErgoTable desks near the
robot, imports+mounts the CR5 cobot (or, temporarily, a Franka Panda --
see cr5_mount.robot_override) between them, warms up a matching cuRobo
MotionGen (best-effort -- skipped if cuRobo isn't installed, e.g. the
`base` Docker profile), and -- when cuRobo is available and not running
--headless -- runs an interactive teleop loop: drag the ghost end-effector
target in the GUI viewport and the robot follows via MotionGen.plan_single().

Verified against a live Isaac Sim 5.1.0 install (isaac-cobot-base
container, real GPU). The factory backdrop asset loads asynchronously --
main() pumps a bounded number of frames after building so a one-shot
--headless run sees it fully resolved before pruning/mounting/printing.

Only creates its own SimulationApp when run standalone (`__main__`), same
reasoning as import_cr5.py -- safe to import as a library from a script
that already has one running (confirmed the hard way: importing this
module after a second SimulationApp already exists segfaults instead of
raising).

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/build_scene.py
"""

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
    """Copies the vendored ErgoTable desk prop to two positions near the
    robot, for holding assembly parts.

    CopyPrim (not MovePrim -- see mount_cr5_pedestal's docstring for why)
    duplicates the source's composition arcs cleanly, so each copy renders
    with full geometry independent of the original.
    """
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
    """References external assembly-part USD files (CAD, converted outside
    this repo's own vendored-asset pipeline -- see assembly_parts' own
    config comments) onto the work surfaces.

    Uses add_reference_to_stage (like build_factory), not CopyPrim (like
    build_ergo_tables): these are standalone external USD files, not prims
    already living on this stage.

    instance.rigid_body: true makes the part a dynamic PhysX rigid body
    (via omni.physx.scripts.utils.setRigidBody(), the same helper behind
    the GUI's own Add > Physics > Rigid Body action -- confirmed that
    action isn't reachable from this stage's right-click menu at all, the
    physics UI extension's menu contribution isn't loaded here). Applies
    a convexHull collision approximation recursively to every mesh under
    the part (setRigidBody's own behavior for an Xformable prim), so it
    behaves as one compound rigid body, not per-sub-mesh independent
    pieces. NOT kinematic: this is meant to be picked up and moved by the
    robot, not a fixed prop like the table/pedestal.
    """
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
    """Repositions the reused RobotPedestal prim (see
    factory.prune_name_startswith's comment in table_layout.yaml) so the
    robot isn't left floating.

    Overrides pose in place rather than moving/renaming the prim out of the
    welding line's hierarchy: RobotPedestal's mesh comes from nested
    `reference` arcs several levels deep in the vendored asset, and
    MovePrim on a prim like that leaves an empty shell behind (0 children).

    Uses set_local_pose(), not set_world_pose(): pedestal.local_translation/
    local_orientation_wxyz (configs/scene/table_layout.yaml) are LOCAL
    values read directly from the GUI's Property panel, since
    RobotPedestal's parent chain has a large offset baked into the vendored
    asset -- set_world_pose() would instead compute a different local
    transform needed to reach that number as a *world* position, which is
    not what these values represent.
    """
    pedestal_cfg = cfg["cr5_mount"]["pedestal"]
    xform = SingleXFormPrim(prim_path=pedestal_cfg["prim_path"])
    xform.set_local_pose(
        translation=np.array(pedestal_cfg["local_translation"]),
        orientation=np.array(pedestal_cfg["local_orientation_wxyz"]),
    )
    xform.set_local_scale(np.array(pedestal_cfg["scale"]))


def build_teleop_target(cfg: dict, robot_prim_path: str, robot_cfg: dict) -> SingleXFormPrim:
    """Creates a draggable target the operator moves in the GUI to command
    the robot's end-effector pose via cuRobo -- a detached copy of the
    robot's own end-effector visual mesh, not a plain marker, so it shows
    exactly what will arrive at that pose.

    CopyPrim correctly preserves the instanceable mesh reference Isaac
    Sim's URDF importer uses for per-link visual geometry: confirmed live
    that the copy shares the same USD prototype as the original and
    renders with full geometry, even though a plain Usd.PrimRange
    traversal of either one shows zero children (instance-proxy content is
    hidden from traversal by default; Usd.TraverseInstanceProxies() is
    needed to see it -- rendering and set_world_pose()/get_world_pose()
    work regardless, without needing that special traversal at all).
    """
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
    """Scans a deliberately narrow set of prims for cuRobo collision
    obstacles -- just the ergo tables and the pedestal, not the whole
    factory backdrop. /World/Factory has thousands of small meshes
    (walkways, fences, racks, part racks); scanning all of that into a
    cuRobo WorldConfig on every refresh would be slow for no benefit, since
    nothing else in the backdrop is within the robot's actual reach.

    Derives the scan scope from ergo_tables/cr5_mount.pedestal's own
    config rather than a separately-maintained path list in
    teleop_target -- a single source of truth, so there's nothing to keep
    in sync if those prims are ever repositioned or renamed.
    """
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
    """Builds and warms up a cuRobo MotionGen for whichever robot is
    actually mounted at cr5_mount.

    Returns (motion_gen, robot_cfg) -- both None (printing why) if cuRobo
    isn't installed, since build_scene.py must keep working in the `base`
    profile, which has no cuRobo, so this step is best-effort rather than a
    hard dependency. `robot_cfg` is the robot's kinematics-schema dict
    (i.e. the *contents* of a robot yml's top-level `robot_cfg` key, not
    the file-shaped wrapper around it) -- returned so callers building the
    teleop target/loop (which need e.g. robot_cfg["kinematics"]["ee_link"]
    and ["cspace"]["joint_names"]) use the exact same robot config
    MotionGen was actually warmed up against, instead of re-resolving the
    robot_override branch a second time and risking the two drifting apart.

    Passes a real, populated world (get_teleop_obstacles's pedestal + ergo
    table scan) to MotionGenConfig.load_from_robot_config() up front rather
    than leaving world_model at its None default -- confirmed live that an
    empty/absent world leaves motion_gen.world_coll_checker as None
    (update_world() then fails with AttributeError), and separately that
    the MESH collision checker's warmup() itself fails ("Primitive
    Collision has no obstacles") if the *first* world it ever sees is
    empty. run_teleop_loop's periodic rescan calls update_world() again
    later with the same scoped scan to pick up any changes.
    """
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
    """Drag `target` in the GUI viewport; the robot follows via cuRobo's
    MotionGen. A from-scratch port of the debounce/plan/apply pattern in
    examples/curobo_reference/motion_gen_reacher.py's main loop (see that
    file -- not modified, just used as a reference), adapted to:
      - This repo's own SingleArticulation convention (see
        scripts/teach_waypoint.py) instead of the reference's
        omni.isaac.core.robots.Robot.
      - No isaacsim.core.api.World: `step_index` is a plain local counter
        (equivalent to World.current_time_step_index for this loop's
        purposes -- staged-startup gating and rescan cadence, neither of
        which cares where the count comes from), and
        omni.timeline.get_timeline_interface().is_playing() replaces
        World.is_playing() -- introducing World here would add machinery
        (stage units, default ground plane, physics-scene-creation timing)
        this file doesn't otherwise touch, for no behavioral gain.
      - robot_cfg-sourced joint names/retract pose (never hardcoded), so
        this works whether the CR5 or the Franka override is mounted.
      - No hardcoded set_max_efforts(5000, ...) -- that's a Franka-tuned
        guess in the reference example, not derived from anything. This
        repo already has a working, per-robot-correct mechanism for pose
        tracking (import_cr5.py's default_drive_strength/
        default_position_drive_damping).

    `max_iterations`: None for the real interactive case (runs until the
    Isaac Sim window closes); set to a finite number for headless
    scripted verification (see the module's own testing notes).
    """
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.state import JointState
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

    # import_cr5() imports with create_physics_scene=False (see its own
    # docstring -- it only authors joint/drive/collider schemas, not a
    # runtime physics scene), so nothing on this stage has created one.
    # SingleArticulation.initialize() needs an actual physics simulation
    # view, which PhysX only produces once a PhysicsScene prim exists --
    # confirmed live that without this, get_physics_sim_view() stays None
    # even after the timeline is playing, and .initialize() raises
    # AttributeError deep in isaacsim.core.prims. This one-line Define()
    # is the minimal fix, well short of introducing isaacsim.core.api.World.
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    tensor_args = TensorDeviceType()
    plan_config = MotionGenPlanConfig()
    timeline = omni.timeline.get_timeline_interface()

    # motion_gen's kinematics/IK/trajopt all operate in the robot's own
    # base-link frame, not USD world space -- confirmed live that passing
    # the target's raw world pose as the IK goal made every plan fail with
    # MotionGenStatus.IK_FAIL, since our robot is mounted away from the USD
    # world origin (cr5_mount.position/orientation_wxyz), unlike the
    # reference example's robot, which happens to sit at world origin so
    # world pose and base-frame pose are numerically identical there. The
    # mount pose is static once set by mount_cr5(), so compute it once here
    # rather than re-deriving it every frame.
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

    # robot.initialize() needs an actual PhysX simulation view, which only
    # exists once physics has actually stepped at least once -- confirmed
    # live that defining /physicsScene alone isn't enough:
    # SimulationManager.get_physics_sim_view() stays None until *after*
    # timeline.play() plus a few simulation_app.update() calls. Calling
    # initialize() unconditionally here (before the loop below ever checks
    # is_playing()) crashed with the same AttributeError this function's
    # own physics-scene fix was supposed to solve, because the timeline
    # isn't playing yet at that point -- the user hasn't clicked Play. So,
    # like the reference example (which only calls
    # robot._articulation_view.initialize() once my_world.is_playing()),
    # defer this until the loop below confirms physics is actually running.
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

        # step_index only advances on frames where physics is actually
        # stepping -- matching World.current_time_step_index in the
        # reference example, which likewise only ticks while playing. If
        # this counted every simulation_app.update() call regardless of
        # play state (as an earlier version of this function did), a user
        # who takes more than a few seconds to click Play would blow past
        # _TELEOP_INIT_FRAMES/_TELEOP_SETTLE_FRAMES before physics ever
        # started, skipping the settle phase entirely.
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
    """Deactivates the welding line's sliding rail and robot pedestals
    under /World/Factory, leaving every other prim (fences, feeders,
    process nodes, roof racks, robot controllers/arms, ErgoTable, etc.)
    untouched.

    Two matching modes, both against `factory` (configs/scene/table_layout.yaml):
      - `prune_name_startswith`: case-insensitive *prefix* (not substring)
        match against a prim's name, applied anywhere under /World/Factory
        -- e.g. "rail" matches `Rail`/`Rail_U20__U23_7` but not `Handrail`
        or `GuardRail`, since those don't start with it.
      - `prune_exact_paths`: exact full prim paths, for names too generic
        to safely prefix-match anywhere in the tree (e.g. "Link1", which
        also names our own CR5's first arm link).
    Verified against a live install: see CLAUDE.md.

    Deactivation (Prim.SetActive(False)), not deletion: reversible, and
    never touches the vendored Factory.usd file on disk.
    """
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


# TEMPORARY -- testing GitHub isaac-sim/IsaacSim#191 (drag-and-drop from the
# Content browser breaks after a URDF import, confirmed live: the corruption
# survives even a save+reopen-fresh cycle, so it's baked into the composed
# stage itself, not just runtime state). Set True to skip mount_cr5()/cuRobo
# setup/teleop-target entirely, so the rest of the scene (factory, ergo
# tables, assembly_parts) still builds and can be used to confirm drag-drop
# works fine when no URDF import has happened. Revert to False (or delete
# this flag and the guards using it) once that's confirmed either way --
# this is not meant to be a permanent mode.
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

        # motion_gen.warmup() blocks the main thread with real GPU work (kernel
        # compilation/loading, pre-tracing batched IK/trajopt solves) and calls
        # no simulation_app.update() of its own -- the viewport will go black
        # and look frozen for however long this takes (seconds to a couple
        # minutes depending on kernel caching). That's expected, not a hang.
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
