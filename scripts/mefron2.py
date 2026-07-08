"""cuRobo teleop for assets/mefron/factory floor/mefron.usd -- target-drag
only, no robot import.

Unlike mefron.py, this script does not import or mount any robot: the
Franka Panda is already saved directly into mefron.usd (added via Isaac
Sim's own GUI robot-asset import, not this repo's URDF-import pipeline --
confirmed live it composes via plain reference arcs straight into NVIDIA's
own bundled Franka Panda Isaac Asset, with no local sublayers and none of
the layered "Robot Description"/ancestral-prim issues mefron.py's own
docstring documents for a from-scratch URDF import). So this file only
needs to attach to what's already there and run cuRobo + the draggable
teleop target -- no import_cr5, no mount positioning, no gripper-friction/
drive-stiffness authoring.

Two things needed adapting from mefron.py, both confirmed live against the
actual saved file, not assumed:
  - The end-effector visuals path mefron.py hardcodes
    (f"{ROBOT_PRIM_PATH}/{ee_link}/visuals") does not exist on this robot
    asset -- its layout uses "<link>/geometry" (an instanceable prim), not
    "<link>/visuals". build_teleop_target() below references the ee_link
    prim itself instead.
  - MOUNT_POSITION/MOUNT_ORIENTATION_WXYZ used to be the pose mefron.py's
    mount_franka() actively *imposed* on the robot. With no mount step
    here, the robot's actual base pose is read directly off the stage once
    at startup (get_robot_base_pose()) instead of trusted as a hardcoded
    constant -- correct regardless of exactly where the robot was placed.

Everything else (obstacle scan, MotionGen setup, the drag-follow loop,
gripper keyboard control, grasp/assembly pose helpers) is ported verbatim
from mefron.py.

Only creates its own SimulationApp when run standalone.

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/mefron2.py [--headless]
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    experience = "" if _headless else f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    simulation_app = SimulationApp({"headless": _headless}, experience=experience)

# See mefron.py's own module comment for why this is needed under the full
# Kit experience: cuRobo's torch_utils does `from packaging import version`,
# which resolves to a broken internal pip-bootstrap bundle instead of the
# real site-packages install unless pre-loaded explicitly first.
import importlib.util  # noqa: E402

_REAL_PACKAGING_DIR = "/isaac-sim/kit/python/lib/python3.11/site-packages/packaging"


def _preload_real_submodule(pkg_module, name):
    spec = importlib.util.spec_from_file_location(f"packaging.{name}", f"{_REAL_PACKAGING_DIR}/{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[f"packaging.{name}"] = module
    spec.loader.exec_module(module)
    setattr(pkg_module, name, module)


if "packaging" not in sys.modules and os.path.isdir(_REAL_PACKAGING_DIR):
    _spec = importlib.util.spec_from_file_location(
        "packaging", f"{_REAL_PACKAGING_DIR}/__init__.py", submodule_search_locations=[_REAL_PACKAGING_DIR]
    )
    _packaging_module = importlib.util.module_from_spec(_spec)
    sys.modules["packaging"] = _packaging_module
    _spec.loader.exec_module(_packaging_module)
    _preload_real_submodule(_packaging_module, "version")

import carb.settings  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
MEFRON_USD = REPO_ROOT / "assets" / "mefron" / "factory floor" / "mefron.usd"

ROBOT_PRIM_PATH = "/World/Franka"
TARGET_PRIM_PATH = "/World/target"
MOUNT_PLATE_PRIM_PATH = "/World/sektion_cabinet_instanceable"

FRANKA_MOTION_GEN_ROBOT_CFG = "franka.yml"

# Nearby scene objects actually within the Franka's reach envelope -- not
# the whole /World/Factory backdrop.
OBSTACLE_PRIM_PATHS = [
    "/World/packing_table",
    "/World/packing_table_01",
    "/World/finger_print_scanner",
    "/World/main_holder",
    "/World/screen",
    "/World/backpanel_support",
    MOUNT_PLATE_PRIM_PATH,
]

# Loop-timing constants, ported verbatim from mefron.py's own run_teleop_loop.
_TELEOP_INIT_FRAMES = 10
_TELEOP_SETTLE_FRAMES = 20
_TELEOP_OBSTACLE_RESCAN_INTERVAL = 1000
_POSE_DELTA_THRESHOLD = 1.0e-3
_STATIC_JOINT_VELOCITY_THRESHOLD = 0.5
_ROBOT_INIT_SETTLE_FRAMES = 5
_TELEOP_TIME_DILATION_FACTOR = 0.3

# Gripper actuation constants -- confirmed via franka_panda.urdf's actual
# joint limits (panda_finger_joint1/2, prismatic, lower=0.0 upper=0.04).
# No friction-material/drive-stiffening constants here (unlike mefron.py):
# this robot asset's own defaults are used as-is, since that setup is
# import-time physics tuning, not teleop/target-drag logic.
GRIPPER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]
GRIPPER_OPEN_POSITION = 0.04
GRIPPER_CLOSED_POSITION = 0.0
# Load-bearing even without any friction-authoring code: this is
# compute_grasp_approach_pose()'s default argument below.
HIGH_FRICTION_PRIM_PATHS = ["/World/finger_print_scanner"]

# T_S_G / T_H_S -- ported verbatim from mefron.py, see that file's own
# comments for how these were derived (compute_relative_pose() on live
# world poses, not hand Euler-angle math).
GRASP_OFFSET_POSITION = [0.01277519, -0.02169126, -0.02863107]
GRASP_OFFSET_ORIENTATION_WXYZ = [-0.000518294608, -0.00348700255, 0.000751325308, 0.999993504]

ASSEMBLY_RELATIONSHIPS = {
    "finger_print_scanner_on_main_holder": {
        "part_prim_path": "/World/finger_print_scanner",
        "mount_prim_path": "/World/main_holder",
        "local_position": [-0.05765023, 0.02069006, 0.01875005],
        "local_orientation_wxyz": [0.999973595, -0.00618904850, 0.000842160478, -0.00371422408],
    }
}


def get_robot_base_pose():
    """Reads the already-mounted robot's real world pose directly off the
    stage, instead of trusting a hardcoded mount-position constant -- correct
    regardless of exactly where the robot was placed when it was saved into
    mefron.usd."""
    position, orientation_wxyz = SingleXFormPrim(prim_path=ROBOT_PRIM_PATH).get_world_pose()
    return np.array(position), np.array(orientation_wxyz)


def compute_grasp_approach_pose(part_prim_path: str = HIGH_FRICTION_PRIM_PATHS[0]):
    """Returns the world pose /World/target should be set to in order to grasp
    the named part (finger_print_scanner by default) at the fixed relative
    offset GRASP_OFFSET_*, wherever that part currently sits on the table.
    Table-position-independent by construction -- recomputes from the part's
    live pose every call, not a value baked in once."""
    part_trans, part_quat = SingleXFormPrim(prim_path=part_prim_path).get_world_pose()
    return compute_dependent_world_pose(part_trans, part_quat, GRASP_OFFSET_POSITION, GRASP_OFFSET_ORIENTATION_WXYZ)


def compute_assembly_grasp_target(relationship_name: str = "finger_print_scanner_on_main_holder"):
    """Returns the world pose /World/target should be set to in order to place
    the already-grasped part at its correctly assembled position on its mount,
    wherever the mount currently sits."""
    relationship = ASSEMBLY_RELATIONSHIPS[relationship_name]
    mount_trans, mount_quat = SingleXFormPrim(prim_path=relationship["mount_prim_path"]).get_world_pose()
    part_target_trans, part_target_quat = compute_dependent_world_pose(
        mount_trans, mount_quat, relationship["local_position"], relationship["local_orientation_wxyz"]
    )
    return compute_dependent_world_pose(
        part_target_trans, part_target_quat, GRASP_OFFSET_POSITION, GRASP_OFFSET_ORIENTATION_WXYZ
    )


class GripperKeyboardControl:
    """Open/closed request for the Franka's gripper, read once per teleop
    frame, plus two one-shot snap-to-pose requests (G: grasp approach, P:
    assembly placement). Ported verbatim from mefron.py."""

    def __init__(self) -> None:
        self.closed = False
        self._grasp_approach_requested = False
        self._assembly_target_requested = False

    def set_closed(self, closed: bool) -> None:
        self.closed = closed

    def request_grasp_approach(self) -> None:
        self._grasp_approach_requested = True

    def consume_grasp_approach_request(self) -> bool:
        requested = self._grasp_approach_requested
        self._grasp_approach_requested = False
        return requested

    def request_assembly_target(self) -> None:
        self._assembly_target_requested = True

    def consume_assembly_target_request(self) -> bool:
        requested = self._assembly_target_requested
        self._assembly_target_requested = False
        return requested


def build_gripper_keyboard_control() -> GripperKeyboardControl:
    """Subscribes to real keyboard events: C closes the gripper, O opens it,
    G snaps /World/target to the grasp-approach pose, P snaps it to the
    assembly-placement pose. Ported verbatim from mefron.py."""
    import carb.input
    import omni.appwindow

    control = GripperKeyboardControl()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == carb.input.KeyboardInput.C:
                control.set_closed(True)
            elif event.input == carb.input.KeyboardInput.O:
                control.set_closed(False)
            elif event.input == carb.input.KeyboardInput.G:
                control.request_grasp_approach()
            elif event.input == carb.input.KeyboardInput.P:
                control.request_assembly_target()
        return True

    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control


def get_obstacles():
    from curobo.util.usd_helper import UsdHelper

    usd_help = UsdHelper()
    usd_help.load_stage(omni.usd.get_context().get_stage())
    return usd_help.get_obstacles_from_stage(
        only_paths=list(OBSTACLE_PRIM_PATHS),
        reference_prim_path=ROBOT_PRIM_PATH,
        ignore_substring=[ROBOT_PRIM_PATH, TARGET_PRIM_PATH, "/curobo"],
    ).get_collision_check_world()


def setup_motion_gen():
    from curobo.types.base import TensorDeviceType
    from curobo.util_file import get_robot_configs_path, join_path, load_yaml
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

    robot_cfg = load_yaml(join_path(get_robot_configs_path(), FRANKA_MOTION_GEN_ROBOT_CFG))["robot_cfg"]
    world_cfg = get_obstacles()
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        {"robot_cfg": robot_cfg}, world_cfg, tensor_args=TensorDeviceType()
    )
    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup()
    return motion_gen, robot_cfg


def motion_gen_kinematics_get_state(robot_cfg, q):
    from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig

    kinematics_config = CudaRobotModelConfig.from_data_dict(robot_cfg["kinematics"])
    kinematics = CudaRobotModel(kinematics_config)
    return kinematics.get_state(q).ee_pose


def build_teleop_target(robot_cfg: dict, robot_base_position, robot_base_orientation_wxyz) -> SingleXFormPrim:
    """Creates a draggable target at the robot's own retract_config
    end-effector pose, as a real, freely-editable copy of the end-effector's
    visual mesh -- not a reference, and not the ee_link/geometry prim
    directly.

    Two real problems, both confirmed live and both fixed by *how* this
    copy is made:

    1. **Ancestral prim (can't be reparented in the Stage tree).**
       mefron.py's equivalent uses `AddInternalReference()` -- that produces
       a working, correctly-sized target, but `check_ancestral()` (confirmed
       against real Kit source) recurses into whatever a reference points
       at, and "{ee_link}/geometry" is itself a descendant of the referenced
       Franka subtree, so the reference target stays ancestral no matter
       what. A plain `CopyPrim` from "{ee_link}/geometry" itself doesn't
       work either -- confirmed live it produces an EMPTY bounding box,
       because "geometry" is only `instanceable=True` metadata pointing at
       an instance; CopyPrim's shallow copy carries the instanceable flag
       but not the composition arc needed to resolve it, leaving a hollow
       shell. The fix: resolve the actual instance-proxy Mesh prim
       underneath "geometry" first (via `Usd.TraverseInstanceProxies()`),
       then `CopyPrim` from THAT already-resolved path. Confirmed live this
       produces a correct non-empty bbox, `check_ancestral()==False`, and a
       real `MovePrim` reparent actually succeeds.
    2. **Unwanted collision.** NVIDIA's asset bakes collision
       (UsdPhysics.CollisionAPI + PhysxCollisionAPI + PhysicsMeshCollisionAPI,
       approximation=convexHull) onto the very same mesh prim as the
       visuals -- confirmed live via a real PhysX overlap query that a
       referenced copy is a genuine static collider, already overlapping the
       robot's own nearby arm links (the target starts right at the current
       end-effector pose). Since the CopyPrim-from-resolved-path result
       above is a real, non-instance-proxy prim (not read-only), the
       collision API can be removed directly with a plain `RemoveAPI` --
       confirmed live this works and leaves a correct, non-empty mesh with
       zero collision APIs.
    """
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose as CuroboPose

    ee_link = robot_cfg["kinematics"]["ee_link"]
    geometry_path = f"{ROBOT_PRIM_PATH}/{ee_link}/geometry"

    stage = omni.usd.get_context().get_stage()
    geometry_prim = stage.GetPrimAtPath(geometry_path)
    mesh_proxy_path = None
    for prim in Usd.PrimRange(geometry_prim, Usd.TraverseInstanceProxies()):
        if prim.GetTypeName() == "Mesh":
            mesh_proxy_path = prim.GetPath()
            break
    if mesh_proxy_path is None:
        raise RuntimeError(f"No Mesh prim found under {geometry_path} -- can't build the teleop target.")

    omni.kit.commands.execute("CopyPrim", path_from=str(mesh_proxy_path), path_to=TARGET_PRIM_PATH)
    target_prim = stage.GetPrimAtPath(TARGET_PRIM_PATH)
    if target_prim.HasAPI(UsdPhysics.CollisionAPI):
        target_prim.RemoveAPI(UsdPhysics.CollisionAPI)

    tensor_args = TensorDeviceType()
    retract_config = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])
    q = tensor_args.to_device(retract_config).unsqueeze(0)
    local_ee_pose = motion_gen_kinematics_get_state(robot_cfg, q)

    robot_base_pose = CuroboPose(
        position=tensor_args.to_device(robot_base_position),
        quaternion=tensor_args.to_device(robot_base_orientation_wxyz),
    )
    world_ee_pose = robot_base_pose.multiply(local_ee_pose)

    xform = SingleXFormPrim(prim_path=TARGET_PRIM_PATH)
    xform.set_world_pose(
        position=world_ee_pose.position.squeeze(0).cpu().numpy(),
        orientation=world_ee_pose.quaternion.squeeze(0).cpu().numpy(),
    )
    return xform


def compute_relative_pose(reference_trans, reference_quat, dependent_trans, dependent_quat):
    """Given two live world poses, returns the dependent object's pose
    expressed in the reference object's own frame. Ported verbatim from
    mefron.py."""
    from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats

    ref_rot, dep_rot = quats_to_rot_matrices(np.array([reference_quat, dependent_quat]))
    rel_rot = ref_rot.T @ dep_rot
    rel_trans = ref_rot.T @ (np.array(dependent_trans) - np.array(reference_trans))
    return rel_trans, rot_matrices_to_quats(np.array([rel_rot]))[0]


def compute_dependent_world_pose(reference_trans, reference_quat, relative_trans, relative_quat_wxyz):
    """Inverse of compute_relative_pose(). Ported verbatim from mefron.py."""
    from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats

    (ref_rot,) = quats_to_rot_matrices(np.array([reference_quat]))
    trans = ref_rot @ np.array(relative_trans) + np.array(reference_trans)
    rot = ref_rot @ quats_to_rot_matrices(np.array([relative_quat_wxyz]))[0]
    return trans, rot_matrices_to_quats(np.array([rot]))[0]


def run_teleop_loop(
    motion_gen,
    robot_cfg: dict,
    target: SingleXFormPrim,
    robot_base_position,
    robot_base_orientation_wxyz,
    max_iterations: int | None = None,
    gripper_control: GripperKeyboardControl | None = None,
) -> None:
    """Drag `target` in the GUI viewport; the robot follows via cuRobo's
    MotionGen. Ported verbatim from mefron.py's run_teleop_loop() (see that
    file for the full design history -- physics-scene dedup, Stop/Play
    articulation rebuild, interpolation_dt-gated playback, gripper
    actuation, G/P snap-to-pose requests) except robot_base_position/
    robot_base_orientation_wxyz are now passed in (read live from the stage
    by get_robot_base_pose(), see main()) instead of hardcoded constants.
    """
    import time

    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.state import JointState
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    tensor_args = TensorDeviceType()
    plan_config = MotionGenPlanConfig(time_dilation_factor=_TELEOP_TIME_DILATION_FACTOR)
    timeline = omni.timeline.get_timeline_interface()

    robot_base_pose = Pose(
        position=tensor_args.to_device(robot_base_position),
        quaternion=tensor_args.to_device(robot_base_orientation_wxyz),
    )

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])

    robot = None
    idx_list = None
    gripper_idx_list = None
    articulation_controller = None

    past_pose = None
    past_orientation = None
    target_pose = None
    target_orientation = None
    cmd_plan = None
    cmd_idx = 0
    last_cmd_time = None
    interpolation_dt = 0.02
    obstacles = None
    step_index = 0
    not_playing_frames = 0
    was_playing = False

    while simulation_app.is_running():
        simulation_app.update()

        if not timeline.is_playing():
            was_playing = False
            not_playing_frames += 1
            if not_playing_frames % 100 == 0:
                print("[mefron2] Click Play to start cuRobo teleop.", flush=True)
            continue

        if not was_playing:
            idx_list = None
            gripper_idx_list = None
            articulation_controller = None
            past_pose = None
            past_orientation = None
            target_pose = None
            target_orientation = None
            cmd_plan = None
            cmd_idx = 0
            last_cmd_time = None
            obstacles = None
            step_index = 0
            was_playing = True

        step_index += 1
        if max_iterations is not None and step_index > max_iterations:
            return

        if idx_list is None:
            if step_index < _ROBOT_INIT_SETTLE_FRAMES:
                continue
            robot = SingleArticulation(prim_path=ROBOT_PRIM_PATH, name="mefron2_teleop_robot")
            robot.initialize()
            idx_list = [robot.get_dof_index(x) for x in j_names]
            gripper_idx_list = [robot.get_dof_index(x) for x in GRIPPER_JOINT_NAMES]
            articulation_controller = robot.get_articulation_controller()

        if step_index < _TELEOP_INIT_FRAMES:
            robot.set_joint_positions(default_config, idx_list)
            continue
        if step_index < _TELEOP_SETTLE_FRAMES:
            continue

        if obstacles is None or step_index % _TELEOP_OBSTACLE_RESCAN_INTERVAL == 0:
            obstacles = get_obstacles()
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

        if gripper_control is not None:
            if gripper_control.consume_grasp_approach_request():
                cube_position, cube_orientation = compute_grasp_approach_pose()
                target.set_world_pose(position=cube_position, orientation=cube_orientation)
            elif gripper_control.consume_assembly_target_request():
                cube_position, cube_orientation = compute_assembly_grasp_target()
                target.set_world_pose(position=cube_position, orientation=cube_orientation)

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

        robot_static = bool(np.max(np.abs(sim_js.velocities)) < _STATIC_JOINT_VELOCITY_THRESHOLD)

        if (
            (
                np.linalg.norm(cube_position - target_pose) > _POSE_DELTA_THRESHOLD
                or np.linalg.norm(cube_orientation - target_orientation) > _POSE_DELTA_THRESHOLD
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
            print(f"[mefron2] teleop plan_single success={result.success.item()}", flush=True)
            if result.success.item():
                cmd_plan = motion_gen.get_full_js(result.get_interpolated_plan())
                cmd_plan = cmd_plan.get_ordered_joint_state(sim_js_names)
                cmd_idx = 0
                interpolation_dt = result.interpolation_dt
                last_cmd_time = None
            target_pose = cube_position
            target_orientation = cube_orientation

        past_pose = cube_position
        past_orientation = cube_orientation

        if cmd_plan is not None:
            now = time.time()
            if last_cmd_time is None or (now - last_cmd_time) >= interpolation_dt:
                cmd_state = cmd_plan[cmd_idx]
                art_action = ArticulationAction(
                    cmd_state.position.cpu().numpy(),
                    cmd_state.velocity.cpu().numpy(),
                    joint_indices=idx_list,
                )
                articulation_controller.apply_action(art_action)
                cmd_idx += 1
                last_cmd_time = now
                if cmd_idx >= len(cmd_plan.position):
                    cmd_idx = 0
                    cmd_plan = None

        if gripper_control is not None:
            gripper_target = GRIPPER_CLOSED_POSITION if gripper_control.closed else GRIPPER_OPEN_POSITION
            gripper_action = ArticulationAction(
                np.array([gripper_target, gripper_target]),
                joint_indices=gripper_idx_list,
            )
            articulation_controller.apply_action(gripper_action)


def main() -> None:
    # See mefron.py's own main() comment for why this is needed: pressing
    # Play only creates a real PhysX simulation view if this setting is on
    # (a real toggle in the Play button's own dropdown).
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    omni.usd.get_context().open_stage(str(MEFRON_USD))
    # mefron.usd's own content resolves asynchronously.
    for _ in range(120):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        raise RuntimeError(
            f"{ROBOT_PRIM_PATH} not found in {MEFRON_USD} -- this script does not import a robot, "
            "it expects one already saved into the file."
        )
    robot_base_position, robot_base_orientation_wxyz = get_robot_base_pose()

    for status_path in [ROBOT_PRIM_PATH, *OBSTACLE_PRIM_PATHS]:
        prim = stage.GetPrimAtPath(status_path)
        print(f"[mefron2] {status_path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

    print("[mefron2] warming up cuRobo motion_gen (viewport will look frozen/black until this finishes)...", flush=True)
    motion_gen, robot_cfg = setup_motion_gen()
    print("[mefron2] curobo motion_gen: READY", flush=True)
    # See mefron.py's own main() comment: motion_gen.warmup() is a ~30s
    # blocking call with no simulation_app.update() of its own, so physics
    # "playing" across that gap corrupts PhysX's simulation view. Force a
    # stop here; run_teleop_loop() already rebuilds everything on Play.
    omni.timeline.get_timeline_interface().stop()

    target = build_teleop_target(robot_cfg, robot_base_position, robot_base_orientation_wxyz)
    target_prim = stage.GetPrimAtPath(TARGET_PRIM_PATH)
    print(f"[mefron2] {TARGET_PRIM_PATH}: {'OK' if target_prim.IsValid() else 'MISSING'}", flush=True)

    if _headless:
        simulation_app.close()
        return

    gripper_control = build_gripper_keyboard_control()
    print("[mefron2] Gripper: press C to close, O to open. G: grasp approach, P: assembly placement.", flush=True)
    print("[mefron2] click Play in the GUI to start teleop.", flush=True)
    run_teleop_loop(
        motion_gen, robot_cfg, target, robot_base_position, robot_base_orientation_wxyz, gripper_control=gripper_control
    )
    simulation_app.close()


if __name__ == "__main__":
    main()
