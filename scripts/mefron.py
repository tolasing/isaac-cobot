"""Interactive cuRobo teleop + pick-and-place for the mefron scanner-assembly scene.
See docs/mefron-history.md for bug history and docs/grasp-and-assembly-offsets.md for how the grasp/assembly poses were derived.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # Use the full isaac-sim.sh experience (UI extensions like Physics debug viz) for
    # interactive runs; headless verification doesn't need it.
    experience = "" if _headless else f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    simulation_app = SimulationApp({"headless": _headless}, experience=experience)

# Pre-loads real `packaging`/`packaging.version` from site-packages before cuRobo imports them,
# since the full SimulationApp experience shadows packaging.version with a broken bundle (see docs/mefron-history.md).
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
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Sdf, UsdPhysics  # noqa: E402

from import_cr5 import import_cr5  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
MEFRON_USD = REPO_ROOT / "assets" / "mefron" / "factory floor" / "mefron.usd"
# Disk-persisted "Robot Description" dir the URDF importer writes on every import into
# this file-backed stage; cleared at the top of main() so a crash's stub files can't break the next import.
MEFRON_CONFIGURATION_DIR = MEFRON_USD.parent / "configuration"

ROBOT_PRIM_PATH = "/World/Franka"
TARGET_PRIM_PATH = "/World/target"
# SEKTION cabinet table the Franka mounts on (replaced the original Pedestal_plates/Cube_05 plate).
# No /Factory prefix: this script opens mefron.usd directly, one level shallower than build_scene_mefron.py's reference.
MOUNT_PLATE_PRIM_PATH = "/World/sektion_cabinet_instanceable"
MOUNT_POSITION = [2.74097, -4.782, 0.7924]
MOUNT_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

FRANKA_URDF_RELATIVE_PATH = "robot/franka_description/franka_panda.urdf"
FRANKA_DRIVE_STRENGTH = 1047.19751
FRANKA_DRIVE_DAMPING = 52.35988
FRANKA_MOTION_GEN_ROBOT_CFG = "franka.yml"

# Nearby scene objects within the Franka's reach envelope, not the whole /World/Factory backdrop
# (which would add scan time for no benefit).
OBSTACLE_PRIM_PATHS = [
    "/World/packing_table",
    "/World/packing_table_01",
    "/World/finger_print_scanner",
    "/World/main_holder",
    "/World/screen",
    "/World/backpanel_support",
    MOUNT_PLATE_PRIM_PATH,
]

# Loop-timing constants for run_teleop_loop(), ported from build_scene.py.
_TELEOP_INIT_FRAMES = 10
_TELEOP_SETTLE_FRAMES = 20
_TELEOP_OBSTACLE_RESCAN_INTERVAL = 1000
_POSE_DELTA_THRESHOLD = 1.0e-3
_STATIC_JOINT_VELOCITY_THRESHOLD = 0.5

# Frames to wait after is_playing() first turns True before constructing SingleArticulation --
# PhysX needs a few real steps before its simulation view is actually ready.
_ROBOT_INIT_SETTLE_FRAMES = 5

# Uniformly re-times the already-planned trajectory to play out slower; does not change the
# optimizer's relative speed profile or planning success. See _TELEOP_VELOCITY_SCALE for capping actual limits.
_TELEOP_TIME_DILATION_FACTOR = 0.3

# Caps velocity/acceleration limits used during trajectory optimization. cuRobo treats scale <= 0.25 as a
# special case: it swaps in finetune_trajopt_slow.yml and raises maximum_trajectory_dt to compensate; 0.2 stays under that threshold.
_TELEOP_VELOCITY_SCALE = 0.2
_TELEOP_ACCELERATION_SCALE = 0.2

# Grasp-physics constants, ported from build_scene_mefron.py's apply_gripper_friction()/stiffen_gripper_drive().
GRIPPER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]
GRIPPER_OPEN_POSITION = 0.04
GRIPPER_CLOSED_POSITION = 0.0
# Rate (m/s) the commanded gripper position is ramped toward open/closed, instead of stepping instantly --
# avoids a snap shut under the high drive stiffness. 0.02 m/s takes ~2s for the full 0.04m travel.
GRIPPER_CLOSE_SPEED = 0.02
GRIPPER_FRICTION_MATERIAL_PATH = "/World/GripperFrictionMaterial"
GRIPPER_STATIC_FRICTION = 0.9
GRIPPER_DYNAMIC_FRICTION = 0.8
GRIPPER_FINGER_LINK_NAMES = ["panda_leftfinger", "panda_rightfinger"]
GRIPPER_DRIVE_STIFFNESS = 10000.0
GRIPPER_DRIVE_DAMPING = 200.0
HIGH_FRICTION_PRIM_PATHS = ["/World/finger_print_scanner"]

# T_S_G: the Franka's ee_link pose expressed in finger_print_scanner's own local frame at a known-good
# grasp, derived via compute_relative_pose() against a manually-jogged /World/target (see docs/grasp-and-assembly-offsets.md).
GRASP_OFFSET_POSITION = [0.00027002069774515104, -0.021693730387954874, -0.1271989186209571]
GRASP_OFFSET_ORIENTATION_WXYZ = [
    -2.1523912431273915e-05,
    -8.089888886539503e-06,
    5.762411090611313e-06,
    0.9999999997190347,
]

# T_H_S: finger_print_scanner's pose expressed in main_holder's own local frame at the correctly
# assembled position, derived via compute_relative_pose() after temporarily reparenting in mefron.usd.
ASSEMBLY_RELATIONSHIPS = {
    "finger_print_scanner_on_main_holder": {
        "part_prim_path": "/World/finger_print_scanner",
        "mount_prim_path": "/World/main_holder",
        "local_position": [-0.05765001316747483, 0.02068996147910942, 0.01500000425999065],
        "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
}


def compute_grasp_approach_pose(part_prim_path: str = HIGH_FRICTION_PRIM_PATHS[0]):
    """Returns the world pose /World/target should be set to in order to grasp the named part
    (finger_print_scanner by default) at the fixed relative offset GRASP_OFFSET_*, recomputed from its live pose."""
    part_trans, part_quat = SingleXFormPrim(prim_path=part_prim_path).get_world_pose()
    return compute_dependent_world_pose(part_trans, part_quat, GRASP_OFFSET_POSITION, GRASP_OFFSET_ORIENTATION_WXYZ)


def compute_assembly_grasp_target(relationship_name: str = "finger_print_scanner_on_main_holder"):
    """Returns the world pose /World/target should be set to in order to place the already-grasped part at
    its assembled position, by composing the mount's live pose with ASSEMBLY_RELATIONSHIPS and GRASP_OFFSET_*."""
    relationship = ASSEMBLY_RELATIONSHIPS[relationship_name]
    mount_trans, mount_quat = SingleXFormPrim(prim_path=relationship["mount_prim_path"]).get_world_pose()
    part_target_trans, part_target_quat = compute_dependent_world_pose(
        mount_trans, mount_quat, relationship["local_position"], relationship["local_orientation_wxyz"]
    )
    return compute_dependent_world_pose(
        part_target_trans, part_target_quat, GRASP_OFFSET_POSITION, GRASP_OFFSET_ORIENTATION_WXYZ
    )


def mount_franka() -> None:
    from curobo.util_file import get_assets_path, join_path

    urdf_path = Path(join_path(get_assets_path(), FRANKA_URDF_RELATIVE_PATH))
    import_cr5(
        urdf_path=urdf_path,
        prim_path=ROBOT_PRIM_PATH,
        default_drive_strength=FRANKA_DRIVE_STRENGTH,
        default_position_drive_damping=FRANKA_DRIVE_DAMPING,
    )
    xform = SingleXFormPrim(prim_path=ROBOT_PRIM_PATH)
    xform.set_world_pose(
        position=np.array(MOUNT_POSITION),
        orientation=np.array(MOUNT_ORIENTATION_WXYZ),
    )


def apply_gripper_friction() -> None:
    """Authors one high-friction physics material and binds it to the Franka's fingertip links and
    HIGH_FRICTION_PRIM_PATHS. Runtime-only (never persisted via stage.Save()); re-authored fresh every run."""
    from omni.physx.scripts import utils as physx_utils
    from omni.physx.scripts.physicsUtils import add_physics_material_to_prim

    stage = omni.usd.get_context().get_stage()
    physx_utils.addRigidBodyMaterial(
        stage,
        GRIPPER_FRICTION_MATERIAL_PATH,
        staticFriction=GRIPPER_STATIC_FRICTION,
        dynamicFriction=GRIPPER_DYNAMIC_FRICTION,
        restitution=0.0,
    )

    target_paths = [f"{ROBOT_PRIM_PATH}/{name}" for name in GRIPPER_FINGER_LINK_NAMES]
    target_paths += HIGH_FRICTION_PRIM_PATHS
    for prim_path in target_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            print(f"[mefron] WARNING: {prim_path} not found -- skipping friction bind.", flush=True)
            continue
        add_physics_material_to_prim(stage, prim, GRIPPER_FRICTION_MATERIAL_PATH)


def stiffen_gripper_drive() -> None:
    """Raises the finger joints' position-drive stiffness/damping above the whole-robot import-time
    default, so their maxForce budget isn't left mostly unused."""
    stage = omni.usd.get_context().get_stage()
    for joint_name in GRIPPER_JOINT_NAMES:
        joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/joints/{joint_name}")
        if not joint_prim.IsValid():
            print(f"[mefron] WARNING: {joint_prim.GetPath()} not found -- skipping stiffen.", flush=True)
            continue
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "linear")
        drive.CreateStiffnessAttr().Set(GRIPPER_DRIVE_STIFFNESS)
        drive.CreateDampingAttr().Set(GRIPPER_DRIVE_DAMPING)


class GripperKeyboardControl:
    """Open/closed request for the Franka's gripper, read once per teleop frame, plus two one-shot
    snap-to-pose requests (G: grasp approach, P: assembly placement) consumed exactly once via request_*/consume_*."""

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
    """Subscribes to keyboard events: C closes the gripper, O opens it, G snaps /World/target to the
    grasp-approach pose, P snaps it to the assembly-placement pose."""
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

    # Kept alive on the control object so the subscription isn't garbage-collected.
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
    # A real, populated world must be passed at construction time, or update_world()/warmup() later fail.
    world_cfg = get_obstacles()
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        {"robot_cfg": robot_cfg},
        world_cfg,
        tensor_args=TensorDeviceType(),
        velocity_scale=_TELEOP_VELOCITY_SCALE,
        acceleration_scale=_TELEOP_ACCELERATION_SCALE,
    )
    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup()
    return motion_gen, robot_cfg


def build_teleop_target(robot_cfg: dict) -> SingleXFormPrim:
    """Creates a draggable target at the robot's retract_config end-effector pose (guaranteed reachable),
    displaying an internally-referenced (not CopyPrim'd) live view of the real end-effector mesh."""
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose as CuroboPose

    ee_link = robot_cfg["kinematics"]["ee_link"]
    source_path = f"{ROBOT_PRIM_PATH}/{ee_link}/visuals"

    stage = omni.usd.get_context().get_stage()
    target_prim = stage.DefinePrim(TARGET_PRIM_PATH, "Xform")
    target_prim.GetReferences().AddInternalReference(Sdf.Path(source_path))

    tensor_args = TensorDeviceType()
    retract_config = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])
    q = tensor_args.to_device(retract_config).unsqueeze(0)
    local_ee_pose = motion_gen_kinematics_get_state(robot_cfg, q)

    robot_base_pose = CuroboPose(
        position=tensor_args.to_device(np.array(MOUNT_POSITION)),
        quaternion=tensor_args.to_device(np.array(MOUNT_ORIENTATION_WXYZ)),
    )
    world_ee_pose = robot_base_pose.multiply(local_ee_pose)

    xform = SingleXFormPrim(prim_path=TARGET_PRIM_PATH)
    xform.set_world_pose(
        position=world_ee_pose.position.squeeze(0).cpu().numpy(),
        orientation=world_ee_pose.quaternion.squeeze(0).cpu().numpy(),
    )
    return xform


def motion_gen_kinematics_get_state(robot_cfg, q):
    # Deferred import + tiny standalone CudaRobotModel, so build_teleop_target()
    # doesn't need a live MotionGen passed in just for forward kinematics.
    from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig

    kinematics_config = CudaRobotModelConfig.from_data_dict(robot_cfg["kinematics"])
    kinematics = CudaRobotModel(kinematics_config)
    return kinematics.get_state(q).ee_pose


def compute_relative_pose(reference_trans, reference_quat, dependent_trans, dependent_quat):
    """Given two live world poses, returns the dependent object's pose expressed in the reference
    object's own frame -- the derivation direction, opposite of compute_dependent_world_pose()'s consumption direction."""
    from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats

    ref_rot, dep_rot = quats_to_rot_matrices(np.array([reference_quat, dependent_quat]))
    rel_rot = ref_rot.T @ dep_rot
    rel_trans = ref_rot.T @ (np.array(dependent_trans) - np.array(reference_trans))
    return rel_trans, rot_matrices_to_quats(np.array([rel_rot]))[0]


def compute_dependent_world_pose(reference_trans, reference_quat, relative_trans, relative_quat_wxyz):
    """Inverse of compute_relative_pose(): given a live reference world pose and a fixed relative
    offset in its frame, returns the dependent object's resulting world pose."""
    from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats

    (ref_rot,) = quats_to_rot_matrices(np.array([reference_quat]))
    trans = ref_rot @ np.array(relative_trans) + np.array(reference_trans)
    rot = ref_rot @ quats_to_rot_matrices(np.array([relative_quat_wxyz]))[0]
    return trans, rot_matrices_to_quats(np.array([rot]))[0]


def run_teleop_loop(
    motion_gen,
    robot_cfg: dict,
    target: SingleXFormPrim,
    max_iterations: int | None = None,
    gripper_control: GripperKeyboardControl | None = None,
) -> None:
    """Drag `target` in the GUI viewport; the robot follows via cuRobo's MotionGen plan/apply loop, rebuilding
    the articulation on every fresh Play and supporting gripper open/close plus G/P grasp/assembly pose snaps."""
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
        position=tensor_args.to_device(np.array(MOUNT_POSITION)),
        quaternion=tensor_args.to_device(np.array(MOUNT_ORIENTATION_WXYZ)),
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
    # Real elapsed time since the last waypoint was applied, and the plan's intended per-waypoint duration.
    last_cmd_time = None
    interpolation_dt = 0.02
    obstacles = None
    step_index = 0
    not_playing_frames = 0
    was_playing = False
    # Ramped gripper setpoint state -- see GRIPPER_CLOSE_SPEED for why it moves gradually.
    gripper_setpoint = None
    last_gripper_time = None

    while simulation_app.is_running():
        simulation_app.update()

        if not timeline.is_playing():
            was_playing = False
            not_playing_frames += 1
            if not_playing_frames % 100 == 0:
                print("[mefron] Click Play to start cuRobo teleop.", flush=True)
            continue

        if not was_playing:
            # Fresh Play (first ever, or after a Stop) -- rebuild everything bound to the previous physics view.
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
            gripper_setpoint = None
            last_gripper_time = None
            was_playing = True

        step_index += 1
        if max_iterations is not None and step_index > max_iterations:
            return

        if idx_list is None:
            if step_index < _ROBOT_INIT_SETTLE_FRAMES:
                continue
            robot = SingleArticulation(prim_path=ROBOT_PRIM_PATH, name="mefron_teleop_robot")
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

        # One-shot G/P snap requests. Must run AFTER the past_pose/target_pose bootstrap above, not before --
        # otherwise cube_position would already reflect the post-snap pose when target_pose is seeded, making the debounce distance 0 forever.
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
            print(f"[mefron] teleop plan_single success={result.success.item()}", flush=True)
            if result.success.item():
                cmd_plan = motion_gen.get_full_js(result.get_interpolated_plan())
                cmd_plan = cmd_plan.get_ordered_joint_state(sim_js_names)
                cmd_idx = 0
                # This specific plan's intended per-waypoint duration (MotionGenResult-level, not MotionGen-level).
                interpolation_dt = result.interpolation_dt
                last_cmd_time = None
            target_pose = cube_position
            target_orientation = cube_orientation

        past_pose = cube_position
        past_orientation = cube_orientation

        if cmd_plan is not None:
            # Gate on real elapsed time, not frame count.
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

        # Independent of cmd_plan/cuRobo -- applied every frame so it always wins the finger indices'
        # drive-target write, even though get_full_js() re-applies lock_joints on every planned frame too.
        if gripper_control is not None:
            gripper_target = GRIPPER_CLOSED_POSITION if gripper_control.closed else GRIPPER_OPEN_POSITION
            if gripper_setpoint is None:
                gripper_setpoint = gripper_target
            now = time.time()
            if last_gripper_time is not None:
                max_step = GRIPPER_CLOSE_SPEED * (now - last_gripper_time)
                if gripper_setpoint < gripper_target:
                    gripper_setpoint = min(gripper_setpoint + max_step, gripper_target)
                elif gripper_setpoint > gripper_target:
                    gripper_setpoint = max(gripper_setpoint - max_step, gripper_target)
            last_gripper_time = now
            gripper_action = ArticulationAction(
                np.array([gripper_setpoint, gripper_setpoint]),
                joint_indices=gripper_idx_list,
            )
            articulation_controller.apply_action(gripper_action)


def clear_stale_robot_configuration() -> None:
    """Deletes any pre-existing files under MEFRON_CONFIGURATION_DIR before the URDF importer writes
    fresh ones, so a past crash's truncated stub files can't break the next import."""
    if not MEFRON_CONFIGURATION_DIR.is_dir():
        return
    for stale_file in MEFRON_CONFIGURATION_DIR.glob("*.usd"):
        stale_file.unlink()
        print(f"[mefron] cleared stale robot-description file: {stale_file}", flush=True)


def main() -> None:
    # Ensures Play actually creates a PhysX simulation view -- otherwise this is a GUI toggle that's
    # easy to have off, in which case is_playing() lies and SingleArticulation.initialize() never gets a real view.
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    # Must run BEFORE open_stage(): mefron.usd has a persisted, broken /panda prim reference, and
    # resolving it against stale files caches an Sdf.Layer that later crashes mount_franka()'s import ("a layer already exists").
    clear_stale_robot_configuration()

    omni.usd.get_context().open_stage(str(MEFRON_USD))
    # mefron.usd's own content resolves asynchronously, same reasoning as
    # build_scene.py's own post-build_factory() frame pump.
    for _ in range(120):
        simulation_app.update()

    mount_franka()
    apply_gripper_friction()
    stiffen_gripper_drive()

    stage = omni.usd.get_context().get_stage()
    for status_path in [ROBOT_PRIM_PATH, *OBSTACLE_PRIM_PATHS]:
        prim = stage.GetPrimAtPath(status_path)
        print(f"[mefron] {status_path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

    print("[mefron] warming up cuRobo motion_gen (viewport will look frozen/black until this finishes)...", flush=True)
    motion_gen, robot_cfg = setup_motion_gen()
    print("[mefron] curobo motion_gen: READY", flush=True)
    # Force a stop unconditionally: if physics was left playing across warmup()'s ~30s unpumped gap,
    # PhysX's simulation view gets corrupted; the loop rebuilds cleanly on the next fresh Play regardless.
    omni.timeline.get_timeline_interface().stop()

    target = build_teleop_target(robot_cfg)
    target_prim = stage.GetPrimAtPath(TARGET_PRIM_PATH)
    print(f"[mefron] {TARGET_PRIM_PATH}: {'OK' if target_prim.IsValid() else 'MISSING'}", flush=True)

    if _headless:
        simulation_app.close()
        return

    gripper_control = build_gripper_keyboard_control()
    print("[mefron] Gripper: press C to close, O to open.", flush=True)
    print("[mefron] click Play in the GUI to start teleop.", flush=True)
    run_teleop_loop(motion_gen, robot_cfg, target, gripper_control=gripper_control)
    simulation_app.close()


if __name__ == "__main__":
    main()
