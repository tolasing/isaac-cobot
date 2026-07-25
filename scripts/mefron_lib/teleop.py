"""Interactive cuRobo teleop loop: builds each arm's draggable target, warms up its MotionGen, and
runs the drag-follow plan/apply loop with gripper open/close, arm 1's J/B/K assembly/grasp-editor
pose snaps (one key per config.GRASP_TARGETS entry), and P -- assembly placement, wired to whichever
arms have an assembly_control in their arm dict (arm 1 via GripperKeyboardControl, arm 2 via
AssemblyPlacementControl -- see build_assembly_placement_keyboard_control()). run_teleop_loop() takes
a list of per-arm dicts so multiple robots can each drag-follow their own target off one shared
timeline/simulation_app.update() tick -- see its own docstring for the required dict shape. See
docs/mefron-history.md for the Stop/Play-rebuild and physics-timing gotchas this loop works around.
"""

from __future__ import annotations

import numpy as np
import omni.timeline
import omni.usd
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.types import ArticulationAction
from pxr import Sdf, UsdPhysics

from . import config
from .grasp import (
    compute_assembly_grasp_target,
    compute_grasp_approach_pose_from_file,
    compute_grasp_finger_widths_from_file,
    compute_part_target_pose,
)


class GripperKeyboardControl:
    """Open/closed request for the Franka's gripper, read once per teleop frame, plus two one-shot
    snap-to-pose requests (P: assembly placement, J/B/...: grasp-editor-yaml grasp approach for
    whichever config.GRASP_TARGETS object the key maps to) consumed exactly once via
    request_*/consume_*. open_position/closed_position start at the global config defaults and are
    overwritten by set_grasp_widths() once a grasp-approach request has been consumed, so C/O ramp
    toward whichever object was last selected instead of one fixed global width."""

    def __init__(self) -> None:
        self.closed = False
        self.open_position = config.GRIPPER_OPEN_POSITION
        self.closed_position = config.GRIPPER_CLOSED_POSITION
        self._assembly_target_requested = False
        self._grasp_approach_object_requested: str | None = None
        # Last config.GRASP_TARGETS key whose grasp-approach request was made -- lets P look up the
        # matching config.ASSEMBLY_RELATIONSHIPS entry instead of a single hardcoded object.
        self.last_grasped_object: str | None = None

    def set_closed(self, closed: bool) -> None:
        self.closed = closed

    def set_grasp_widths(self, open_position: float, closed_position: float) -> None:
        self.open_position = open_position
        self.closed_position = closed_position

    def request_assembly_target(self) -> None:
        self._assembly_target_requested = True

    def has_pending_assembly_target_request(self) -> bool:
        """Peek without consuming -- lets the teleop loop hold a P request open across frames
        until the robot goes idle, instead of consuming it (and snapping /World/target) while a
        plan is still in flight, where the snap would be silently discarded."""
        return self._assembly_target_requested

    def consume_assembly_target_request(self) -> bool:
        requested = self._assembly_target_requested
        self._assembly_target_requested = False
        return requested

    def request_grasp_approach_from_file(self, object_name: str) -> None:
        self._grasp_approach_object_requested = object_name
        self.last_grasped_object = object_name

    def consume_grasp_approach_from_file_request(self) -> str | None:
        requested = self._grasp_approach_object_requested
        self._grasp_approach_object_requested = None
        return requested

    def reset(self) -> None:
        """Called on every fresh Play (see run_teleop_loop()), same reasoning/pattern as
        ConveyorControl.reset(): this object is built once in mefron.py's setup, outside the
        per-Play arm["_state"] rebuild, so closed/last_grasped_object would otherwise silently
        survive a Stop even though the actual gripper and part have both reset to their pre-Play
        state. Without this, P can fire arm 1's placement snap on a fresh Play using a stale
        last_grasped_object from a previous session (see _step_arm()'s P-handling gate, which also
        now checks `closed` for the same reason within a single session -- e.g. grasp then O without
        ever placing)."""
        self.closed = False
        self.last_grasped_object = None
        self._assembly_target_requested = False
        self._grasp_approach_object_requested = None


def build_gripper_keyboard_control(
    close_key: str = "C",
    open_key: str = "O",
    grasp_key_bindings: dict[str, str] | None = None,
) -> GripperKeyboardControl:
    """Subscribes to keyboard events: close_key closes the gripper, open_key opens it. When
    grasp_key_bindings is left at its default (None), it's built from config.GRASP_TARGETS (J for
    finger_print_scanner, B for backpanel_support, ...) and P becomes active,
    snapping /World/target to the assembly-placement pose for whichever object was last grasped.
    Pass grasp_key_bindings={} for an arm with no grasp/assembly task wired up yet (e.g. arm 2) --
    that arm's P key (and any grasp keys) then does nothing, since a second concurrent keyboard
    subscription for a different arm would otherwise also see the same keypress."""
    import carb.input
    import omni.appwindow

    control = GripperKeyboardControl()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()

    if grasp_key_bindings is None:
        grasp_key_bindings = {
            getattr(carb.input.KeyboardInput, target["key"]): object_name
            for object_name, target in config.GRASP_TARGETS.items()
        }
        supports_assembly_snap = True
    else:
        grasp_key_bindings = {
            getattr(carb.input.KeyboardInput, key): object_name for key, object_name in grasp_key_bindings.items()
        }
        supports_assembly_snap = bool(grasp_key_bindings)

    close_input = getattr(carb.input.KeyboardInput, close_key)
    open_input = getattr(carb.input.KeyboardInput, open_key)

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == close_input:
                control.set_closed(True)
            elif event.input == open_input:
                control.set_closed(False)
            elif supports_assembly_snap and event.input == carb.input.KeyboardInput.P:
                control.request_assembly_target()
            elif event.input in grasp_key_bindings:
                control.request_grasp_approach_from_file(grasp_key_bindings[event.input])
        return True

    # Kept alive on the control object so the subscription isn't garbage-collected.
    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control


class SuctionApproachControl:
    """One-shot 'snap arm 2's target to whichever object's suction-approach pose' request (N: screen,
    M: pcb_assembly -- one key per config.SUCTION_TARGETS entry, same shape as GripperKeyboardControl's
    J/B grasp-approach keys), independent of GripperKeyboardControl -- arm 2's gripper_control must stay
    None (its parallel-jaw finger joints are deactivated, so a real GripperKeyboardControl would try to
    resolve config.GRIPPER_JOINT_NAMES via get_dof_index() and hit the unresolved-joint-index
    RuntimeError in _step_arm()'s init block). No has_pending/peek pair needed, unlike P: arm 2 has no
    carried-object two-stage-lift concern, so an immediate one-shot consume (same shape as J/B's
    grasp-approach request) is enough."""

    def __init__(self) -> None:
        self._requested_object: str | None = None
        # Last config.SUCTION_TARGETS key an approach request was made for -- lets P look up the
        # matching assembly_relationship instead of a single hardcoded object. Mirrors
        # GripperKeyboardControl.last_grasped_object.
        self.last_approached_object: str | None = None

    def request_approach(self, object_name: str) -> None:
        self._requested_object = object_name
        self.last_approached_object = object_name

    def consume_approach_request(self) -> str | None:
        requested = self._requested_object
        self._requested_object = None
        return requested

    def reset(self) -> None:
        """Called on every fresh Play (see run_teleop_loop()), same reasoning as
        GripperKeyboardControl.reset(): last_approached_object is built once outside the per-Play
        arm["_state"] rebuild, so it would otherwise silently survive a Stop and let a fresh Play's P
        route to whatever was last approached in a previous session, with nothing actually attached."""
        self._requested_object = None
        self.last_approached_object = None


def build_suction_approach_keyboard_control(
    key_bindings: dict[str, str] | None = None,
) -> SuctionApproachControl:
    """key_bindings defaults to config.SUCTION_TARGETS (N for screen, M for pcb_assembly)."""
    import carb.input
    import omni.appwindow

    if key_bindings is None:
        key_bindings = {target["key"]: object_name for object_name, target in config.SUCTION_TARGETS.items()}

    control = SuctionApproachControl()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()
    request_bindings = {
        getattr(carb.input.KeyboardInput, key): object_name for key, object_name in key_bindings.items()
    }

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input in request_bindings:
            control.request_approach(request_bindings[event.input])
        return True

    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control


class AssemblyPlacementControl:
    """One-shot 'place the carried part at its assembly pose' request for an arm with no
    GripperKeyboardControl of its own (arm 2 -- see that class's own docstring for why a real one
    would crash it). Same has_pending/consume peek pair as GripperKeyboardControl's P handling
    (not SuctionApproachControl's immediate-consume shape): the carried part needs the same
    two-stage lift-then-drop as arm 1's P, so the request must survive across frames until the robot
    goes idle, not be consumed while a plan is still in flight."""

    def __init__(self) -> None:
        self._requested = False

    def request_placement(self) -> None:
        self._requested = True

    def has_pending_placement_request(self) -> bool:
        return self._requested

    def consume_placement_request(self) -> bool:
        requested = self._requested
        self._requested = False
        return requested


def build_assembly_placement_keyboard_control(key: str = "P") -> AssemblyPlacementControl:
    """A second, independent keyboard subscription for P -- arm 1's GripperKeyboardControl already
    owns its own P subscription (see build_gripper_keyboard_control()'s docstring: concurrent
    subscriptions each see the same keypress), so one P press fires both arms' handlers."""
    import carb.input
    import omni.appwindow

    control = AssemblyPlacementControl()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()
    request_input = getattr(carb.input.KeyboardInput, key)

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input == request_input:
            control.request_placement()
        return True

    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control


class SurfaceGripperKeyboardControl:
    """Fires close_gripper()/open_gripper() once per keypress via the real Surface Gripper runtime
    interface (isaacsim.robot.surface_gripper). No per-frame state machine needed here -- unlike
    GripperKeyboardControl's ramped open/close, the extension's own SurfaceGripperManager (C++) owns
    the Open/Closing/Closed/retry state machine; Python only needs to request a transition once.
    is_closed() reads that same manager-owned state back (via get_gripper_status(), not anything
    tracked locally), for callers that need to know whether something is actually gripped right now
    -- see _step_arm()'s assembly_control gating for why that distinction matters."""

    def __init__(self, gripper_prim_path: str) -> None:
        import isaacsim.robot.surface_gripper._surface_gripper as surface_gripper

        self.gripper_prim_path = gripper_prim_path
        self._interface = surface_gripper.acquire_surface_gripper_interface()

    def close(self) -> None:
        self._interface.close_gripper(self.gripper_prim_path)

    def open(self) -> None:
        self._interface.open_gripper(self.gripper_prim_path)

    def is_closed(self) -> bool:
        import isaacsim.robot.surface_gripper._surface_gripper as surface_gripper

        status = self._interface.get_gripper_status(self.gripper_prim_path)
        return surface_gripper.GripperStatus(status) == surface_gripper.GripperStatus.Closed


def build_surface_gripper_keyboard_control(
    gripper_prim_path: str,
    close_key: str = config.SUCTION_ATTACH_KEY,
    open_key: str = config.SUCTION_DETACH_KEY,
) -> SurfaceGripperKeyboardControl:
    import carb.input
    import omni.appwindow

    control = SurfaceGripperKeyboardControl(gripper_prim_path)
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()
    close_input = getattr(carb.input.KeyboardInput, close_key)
    open_input = getattr(carb.input.KeyboardInput, open_key)

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == close_input:
                control.close()
            elif event.input == open_input:
                control.open()
        return True

    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control


def get_obstacles(robot_prim_path: str = config.ROBOT_PRIM_PATH, target_prim_path: str = config.TARGET_PRIM_PATH):
    from curobo.util.usd_helper import UsdHelper

    usd_help = UsdHelper()
    usd_help.load_stage(omni.usd.get_context().get_stage())
    return usd_help.get_obstacles_from_stage(
        only_paths=list(config.OBSTACLE_PRIM_PATHS),
        reference_prim_path=robot_prim_path,
        ignore_substring=[robot_prim_path, target_prim_path, "/curobo"],
    ).get_collision_check_world()


def _robot_cfg_without_gripper_joints(robot_cfg: dict) -> dict:
    """Returns a copy of robot_cfg with config.GRIPPER_JOINT_NAMES stripped out of cspace.joint_names
    and its parallel per-joint lists (retract_config, null_space_weight, cspace_distance_weight) --
    for an arm whose parallel-jaw gripper joints don't physically exist on the live USD stage (see
    robot.remove_parallel_jaw_gripper()). Without this, _step_arm()'s state["idx_list"] -- built by
    calling get_dof_index() on every name in this same cspace.joint_names -- KeyErrors on the first
    frame, since those DOF names no longer resolve on that arm's (fingerless) SingleArticulation.
    lock_joints/collision_spheres/mesh_link_names are left untouched: those describe cuRobo's own
    internal analytical kinematic/collision model (built from the URDF, independent of the live
    stage), not the live articulation's actual DOF set, so they don't need to match."""
    import copy

    robot_cfg = copy.deepcopy(robot_cfg)
    cspace = robot_cfg["kinematics"]["cspace"]
    keep_idx = [i for i, name in enumerate(cspace["joint_names"]) if name not in config.GRIPPER_JOINT_NAMES]
    for key in ("joint_names", "retract_config", "null_space_weight", "cspace_distance_weight"):
        cspace[key] = [cspace[key][i] for i in keep_idx]
    return robot_cfg


def setup_motion_gen(
    robot_prim_path: str = config.ROBOT_PRIM_PATH,
    target_prim_path: str = config.TARGET_PRIM_PATH,
    has_parallel_jaw_gripper: bool = True,
):
    from curobo.types.base import TensorDeviceType
    from curobo.util_file import get_robot_configs_path, join_path, load_yaml
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

    robot_cfg = load_yaml(join_path(get_robot_configs_path(), config.FRANKA_MOTION_GEN_ROBOT_CFG))["robot_cfg"]
    if not has_parallel_jaw_gripper:
        robot_cfg = _robot_cfg_without_gripper_joints(robot_cfg)
    # A real, populated world must be passed at construction time, or update_world()/warmup() later fail.
    world_cfg = get_obstacles(robot_prim_path, target_prim_path)
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        {"robot_cfg": robot_cfg},
        world_cfg,
        tensor_args=TensorDeviceType(),
        velocity_scale=config._TELEOP_VELOCITY_SCALE,
        acceleration_scale=config._TELEOP_ACCELERATION_SCALE,
    )
    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup()
    return motion_gen, robot_cfg


def motion_gen_kinematics_get_state(robot_cfg, q):
    # Deferred import + tiny standalone CudaRobotModel, so build_teleop_target()
    # doesn't need a live MotionGen passed in just for forward kinematics.
    from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig

    kinematics_config = CudaRobotModelConfig.from_data_dict(robot_cfg["kinematics"])
    kinematics = CudaRobotModel(kinematics_config)
    return kinematics.get_state(q).ee_pose


def build_teleop_target(
    robot_cfg: dict,
    robot_prim_path: str = config.ROBOT_PRIM_PATH,
    target_prim_path: str = config.TARGET_PRIM_PATH,
    mount_position=config.MOUNT_POSITION,
    mount_orientation_wxyz=config.MOUNT_ORIENTATION_WXYZ,
) -> SingleXFormPrim:
    """Creates a draggable target at the robot's retract_config end-effector pose (guaranteed reachable),
    displaying an internally-referenced (not CopyPrim'd) live view of the real end-effector mesh."""
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose as CuroboPose

    ee_link = robot_cfg["kinematics"]["ee_link"]
    source_path = f"{robot_prim_path}/{ee_link}/visuals"

    stage = omni.usd.get_context().get_stage()
    target_prim = stage.DefinePrim(target_prim_path, "Xform")
    target_prim.GetReferences().AddInternalReference(Sdf.Path(source_path))

    tensor_args = TensorDeviceType()
    retract_config = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])
    q = tensor_args.to_device(retract_config).unsqueeze(0)
    local_ee_pose = motion_gen_kinematics_get_state(robot_cfg, q)

    robot_base_pose = CuroboPose(
        position=tensor_args.to_device(np.array(mount_position)),
        quaternion=tensor_args.to_device(np.array(mount_orientation_wxyz)),
    )
    world_ee_pose = robot_base_pose.multiply(local_ee_pose)

    xform = SingleXFormPrim(prim_path=target_prim_path)
    xform.set_world_pose(
        position=world_ee_pose.position.squeeze(0).cpu().numpy(),
        orientation=world_ee_pose.quaternion.squeeze(0).cpu().numpy(),
    )
    return xform


def _fresh_arm_state() -> dict:
    """Per-arm state rebuilt on every fresh Play (first ever, or after a Stop) -- everything here is
    bound to the physics view that existed when it was built, same as run_teleop_loop's own original
    single-arm locals."""
    return {
        "robot": None,
        "idx_list": None,
        "gripper_idx_list": None,
        "articulation_controller": None,
        "past_pose": None,
        "past_orientation": None,
        "target_pose": None,
        "target_orientation": None,
        "cmd_plan": None,
        "cmd_idx": 0,
        # Real elapsed time since the last waypoint was applied, and the plan's intended per-waypoint duration.
        "last_cmd_time": None,
        "interpolation_dt": 0.02,
        # Set by the P handler to the real assembly-placement pose while `target` is snapped to an
        # intermediate straight-up lift waypoint first; applied once that lift plan finishes executing.
        "pending_final_pose": None,
        "obstacles": None,
        # Ramped gripper setpoint state -- see config.GRIPPER_CLOSE_SPEED for why it moves gradually.
        "gripper_setpoint": None,
        "last_gripper_time": None,
    }


def _snap_target_to_assembly_lift_waypoint(state: dict, target, ee_link_prim_path: str, relationship_name: str):
    """Shared by every arm's P handling (GripperKeyboardControl for arm 1, AssemblyPlacementControl
    for arm 2): computes the carried part's final assembly-placement pose via
    compute_assembly_grasp_target(), stages it in state["pending_final_pose"], and snaps `target` to
    an intermediate waypoint first -- final X/Y and final orientation, but held at the constant
    ASSEMBLY_LIFT_HEIGHT -- so the only motion left once that lift plan finishes is a straight drop
    in Z. A direct plan to the final pose was dragging/clipping the carried object through the table
    and nearby props. Returns the new (cube_position, cube_orientation) for the caller's locals."""
    final_position, final_orientation = compute_assembly_grasp_target(ee_link_prim_path, relationship_name)
    state["pending_final_pose"] = (final_position, final_orientation)
    cube_position = np.array([final_position[0], final_position[1], config.ASSEMBLY_LIFT_HEIGHT])
    cube_orientation = final_orientation
    target.set_world_pose(position=cube_position, orientation=cube_orientation)
    return cube_position, cube_orientation


def _step_arm(arm: dict, step_index: int, tensor_args) -> None:
    """One frame's worth of drag-follow-plan/apply + gripper-apply logic for a single arm, mutating
    arm["_state"] in place. Split out of run_teleop_loop() so multiple arms can each run their own
    copy of this per tick, off the one shared timeline/simulation_app.update() loop."""
    import time

    from curobo.types.math import Pose
    from curobo.types.state import JointState

    state = arm["_state"]
    motion_gen = arm["motion_gen"]
    robot_cfg = arm["robot_cfg"]
    target = arm["target"]
    gripper_control = arm.get("gripper_control")
    robot_prim_path = arm["robot_prim_path"]
    target_prim_path = arm["target_prim_path"]
    robot_base_pose = arm["_robot_base_pose"]
    j_names = arm["_j_names"]
    default_config = arm["_default_config"]
    ee_link_prim_path = arm["_ee_link_prim_path"]
    plan_config = arm["_plan_config"]

    if state["idx_list"] is None:
        if step_index < config._ROBOT_INIT_SETTLE_FRAMES:
            return
        state["robot"] = SingleArticulation(prim_path=robot_prim_path, name=f"mefron_teleop_robot_{arm['_name']}")
        state["robot"].initialize()
        state["idx_list"] = [state["robot"].get_dof_index(x) for x in j_names]
        if gripper_control is not None:
            state["gripper_idx_list"] = [state["robot"].get_dof_index(x) for x in config.GRIPPER_JOINT_NAMES]
        else:
            state["gripper_idx_list"] = []
        state["articulation_controller"] = state["robot"].get_articulation_controller()
        # get_dof_index() returning None for a joint name it can't resolve is exactly the kind of
        # thing that, left unchecked, feeds a bad index into apply_action()'s native PhysX tensor
        # call below -- which can crash the whole process rather than raise a catchable exception.
        # Fail loudly here instead.
        if any(i is None for i in state["idx_list"]) or any(i is None for i in state["gripper_idx_list"]):
            raise RuntimeError(
                f"[mefron_lib] {arm['_name']}: get_dof_index() could not resolve one or more joints "
                f"(idx_list={state['idx_list']}, gripper_idx_list={state['gripper_idx_list']}) -- "
                "refusing to drive this arm with an unresolved joint index."
            )

    if step_index < config._TELEOP_INIT_FRAMES:
        state["robot"].set_joint_positions(default_config, state["idx_list"])
        return
    if step_index < config._TELEOP_SETTLE_FRAMES:
        return

    if state["obstacles"] is None or step_index % config._TELEOP_OBSTACLE_RESCAN_INTERVAL == 0:
        state["obstacles"] = get_obstacles(robot_prim_path, target_prim_path)
        motion_gen.update_world(state["obstacles"])

    cube_position, cube_orientation = target.get_world_pose()
    if state["past_pose"] is None:
        state["past_pose"] = cube_position
    if state["target_pose"] is None:
        state["target_pose"] = cube_position
    if state["target_orientation"] is None:
        state["target_orientation"] = cube_orientation
    if state["past_orientation"] is None:
        state["past_orientation"] = cube_orientation

    # One-shot P/J snap requests. Must run AFTER the past_pose/target_pose bootstrap above, not before --
    # otherwise cube_position would already reflect the post-snap pose when target_pose is seeded, making the debounce distance 0 forever.
    if gripper_control is not None:
        if gripper_control.has_pending_assembly_target_request():
            # Only actually kick off the align/drop sequence once the robot is idle. Consuming
            # the request and snapping `target` while cmd_plan is still in flight (e.g. P
            # pressed a beat before the previous drag/grasp motion finished settling) would be
            # silently discarded -- the trigger-if below requires cmd_plan is None to plan
            # anything -- and the CURRENT in-flight plan's completion would then wrongly consume
            # pending_final_pose, sending the robot straight from wherever it was to the final
            # assembly pose with no hover stop at all.
            if gripper_control.last_grasped_object is None:
                # Nothing grasped yet this session -- discard the stale request instead of
                # defaulting to finger_print_scanner sight-unseen. Arm 1 sharing the P key with arm
                # 2 (see AssemblyPlacementControl) means every P press reaches here even when the
                # user only meant to place arm 2's screen; without this guard arm 1 would swing
                # toward finger_print_scanner's mount pose on every such press, both moving an arm
                # the user didn't intend to move and cluttering arm 2's obstacle-avoided workspace
                # right as it plans its own placement.
                gripper_control.consume_assembly_target_request()
            elif state["cmd_plan"] is None:
                gripper_control.consume_assembly_target_request()
                # Same "actually holding it right now" gate as arm 2's surface_gripper_control.is_closed()
                # check below (see a19b672) -- last_grasped_object is sticky (set once by J/B/K, never
                # cleared by O), so without this a press of O then P still fires the placement snap for
                # whatever was last grasped, and a stale last_grasped_object surviving a Stop/Play cycle
                # (this control object isn't rebuilt by _fresh_arm_state(), see reset()) does the same on
                # a fresh Play with nothing actually held.
                if gripper_control.closed:
                    object_name = gripper_control.last_grasped_object
                    # Looked up by part_prim_path rather than assuming a "{object_name}_on_main_holder"
                    # key -- not every object mounts onto main_holder (e.g. pcb_assembly_on_backpanel_support).
                    part_prim_path = config.GRASP_TARGETS[object_name]["part_prim_path"]
                    relationship_name = next(
                        name
                        for name, relationship in config.ASSEMBLY_RELATIONSHIPS.items()
                        if relationship["part_prim_path"] == part_prim_path
                    )
                    cube_position, cube_orientation = _snap_target_to_assembly_lift_waypoint(
                        state, target, ee_link_prim_path, relationship_name
                    )
        else:
            requested_object = gripper_control.consume_grasp_approach_from_file_request()
            if requested_object is not None:
                grasp_target = config.GRASP_TARGETS[requested_object]
                cube_position, cube_orientation = compute_grasp_approach_pose_from_file(
                    grasp_target["yaml_path"],
                    grasp_target["grasp_name"],
                    part_prim_path=grasp_target["part_prim_path"],
                )
                target.set_world_pose(position=cube_position, orientation=cube_orientation)
                open_position, closed_position = compute_grasp_finger_widths_from_file(
                    grasp_target["yaml_path"], grasp_target["grasp_name"]
                )
                gripper_control.set_grasp_widths(open_position, closed_position)
                gripper_control.set_closed(False)

    # One-shot suction-approach snap (arm 2 only) -- one key per config.SUCTION_TARGETS entry (N:
    # screen, M: pcb_assembly). Same "must run after the past_pose/target_pose bootstrap" ordering
    # rule as the gripper_control block above. Ungated (no cmd_plan is None check, unlike P) --
    # matches J/B's shape instead: arm 2 has no carried-object two-stage-lift concern, and the
    # debounce/plan-trigger logic below re-checks state["past_pose"] against the fresh cube_position
    # on the next frame regardless of which branch wrote it, so an immediate consume here is safe.
    suction_control = arm.get("suction_control")
    if suction_control is not None:
        requested_object = suction_control.consume_approach_request()
        if requested_object is not None:
            approach_relationship = config.SUCTION_TARGETS[requested_object]["approach_relationship"]
            cube_position, cube_orientation = compute_part_target_pose(approach_relationship)
            target.set_world_pose(position=cube_position, orientation=cube_orientation)

    # One-shot assembly-placement snap for arms with no GripperKeyboardControl of their own (arm 2's
    # AssemblyPlacementControl -- bound to the same P key as arm 1's gripper_control above via a
    # second, independent keyboard subscription, so one P press can drive both arms). Same
    # idle-gating as arm 1's P and the same reason: peek, don't consume, until state["cmd_plan"] is
    # None, or an in-flight plan's completion would wrongly consume pending_final_pose.
    assembly_control = arm.get("assembly_control")
    if (
        assembly_control is not None
        and assembly_control.has_pending_placement_request()
        and state["cmd_plan"] is None
    ):
        assembly_control.consume_placement_request()
        # Looked up from suction_control.last_approached_object instead of a fixed relationship
        # string -- mirrors arm 1's GRASP_TARGETS[last_grasped_object] lookup, since arm 2 can now
        # place either screen or pcb_assembly depending on which was last approached (N vs M).
        object_name = suction_control.last_approached_object if suction_control is not None else None
        surface_gripper_control = arm.get("surface_gripper_control")
        # Same "discard the stale request" shape as arm 1's last_grasped_object is None branch
        # above: without this, arm 2 would swing toward whichever object happens to be looked up
        # even when nothing was ever approached this session (N/M), or after an attach/release with
        # nothing actually held.
        if object_name is not None and (surface_gripper_control is None or surface_gripper_control.is_closed()):
            relationship_name = config.SUCTION_TARGETS[object_name]["assembly_relationship"]
            cube_position, cube_orientation = _snap_target_to_assembly_lift_waypoint(
                state, target, ee_link_prim_path, relationship_name
            )

    sim_js = state["robot"].get_joints_state()
    if sim_js is None:
        return
    sim_js_names = state["robot"].dof_names
    cu_js = JointState(
        position=tensor_args.to_device(sim_js.positions),
        velocity=tensor_args.to_device(sim_js.velocities) * 0.0,
        acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
        jerk=tensor_args.to_device(sim_js.velocities) * 0.0,
        joint_names=sim_js_names,
    )
    cu_js = cu_js.get_ordered_joint_state(motion_gen.kinematics.joint_names)

    robot_static = bool(np.max(np.abs(sim_js.velocities)) < config._STATIC_JOINT_VELOCITY_THRESHOLD)

    if (
        (
            np.linalg.norm(cube_position - state["target_pose"]) > config._POSE_DELTA_THRESHOLD
            or np.linalg.norm(cube_orientation - state["target_orientation"]) > config._POSE_DELTA_THRESHOLD
        )
        and np.linalg.norm(state["past_pose"] - cube_position) == 0.0
        and np.linalg.norm(state["past_orientation"] - cube_orientation) == 0.0
        and robot_static
        and state["cmd_plan"] is None
    ):
        world_target_pose = Pose(
            position=tensor_args.to_device(cube_position),
            quaternion=tensor_args.to_device(cube_orientation),
        )
        ik_goal = robot_base_pose.compute_local_pose(world_target_pose)
        result = motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, plan_config)
        print(f"[mefron] {arm['_name']} teleop plan_single success={result.success.item()}", flush=True)
        if result.success.item():
            cmd_plan = motion_gen.get_full_js(result.get_interpolated_plan())
            state["cmd_plan"] = cmd_plan.get_ordered_joint_state(sim_js_names)
            state["cmd_idx"] = 0
            # This specific plan's intended per-waypoint duration (MotionGenResult-level, not MotionGen-level).
            state["interpolation_dt"] = result.interpolation_dt
            state["last_cmd_time"] = None
        state["target_pose"] = cube_position
        state["target_orientation"] = cube_orientation

    state["past_pose"] = cube_position
    state["past_orientation"] = cube_orientation

    if state["cmd_plan"] is not None:
        # Gate on real elapsed time, not frame count.
        now = time.time()
        if state["last_cmd_time"] is None or (now - state["last_cmd_time"]) >= state["interpolation_dt"]:
            cmd_state = state["cmd_plan"][state["cmd_idx"]]
            art_action = ArticulationAction(
                cmd_state.position.cpu().numpy(),
                cmd_state.velocity.cpu().numpy(),
                joint_indices=state["idx_list"],
            )
            state["articulation_controller"].apply_action(art_action)
            state["cmd_idx"] += 1
            state["last_cmd_time"] = now
            if state["cmd_idx"] >= len(state["cmd_plan"].position):
                state["cmd_idx"] = 0
                state["cmd_plan"] = None
                if state["pending_final_pose"] is not None:
                    final_position, final_orientation = state["pending_final_pose"]
                    state["pending_final_pose"] = None
                    target.set_world_pose(position=final_position, orientation=final_orientation)

    # Independent of cmd_plan/cuRobo -- applied every frame so it always wins the finger indices'
    # drive-target write, even though get_full_js() re-applies lock_joints on every planned frame too.
    if gripper_control is not None:
        gripper_target = gripper_control.closed_position if gripper_control.closed else gripper_control.open_position
        if state["gripper_setpoint"] is None:
            state["gripper_setpoint"] = gripper_target
        now = time.time()
        if state["last_gripper_time"] is not None:
            max_step = config.GRIPPER_CLOSE_SPEED * (now - state["last_gripper_time"])
            if state["gripper_setpoint"] < gripper_target:
                state["gripper_setpoint"] = min(state["gripper_setpoint"] + max_step, gripper_target)
            elif state["gripper_setpoint"] > gripper_target:
                state["gripper_setpoint"] = max(state["gripper_setpoint"] - max_step, gripper_target)
        state["last_gripper_time"] = now
        gripper_action = ArticulationAction(
            np.array([state["gripper_setpoint"], state["gripper_setpoint"]]),
            joint_indices=state["gripper_idx_list"],
        )
        state["articulation_controller"].apply_action(gripper_action)


def run_teleop_loop(
    simulation_app,
    arms: list[dict],
    max_iterations: int | None = None,
    # Duck-typed (not conveyor.ConveyorControl) -- this loop only ever calls .reset()/.step() on
    # whatever's passed in, deliberately staying decoupled from conveyor.py.
    conveyor_control: object | None = None,
) -> None:
    """Drag each arm's own `target` in the GUI viewport; each robot follows via its own cuRobo
    MotionGen plan/apply loop, rebuilding every arm's articulation on every fresh Play and supporting
    gripper open/close (plus, for whichever arm's gripper_control has grasp key bindings wired up,
    P/J/B/... grasp/assembly pose snaps). `arms` is a list of dicts, one per robot, each requiring:
    "motion_gen", "robot_cfg", "target", "robot_prim_path", "target_prim_path", "mount_position",
    "mount_orientation_wxyz"; "gripper_control" and "name" are optional (name defaults to
    robot_prim_path and must be unique across arms -- it becomes each arm's SingleArticulation view
    name, and Isaac Sim's core registry requires that to be unique per articulation). conveyor_control
    is stepped once per playing frame alongside the arms, independent of any of them."""
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    tensor_args = TensorDeviceType()
    timeline = omni.timeline.get_timeline_interface()

    # Freeze each arm's static, per-arm-but-not-per-frame data once up front.
    for arm in arms:
        arm["_name"] = arm.get("name") or arm["robot_prim_path"]
        arm["_robot_base_pose"] = Pose(
            position=tensor_args.to_device(np.array(arm["mount_position"])),
            quaternion=tensor_args.to_device(np.array(arm["mount_orientation_wxyz"])),
        )
        arm["_j_names"] = arm["robot_cfg"]["kinematics"]["cspace"]["joint_names"]
        arm["_default_config"] = np.array(arm["robot_cfg"]["kinematics"]["cspace"]["retract_config"])
        arm["_ee_link_prim_path"] = f"{arm['robot_prim_path']}/{arm['robot_cfg']['kinematics']['ee_link']}"
        arm["_plan_config"] = MotionGenPlanConfig(time_dilation_factor=config._TELEOP_TIME_DILATION_FACTOR)
        arm["_state"] = _fresh_arm_state()

    step_index = 0
    not_playing_frames = 0
    was_playing = False

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
            for arm in arms:
                arm["_state"] = _fresh_arm_state()
                gripper_control = arm.get("gripper_control")
                if gripper_control is not None:
                    gripper_control.reset()
                suction_control = arm.get("suction_control")
                if suction_control is not None:
                    suction_control.reset()
            if conveyor_control is not None:
                conveyor_control.reset()
            step_index = 0
            was_playing = True

        step_index += 1
        if max_iterations is not None and step_index > max_iterations:
            return

        for arm in arms:
            _step_arm(arm, step_index, tensor_args)

        if conveyor_control is not None:
            conveyor_control.step()
