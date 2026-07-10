"""Interactive cuRobo teleop loop: builds the draggable target, warms up MotionGen, and runs the
drag-follow plan/apply loop with gripper open/close and G/P/J grasp/assembly/grasp-editor pose snaps. See
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
    compute_assembly_placement_error,
    compute_grasp_approach_pose,
    compute_grasp_approach_pose_from_file,
    compute_reactive_assembly_target,
)


class GripperKeyboardControl:
    """Open/closed request for the Franka's gripper, read once per teleop frame, plus four one-shot
    snap-to-pose requests (G: grasp approach, P: assembly placement, J: grasp-editor-yaml grasp approach,
    for live comparison against G, M: reactive MPC-tracked assembly placement) consumed exactly once via
    request_*/consume_*."""

    def __init__(self) -> None:
        self.closed = False
        self._grasp_approach_requested = False
        self._assembly_target_requested = False
        self._grasp_approach_from_file_requested = False
        self._reactive_assembly_requested = False

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

    def request_grasp_approach_from_file(self) -> None:
        self._grasp_approach_from_file_requested = True

    def consume_grasp_approach_from_file_request(self) -> bool:
        requested = self._grasp_approach_from_file_requested
        self._grasp_approach_from_file_requested = False
        return requested

    def request_reactive_assembly(self) -> None:
        self._reactive_assembly_requested = True

    def consume_reactive_assembly_request(self) -> bool:
        requested = self._reactive_assembly_requested
        self._reactive_assembly_requested = False
        return requested


def build_gripper_keyboard_control() -> GripperKeyboardControl:
    """Subscribes to keyboard events: C closes the gripper, O opens it, G snaps /World/target to the
    constants-based grasp-approach pose, P snaps it to the assembly-placement pose, J snaps it to the
    Grasp Editor-exported yaml's grasp-approach pose (config.GRASP_EDITOR_YAML_PATH/GRASP_NAME), M
    triggers the discrete-approach-then-MPC-reactive-tracking two-phase assembly placement."""
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
            elif event.input == carb.input.KeyboardInput.J:
                control.request_grasp_approach_from_file()
            elif event.input == carb.input.KeyboardInput.M:
                control.request_reactive_assembly()
        return True

    # Kept alive on the control object so the subscription isn't garbage-collected.
    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control


def get_obstacles(exclude_paths=()):
    """exclude_paths: additional prim paths to drop beyond the always-excluded robot/target/curobo --
    e.g. mpc.setup_mpc_solver() and the mpc_solver.update_world() rescan pass config._MPC_COLLISION_EXCLUDE_PATHS
    so MPC gets its own, smaller collision world than motion_gen's."""
    from curobo.util.usd_helper import UsdHelper

    usd_help = UsdHelper()
    usd_help.load_stage(omni.usd.get_context().get_stage())
    return usd_help.get_obstacles_from_stage(
        only_paths=list(config.OBSTACLE_PRIM_PATHS),
        reference_prim_path=config.ROBOT_PRIM_PATH,
        ignore_substring=[config.ROBOT_PRIM_PATH, config.TARGET_PRIM_PATH, "/curobo", *exclude_paths],
    ).get_collision_check_world()


def setup_motion_gen():
    from curobo.types.base import TensorDeviceType
    from curobo.util_file import get_robot_configs_path, join_path, load_yaml
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

    robot_cfg = load_yaml(join_path(get_robot_configs_path(), config.FRANKA_MOTION_GEN_ROBOT_CFG))["robot_cfg"]
    # A real, populated world must be passed at construction time, or update_world()/warmup() later fail.
    world_cfg = get_obstacles()
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


def build_teleop_target(robot_cfg: dict) -> SingleXFormPrim:
    """Creates a draggable target at the robot's retract_config end-effector pose (guaranteed reachable),
    displaying an internally-referenced (not CopyPrim'd) live view of the real end-effector mesh."""
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose as CuroboPose

    ee_link = robot_cfg["kinematics"]["ee_link"]
    source_path = f"{config.ROBOT_PRIM_PATH}/{ee_link}/visuals"

    stage = omni.usd.get_context().get_stage()
    target_prim = stage.DefinePrim(config.TARGET_PRIM_PATH, "Xform")
    target_prim.GetReferences().AddInternalReference(Sdf.Path(source_path))

    tensor_args = TensorDeviceType()
    retract_config = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])
    q = tensor_args.to_device(retract_config).unsqueeze(0)
    local_ee_pose = motion_gen_kinematics_get_state(robot_cfg, q)

    robot_base_pose = CuroboPose(
        position=tensor_args.to_device(np.array(config.MOUNT_POSITION)),
        quaternion=tensor_args.to_device(np.array(config.MOUNT_ORIENTATION_WXYZ)),
    )
    world_ee_pose = robot_base_pose.multiply(local_ee_pose)

    xform = SingleXFormPrim(prim_path=config.TARGET_PRIM_PATH)
    xform.set_world_pose(
        position=world_ee_pose.position.squeeze(0).cpu().numpy(),
        orientation=world_ee_pose.quaternion.squeeze(0).cpu().numpy(),
    )
    return xform


def run_teleop_loop(
    simulation_app,
    motion_gen,
    robot_cfg: dict,
    target: SingleXFormPrim,
    max_iterations: int | None = None,
    gripper_control: GripperKeyboardControl | None = None,
    mpc_solver=None,
) -> None:
    """Drag `target` in the GUI viewport; the robot follows via cuRobo's MotionGen plan/apply loop, rebuilding
    the articulation on every fresh Play and supporting gripper open/close plus G/P/J grasp/assembly pose snaps.
    If mpc_solver is provided, M triggers a two-phase placement: a discrete MotionGen approach to the nominal
    assembly pose, then a handoff to mpc_solver for continuous slip-corrected tracking via
    grasp.compute_reactive_assembly_target() (see cuRobo's own MpcSolver docstring for the global/local split)."""
    import time

    from curobo.rollout.rollout_base import Goal
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.state import JointState
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    tensor_args = TensorDeviceType()
    plan_config = MotionGenPlanConfig(time_dilation_factor=config._TELEOP_TIME_DILATION_FACTOR)
    timeline = omni.timeline.get_timeline_interface()

    robot_base_pose = Pose(
        position=tensor_args.to_device(np.array(config.MOUNT_POSITION)),
        quaternion=tensor_args.to_device(np.array(config.MOUNT_ORIENTATION_WXYZ)),
    )

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])
    ee_link_prim_path = f"{config.ROBOT_PRIM_PATH}/{robot_cfg['kinematics']['ee_link']}"

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
    # Ramped gripper setpoint state -- see config.GRIPPER_CLOSE_SPEED for why it moves gradually.
    gripper_setpoint = None
    last_gripper_time = None
    # M-key reactive-placement state: pending_mpc_handoff bridges the discrete MotionGen approach leg
    # (still driven by the existing cmd_plan machinery below) to the MPC local-tracking leg.
    mpc_active = False
    pending_mpc_handoff = False
    mpc_goal_buffer = None
    mpc_step_count = 0
    mpc_converged_count = 0
    last_mpc_time = None
    # Best (minimum) part-placement error seen this MPC run -- diagnostic only, so a timeout print
    # shows how close it actually got, not just wherever it ended up after the fact.
    mpc_best_position_error = None
    mpc_best_rotation_error = None

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
            mpc_active = False
            pending_mpc_handoff = False
            mpc_goal_buffer = None
            mpc_step_count = 0
            mpc_converged_count = 0
            last_mpc_time = None
            if mpc_solver is not None:
                mpc_solver.reset()
            was_playing = True

        step_index += 1
        if max_iterations is not None and step_index > max_iterations:
            return

        if idx_list is None:
            if step_index < config._ROBOT_INIT_SETTLE_FRAMES:
                continue
            robot = SingleArticulation(prim_path=config.ROBOT_PRIM_PATH, name="mefron_teleop_robot")
            robot.initialize()
            idx_list = [robot.get_dof_index(x) for x in j_names]
            gripper_idx_list = [robot.get_dof_index(x) for x in config.GRIPPER_JOINT_NAMES]
            articulation_controller = robot.get_articulation_controller()

        if step_index < config._TELEOP_INIT_FRAMES:
            robot.set_joint_positions(default_config, idx_list)
            continue
        if step_index < config._TELEOP_SETTLE_FRAMES:
            continue

        if obstacles is None or step_index % config._TELEOP_OBSTACLE_RESCAN_INTERVAL == 0:
            obstacles = get_obstacles()
            motion_gen.update_world(obstacles)
            if mpc_solver is not None:
                mpc_solver.update_world(get_obstacles(exclude_paths=config._MPC_COLLISION_EXCLUDE_PATHS))

        cube_position, cube_orientation = target.get_world_pose()
        if past_pose is None:
            past_pose = cube_position
        if target_pose is None:
            target_pose = cube_position
        if target_orientation is None:
            target_orientation = cube_orientation
        if past_orientation is None:
            past_orientation = cube_orientation

        # One-shot G/P/J snap requests. Must run AFTER the past_pose/target_pose bootstrap above, not before --
        # otherwise cube_position would already reflect the post-snap pose when target_pose is seeded, making the debounce distance 0 forever.
        if gripper_control is not None:
            if gripper_control.consume_grasp_approach_request():
                cube_position, cube_orientation = compute_grasp_approach_pose()
                target.set_world_pose(position=cube_position, orientation=cube_orientation)
            elif gripper_control.consume_assembly_target_request():
                cube_position, cube_orientation = compute_assembly_grasp_target(ee_link_prim_path)
                target.set_world_pose(position=cube_position, orientation=cube_orientation)
            elif gripper_control.consume_grasp_approach_from_file_request():
                cube_position, cube_orientation = compute_grasp_approach_pose_from_file(
                    config.GRASP_EDITOR_YAML_PATH, config.GRASP_EDITOR_GRASP_NAME
                )
                target.set_world_pose(position=cube_position, orientation=cube_orientation)
            elif gripper_control.consume_reactive_assembly_request():
                if not mpc_active:
                    cube_position, cube_orientation = compute_assembly_grasp_target(ee_link_prim_path)
                    target.set_world_pose(position=cube_position, orientation=cube_orientation)
                    if mpc_solver is not None:
                        pending_mpc_handoff = True
                    else:
                        print(
                            "[mefron] WARNING: reactive assembly requested but no mpc_solver was provided.",
                            flush=True,
                        )

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

        robot_static = bool(np.max(np.abs(sim_js.velocities)) < config._STATIC_JOINT_VELOCITY_THRESHOLD)

        if (
            (
                np.linalg.norm(cube_position - target_pose) > config._POSE_DELTA_THRESHOLD
                or np.linalg.norm(cube_orientation - target_orientation) > config._POSE_DELTA_THRESHOLD
            )
            and np.linalg.norm(past_pose - cube_position) == 0.0
            and np.linalg.norm(past_orientation - cube_orientation) == 0.0
            and robot_static
            and cmd_plan is None
            and not mpc_active
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

        if mpc_active:
            # Gate on real elapsed time, mirroring the cmd_plan branch's own interpolation_dt gate below.
            now = time.time()
            if last_mpc_time is None or (now - last_mpc_time) >= config._MPC_STEP_DT:
                try:
                    gripper_target_trans, gripper_target_quat = compute_reactive_assembly_target(ee_link_prim_path)
                    world_target_pose = Pose(
                        position=tensor_args.to_device(gripper_target_trans),
                        quaternion=tensor_args.to_device(gripper_target_quat),
                    )
                    local_target_pose = robot_base_pose.compute_local_pose(world_target_pose)

                    # Separate sim_js_mpc/cu_js_mpc from the sim_js/cu_js already read above -- those keep
                    # feeding the (still-evaluated-every-frame) discrete-trigger condition, and MPC needs the
                    # robot's real current velocity to track well, not the zeroed-velocity cu_js used there.
                    sim_js_mpc = robot.get_joints_state()
                    cu_js_mpc = JointState(
                        position=tensor_args.to_device(sim_js_mpc.positions),
                        velocity=tensor_args.to_device(sim_js_mpc.velocities),
                        acceleration=tensor_args.to_device(sim_js_mpc.velocities) * 0.0,
                        jerk=tensor_args.to_device(sim_js_mpc.velocities) * 0.0,
                        joint_names=robot.dof_names,
                    ).unsqueeze(0)
                    cu_js_mpc = mpc_solver.get_active_js(cu_js_mpc)

                    if mpc_goal_buffer is None:
                        mpc_goal_buffer = mpc_solver.setup_solve_single(
                            Goal(current_state=cu_js_mpc, goal_pose=local_target_pose), num_seeds=1
                        )
                    else:
                        mpc_goal_buffer.goal_pose.copy_(local_target_pose)
                        mpc_solver.update_goal(mpc_goal_buffer)

                    mpc_result = mpc_solver.step(cu_js_mpc, max_attempts=config._MPC_STEP_MAX_ATTEMPTS)
                    mpc_js_action = mpc_result.js_action.get_ordered_joint_state(sim_js_names)
                    articulation_controller.apply_action(
                        ArticulationAction(
                            mpc_js_action.position.squeeze(0).cpu().numpy(),
                            mpc_js_action.velocity.squeeze(0).cpu().numpy(),
                            joint_indices=idx_list,
                        )
                    )
                    last_mpc_time = now
                    mpc_step_count += 1

                    # What actually matters is whether the PART is correctly placed on its mount, not
                    # whether the gripper reached its own last-computed goal_pose -- those are only
                    # equivalent if the grasp offset hasn't changed since that goal was computed. Convergence
                    # is gated on the direct part-to-target error; mpc_result.metrics (gripper-to-goal error,
                    # cuRobo's own solver-internal signal) is logged alongside for diagnostics only.
                    position_error, rotation_error = compute_assembly_placement_error()
                    gripper_position_error = float(mpc_result.metrics.position_error.item())
                    gripper_rotation_error = float(mpc_result.metrics.rotation_error.item())
                    if (
                        position_error < config._MPC_POSITION_CONVERGENCE_THRESHOLD_M
                        and rotation_error < config._MPC_ROTATION_CONVERGENCE_THRESHOLD_RAD
                    ):
                        mpc_converged_count += 1
                    else:
                        mpc_converged_count = 0

                    if mpc_converged_count >= config._MPC_CONVERGED_STEPS_REQUIRED:
                        print(
                            f"[mefron] MPC reactive placement converged after {mpc_step_count} steps "
                            f"(part_pos_err={position_error:.4f} m, part_rot_err={rotation_error:.4f} rad; "
                            f"gripper_pos_err={gripper_position_error:.4f}, gripper_rot_err={gripper_rotation_error:.4f}).",
                            flush=True,
                        )
                        mpc_active = False
                    elif mpc_step_count >= config._MPC_MAX_TRACKING_STEPS:
                        print(
                            f"[mefron] MPC reactive placement TIMED OUT after {mpc_step_count} steps "
                            f"(part_pos_err={position_error:.4f} m, part_rot_err={rotation_error:.4f} rad; "
                            f"gripper_pos_err={gripper_position_error:.4f}, gripper_rot_err={gripper_rotation_error:.4f}).",
                            flush=True,
                        )
                        mpc_active = False
                except Exception as exc:
                    print(f"[mefron] MPC branch failed, aborting reactive tracking: {exc}", flush=True)
                    mpc_active = False
                # Falls through to the unconditional gripper-ramp block below either way.
        elif cmd_plan is not None:
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
                    if pending_mpc_handoff:
                        pending_mpc_handoff = False
                        mpc_active = True
                        mpc_goal_buffer = None
                        mpc_step_count = 0
                        mpc_converged_count = 0
                        last_mpc_time = None
                        mpc_solver.reset()

        # Independent of cmd_plan/cuRobo -- applied every frame so it always wins the finger indices'
        # drive-target write, even though get_full_js() re-applies lock_joints on every planned frame too.
        if gripper_control is not None:
            gripper_target = config.GRIPPER_CLOSED_POSITION if gripper_control.closed else config.GRIPPER_OPEN_POSITION
            if gripper_setpoint is None:
                gripper_setpoint = gripper_target
            now = time.time()
            if last_gripper_time is not None:
                max_step = config.GRIPPER_CLOSE_SPEED * (now - last_gripper_time)
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
