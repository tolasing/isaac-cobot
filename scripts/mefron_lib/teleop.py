"""The per-frame teleop loop: drag-follow plan/apply per arm, key-request dispatch, and the
Stop/Play rebuild. Physics-timing and rebuild gotchas: docs/mefron-history.md."""

from __future__ import annotations

import numpy as np
import omni.timeline
import omni.usd
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdPhysics

from . import assembly, config, feeder, motion, toolchanger
from .grasp import (
    compute_grasp_approach_pose_from_file,
    compute_grasp_finger_widths_from_file,
    compute_part_target_pose,
)


def _fresh_arm_state() -> dict:
    """Per-arm state rebuilt on every fresh Play -- everything here is bound to the physics view
    that existed when it was built."""
    return {
        "robot": None,
        "idx_list": None,
        "articulation_controller": None,
        "past_pose": None,
        "past_orientation": None,
        "target_pose": None,
        "target_orientation": None,
        "cmd_plan": None,
        "cmd_idx": 0,
        # Real elapsed time since the last waypoint was applied, and the plan's per-waypoint duration.
        "last_cmd_time": None,
        "interpolation_dt": 0.02,
        # P's real assembly pose, staged while `target` is snapped to the intermediate lift waypoint
        # first; applied once that lift plan finishes executing.
        "pending_final_pose": None,
        "obstacles": None,
        # Ramped gripper setpoint state -- see config.GRIPPER_CLOSE_SPEED.
        "gripper_setpoint": None,
        "last_gripper_time": None,
        # Ordered (position, orientation, on_arrival) waypoints for a multi-leg sequence -- an
        # arbitrary-length chain with side effects at specific legs, unlike pending_final_pose.
        "motion_queue": [],
        # True once a plan reaches a waypoint with a real on_arrival side effect but before that
        # effect has run -- see the robot_static-gated firing in _step_arm().
        "awaiting_arrival_settle": False,
    }


def _invalidate_articulation_handles(state: dict) -> None:
    """Forces _step_arm()'s rebuild-on-None block to rebind the articulation next frame. Authoring a
    joint prim live mid-Play silently stales the cached handles, so every live joint edit goes here."""
    state["idx_list"] = None
    state["articulation_controller"] = None


def _step_arm(arm: dict, step_index: int, tensor_args) -> None:
    """One frame of drag-follow plan/apply + key dispatch for a single arm, mutating arm["_state"]
    in place. Split out of run_teleop_loop() so several arms can share one timeline tick."""
    import time

    from curobo.types.math import Pose
    from curobo.types.state import JointState

    state = arm["_state"]
    motion_gen = arm["motion_gen"]
    target = arm["target"]
    gripper_control = arm.get("gripper_control")
    tool_changer_control = arm.get("tool_changer_control")
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
        # No gripper joint indices to resolve: the tool's finger joints are not in this arm's
        # own articulation and driven via toolchanger.set_gripper_tool_finger_target() instead.
        state["articulation_controller"] = state["robot"].get_articulation_controller()
        # A None index would feed apply_action()'s native PhysX call below and can crash the process
        # rather than raise. Fail loudly here instead.
        if any(i is None for i in state["idx_list"]):
            raise RuntimeError(
                f"[mefron_lib] {arm['_name']}: get_dof_index() could not resolve one or more joints "
                f"(idx_list={state['idx_list']}) -- refusing to drive this arm with an unresolved "
                "joint index."
            )

    if step_index < config._TELEOP_INIT_FRAMES:
        state["robot"].set_joint_positions(default_config, state["idx_list"])
        return
    if step_index < config._TELEOP_SETTLE_FRAMES:
        return

    if state["obstacles"] is None or step_index % config._TELEOP_OBSTACLE_RESCAN_INTERVAL == 0:
        state["obstacles"] = motion.get_obstacles(robot_prim_path, target_prim_path)
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

    suction_control = arm.get("suction_control")
    surface_gripper_control = arm.get("surface_gripper_control")

    # One-shot P/J/N/M snap requests. Must run AFTER the past_pose/target_pose bootstrap above, or
    # target_pose would be seeded from the post-snap pose and the debounce distance stays 0 forever.
    if gripper_control is not None:
        if gripper_control.has_pending_assembly_target_request():
            # Idle-gated: consuming mid-plan silently discards the request, and lets the in-flight
            # plan's completion wrongly consume pending_final_pose with no hover stop.
            if state["cmd_plan"] is None:
                docked_tool = tool_changer_control.currently_docked_tool if tool_changer_control is not None else None
                # "Actually holding it right now" gate -- last_grasped_object/last_approached_object
                # are sticky, so without this an O or L followed by P would still fire a placement.
                holding = (docked_tool == "gripper" and gripper_control.closed) or (
                    docked_tool == "suction"
                    and (surface_gripper_control is None or surface_gripper_control.is_closed())
                )
                relationship_name = (
                    motion.assembly_relationship_for_docked_tool(docked_tool, gripper_control, suction_control)
                    if holding
                    else None
                )

                gripper_control.consume_assembly_target_request()
                if relationship_name is not None:
                    cube_position, cube_orientation = motion.snap_target_to_assembly_lift_waypoint(
                        state, target, ee_link_prim_path, relationship_name
                    )
                # else: nothing valid to place -- discard rather than guessing.
        else:
            requested_object = gripper_control.consume_grasp_approach_from_file_request()
            if requested_object is not None:
                # Only meaningful with the gripper tool docked -- otherwise there are no fingers to
                # grasp with, wherever target snaps to.
                if tool_changer_control is not None and tool_changer_control.currently_docked_tool != "gripper":
                    print(
                        f"[mefron] {arm['_name']}: ignoring grasp-approach request -- the gripper tool isn't docked.",
                        flush=True,
                    )
                else:
                    grasp_target = config.GRASP_TARGETS[requested_object]
                    # Pin the whole grasp->place->weld cycle to the copy at the station now; a lifted
                    # part leaves the belt queue, so nothing later could still name it.
                    part_prim_path = feeder.latch(grasp_target["part_prim_path"])
                    # Free an already-assembled part first, or its weld joint pins it in place and
                    # the failure looks like a broken gripper.
                    if assembly.release_assembly_weld(part_prim_path):
                        _invalidate_articulation_handles(state)
                    cube_position, cube_orientation = compute_grasp_approach_pose_from_file(
                        grasp_target["yaml_path"],
                        grasp_target["grasp_name"],
                        part_prim_path=part_prim_path,
                    )
                    target.set_world_pose(position=cube_position, orientation=cube_orientation)
                    # Only the ignore paths printed before, so a snap that landed somewhere
                    # unexpected was indistinguishable from a key that never registered.
                    print(
                        f"[mefron] {arm['_name']}: grasp approach '{requested_object}' on "
                        f"{part_prim_path} -- target snapped to {np.round(cube_position, 4).tolist()}.",
                        flush=True,
                    )
                    open_position, closed_position = compute_grasp_finger_widths_from_file(
                        grasp_target["yaml_path"], grasp_target["grasp_name"]
                    )
                    gripper_control.set_grasp_widths(open_position, closed_position)
                    gripper_control.set_closed(False)

    # One-shot suction-approach snap. Ungated on cmd_plan unlike P -- no carried-object two-stage
    # lift concern here, so an immediate consume is safe; gated on the suction tool being docked.
    if suction_control is not None:
        requested_object = suction_control.consume_approach_request()
        if requested_object is not None:
            if tool_changer_control is not None and tool_changer_control.currently_docked_tool != "suction":
                print(
                    f"[mefron] {arm['_name']}: ignoring suction-approach request -- the suction tool isn't docked.",
                    flush=True,
                )
            else:
                approach_relationship = config.SUCTION_TARGETS[requested_object]["approach_relationship"]
                # Same latch + un-weld as the grasp branch above -- see there.
                part_prim_path = feeder.latch(config.ASSEMBLY_RELATIONSHIPS[approach_relationship]["part_prim_path"])
                if assembly.release_assembly_weld(part_prim_path):
                    _invalidate_articulation_handles(state)
                cube_position, cube_orientation = compute_part_target_pose(approach_relationship)
                target.set_world_pose(position=cube_position, orientation=cube_orientation)

    # One-shot O/L release weld. Both flags are consumed unconditionally so a stale one can't fire
    # later; the proximity gate lives in assembly.weld_part_at_assembly_pose().
    released_by_gripper = gripper_control is not None and gripper_control.consume_release_request()
    released_by_suction = surface_gripper_control is not None and surface_gripper_control.consume_release_request()
    if released_by_gripper or released_by_suction:
        docked_tool = tool_changer_control.currently_docked_tool if tool_changer_control is not None else None
        if (released_by_gripper and docked_tool == "gripper") or (released_by_suction and docked_tool == "suction"):
            relationship_name = motion.assembly_relationship_for_docked_tool(
                docked_tool, gripper_control, suction_control
            )
            if relationship_name is not None and assembly.weld_part_at_assembly_pose(relationship_name):
                _invalidate_articulation_handles(state)

    # One-shot tool-change request (Y/U/I), idle-gated: no in-flight plan and no leftover
    # waypoints, since each leg's dock/undock side effect must run in the right order.
    if (
        tool_changer_control is not None
        and tool_changer_control.has_pending_request()
        and state["cmd_plan"] is None
        and not state["motion_queue"]
    ):
        requested_tool = tool_changer_control.consume_request()
        if requested_tool == tool_changer_control.currently_docked_tool:
            print(f"[mefron] {arm['_name']}: {requested_tool} is already docked -- ignoring.", flush=True)
        else:
            queue = motion.build_tool_change_queue(tool_changer_control, requested_tool)
            cube_position, cube_orientation = motion.start_motion_queue(state, target, queue)

    # One-shot screw pick/place, idle-gated exactly like the tool-change block. Pick takes priority
    # over a place pressed in the same window; the place waits for the next idle frame.
    screw_control = arm.get("screw_control")
    if screw_control is not None and state["cmd_plan"] is None and not state["motion_queue"]:
        docked_tool = tool_changer_control.currently_docked_tool if tool_changer_control is not None else None
        holes_left = screw_control.hole_index < len(config.SCREW_HOLES)
        if screw_control.has_pending_pick_request():
            screw_control.consume_pick_request()
            if docked_tool != "screwdriver":
                print(f"[mefron] {arm['_name']}: ignoring screw pick -- the screwdriver tool isn't docked.", flush=True)
            elif screw_control.carried_screw_index is not None:
                print(f"[mefron] {arm['_name']}: ignoring screw pick -- a screw is already on the bit.", flush=True)
            elif not holes_left:
                print(f"[mefron] {arm['_name']}: ignoring screw pick -- all screw holes are filled.", flush=True)
            else:
                cube_position, cube_orientation = motion.start_motion_queue(
                    state, target, motion.build_screw_pick_queue(screw_control, ee_link_prim_path)
                )
        elif screw_control.has_pending_place_request():
            screw_control.consume_place_request()
            if docked_tool != "screwdriver":
                print(f"[mefron] {arm['_name']}: ignoring screw place -- the screwdriver tool isn't docked.", flush=True)
            elif screw_control.carried_screw_index is None:
                print(f"[mefron] {arm['_name']}: ignoring screw place -- no screw on the bit.", flush=True)
            elif not holes_left:
                print(f"[mefron] {arm['_name']}: ignoring screw place -- all screw holes are filled.", flush=True)
            else:
                cube_position, cube_orientation = motion.start_motion_queue(
                    state, target, motion.build_screw_place_queue(screw_control, ee_link_prim_path)
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

    if state["awaiting_arrival_settle"] and robot_static:
        # cmd_idx reaching the plan's end only says the COMMANDED velocity is ~0, not that the real
        # arm caught up -- docking onto a moving wrist is a coupling shock. docs/ee-arrival-accuracy.md.
        state["awaiting_arrival_settle"] = False
        _, _, on_arrival = state["motion_queue"][0]
        state["motion_queue"] = state["motion_queue"][1:]
        if on_arrival is not None:
            on_arrival()
            # A dock/undock authors a joint prim live mid-Play, which silently stales the cached
            # articulation handles -- apply_action() keeps succeeding but the arm stops responding.
            _invalidate_articulation_handles(state)
        if state["motion_queue"]:
            next_position, next_orientation, _ = state["motion_queue"][0]
            target.set_world_pose(position=next_position, orientation=next_orientation)

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
            # This specific plan's per-waypoint duration (MotionGenResult-level, not MotionGen-level).
            state["interpolation_dt"] = result.interpolation_dt
            state["last_cmd_time"] = None
        state["target_pose"] = cube_position
        state["target_orientation"] = cube_orientation

    state["past_pose"] = cube_position
    state["past_orientation"] = cube_orientation

    # articulation_controller can be None for exactly one frame: a live joint edit mid-plan (an O/L
    # release weld) invalidates the handles, and the block at the top rebinds them next frame.
    if state["cmd_plan"] is not None and state["articulation_controller"] is not None:
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
                elif state["motion_queue"]:
                    # A hover leg (on_arrival=None) has no side effect, so advance immediately; a
                    # dock/undock leg defers until robot_static confirms the real arm has settled.
                    _, _, on_arrival = state["motion_queue"][0]
                    if on_arrival is not None:
                        state["awaiting_arrival_settle"] = True
                    else:
                        state["motion_queue"] = state["motion_queue"][1:]
                        if state["motion_queue"]:
                            next_position, next_orientation, _ = state["motion_queue"][0]
                            target.set_world_pose(position=next_position, orientation=next_orientation)

    # Independent of cmd_plan/cuRobo -- applied every frame so it always wins the drive-target write.
    # The docked tool's fingers aren't part of this arm's articulation, so write their DriveAPI direct.
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
        toolchanger.set_gripper_tool_finger_target(state["gripper_setpoint"])


def run_teleop_loop(
    simulation_app,
    arms: list[dict],
    max_iterations: int | None = None,
    # Duck-typed (not conveyor.ConveyorControl / feeder.FeederControl) -- this loop only calls
    # .reset()/.step() on either, deliberately staying decoupled from both modules.
    conveyor_control: object | None = None,
    feeder_control: object | None = None,
) -> None:
    """Drags each arm's own `target`; each robot follows via cuRobo MotionGen plan/apply, rebuilding
    on every fresh Play. `arms` is a list of per-robot dicts (see mefron.py for the shape)."""
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
            # Once per not-playing stretch (startup, and after each Stop) rather than every 100
            # frames -- the repeat buried every other print in the terminal.
            if was_playing or not_playing_frames == 0:
                print("[mefron] Click Play to start cuRobo teleop.", flush=True)
            was_playing = False
            not_playing_frames += 1
            continue

        if not was_playing:
            # Fresh Play -- rebuild everything bound to the previous physics view.
            for arm in arms:
                arm["_state"] = _fresh_arm_state()
                for key in ("gripper_control", "suction_control", "tool_changer_control", "screw_control"):
                    control = arm.get(key)
                    if control is not None:
                        control.reset()
            if conveyor_control is not None:
                conveyor_control.reset()
            if feeder_control is not None:
                feeder_control.reset()
            step_index = 0
            was_playing = True

        step_index += 1
        if max_iterations is not None and step_index > max_iterations:
            return

        for arm in arms:
            _step_arm(arm, step_index, tensor_args)

        # Scene-level, not per-arm: keeps each welded part's kinematic anchor on its mount, so an
        # assembled part rides main_holder when the conveyor below moves it.
        assembly.sync_assembly_anchors()

        if conveyor_control is not None:
            conveyor_control.step()
        if feeder_control is not None:
            feeder_control.step()
