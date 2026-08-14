"""cuRobo setup (obstacles, MotionGen, the draggable target) and the multi-leg waypoint queues
_step_arm() plays back for tool changes, screw picks/places and assembly placement."""

from __future__ import annotations

import numpy as np
import omni.usd
from isaacsim.core.prims import SingleXFormPrim
from pxr import Sdf

from . import config, feeder
from .grasp import compute_assembly_grasp_target
from .keyboard import ScrewControl, ToolChangerControl


def get_obstacles(robot_prim_path: str = config.ROBOT_PRIM_PATH, target_prim_path: str = config.TARGET_PRIM_PATH):
    from curobo.util.usd_helper import UsdHelper

    usd_help = UsdHelper()
    usd_help.load_stage(omni.usd.get_context().get_stage())
    return usd_help.get_obstacles_from_stage(
        only_paths=list(config.OBSTACLE_PRIM_PATHS),
        reference_prim_path=robot_prim_path,
        ignore_substring=[robot_prim_path, target_prim_path, "/curobo"],
    ).get_collision_check_world()


def load_robot_cfg() -> dict:
    """The FR5's cuRobo kinematics dict, converted from config.FR5_XRDF_PATH. Absolute paths
    throughout: cuRobo resolves relative ones against its OWN bundled dirs, not this repo."""
    from curobo.types.file_path import ContentPath
    from curobo.util.xrdf_utils import convert_xrdf_to_curobo

    content_path = ContentPath(
        robot_xrdf_absolute_path=str(config.FR5_XRDF_PATH),
        robot_urdf_absolute_path=str(config.FR5_URDF_PATH),
        robot_asset_absolute_path=str(config.FR5_URDF_ASSET_ROOT),
    )
    return convert_xrdf_to_curobo(content_path)["robot_cfg"]


def setup_motion_gen(
    robot_prim_path: str = config.ROBOT_PRIM_PATH,
    target_prim_path: str = config.TARGET_PRIM_PATH,
):
    from curobo.types.base import TensorDeviceType
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

    robot_cfg = load_robot_cfg()
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
    # Deferred import + tiny standalone CudaRobotModel, so build_teleop_target() doesn't need a live
    # MotionGen passed in just for forward kinematics.
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
    """Creates a draggable target at the robot's retract_config ee pose (guaranteed reachable),
    showing an internally-referenced (not CopyPrim'd) live view of the real end-effector mesh."""
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose as CuroboPose

    # ee_link is the synthetic tool_flange frame, which has no geometry -- show wrist3_link's mesh
    # instead, pushed back by the flange offset so the target's ORIGIN is where tools dock.
    source_path = f"{robot_prim_path}/{config.FR5_EE_LINK}/visuals"

    stage = omni.usd.get_context().get_stage()
    target_prim = stage.DefinePrim(target_prim_path, "Xform")
    visual_prim = stage.DefinePrim(f"{target_prim_path}/ee_visual", "Xform")
    visual_prim.GetReferences().AddInternalReference(Sdf.Path(source_path))
    SingleXFormPrim(prim_path=f"{target_prim_path}/ee_visual").set_local_pose(
        translation=np.array([0.0, 0.0, -config.FR5_TOOL_FLANGE_OFFSET]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )

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


def start_motion_queue(state: dict, target, queue: list):
    """Installs a fresh waypoint queue and snaps `target` to its first leg. The +1.0e6 forces a
    re-plan even when the new waypoint coincides with the last one -- docs/tool-changer.md gotcha 9."""
    state["motion_queue"] = queue
    position, orientation, _ = queue[0]
    target.set_world_pose(position=position, orientation=orientation)
    state["target_pose"] = position + 1.0e6
    state["target_orientation"] = orientation + 1.0e6
    return position, orientation


def _add_hover_descend_retract_leg(queue: list, position, orientation, on_arrival, clearance: float) -> None:
    """Appends one hover -> descend -> act -> retract leg. Hover clearance is relative to the leg's
    own target, never a fixed world-Z constant -- see CLAUDE.md's ASSEMBLY_LIFT_HEIGHT open issue."""
    hover_position = position + np.array([0.0, 0.0, clearance])
    queue.append((hover_position, orientation, None))
    queue.append((position, orientation, on_arrival))
    queue.append((hover_position, orientation, None))


def build_tool_change_queue(tool_changer_control: ToolChangerControl, requested_tool: str) -> list:
    """One tool swap: return the currently-docked tool to its rack first (skipped from a bare
    wrist), then approach the requested tool's rack. Both legs hover -> descend -> act -> retract."""
    from . import toolchanger
    from .grasp import compute_tool_dock_target, compute_tool_rack_return_target

    queue = []

    def _add_leg(dock_position, dock_orientation, on_arrival) -> None:
        _add_hover_descend_retract_leg(
            queue, dock_position, dock_orientation, on_arrival, config.TOOL_RACK_APPROACH_CLEARANCE
        )

    current_tool = tool_changer_control.currently_docked_tool
    if current_tool is not None and current_tool != requested_tool:

        def _on_undock(tool_name=current_tool) -> None:
            toolchanger.undock_tool_to_rack(tool_name)
            tool_changer_control.currently_docked_tool = None

        return_position, return_orientation = compute_tool_rack_return_target(current_tool)
        _add_leg(return_position, return_orientation, _on_undock)

    def _on_dock(tool_name=requested_tool) -> None:
        toolchanger.dock_tool_to_wrist(tool_name)
        tool_changer_control.currently_docked_tool = tool_name

    dock_position, dock_orientation = compute_tool_dock_target(requested_tool)
    _add_leg(dock_position, dock_orientation, _on_dock)
    return queue


def build_screw_pick_queue(screw_control: ScrewControl, ee_link_prim_path: str) -> list:
    """One hover -> descend -> weld-to-wrist -> retract leg onto the presented screw. Poses are
    computed here at request time off live poses, the same as P does."""
    from . import screws
    from .grasp import compute_ee_target_for_screw_pose, compute_screw_presenter_pose

    screw_index = screw_control.hole_index
    screw_trans, screw_quat = compute_screw_presenter_pose()
    pick_position, pick_orientation = compute_ee_target_for_screw_pose(ee_link_prim_path, screw_trans, screw_quat)

    def _on_pick(index=screw_index) -> None:
        screws.attach_screw_to_wrist(index)
        screw_control.carried_screw_index = index
        print(f"[mefron] screw {index} picked off the presenter.", flush=True)

    queue: list = []
    _add_hover_descend_retract_leg(
        queue, pick_position, pick_orientation, _on_pick, config.SCREW_APPROACH_CLEARANCE
    )
    return queue


def _warn_if_screw_mount_unassembled() -> None:
    """Holes are read off the back cover's LIVE pose, so placing before it's assembled seats screws
    wherever it's parked. Warns rather than refuses -- nothing here is unrecoverable."""
    from .grasp import compute_part_weld_pose

    relationship_name = next(
        (
            name
            for name, relationship in config.ASSEMBLY_RELATIONSHIPS.items()
            if relationship["part_prim_path"] == config.SCREW_HOLE_MOUNT_PRIM_PATH
        ),
        None,
    )
    if relationship_name is None:
        return
    # The assembled COPY, matching compute_screw_hole_pose()'s own resolution -- comparing against a
    # different copy than the holes are read off would make this warning meaningless.
    mount_prim_path = feeder.resolve_assembled(config.SCREW_HOLE_MOUNT_PRIM_PATH)
    live_trans, _ = SingleXFormPrim(prim_path=mount_prim_path, reset_xform_properties=False).get_world_pose()
    assembled_trans, _ = compute_part_weld_pose(relationship_name)
    distance = float(np.linalg.norm(np.array(live_trans) - np.array(assembled_trans)))
    if distance > config.ASSEMBLY_WELD_MAX_DISTANCE:
        print(
            f"[mefron] {mount_prim_path} is {distance:.3f}m from its assembled pose -- screws will be "
            "placed wherever it currently sits.",
            flush=True,
        )


def build_screw_place_queue(screw_control: ScrewControl, ee_link_prim_path: str) -> list:
    """One hover -> descend -> release-into-hole -> retract leg onto config.SCREW_HOLES[hole_index].
    On arrival the screw is welded in and the NEXT one is presented, so one is always waiting."""
    from . import screws
    from .grasp import compute_ee_target_for_screw_pose, compute_screw_hole_pose

    hole_index = screw_control.hole_index
    screw_index = screw_control.carried_screw_index
    _warn_if_screw_mount_unassembled()
    hole_trans, hole_quat = compute_screw_hole_pose(hole_index)
    place_position, place_orientation = compute_ee_target_for_screw_pose(ee_link_prim_path, hole_trans, hole_quat)

    def _on_place(index=screw_index, hole=hole_index) -> None:
        screws.weld_screw_into_hole(index, hole)
        screw_control.carried_screw_index = None
        screw_control.hole_index = hole + 1
        print(f"[mefron] screw {index} placed in hole {hole}.", flush=True)
        if screw_control.hole_index < len(config.SCREW_HOLES):
            screws.present_screw(screw_control.hole_index)
        else:
            print("[mefron] all screw holes filled.", flush=True)

    queue: list = []
    _add_hover_descend_retract_leg(
        queue, place_position, place_orientation, _on_place, config.SCREW_APPROACH_CLEARANCE
    )
    return queue


def assembly_relationship_for_docked_tool(docked_tool, gripper_control, suction_control) -> str | None:
    """Which config.ASSEMBLY_RELATIONSHIPS entry the docked tool's last-touched object maps to --
    shared by P and the O/L release weld. None if that tool hasn't touched anything yet."""
    if docked_tool == "gripper" and gripper_control is not None and gripper_control.last_grasped_object is not None:
        # Looked up by part_prim_path, not a "{object}_on_main_holder" key -- not every object mounts
        # onto main_holder (e.g. pcb_assembly_on_backpanel_support).
        part_prim_path = config.GRASP_TARGETS[gripper_control.last_grasped_object]["part_prim_path"]
        return next(
            (
                name
                for name, relationship in config.ASSEMBLY_RELATIONSHIPS.items()
                if relationship["part_prim_path"] == part_prim_path
            ),
            None,
        )
    if docked_tool == "suction" and suction_control is not None and suction_control.last_approached_object is not None:
        return config.SUCTION_TARGETS[suction_control.last_approached_object]["assembly_relationship"]
    return None


def snap_target_to_assembly_lift_waypoint(state: dict, target, ee_link_prim_path: str, relationship_name: str):
    """P's first half: stages the final pose and snaps `target` to an intermediate waypoint (final
    X/Y at ASSEMBLY_LIFT_HEIGHT). A direct plan dragged the carried object through the table."""
    final_position, final_orientation = compute_assembly_grasp_target(ee_link_prim_path, relationship_name)
    state["pending_final_pose"] = (final_position, final_orientation)
    cube_position = np.array([final_position[0], final_position[1], config.ASSEMBLY_LIFT_HEIGHT])
    cube_orientation = final_orientation
    target.set_world_pose(position=cube_position, orientation=cube_orientation)
    return cube_position, cube_orientation
