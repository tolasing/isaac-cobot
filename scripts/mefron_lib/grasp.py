"""Pose math for deriving and applying the grasp/assembly/screw relative-pose constants.
See docs/grasp-and-assembly-offsets.md for how these were derived."""

from __future__ import annotations

import numpy as np
from isaacsim.core.prims import SingleXFormPrim

from . import config
from .feeder import resolve_assembled, resolve_for_pick


def relationship_pose_prim_paths(relationship_name: str) -> tuple[str, str]:
    """(part, mount) resolved to live instances. A part resolves to the copy being picked, a mount to
    the copy already assembled -- except the mount == part suction-approach entries. docs/part-feeders.md."""
    relationship = config.ASSEMBLY_RELATIONSHIPS[relationship_name]
    part_prim_path = resolve_for_pick(relationship["part_prim_path"])
    if relationship["mount_prim_path"] == relationship["part_prim_path"]:
        return part_prim_path, part_prim_path
    return part_prim_path, resolve_assembled(relationship["mount_prim_path"])


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


def compute_reference_world_pose(dependent_trans, dependent_quat, relative_trans, relative_quat_wxyz):
    """True inverse of compute_dependent_world_pose(): given where a rigidly-offset CHILD frame must
    end up, returns the pose the reference frame must reach. A screw's hole pose is the ground truth."""
    from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats

    dep_rot, rel_rot = quats_to_rot_matrices(np.array([dependent_quat, relative_quat_wxyz]))
    ref_rot = dep_rot @ rel_rot.T
    ref_trans = np.array(dependent_trans) - ref_rot @ np.array(relative_trans)
    return ref_trans, rot_matrices_to_quats(np.array([ref_rot]))[0]


def compute_grasp_approach_pose_from_file(
    yaml_path: str,
    grasp_name: str,
    part_prim_path: str = config.HIGH_FRICTION_PRIM_PATHS[0],
):
    """Loads a Grasp-Editor-exported yaml, recomputed from the part's live pose every call. The
    grasp is relative to panda_hand, which franka.yml's ee_link already is -- no conversion needed."""
    from isaacsim.robot_setup.grasp_editor import import_grasps_from_file

    grasp_spec = import_grasps_from_file(str(yaml_path))
    # reset_xform_properties=False -- parts carry an xformOp:scale:unitsResolve op the default
    # would silently strip. See docs/mefron-history.md.
    part_trans, part_quat = SingleXFormPrim(
        prim_path=resolve_for_pick(part_prim_path), reset_xform_properties=False
    ).get_world_pose()
    return grasp_spec.compute_gripper_pose_from_rigid_body_pose(grasp_name, part_trans, part_quat)


def compute_grasp_finger_widths_from_file(
    yaml_path: str,
    grasp_name: str,
    finger_joint_name: str = "panda_finger_joint1",
):
    """Reads the yaml's pregrasp_cspace_position (open) and cspace_position (closed) widths --
    the object-specific values that override config's global defaults once a grasp is selected."""
    from isaacsim.robot_setup.grasp_editor import import_grasps_from_file

    grasp_spec = import_grasps_from_file(str(yaml_path))
    grasp_dict = grasp_spec.get_grasp_dict_by_name(grasp_name)
    open_position = grasp_dict["pregrasp_cspace_position"][finger_joint_name]
    closed_position = grasp_dict["cspace_position"][finger_joint_name]
    return open_position, closed_position


def measure_grasp_offset(gripper_trans, gripper_quat, part_trans, part_quat):
    """CURRENT live gripper-to-part relative pose (T_part_gripper) -- not a fixed nominal offset,
    since J's grasp (not a hand-derived constant) is what actually determines this now."""
    return compute_relative_pose(part_trans, part_quat, gripper_trans, gripper_quat)


def compute_part_target_pose(relationship_name: str = "finger_print_scanner_on_main_holder"):
    """The part's own target world pose on its mount, independent of any grasp offset."""
    relationship = config.ASSEMBLY_RELATIONSHIPS[relationship_name]
    _, mount_prim_path = relationship_pose_prim_paths(relationship_name)
    # reset_xform_properties=False -- mount_prim_path carries the same unitsResolve op as above.
    mount_trans, mount_quat = SingleXFormPrim(
        prim_path=mount_prim_path, reset_xform_properties=False
    ).get_world_pose()
    return compute_dependent_world_pose(
        mount_trans, mount_quat, relationship["local_position"], relationship["local_orientation_wxyz"]
    )


def assembly_weld_local_pose(relationship_name: str):
    """Where the O/L release weld seats a part: ASSEMBLY_WELD_POSES when it has an entry, else the
    ASSEMBLY_RELATIONSHIPS offset P drives to. Split on purpose -- see that dict's own comment."""
    weld = config.ASSEMBLY_WELD_POSES.get(relationship_name) or config.ASSEMBLY_RELATIONSHIPS[relationship_name]
    return weld["local_position"], weld["local_orientation_wxyz"]


def compute_part_weld_pose(relationship_name: str):
    """The world pose that weld seats the part at -- the mount's LIVE pose composed with the offset
    above. Same live-relative principle as compute_part_target_pose(), just the weld's own offset."""
    _, mount_prim_path = relationship_pose_prim_paths(relationship_name)
    mount_trans, mount_quat = SingleXFormPrim(
        prim_path=mount_prim_path, reset_xform_properties=False
    ).get_world_pose()
    return compute_dependent_world_pose(mount_trans, mount_quat, *assembly_weld_local_pose(relationship_name))


def compute_assembly_grasp_target_from_offset(
    grasp_offset_position,
    grasp_offset_orientation_wxyz,
    relationship_name: str = "finger_print_scanner_on_main_holder",
):
    """Same composition as compute_assembly_grasp_target(), but takes the grasp offset as a
    parameter instead of measuring it."""
    part_target_trans, part_target_quat = compute_part_target_pose(relationship_name)
    return compute_dependent_world_pose(
        part_target_trans, part_target_quat, grasp_offset_position, grasp_offset_orientation_wxyz
    )


def compute_tool_dock_target(tool_name: str):
    """ee_link's target pose for docking a tool: its LIVE female-coupler pose composed with the
    fixed mate offset -- a standardized coupler mates the same way every time, nothing per-tool."""
    from . import toolchanger

    female_coupler_path = toolchanger._female_coupler_prim_path(tool_name)
    coupler_trans, coupler_quat = SingleXFormPrim(
        prim_path=female_coupler_path, reset_xform_properties=False
    ).get_world_pose()
    return compute_dependent_world_pose(
        coupler_trans,
        coupler_quat,
        config.TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_POSITION,
        config.TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_ORIENTATION_WXYZ,
    )


def compute_tool_rack_return_target(tool_name: str):
    """ee_link's target pose for RETURNING a docked tool to its rack. Can't read the tool's live
    coupler pose -- it's riding the wrist -- so it uses the rack's own recorded dock pose instead."""
    target = config.TOOL_CHANGE_TARGETS[tool_name]
    female_coupler_trans, female_coupler_quat = compute_dependent_world_pose(
        np.array(target["dock_position"]),
        np.array(target["dock_orientation_wxyz"]),
        target["female_coupler_local_position"],
        target["female_coupler_local_orientation_wxyz"],
    )
    return compute_dependent_world_pose(
        female_coupler_trans,
        female_coupler_quat,
        config.TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_POSITION,
        config.TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_ORIENTATION_WXYZ,
    )


def compute_screw_presenter_pose():
    """The presented screw's world pose: the live presenter prim composed with the seat offset,
    since the prim origin is the CAD base plate. Read live, so a hand-placed presenter wins."""
    # reset_xform_properties=False -- a hand-placed/CAD-referenced presenter can carry the same
    # xformOp:scale:unitsResolve op the default would silently strip.
    presenter_trans, presenter_quat = SingleXFormPrim(
        prim_path=config.SCREW_PRESENTER_PRIM_PATH, reset_xform_properties=False
    ).get_world_pose()
    return compute_dependent_world_pose(
        presenter_trans,
        presenter_quat,
        config.SCREW_PRESENTER_SEAT_LOCAL_POSITION,
        config.SCREW_PRESENTER_SEAT_LOCAL_ORIENTATION_WXYZ,
    )


def screw_hole_local_pose(hole_index: int):
    """Where a seated screw sits in the MOUNT's frame: the hole entry pose pushed along the hole's
    OWN +Z. Shared with weld_screw_into_hole()'s joint frame -- one source of truth, or it fights."""
    hole = config.SCREW_HOLES[hole_index]
    return compute_dependent_world_pose(
        hole["local_position"],
        hole["local_orientation_wxyz"],
        [0.0, 0.0, config.SCREW_HOLE_INSERTION_DEPTH],
        [1.0, 0.0, 0.0, 0.0],
    )


def compute_screw_hole_pose(hole_index: int):
    """The world pose a screw should end up at for config.SCREW_HOLES[hole_index]: the back cover's
    LIVE pose composed with the local pose above."""
    # reset_xform_properties=False -- the cover carries an xformOp:scale:unitsResolve op; see
    # compute_part_target_pose() above. resolve_assembled: the holes are in the copy already fitted.
    mount_trans, mount_quat = SingleXFormPrim(
        prim_path=resolve_assembled(config.SCREW_HOLE_MOUNT_PRIM_PATH), reset_xform_properties=False
    ).get_world_pose()
    return compute_dependent_world_pose(mount_trans, mount_quat, *screw_hole_local_pose(hole_index))


def compute_ee_target_for_screw_pose(ee_link_prim_path: str, screw_trans, screw_quat):
    """ee_link's target pose for putting a carried screw at screw_trans/quat. SCREW_CARRY_LOCAL_*
    inverts to the tool root's pose, then the LIVE (not constant) tool-to-ee offset converts it."""
    from . import toolchanger

    # reset_xform_properties=False on the tool -- it carries an xformOp:scale:unitsResolve op.
    tool_trans, tool_quat = SingleXFormPrim(
        prim_path=toolchanger._tool_prim_path("screwdriver"), reset_xform_properties=False
    ).get_world_pose()
    ee_trans, ee_quat = SingleXFormPrim(prim_path=ee_link_prim_path, reset_xform_properties=False).get_world_pose()
    ee_wrt_tool_trans, ee_wrt_tool_quat = compute_relative_pose(tool_trans, tool_quat, ee_trans, ee_quat)

    target_tool_trans, target_tool_quat = compute_reference_world_pose(
        screw_trans, screw_quat, config.SCREW_CARRY_LOCAL_POSITION, config.SCREW_CARRY_LOCAL_ORIENTATION_WXYZ
    )
    return compute_dependent_world_pose(
        target_tool_trans, target_tool_quat, ee_wrt_tool_trans, ee_wrt_tool_quat
    )


def compute_assembly_grasp_target(ee_link_prim_path: str, relationship_name: str = "finger_print_scanner_on_main_holder"):
    """The world pose /World/target takes for P: the part's target pose from ASSEMBLY_RELATIONSHIPS,
    with the CURRENT live gripper-to-part offset applied on top. Computed once, on the keypress."""
    part_prim_path, _ = relationship_pose_prim_paths(relationship_name)
    # reset_xform_properties=False on both -- ee_link_prim_path has no unitsResolve op to lose
    # either way, but the part can carry one (see above).
    gripper_trans, gripper_quat = SingleXFormPrim(
        prim_path=ee_link_prim_path, reset_xform_properties=False
    ).get_world_pose()
    part_trans, part_quat = SingleXFormPrim(
        prim_path=part_prim_path, reset_xform_properties=False
    ).get_world_pose()
    offset_trans, offset_quat = measure_grasp_offset(gripper_trans, gripper_quat, part_trans, part_quat)
    return compute_assembly_grasp_target_from_offset(offset_trans, offset_quat, relationship_name)
