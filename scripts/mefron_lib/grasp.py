"""Pose math for deriving and applying the grasp/assembly relative-pose constants. See
docs/grasp-and-assembly-offsets.md for how compute_relative_pose() was used to derive
config.ASSEMBLY_RELATIONSHIPS.
"""

from __future__ import annotations

import numpy as np
from isaacsim.core.prims import SingleXFormPrim

from . import config


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
    """True inverse of compute_dependent_world_pose(): given the world pose a rigidly-offset CHILD
    frame should end up at, plus that fixed offset, returns the pose the reference frame itself must
    reach. Needed because a screw's presented/hole pose is the ground truth, while the tool holding
    it is what actually gets driven there."""
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
    """Loads a Grasp-Editor-exported isaac_grasp yaml via Isaac Sim's own grasp_editor API,
    recomputed from the part's live pose on every call. The exported grasp is relative to
    panda_hand -- no further conversion needed, since franka.yml's ee_link already is panda_hand
    (not the URDF's separate, unused ee_link link 0.1m further out)."""
    from isaacsim.robot_setup.grasp_editor import import_grasps_from_file

    grasp_spec = import_grasps_from_file(str(yaml_path))
    # reset_xform_properties=False -- several parts carry an xformOp:scale:unitsResolve op the
    # default would silently strip. See docs/mefron-history.md (conveyor.py section).
    part_trans, part_quat = SingleXFormPrim(prim_path=part_prim_path, reset_xform_properties=False).get_world_pose()
    return grasp_spec.compute_gripper_pose_from_rigid_body_pose(grasp_name, part_trans, part_quat)


def compute_grasp_finger_widths_from_file(
    yaml_path: str,
    grasp_name: str,
    finger_joint_name: str = "panda_finger_joint1",
):
    """Reads the yaml's pregrasp_cspace_position (approach/open width) and cspace_position
    (grasp/closed width) for finger_joint_name -- the object-specific widths Grasp Editor authored,
    to replace the single global GRIPPER_OPEN_POSITION/GRIPPER_CLOSED_POSITION once a grasp is selected."""
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
    # reset_xform_properties=False -- mount_prim_path carries the same unitsResolve op as above.
    mount_trans, mount_quat = SingleXFormPrim(
        prim_path=relationship["mount_prim_path"], reset_xform_properties=False
    ).get_world_pose()
    return compute_dependent_world_pose(
        mount_trans, mount_quat, relationship["local_position"], relationship["local_orientation_wxyz"]
    )


def compute_assembly_grasp_target_from_offset(
    grasp_offset_position,
    grasp_offset_orientation_wxyz,
    relationship_name: str = "finger_print_scanner_on_main_holder",
):
    """Same composition as compute_assembly_grasp_target(), but takes the grasp offset as a
    parameter instead of measuring it -- lets compute_assembly_grasp_target() supply the CURRENT
    live-measured offset instead of a fixed constant."""
    part_target_trans, part_target_quat = compute_part_target_pose(relationship_name)
    return compute_dependent_world_pose(
        part_target_trans, part_target_quat, grasp_offset_position, grasp_offset_orientation_wxyz
    )


def compute_tool_dock_target(tool_name: str):
    """ee_link's target world pose for docking/undocking config.TOOL_CHANGE_TARGETS[tool_name]:
    composes the tool's LIVE female-coupler world pose with the fixed
    TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_* mate offset -- same composition direction as
    compute_assembly_grasp_target_from_offset(), just with a fixed offset instead of a measured one
    (a standardized coupler mates the same way every time, nothing to measure per-tool)."""
    from . import robot

    female_coupler_path = robot._female_coupler_prim_path(tool_name)
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
    """ee_link's target world pose for RETURNING a currently-docked tool to its own rack. Unlike
    compute_tool_dock_target(), this can't read the tool's live female-coupler pose -- it's riding
    on the wrist right now, not sitting statically at the rack, so its live pose reflects the
    wrist's current position, not the rack's. Uses the rack's fixed dock_position/orientation from
    config instead, composed the same way spawn_dockable_tool() placed the tool there originally."""
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
    """The presented screw's own world pose -- read live off config.SCREW_PRESENTER_PRIM_PATH, so a
    hand-placed presenter prim in mefron.usd is honored without touching config."""
    # reset_xform_properties=False -- a hand-placed/CAD-referenced presenter can carry the same
    # xformOp:scale:unitsResolve op the default would silently strip.
    return SingleXFormPrim(
        prim_path=config.SCREW_PRESENTER_PRIM_PATH, reset_xform_properties=False
    ).get_world_pose()


def compute_screw_hole_pose(hole_index: int):
    """The world pose a screw's own frame should end up at for config.SCREW_HOLES[hole_index]:
    main_holder's LIVE pose composed with that hole's local pose, then pushed
    SCREW_HOLE_INSERTION_DEPTH along the hole's OWN +Z (not world -Z) so a re-oriented hole entry
    still seats inward. Same live-relative principle as compute_part_target_pose()."""
    hole = config.SCREW_HOLES[hole_index]
    # reset_xform_properties=False -- main_holder carries an xformOp:scale:unitsResolve op; see
    # compute_part_target_pose() above.
    mount_trans, mount_quat = SingleXFormPrim(
        prim_path=config.SCREW_HOLE_MOUNT_PRIM_PATH, reset_xform_properties=False
    ).get_world_pose()
    entry_trans, entry_quat = compute_dependent_world_pose(
        mount_trans, mount_quat, hole["local_position"], hole["local_orientation_wxyz"]
    )
    return compute_dependent_world_pose(
        entry_trans, entry_quat, [0.0, 0.0, config.SCREW_HOLE_INSERTION_DEPTH], [1.0, 0.0, 0.0, 0.0]
    )


def compute_ee_target_for_screw_pose(ee_link_prim_path: str, screw_trans, screw_quat):
    """ee_link's target world pose for putting a screw carried on the bit at screw_trans/quat.
    Two steps: SCREW_CARRY_LOCAL_* inverts to the docked tool root's required pose, then the LIVE
    tool-root-to-ee_link offset (measured, not a constant -- so this stays right even though
    TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_* is still a placeholder) converts that to ee_link's."""
    from . import robot

    # reset_xform_properties=False on the tool -- it carries an xformOp:scale:unitsResolve op.
    tool_trans, tool_quat = SingleXFormPrim(
        prim_path=robot._tool_prim_path("screwdriver"), reset_xform_properties=False
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
    """Returns the world pose /World/target should be set to for P: main_holder's live pose composed
    with ASSEMBLY_RELATIONSHIPS gives the part's target pose; the CURRENT live gripper-to-part offset
    (not a fixed constant -- J, not G, does the grasp, so there's no separate grasp constant to fall
    back on) is applied on top to get the gripper's target. Computed once, on the P keypress."""
    relationship = config.ASSEMBLY_RELATIONSHIPS[relationship_name]
    # reset_xform_properties=False on both -- ee_link_prim_path has no unitsResolve op to lose
    # either way, but relationship["part_prim_path"] can carry one (see above).
    gripper_trans, gripper_quat = SingleXFormPrim(
        prim_path=ee_link_prim_path, reset_xform_properties=False
    ).get_world_pose()
    part_trans, part_quat = SingleXFormPrim(
        prim_path=relationship["part_prim_path"], reset_xform_properties=False
    ).get_world_pose()
    offset_trans, offset_quat = measure_grasp_offset(gripper_trans, gripper_quat, part_trans, part_quat)
    return compute_assembly_grasp_target_from_offset(offset_trans, offset_quat, relationship_name)
