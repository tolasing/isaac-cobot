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


def compute_grasp_approach_pose_from_file(
    yaml_path: str,
    grasp_name: str,
    part_prim_path: str = config.HIGH_FRICTION_PRIM_PATHS[0],
):
    """Loads a Grasp-Editor-exported isaac_grasp yaml via Isaac Sim's own
    isaacsim.robot_setup.grasp_editor API, recomputed from the part's live pose on every call.
    The exported grasp is relative to panda_hand (Grasp Editor's own gripper_frame) -- no further
    conversion needed, since cuRobo's own franka.yml sets `kinematics.ee_link: "panda_hand"`, i.e.
    /World/target (what this feeds) already *is* panda_hand's frame, not the URDF's separate, unused
    `ee_link` link 0.1m further out (confirmed by reading franka.yml directly, not assumed from the URDF
    alone -- an earlier version of this function wrongly composed that 0.1m offset in)."""
    from isaacsim.robot_setup.grasp_editor import import_grasps_from_file

    grasp_spec = import_grasps_from_file(str(yaml_path))
    part_trans, part_quat = SingleXFormPrim(prim_path=part_prim_path).get_world_pose()
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
    mount_trans, mount_quat = SingleXFormPrim(prim_path=relationship["mount_prim_path"]).get_world_pose()
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


def compute_assembly_grasp_target(ee_link_prim_path: str, relationship_name: str = "finger_print_scanner_on_main_holder"):
    """Returns the world pose /World/target should be set to for P: main_holder's live pose composed
    with ASSEMBLY_RELATIONSHIPS gives the part's target pose; the CURRENT live gripper-to-part offset
    (not a fixed constant -- J, not G, does the grasp, so there's no separate grasp constant to fall
    back on) is applied on top to get the gripper's target. Computed once, on the P keypress."""
    relationship = config.ASSEMBLY_RELATIONSHIPS[relationship_name]
    gripper_trans, gripper_quat = SingleXFormPrim(prim_path=ee_link_prim_path).get_world_pose()
    part_trans, part_quat = SingleXFormPrim(prim_path=relationship["part_prim_path"]).get_world_pose()
    offset_trans, offset_quat = measure_grasp_offset(gripper_trans, gripper_quat, part_trans, part_quat)
    return compute_assembly_grasp_target_from_offset(offset_trans, offset_quat, relationship_name)
