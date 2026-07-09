"""Pose math for deriving and applying the grasp/assembly relative-pose constants. See
docs/grasp-and-assembly-offsets.md for how compute_relative_pose() was used to derive
config.GRASP_OFFSET_POSITION/ORIENTATION_WXYZ and config.ASSEMBLY_RELATIONSHIPS.
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


def compute_grasp_approach_pose(part_prim_path: str = config.HIGH_FRICTION_PRIM_PATHS[0]):
    """Returns the world pose /World/target should be set to in order to grasp the named part
    (finger_print_scanner by default) at the fixed relative offset GRASP_OFFSET_*, recomputed from its live pose."""
    part_trans, part_quat = SingleXFormPrim(prim_path=part_prim_path).get_world_pose()
    return compute_dependent_world_pose(
        part_trans, part_quat, config.GRASP_OFFSET_POSITION, config.GRASP_OFFSET_ORIENTATION_WXYZ
    )


def compute_grasp_approach_pose_from_file(
    yaml_path: str,
    grasp_name: str,
    part_prim_path: str = config.HIGH_FRICTION_PRIM_PATHS[0],
):
    """Alternative to compute_grasp_approach_pose(): loads a Grasp-Editor-exported isaac_grasp yaml via
    Isaac Sim's own isaacsim.robot_setup.grasp_editor API instead of the hand-derived GRASP_OFFSET_*
    constants, recomputed from the part's live pose on every call, same as compute_grasp_approach_pose().
    The exported grasp is relative to panda_hand (Grasp Editor's own gripper_frame) -- no further
    conversion needed, since cuRobo's own franka.yml sets `kinematics.ee_link: "panda_hand"`, i.e.
    /World/target (what this feeds) already *is* panda_hand's frame, not the URDF's separate, unused
    `ee_link` link 0.1m further out (confirmed by reading franka.yml directly, not assumed from the URDF
    alone -- an earlier version of this function wrongly composed that 0.1m offset in)."""
    from isaacsim.robot_setup.grasp_editor import import_grasps_from_file

    grasp_spec = import_grasps_from_file(str(yaml_path))
    part_trans, part_quat = SingleXFormPrim(prim_path=part_prim_path).get_world_pose()
    return grasp_spec.compute_gripper_pose_from_rigid_body_pose(grasp_name, part_trans, part_quat)


def compute_assembly_grasp_target(relationship_name: str = "finger_print_scanner_on_main_holder"):
    """Returns the world pose /World/target should be set to in order to place the already-grasped part at
    its assembled position, by composing the mount's live pose with ASSEMBLY_RELATIONSHIPS and GRASP_OFFSET_*."""
    relationship = config.ASSEMBLY_RELATIONSHIPS[relationship_name]
    mount_trans, mount_quat = SingleXFormPrim(prim_path=relationship["mount_prim_path"]).get_world_pose()
    part_target_trans, part_target_quat = compute_dependent_world_pose(
        mount_trans, mount_quat, relationship["local_position"], relationship["local_orientation_wxyz"]
    )
    return compute_dependent_world_pose(
        part_target_trans, part_target_quat, config.GRASP_OFFSET_POSITION, config.GRASP_OFFSET_ORIENTATION_WXYZ
    )
