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


def measure_grasp_offset(gripper_trans, gripper_quat, part_trans, part_quat):
    """Measures the CURRENT live gripper-to-part relative pose (same shape/semantics as the fixed
    GRASP_OFFSET_POSITION/ORIENTATION_WXYZ constants: T_part_gripper) instead of assuming the fixed
    nominal offset -- corrects for whatever grasp slip has actually happened this pick."""
    return compute_relative_pose(part_trans, part_quat, gripper_trans, gripper_quat)


def compute_part_target_pose(relationship_name: str = "finger_print_scanner_on_main_holder"):
    """Returns the part's own target world pose on its mount (e.g. finger_print_scanner's correctly-
    assembled pose on main_holder) -- independent of any grasp offset. This is the pose that actually
    determines whether the assembly is correct; compute_assembly_grasp_target*() converts it into an
    equivalent GRIPPER target only because that's the sole goal type cuRobo's MotionGen/MpcSolver can
    directly command. See compute_assembly_placement_error() for checking against this directly."""
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
    parameter instead of hardcoding config.GRASP_OFFSET_POSITION/ORIENTATION_WXYZ."""
    part_target_trans, part_target_quat = compute_part_target_pose(relationship_name)
    return compute_dependent_world_pose(
        part_target_trans, part_target_quat, grasp_offset_position, grasp_offset_orientation_wxyz
    )


def quaternion_angular_distance(quat_a_wxyz, quat_b_wxyz) -> float:
    """Standard quaternion angular distance in radians: 2*arccos(|dot(a, b)|). The abs() makes it
    robust to the double-cover sign ambiguity (q and -q represent the same rotation, and USD/cuRobo
    give no guarantee which sign a given pose read-back returns)."""
    dot = np.clip(np.abs(np.dot(np.array(quat_a_wxyz), np.array(quat_b_wxyz))), -1.0, 1.0)
    return float(2.0 * np.arccos(dot))


def compute_assembly_placement_error(relationship_name: str = "finger_print_scanner_on_main_holder"):
    """Returns (position_error_m, rotation_error_rad): how far the part's CURRENT live pose is from
    its correctly-assembled target pose on its mount -- the thing that actually matters for a correct
    assembly, independent of the gripper/grasp-offset indirection compute_reactive_assembly_target()
    needs. Use this (not mpc_result.metrics, which measures gripper-to-its-own-last-goal error) to
    decide whether reactive placement has actually converged -- gripper-converged only implies
    part-converged if the grasp offset hasn't changed since that particular goal was computed, which
    isn't guaranteed if slip is still happening in the final ticks."""
    relationship = config.ASSEMBLY_RELATIONSHIPS[relationship_name]
    part_trans, part_quat = SingleXFormPrim(prim_path=relationship["part_prim_path"]).get_world_pose()
    target_trans, target_quat = compute_part_target_pose(relationship_name)
    position_error = float(np.linalg.norm(np.array(part_trans) - np.array(target_trans)))
    rotation_error = quaternion_angular_distance(part_quat, target_quat)
    return position_error, rotation_error


def compute_assembly_grasp_target(ee_link_prim_path: str):
    """Returns the world pose /World/target should be set to for the P key's placement: main_holder's
    live pose composed with ASSEMBLY_RELATIONSHIPS gives finger_print_scanner's target pose, then the
    CURRENT live gripper-to-part offset (not a fixed nominal one -- picking is via J, not G, so there's
    no separate grasp constant to fall back on) is applied to get the gripper's target. Computed once,
    on the P keypress, rather than continuously -- see compute_reactive_assembly_target(), which this
    delegates to and which M's phase-2 calls every tick instead of once."""
    return compute_reactive_assembly_target(ee_link_prim_path)


def compute_reactive_assembly_target(
    ee_link_prim_path: str, relationship_name: str = "finger_print_scanner_on_main_holder"
):
    """Per-frame derivation for slip-corrected placement: measures the current live gripper-to-part
    offset (not the fixed nominal GRASP_OFFSET_*), then applies it to the part's target pose on
    main_holder -- so the commanded gripper pose corrects for whatever slip has actually happened,
    rather than assuming the grasp came out exactly as planned."""
    relationship = config.ASSEMBLY_RELATIONSHIPS[relationship_name]
    gripper_trans, gripper_quat = SingleXFormPrim(prim_path=ee_link_prim_path).get_world_pose()
    part_trans, part_quat = SingleXFormPrim(prim_path=relationship["part_prim_path"]).get_world_pose()
    offset_trans, offset_quat = measure_grasp_offset(gripper_trans, gripper_quat, part_trans, part_quat)
    return compute_assembly_grasp_target_from_offset(offset_trans, offset_quat, relationship_name)
