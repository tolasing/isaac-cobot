"""Constants shared across the mefron family of scripts: paths, mount pose, gripper/friction/drive
tuning, and the derived grasp/assembly relative poses. Pure data -- no omni/curobo imports, safe to
import at any point.
"""

from __future__ import annotations

import math
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MEFRON_USD = REPO_ROOT / "assets" / "mefron" / "factory floor" / "mefron.usd"
# Disk-persisted "Robot Description" dir the URDF importer writes on every import into this
# file-backed stage; see kit_bootstrap.clear_stale_robot_configuration().
MEFRON_CONFIGURATION_DIR = MEFRON_USD.parent / "configuration"

ROBOT_PRIM_PATH = "/World/Franka"
TARGET_PRIM_PATH = "/World/target"
# SEKTION cabinet table the Franka mounts on (replaced the original Pedestal_plates/Cube_05 plate).
# No /Factory prefix: mefron.py opens mefron.usd directly, one level shallower than build_scene_mefron.py's reference.
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

# Loop-timing constants for teleop.run_teleop_loop(), ported from build_scene.py.
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
_TELEOP_VELOCITY_SCALE = 0.5
_TELEOP_ACCELERATION_SCALE = 0.5

# Grasp-physics constants, ported from build_scene_mefron.py's apply_gripper_friction()/stiffen_gripper_drive().
GRIPPER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]
# Narrowed from the full 0-0.04m stroke to bracket finger_print_scanner's actual 12mm grip width
# (measured via UsdGeom.BBoxCache local bound) -- the full stroke let one finger contact and drag the
# part sideways well before the other closed the remaining distance. CLOSED is the symmetric half-width
# (6mm/side); OPEN adds a 4mm/side clearance margin for approach.
GRIPPER_OPEN_POSITION = 0.010
GRIPPER_CLOSED_POSITION = 0.000
# Rate (m/s) the commanded gripper position is ramped toward open/closed, instead of stepping instantly --
# avoids a snap shut under the high drive stiffness. 0.02 m/s takes ~0.2s for the now-narrowed 0.004m travel.
GRIPPER_CLOSE_SPEED = 0.02
GRIPPER_FRICTION_MATERIAL_PATH = "/World/GripperFrictionMaterial"
GRIPPER_STATIC_FRICTION = 1.5
GRIPPER_DYNAMIC_FRICTION = 1.5
GRIPPER_FINGER_LINK_NAMES = ["panda_leftfinger", "panda_rightfinger"]
GRIPPER_DRIVE_STIFFNESS = 10000.0
GRIPPER_DRIVE_DAMPING = 200.0
HIGH_FRICTION_PRIM_PATHS = ["/World/finger_print_scanner"]

# T_S_G: the Franka's ee_link pose expressed in finger_print_scanner's own local frame at a known-good
# grasp, derived via grasp.compute_relative_pose() against a manually-jogged /World/target (see docs/grasp-and-assembly-offsets.md).
GRASP_OFFSET_POSITION = [0.00027002069774515104, -0.021693730387954874, -0.1271989186209571]
GRASP_OFFSET_ORIENTATION_WXYZ = [
    -2.1523912431273915e-05,
    -8.089888886539503e-06,
    5.762411090611313e-06,
    0.9999999997190347,
]

# Grasp Editor-exported alternative to GRASP_OFFSET_*, wired to the J key for live comparison against
# the G key's constants-based grasp-approach pose. See grasp.compute_grasp_approach_pose_from_file().
GRASP_EDITOR_YAML_PATH = REPO_ROOT / "assets" / "finger_print_scanner.yaml"
GRASP_EDITOR_GRASP_NAME = "grasp_0"

# MPC reactive-tracking constants (see mpc.setup_mpc_solver(), teleop.run_teleop_loop()'s mpc_active branch).
_MPC_STEP_DT = 0.05  # Unvalidated starting guess -- print mpc_result.solve_time on first live use and tune.
# Convergence is gated on grasp.compute_assembly_placement_error() -- the part's actual live pose vs.
# its target on main_holder -- not on mpc_result.metrics (gripper-to-its-own-last-goal error), since
# those are only equivalent if the grasp offset hasn't changed since that goal was computed.
# Loosened from 0.003 m / 1 deg -- every real run so far (headless and live) has printed "TIMED OUT",
# never "converged", meaning the original thresholds were never actually reachable and MPC always
# burned the full step budget regardless of how close it got. Still flagged as needing live tuning.
_MPC_POSITION_CONVERGENCE_THRESHOLD_M = 0.01  # raw Euclidean meters, no internal deadband exists.
_MPC_ROTATION_CONVERGENCE_THRESHOLD_RAD = math.radians(5.0)
_MPC_CONVERGED_STEPS_REQUIRED = 5  # consecutive under-threshold steps before declaring "arrived."
_MPC_MAX_TRACKING_STEPS = 300  # timeout safeguard -- MpcSolver has no built-in "stuck" flag.
_MPC_STEP_MAX_ATTEMPTS = 2
_MPC_WARMUP_STEPS = 5

# MPC's own joint velocity/acceleration ceiling -- set independently in mpc.setup_mpc_solver() rather
# than relying on inheriting whatever _TELEOP_VELOCITY_SCALE/_TELEOP_ACCELERATION_SCALE happened to
# already bake into the shared robot_cfg dict via setup_motion_gen()'s call ordering. Needed because
# MotionGen has a SEPARATE dampener MPC has no equivalent for -- run_teleop_loop() re-times MotionGen's
# played-back trajectory via MotionGenPlanConfig(time_dilation_factor=_TELEOP_TIME_DILATION_FACTOR),
# uniformly slowing it ~3.3x after planning; MpcSolverConfig has no time_dilation_factor parameter at
# all, so MPC otherwise moves as fast as MPPI decides, up to the raw joint-velocity ceiling, with
# nothing analogous to that post-hoc slowdown. Confirmed empirically (scratch script, not committed)
# that this actually changes MPC's resolved velocity limit and does NOT retroactively affect an
# already-constructed motion_gen/mpc_solver -- safe to set independently. Unvalidated starting point,
# same as _MPC_STEP_DT -- tune live.
_MPC_VELOCITY_SCALE = 0.1
_MPC_ACCELERATION_SCALE = 0.1

# Excluded from MPC's own collision world only (not MotionGen's -- its longer-range approach still
# benefits from avoiding these, and doesn't have the fighting-itself problem below since it solves one
# global trajectory once rather than continuously re-optimizing near the goal). main_holder is the
# assembly destination itself, so avoidance directly conflicts with the reaching goal right where MPC
# needs to converge -- confirmed as the actual cause of a live crash (approach, then "flying around" as
# it got close, part slipped/tumbled after an uncontrolled impact). finger_print_scanner is the carried
# part, which would otherwise be treated as a STATIC obstacle at its stale pre-grasp pose (the collision
# world only rescans every _TELEOP_OBSTACLE_RESCAN_INTERVAL frames). The correct long-term fix is wiring
# up cuRobo's attach_objects_to_robot() (see CLAUDE.md's open issues); this is a stopgap.
_MPC_COLLISION_EXCLUDE_PATHS = ["/World/main_holder", "/World/finger_print_scanner"]

# T_H_S: finger_print_scanner's pose expressed in main_holder's own local frame at the correctly
# assembled position, derived via grasp.compute_relative_pose() after temporarily reparenting in mefron.usd.
ASSEMBLY_RELATIONSHIPS = {
    "finger_print_scanner_on_main_holder": {
        "part_prim_path": "/World/finger_print_scanner",
        "mount_prim_path": "/World/main_holder",
        "local_position": [-0.05765, 0.02069, 0.01565],
        "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
}
