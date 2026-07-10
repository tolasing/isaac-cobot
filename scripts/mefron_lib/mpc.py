"""cuRobo MpcSolver setup for reactive, slip-corrected local tracking -- the "local" counterpart to
teleop.setup_motion_gen()'s "global" MotionGen, per cuRobo's own module docstring recommendation
(curobo.wrap.reacher.mpc: "MPC only optimizes locally... To generate global trajectories, use
MotionGen"). See run_teleop_loop()'s mpc_active branch for the per-frame usage.
"""

from __future__ import annotations

import numpy as np

from . import config


def setup_mpc_solver(robot_cfg: dict):
    """Builds and warms up an MpcSolver sharing robot_cfg with the already-constructed MotionGen, so
    both solvers agree on kinematics/collision-sphere geometry. Unlike MotionGen, MpcSolver has no
    .warmup() -- WrapMpc.solve() overrides WrapBase.solve() and skips its warm-up-on-first-call
    block -- so the MPPI optimizer's first-call CUDA-graph-capture cost is paid inline on whichever
    call triggers it first. Pays that cost here, against a throwaway hold-position goal, before the
    live control loop ever calls step()."""
    from curobo.rollout.rollout_base import Goal
    from curobo.types.base import TensorDeviceType
    from curobo.types.state import JointState
    from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig

    from .teleop import get_obstacles, motion_gen_kinematics_get_state

    tensor_args = TensorDeviceType()
    world_cfg = get_obstacles(exclude_paths=config._MPC_COLLISION_EXCLUDE_PATHS)

    # MpcSolverConfig.load_from_robot_config() has no velocity_scale/acceleration_scale kwarg (unlike
    # MotionGenConfig's), but it reads the same robot_cfg["kinematics"]["cspace"] fields MotionGen's
    # kwarg-based call writes -- so without this, MPC would silently inherit whatever scale
    # setup_motion_gen() happened to set, with no dedicated dampener of its own (MotionGen additionally
    # gets slowed via MotionGenPlanConfig(time_dilation_factor=...) at playback time in
    # run_teleop_loop(); MPC has no equivalent). Set independently here so MPC's actual ceiling doesn't
    # depend on call ordering elsewhere. Confirmed this doesn't retroactively affect the already-built
    # motion_gen passed in from setup_motion_gen() -- RobotConfig.from_dict() converts robot_cfg into
    # its own internal representation at construction time, not a live view of the dict.
    robot_cfg["kinematics"]["cspace"]["velocity_scale"] = config._MPC_VELOCITY_SCALE
    robot_cfg["kinematics"]["cspace"]["acceleration_scale"] = config._MPC_ACCELERATION_SCALE

    mpc_config = MpcSolverConfig.load_from_robot_config(
        {"robot_cfg": robot_cfg},
        world_cfg,
        tensor_args=tensor_args,
        step_dt=config._MPC_STEP_DT,
    )
    mpc_solver = MpcSolver(mpc_config)

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    retract_config = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])
    q = tensor_args.to_device(retract_config).unsqueeze(0)
    retract_ee_pose = motion_gen_kinematics_get_state(robot_cfg, q)

    # cspace.joint_names is 9-wide (arm+fingers); MpcSolver's actively-optimized joint set is 7-wide
    # (fingers are locked in franka.yml) -- must reduce via get_active_js(), not just reorder, or
    # step() shape-mismatches. Same reduce/expand dance run_teleop_loop() already does for MotionGen.
    warmup_state_full = JointState(
        position=q, velocity=q * 0.0, acceleration=q * 0.0, jerk=q * 0.0, joint_names=j_names
    )
    warmup_state = mpc_solver.get_active_js(warmup_state_full)
    warmup_goal = Goal(current_state=warmup_state, goal_pose=retract_ee_pose)
    mpc_solver.setup_solve_single(warmup_goal, num_seeds=1)
    for _ in range(config._MPC_WARMUP_STEPS):
        mpc_solver.step(warmup_state, max_attempts=config._MPC_STEP_MAX_ATTEMPTS)
    # Clear the warm-start before real use -- it's currently pointed at the throwaway retract goal.
    mpc_solver.reset()
    return mpc_solver
