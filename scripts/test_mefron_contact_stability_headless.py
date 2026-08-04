"""Headless regression test for the contact-vibration fix (robot.tune_physics_scene() /
tune_assembly_part_stability()): parks the gripper next to the parts, then measures whether they
actually hold still. See docs/mefron-history.md for the measurements the thresholds come from.

Run standalone (pass --no-tuning to capture the pre-fix baseline for comparison):
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/test_mefron_contact_stability_headless.py --headless
"""

from __future__ import annotations

import os
import sys

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
_tuning_enabled = "--no-tuning" not in sys.argv
if __name__ == "__main__":
    # Same reason as test_mefron_assembly_headless.py: the J snap goes through
    # grasp.compute_grasp_approach_pose_from_file(), which needs isaacsim.robot_setup.grasp_editor.
    simulation_app = SimulationApp(
        {"headless": _headless}, experience=f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    )

# Must run before any omni/curobo import -- see mefron_lib/kit_bootstrap.py's docstring.
from mefron_lib.kit_bootstrap import clear_stale_robot_configuration, preload_real_packaging  # noqa: E402

preload_real_packaging()

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleRigidPrim  # noqa: E402
from mefron_lib import config, robot, teleop  # noqa: E402

# 1800, double test_mefron_assembly_headless.py's 900: the trajectory has to actually FINISH before
# the measurement below or it samples a still-in-transit gripper, and the headroom keeps the same
# frame count valid (and the baseline comparable) if config.PHYSICS_TIME_STEPS_PER_SECOND is raised --
# under fixed time stepping that halves the sim-time each frame covers.
_MAX_ITERATIONS_PER_PHASE = 1800
# Frames sampled per measurement window, with the arm parked and no new plan in flight.
_MEASURE_FRAMES = 200

# Measured at-rest noise floor with no robot in the scene at all: max per-frame step 9e-6 m, angular
# velocity ~1e-3 rad/s (see docs/mefron-history.md). These sit ~100x above that -- comfortably clear
# of solver noise, far below the violent shaking being tested for.
_MAX_STEP_THRESHOLD = 1.0e-3  # m per frame
_MAX_ANGULAR_VELOCITY_THRESHOLD = 0.5  # rad/s


def _measure(label: str) -> dict[str, dict[str, float]]:
    """Free-runs _MEASURE_FRAMES frames with whatever pose the arm is parked at, sampling every
    assembly part's per-frame position delta and angular velocity. Rebuilds each SingleRigidPrim
    fresh (this codebase's convention -- never cache a PhysX-backed handle across a Stop)."""
    # A stopped timeline freezes every part, which would read as a perfect PASS -- the exact false
    # negative that hid a missing play() call the first time this test was run.
    assert omni.timeline.get_timeline_interface().is_playing(), f"timeline stopped before '{label}'"

    parts = {}
    for part_path in config.ASSEMBLY_PART_PRIM_PATHS:
        try:
            rigid_prim = SingleRigidPrim(prim_path=part_path, name=f"measure_{label}_{part_path.split('/')[-1]}")
            rigid_prim.initialize()
            parts[part_path] = rigid_prim
        except Exception as exc:  # noqa: BLE001 -- a missing/unprobeable part shouldn't abort the run
            print(f"[test_mefron_contact_stability] WARNING: cannot probe {part_path}: {exc}", flush=True)

    positions = {path: [] for path in parts}
    angular_velocities = {path: [] for path in parts}
    for _ in range(_MEASURE_FRAMES):
        simulation_app.update()
        for path, rigid_prim in parts.items():
            position, _ = rigid_prim.get_world_pose()
            positions[path].append(np.array(position, dtype=float))
            angular_velocities[path].append(float(np.linalg.norm(rigid_prim.get_angular_velocity())))

    results = {}
    print(f"\n[test_mefron_contact_stability] ===== {label} =====", flush=True)
    for path in parts:
        samples = np.stack(positions[path])
        steps = np.linalg.norm(np.diff(samples, axis=0), axis=1)
        results[path] = {
            "max_step": float(steps.max()),
            "mean_step": float(steps.mean()),
            "max_angular_velocity": max(angular_velocities[path]),
            "total_drift": float(np.linalg.norm(samples[-1] - samples[0])),
        }
        stats = results[path]
        print(
            f"  {path}: max_step={stats['max_step']:.6f} m  mean_step={stats['mean_step']:.6f} m  "
            f"max_ang_vel={stats['max_angular_velocity']:.6f} rad/s  drift={stats['total_drift']:.6f} m",
            flush=True,
        )
    return results


def main() -> None:
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)
    carb.settings.get_settings().set_bool("/app/player/useFixedTimeStepping", True)

    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)
    omni.usd.get_context().open_stage(str(config.MEFRON_USD))
    robot.clear_stray_robot_prims()
    for _ in range(120):
        simulation_app.update()

    if _tuning_enabled:
        robot.tune_physics_scene()
        robot.tune_assembly_part_stability()
    else:
        print("[test_mefron_contact_stability] --no-tuning: capturing PRE-FIX baseline.", flush=True)

    robot.mount_franka()
    robot.apply_gripper_friction()
    robot.stiffen_gripper_drive()

    print("[test_mefron_contact_stability] warming up cuRobo motion_gen...", flush=True)
    motion_gen, robot_cfg = teleop.setup_motion_gen()
    target = teleop.build_teleop_target(robot_cfg)
    gripper_control = teleop.GripperKeyboardControl()
    arms = [
        {
            "motion_gen": motion_gen,
            "robot_cfg": robot_cfg,
            "target": target,
            "gripper_control": gripper_control,
            "robot_prim_path": config.ROBOT_PRIM_PATH,
            "target_prim_path": config.TARGET_PRIM_PATH,
            "mount_position": config.MOUNT_POSITION,
            "mount_orientation_wxyz": config.MOUNT_ORIENTATION_WXYZ,
            "name": "arm1",
        }
    ]

    # stop() then play(): warmup()'s long unpumped gap can leave PhysX's simulation view corrupted
    # (same reason mefron.py forces a stop there), and run_teleop_loop() rebuilds every articulation
    # on the not-playing -> playing transition. Nothing clicks Play in headless, so play() is required
    # or the loop just spins printing "Click Play" and the measurement reads a stopped, frozen scene.
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    timeline.play()
    for _ in range(5):
        simulation_app.update()
    assert timeline.is_playing(), "timeline is not playing -- measurements would be meaningless"

    # Phase 1: J -- drive the gripper down onto finger_print_scanner, then hold and measure.
    gripper_control.request_grasp_approach_from_file("finger_print_scanner")
    teleop.run_teleop_loop(simulation_app, arms, max_iterations=_MAX_ITERATIONS_PER_PHASE)
    grasp_results = _measure("gripper parked at finger_print_scanner grasp pose")

    # Phase 2: P -- the reported symptom. Places near main_holder, which is where main_holder and the
    # parts around it were seen shaking.
    gripper_control.set_closed(True)
    gripper_control.request_assembly_target()
    teleop.run_teleop_loop(simulation_app, arms, max_iterations=_MAX_ITERATIONS_PER_PHASE)
    place_results = _measure("gripper parked at main_holder assembly-placement pose")

    failures = []
    for label, results in (("grasp", grasp_results), ("place", place_results)):
        for path, stats in results.items():
            if stats["max_step"] > _MAX_STEP_THRESHOLD:
                failures.append(
                    f"{label}/{path}: max_step {stats['max_step']:.6f} m > {_MAX_STEP_THRESHOLD} m"
                )
            if stats["max_angular_velocity"] > _MAX_ANGULAR_VELOCITY_THRESHOLD:
                failures.append(
                    f"{label}/{path}: max_ang_vel {stats['max_angular_velocity']:.6f} rad/s "
                    f"> {_MAX_ANGULAR_VELOCITY_THRESHOLD} rad/s"
                )

    print("", flush=True)
    if failures:
        print("[test_mefron_contact_stability] FAIL: parts moved more than the thresholds allow:", flush=True)
        for failure in failures:
            print(f"  {failure}", flush=True)
    else:
        print(
            "[test_mefron_contact_stability] PASS: every assembly part held still with the gripper "
            "parked at both the grasp and the placement pose.",
            flush=True,
        )

    simulation_app.close()


if __name__ == "__main__":
    main()
