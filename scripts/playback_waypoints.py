"""Plays back recorded waypoints on the CR5 via cuRobo's plan_single_js(). FIRST DRAFT /
UNVERIFIED: not run against a live install.
Run standalone: ${ISAACSIM_ROOT_PATH}/python.sh scripts/playback_waypoints.py"""

from __future__ import annotations

import argparse
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import torch  # noqa: E402
from curobo.types.state import JointState  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from setup_curobo import build_motion_gen  # noqa: E402
from waypoints import load_waypoints  # noqa: E402

CR5_PRIM_PATH = "/World/CR5"
WAYPOINTS_PATH = Path(__file__).resolve().parent.parent / "data" / "waypoints" / "waypoints.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, default=WAYPOINTS_PATH, help="Waypoints JSON file to play back.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    waypoints = load_waypoints(args.file)

    cr5 = SingleArticulation(prim_path=CR5_PRIM_PATH, name="cr5")
    cr5.initialize()

    motion_gen = build_motion_gen()
    joint_names = motion_gen.kinematics.joint_names
    device = motion_gen.tensor_args.device

    current = JointState.from_position(
        torch.tensor([cr5.get_joint_positions()[:6].tolist()], device=device),
        joint_names=joint_names,
    )

    for waypoint in waypoints:
        goal = JointState.from_position(
            torch.tensor([waypoint.joint_positions], device=device),
            joint_names=joint_names,
        )
        result = motion_gen.plan_single_js(current, goal)
        if not result.success.item():
            print(f"[playback_waypoints] Planning failed for waypoint {waypoint.name!r}, skipping.")
            continue

        trajectory = result.get_interpolated_plan()
        for step in range(trajectory.position.shape[0]):
            cr5.set_joint_position_targets(trajectory.position[step].cpu().numpy())
            simulation_app.update()

        current = goal

    simulation_app.close()


if __name__ == "__main__":
    main()
