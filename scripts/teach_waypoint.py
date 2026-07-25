"""Records the CR5's current joint state as a new waypoint. FIRST DRAFT / UNVERIFIED: written
against the isaacsim 5.1 API surface, not run against a live install. Run inside a session where
/World/CR5 already exists and has been posed via the GUI's joint drives:
${ISAACSIM_ROOT_PATH}/python.sh scripts/teach_waypoint.py --name approach"""

from __future__ import annotations

import argparse
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.prims import SingleArticulation  # noqa: E402
from waypoints import Waypoint, append_waypoint  # noqa: E402

CR5_PRIM_PATH = "/World/CR5"
WAYPOINTS_PATH = Path(__file__).resolve().parent.parent / "data" / "waypoints" / "waypoints.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="", help="Optional label for this waypoint.")
    parser.add_argument("--out", type=Path, default=WAYPOINTS_PATH, help="Waypoints JSON file to append to.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    cr5 = SingleArticulation(prim_path=CR5_PRIM_PATH, name="cr5")
    cr5.initialize()

    joint_positions = cr5.get_joint_positions()[:6].tolist()
    waypoint = Waypoint(joint_positions=joint_positions, name=args.name)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    append_waypoint(args.out, waypoint)
    print(f"[teach_waypoint] Recorded waypoint {waypoint.name!r}: {waypoint.joint_positions}")

    simulation_app.close()


if __name__ == "__main__":
    main()
