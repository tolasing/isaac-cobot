"""Joint-space waypoint schema shared by teach_waypoint.py and playback_waypoints.py. Waypoints
are joint-space, not Cartesian -- radians, 6 values for joint1..joint6."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

NUM_JOINTS = 6


@dataclass
class Waypoint:
    joint_positions: list[float]
    name: str = ""

    def __post_init__(self) -> None:
        if len(self.joint_positions) != NUM_JOINTS:
            raise ValueError(
                f"Waypoint.joint_positions must have {NUM_JOINTS} values (joint1..joint6), "
                f"got {len(self.joint_positions)}"
            )


def load_waypoints(path: Path) -> list[Waypoint]:
    data = json.loads(Path(path).read_text())
    return [Waypoint(**wp) for wp in data["waypoints"]]


def save_waypoints(path: Path, waypoints: list[Waypoint]) -> None:
    payload = {"waypoints": [asdict(wp) for wp in waypoints]}
    Path(path).write_text(json.dumps(payload, indent=2))


def append_waypoint(path: Path, waypoint: Waypoint) -> None:
    path = Path(path)
    waypoints = load_waypoints(path) if path.exists() else []
    waypoints.append(waypoint)
    save_waypoints(path, waypoints)
