# Waypoint data

Recorded CR5 waypoints, written by `scripts/teach_waypoint.py` and
consumed by `scripts/playback_waypoints.py`. Joint-space, not Cartesian —
each waypoint is 6 joint angles in radians, for joint1..joint6, in that
order.

Schema (see `scripts/waypoints.py` for the authoritative dataclass):

```json
{
  "waypoints": [
    { "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "name": "home" },
    { "joint_positions": [0.4, -0.2, 1.1, 0.0, 0.5, 0.0], "name": "approach" }
  ]
}
```

Files here are recorded run data, not vendored or hand-authored config —
expect this directory's `.json` contents to change as waypoints are taught.
