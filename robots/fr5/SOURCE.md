# FR5 Asset Provenance

The URDF and mesh files in this directory are vendored from the official
FAIRINO (FAIR Innovation) GitHub organization, not authored in this repo.

- **Source repo:** https://github.com/FAIR-INNOVATION/frcobot_ros2
- **Path:** `fairino_description/{urdf/fairino5_v6.urdf, meshes/fairino5_v6/*.STL}`
- **Commit vendored:** `867cb32bc24a73c1e60bef4e6c16762e7357c5e1` (branch `main`)
- **Date vendored:** 2026-08-13
- **License: NONE DECLARED UPSTREAM.** See "License" below — this is a real
  provenance gap, unlike [`../cr5/`](../cr5/SOURCE.md) (MIT, full text vendored)
  and [`../pgc140/`](../pgc140/SOURCE.md) on the `dobot` branch (a bare `BSD`
  `package.xml` tag).

## Why `fairino5_v6.urdf` and not `FR5WM.urdf`

`fairino_description/urdf/` ships two FR5-named files. `fairino5_v6.urdf` is the
one this repo uses:

- **Its kinematics are the FR5's.** Joint origins give a 425mm upper arm
  (`j3`), a 395mm forearm (`j4`) and 0.152/0.1021/0.102m base+wrist offsets —
  FAIRINO's published FR5 geometry, 922mm reach. `FR5WM.urdf` has a different
  chain (0.42 / 0.36m, a restructured wrist) and far lighter links (upperarm
  4.53kg vs this file's 10.08kg), so it is a different machine.
- **Every joint has real `effort`/`velocity` limits** (150/28 N·m, 3.15/3.2
  rad/s). This is not cosmetic: the CR5's SolidWorks-exporter `velocity="0"`
  artifact made cuRobo hard-fail inside `bound_cost.set_bounds()` with
  `ValueError: Joint velocity limits is zero`, and had to be patched into the
  URDF itself because cuRobo has no config-level override (see
  [`../cr5/SOURCE.md`](../cr5/SOURCE.md)). This file cannot hit that.
- **`FR5WM.urdf` does not parse.** Its `tool` fixed joint declares
  `<child link="gripper_Link"/>` while the `gripper_Link` definition itself is
  commented out — an unresolvable child link.

## Modifications made to the vendored files

`urdf/fairino5_v6.urdf` had its mesh `<geometry>` URIs rewritten from ROS
package-relative form to plain relative filesystem paths, since this repo has no
ROS package resolver — the same rewrite convention `../cr5/` and `../pgc140/`
already use:

```
package://fairino_description/meshes/fairino5_v6/<file>.STL  →  ../meshes/<file>.STL
```

This is the only content change. No geometry, inertial, joint limit, or
kinematic data was altered, and the upstream quirks below were left in place
rather than silently "cleaned up".

## Known quirks inherited from the upstream file

- **`wrist2_link`'s `<collision>` block spells its origin `<origins>`**
  (line 354). URDF parsers ignore the unknown element and fall back to an
  identity origin — which is exactly what the misspelled element declares
  (`xyz="0 0 0" rpy="0 0 0"`), so nothing is geometrically wrong. Left as-is to
  keep "no data altered" literally true; fix it only if a parser ever rejects it.
- **One mesh set serves both `<visual>` and `<collision>`** — the same STL, no
  separate simplified collision hull and no `.dae` visual variant. Two
  consequences: the arm renders flat grey (no baked materials, unlike the
  Franka's textured props), and every collision mesh is full-resolution CAD.
  The sibling `FR5WM`/`FR3WML` mesh sets upstream *do* split visual `.dae` from
  collision `.STL`, but belong to different robots.
- Same SolidWorks-to-URDF exporter as `../cr5/urdf/cr5_robot.urdf` (see the
  file's own header) — but unlike the CR5, the limits are real (see above).
- Every joint carries a `<safety_controller>` element. Isaac Sim's URDF importer
  ignores these; joint actuation here comes from explicit drive-strength
  parameters set at import time (`config.FR5_DRIVE_STRENGTH` /
  `FR5_DRIVE_DAMPING`), not from them.
- `<dynamics damping="0" friction="0"/>` on all six joints, i.e. no joint
  damping at all from the URDF. Post-import `DriveAPI` values must be read back
  and verified rather than assumed — the practice `../pgc140/SOURCE.md`
  established for the same reason.

## License

**Upstream declares no license anywhere.** Confirmed 2026-08-13, all three of:

- no `LICENSE` file at the repo root (HTTP 404),
- GitHub's own repository `license` field is `null`,
- `fairino_description/package.xml` carries the literal placeholder
  `<license>TODO: License declaration</license>`, with `<maintainer>` likewise
  `fr@todo.todo`.

There is therefore no license text to vendor as a companion file, and no
`LICENSE-fr5-upstream` exists here for that reason. This is recorded, not
resolved: redistributing or shipping these assets outside internal simulation
use needs a license clarification from FAIRINO first.
