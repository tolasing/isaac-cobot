# PGC-140 Asset Provenance

The URDF and mesh files in this directory are vendored from the official
DH-Robotics GitHub organization, not authored in this repo.

- **Source repo:** https://github.com/DH-Robotics/dh_gripper_ros
- **Path:** `dh_pgc140_urdf/{urdf/dh_pgc140_urdf.urdf, meshes/*.STL}`
- **Commit vendored:** `f59f9c2f4bc8eb116448b1d798791424bf64e337` (branch `master`, 2021-12-10)
- **Date vendored:** 2026-07-12
- **License:** BSD, per `dh_pgc140_urdf/package.xml`'s `<license>BSD</license>`
  tag — this is the *only* license signal in the upstream repo. There is no
  repo-root `LICENSE` file, no specific BSD variant (2- vs 3-clause), and no
  copyright-holder text anywhere upstream (unlike `robots/cr5/`'s MIT
  license, which came with a full `LICENSE` file to copy verbatim). No
  `LICENSE-pgc140-upstream` companion file exists here for that reason —
  there is no license text to copy, only the bare package.xml tag.

## Modifications made to the vendored files

Every occurrence of DH-Robotics' original names was renamed to avoid a real
collision with `robots/cr5/urdf/cr5_robot.urdf`'s own `base_link` (the CR5
arm's base link and the PGC-140's base link both happened to be named
`base_link` upstream — confirmed by direct inspection of both files):

```
robot name:  dh_pgc140_urdf   -> pgc140_robot
base_link          -> pgc140_base_link
finger1_link       -> pgc140_finger1_link
finger1_joint      -> pgc140_finger1_joint
finger2_link       -> pgc140_finger2_link
finger2_joint      -> pgc140_finger2_joint
```

The `<mimic joint="finger1_joint" .../>` tag on `finger2_joint` was
**removed entirely**, not just renamed. It was initially kept (with its
`joint=` reference updated to `pgc140_finger1_joint`), but confirmed live
(direct introspection of the imported USD prim, on this repo's pinned
Isaac Sim version) that this importer applies it as
`PhysxMimicJointAPI:rotX` — a **rotational** mimic API — onto what is
actually a **prismatic** (linear) joint, and additionally imports mangled
joint limits for it (`lower=-0.005, upper=0.030` instead of the URDF's own
`lower="0" upper="0.025"`, a uniform -0.005 offset with no evident source).
No `UsdPhysics.DriveAPI` gets attached to a mimic-tagged joint at all on
this importer version (confirmed: `UsdPhysics.DriveAPI.Get(joint_prim,
"linear")` returns nothing for it, only for the un-mimicked
`pgc140_finger1_joint`), so it isn't independently drivable either — the
combination left `pgc140_finger2_joint` stuck at its own (incorrect) upper
limit (~0.030) regardless of what was commanded to `pgc140_finger1_joint`,
confirmed via a headless teleop run. This is a real, version-specific
import bug for prismatic mimic joints on this Isaac Sim build, not
something fixable from the config/Python side — removing the tag makes
both finger joints ordinary, independently-drivable prismatic joints
(each gets a real, correctly-imported linear DriveAPI), and both are then
driven with the identical commanded value at the teleop-control layer
(`scripts/build_scene.py`'s gripper block), exactly mirroring how
`scripts/mefron_lib/teleop.py` already drives the Franka's two
(non-mimic-linked) finger joints. See `configs/curobo/cr5.yml`'s own
`cspace`/`lock_joints` comments for the corresponding config-side change
(both finger joints are now tracked there, matching cuRobo's own bundled
`franka.yml` convention for its two independent finger joints).

Mesh `<geometry>` URIs were rewritten from ROS package-relative form to
plain relative filesystem paths, matching `robots/cr5/urdf/cr5_robot.urdf`'s
own established rewrite convention exactly:

```
package://dh_pgc140_urdf/meshes/<file>.STL  →  ../meshes/<file>.STL
```

No geometry, inertial, or kinematic (link/joint origin) data was altered.

## Known quirks inherited from the upstream file

- Same SolidWorks-to-URDF exporter as `robots/cr5/urdf/cr5_robot.urdf` (see
  the file's own header comment) — but unlike the CR5 URDF, both finger
  joints already have real, non-zero `velocity="1"` and `effort="140"`
  limits. No analogous "velocity=0" fix was needed here.
- Neither finger joint has a `<dynamics damping=.../>` element at all —
  confirmed by direct inspection. (This matters because
  `scripts/mefron_lib/robot.py`'s Franka hand template *does* author an
  explicit `<dynamics damping="10.0"/>` on its own finger joints, which
  `docs/mefron-history.md` documents as having silently overridden that
  pipeline's own post-import DriveAPI reauthoring in one case. Since this
  URDF has no such element, that specific failure mode does not apply here
  — but the post-import DriveAPI values still must be read back and
  verified, not assumed, per `import_cr5.py`'s own established practice.)
- `package.xml`'s `<author>`/`<maintainer>` fields are both the literal
  placeholder string `TODO` / `TODO@email.com` — an upstream authoring gap,
  not a mistake in this vendoring pass.
- Mesh format is STL (not COLLADA like the CR5's `.dae` meshes) — Isaac
  Sim's URDF importer handles both directly; no conversion needed.
- `pgc140_finger2_joint`'s `<mimic>` tag was real upstream content (not
  added by this vendoring pass) but was removed here — see "Modifications
  made" above for the full, empirically-confirmed reason (this importer
  version applies it as a rotational mimic API onto a linear joint, with
  mangled limits and no drive attached). If a future Isaac Sim version
  fixes prismatic mimic-joint import, this could potentially be restored
  and re-verified — don't restore it without re-running the same
  DriveAPI/limit introspection this fix was based on.
