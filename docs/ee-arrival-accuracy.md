# End-effector arrival accuracy: the ee settles ~3mm short of `/World/target`

Investigated **2026-08-04** on branch `atc`, live in the GUI. Symptom: after a
teleop move, the arm looks correct but sits slightly *below* the target ghost.
The initial read — "not an issue with cuRobo, just the joint formation" — is
confirmed correct by measurement. This doc holds the numbers, the probe scripts
that produced them, and the fix options, so none of it needs re-deriving.

Related: `docs/mefron-history.md` (the loop's own history),
`docs/grasp-and-assembly-offsets.md` (the hand-jog methodology this deliberately
does *not* use — everything here is measured in-sim, not eyeballed).

## What was measured

Arm settled after a normal drag-teleop move, one tool docked, reach 0.665m from
the mount:

```
target        [ 2.85562 -5.29367  1.0112 ]
panda_hand    [ 2.85526 -5.29291  1.00823]
  MEASURED ee - target       dx=   -0.37 dy=   +0.77 dz=   -2.97  |d|=   3.09 mm
    in target's own frame    dx=   +0.28 dy=   -0.80 dz=   +2.97  |d|=   3.09 mm
  orientation error 0.378 deg
------------------------------------------------------------------------------
  joint              cmd(rad)  meas(rad)   err(deg)       vel
  panda_joint1        0.86110    0.86108     -0.001   -0.0023
  panda_joint2       -1.54447   -1.54757     -0.178    0.0997
  panda_joint3       -1.63932   -1.64248     -0.181    0.0736
  panda_joint4       -1.78153   -1.78168     -0.008   -0.0101
  panda_joint5       -1.55873   -1.55887     -0.008    0.0466
  panda_joint6        1.64406    1.64093     -0.179    0.0116
  panda_joint7       -0.49956   -0.49954     +0.001    0.0003
------------------------------------------------------------------------------
  FK(cmd) - target           dx=   -0.00 dy=   -0.00 dz=   +0.00  |d|=   0.00 mm
  FK(meas) - FK(cmd)         dx=   -0.36 dy=   +0.77 dz=   -2.97  |d|=   3.09 mm
  FK(meas) - USD panda_hand  dx=   +0.00 dy=   -0.00 dz=   -0.00  |d|=   0.00 mm
------------------------------------------------------------------------------
  /PhysicsScene gravity dir=(0, 0, 0) mag=-inf   <- USD defaults, so -Z 9.81 is active
  arm links with gravity disabled: none
  panda_hand/visuals local offset [0. 0. 0.]
```

### Conclusions

- **cuRobo is exonerated.** `FK(commanded joints) − target` = 0.00mm / 0.001°.
  The planner, the `robot_base_pose.compute_local_pose()` frame handling, and
  every pose transform in the chain are exact. The error lives entirely between
  *commanded* and *achieved* joint positions.
- **It is joint tracking lag**: ~0.18° on joints 2/3/6 (all the same sign),
  ~0.00° on 1/4/5/7. That projects to **−2.97mm along the approach axis** (+Z in
  the target's own frame, i.e. short of it) plus 0.38° of orientation.
- **Not a rendering artifact.** `panda_hand/visuals` has an exactly zero local
  offset, and offline FK agrees with the live USD read to 0.00mm — the ghost is
  an honest stand-in for the ee frame, so what the viewport shows is real.
- **The arm had not stopped moving when it was declared arrived.** Residual
  velocity up to **0.0997 rad/s**, carried by the same three joints that carry
  the position error. `config._STATIC_JOINT_VELOCITY_THRESHOLD` is 0.5 rad/s —
  **5x looser** than that residual — so `robot_static` goes True (and every
  `on_arrival` side effect: dock, undock, screw weld) while the arm is ~3mm out.
- `teleop._step_arm()` marks a target handled at **plan** time (it sets
  `state["target_pose"] = cube_position` right after `plan_single()`) and never
  re-checks the achieved pose, so any steady-state error is permanent by design.
- The loop applies **position *and* velocity** targets per waypoint, then simply
  stops streaming at the last one. Nothing re-asserts the final pose afterward.

## The one open measurement

Does the residual **decay** or **plateau**? That single answer picks the fix.
Run the sampler in [Probe 2](#probe-2-decaying-transient-or-steady-state) while
playing, right after a move settles.

| Outcome | Root cause | Fix |
| --- | --- | --- |
| `applied VEL target` nonzero on j2/j3/j6 | the drive is still chasing the trajectory's final velocity, so the spring/damper balance parks the joint short | after the last waypoint, re-assert the final **position** with **zero velocity** until settled, instead of stopping |
| table decays toward 0 | a slow settle the loop never waits for | tighten `_STATIC_JOINT_VELOCITY_THRESHOLD` 0.5 → ~0.01, and hold the last command until it clears |
| table plateaus, VEL targets all zero | genuine static gravity sag | drive gains (`FRANKA_DRIVE_STRENGTH` = 1047.19751 today; size the bump off the `kp`/`kd` the probe prints) or `physxRigidBody:disableGravity` |

**Do not** try to fix this by re-planning to the same Cartesian target on
arrival: cuRobo returns the same final joint state `q*`, whose sag is identical,
so it cannot converge. The correction has to be in joint space, in the arrival
gate, or in the drive.

## Probe 1: target vs achieved ee

Paste into the Script Editor while playing, with the arm settled after a move.
Read-only by construction: poses come from
`UsdGeom.Xformable(...).ComputeLocalToWorldTransform()`, never from
`SingleXFormPrim(path)` whose default `reset_xform_properties=True` would
rewrite the prim's xform op stack.

```python
# ===== mefron: target-vs-achieved ee diagnostic (read-only) =====
import math
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf

ROBOT   = "/World/Franka"
EE_PATH = ROBOT + "/panda_hand"
TARGET  = "/World/target"
ARM_JOINTS = ["panda_joint%d" % i for i in range(1, 8)]

stage = omni.usd.get_context().get_stage()

def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def world_pose(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None, None
    t = Gf.Transform(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    q = t.GetRotation().GetQuat()
    w, x, y, z = np.array([q.GetReal(), *q.GetImaginary()])
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])
    return np.array(t.GetTranslation()), R

_CHAIN = [((0.0, 0.0, 0.333), 0.0), ((0.0, 0.0, 0.0), -math.pi / 2), ((0.0, -0.316, 0.0), math.pi / 2),
          ((0.0825, 0.0, 0.0), math.pi / 2), ((-0.0825, 0.384, 0.0), -math.pi / 2),
          ((0.0, 0.0, 0.0), math.pi / 2), ((0.088, 0.0, 0.0), math.pi / 2)]

def fk_hand(q, base_p, base_R):
    R, p = np.eye(3), np.zeros(3)
    for (xyz, rx), qi in zip(_CHAIN, q):
        p = p + R @ np.array(xyz)
        R = R @ _rx(rx) @ _rz(qi)
    p = p + R @ np.array([0.0, 0.0, 0.107])                  # panda_joint8
    return base_p + base_R @ p, base_R @ R @ _rz(-0.785398163397)   # panda_hand_joint

def ang_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))

def show(label, d):
    print("  %-26s dx=%+8.2f dy=%+8.2f dz=%+8.2f  |d|=%7.2f mm"
          % (label, d[0] * 1e3, d[1] * 1e3, d[2] * 1e3, np.linalg.norm(d) * 1e3))

tgt_p, tgt_R = world_pose(TARGET)
ee_p,  ee_R  = world_pose(EE_PATH)
base_p, base_R = world_pose(ROBOT + "/panda_link0")
if base_p is None:
    base_p, base_R = world_pose(ROBOT)

print("=" * 78)
if tgt_p is None or ee_p is None:
    print("MISSING: target=%s ee=%s -- is the run live?" % (tgt_p is not None, ee_p is not None))
else:
    print("target       ", np.round(tgt_p, 5))
    print("panda_hand   ", np.round(ee_p, 5), " reach from base %.4f m" % np.linalg.norm(ee_p - base_p))
    d = ee_p - tgt_p
    show("MEASURED ee - target", d)
    show("  in target's own frame", tgt_R.T @ d)   # z = approach axis
    print("  orientation error %.3f deg" % ang_deg(tgt_R, ee_R))

try:
    from isaacsim.core.prims import SingleArticulation
    art = SingleArticulation(prim_path=ROBOT, name="mefron_diag", reset_xform_properties=False)
    art.initialize()
    names = list(art.dof_names)
    meas = np.array(art.get_joint_positions(), dtype=float)
    action = art.get_applied_action()
    cmd = None if action is None or action.joint_positions is None else np.array(action.joint_positions, dtype=float)
    vel = np.array(art.get_joint_velocities(), dtype=float)

    print("-" * 78)
    print("  %-16s %10s %10s %10s %9s" % ("joint", "cmd(rad)", "meas(rad)", "err(deg)", "vel"))
    for name in ARM_JOINTS:
        if name not in names:
            print("  %-16s  (not in DOF list)" % name)
            continue
        i = names.index(name)
        c = float(cmd[i]) if cmd is not None else float("nan")
        print("  %-16s %10.5f %10.5f %+10.3f %9.4f"
              % (name, c, meas[i], math.degrees(meas[i] - c), vel[i]))

    if cmd is not None:
        idx = [names.index(n) for n in ARM_JOINTS if n in names]
        p_cmd, R_cmd = fk_hand(cmd[idx], base_p, base_R)
        p_meas, R_meas = fk_hand(meas[idx], base_p, base_R)
        print("-" * 78)
        show("FK(cmd) - target", p_cmd - tgt_p)            # cuRobo IK/plan residual
        print("  ^ plan residual, orientation %.3f deg" % ang_deg(tgt_R, R_cmd))
        show("FK(meas) - FK(cmd)", p_meas - p_cmd)         # PD / gravity sag
        print("  ^ servo sag, orientation %.3f deg" % ang_deg(R_cmd, R_meas))
        show("FK(meas) - USD panda_hand", p_meas - ee_p)   # sanity: should be ~0
except Exception as exc:
    print("  joint read failed: %r" % (exc,))

print("-" * 78)
for prim in stage.Traverse():
    if prim.GetTypeName() == "PhysicsScene":
        g = UsdPhysics.Scene(prim)
        print("  %s gravity dir=%s mag=%s" % (prim.GetPath(), g.GetGravityDirectionAttr().Get(),
                                              g.GetGravityMagnitudeAttr().Get()))
nograv = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT))
          if p.GetAttribute("physxRigidBody:disableGravity")
          and p.GetAttribute("physxRigidBody:disableGravity").Get()]
print("  arm links with gravity disabled: %s" % (nograv or "none"))

vis = stage.GetPrimAtPath(EE_PATH + "/visuals")
if vis.IsValid():
    off = np.array(Gf.Transform(UsdGeom.Xformable(vis).GetLocalTransformation()).GetTranslation())
    print("  panda_hand/visuals local offset %s  (non-zero => ghost drawn off the ee frame)"
          % np.round(off * 1e3, 3))
    print("  panda_hand/visuals visibility: %s" % UsdGeom.Imageable(vis).ComputeVisibility())
tp = stage.GetPrimAtPath(TARGET)
if tp.IsValid():
    print("  /World/target visibility: %s  children: %s"
          % (UsdGeom.Imageable(tp).ComputeVisibility(), [c.GetName() for c in tp.GetChildren()]))
print("=" * 78)
```

Reading it: a large `FK(cmd) − target` means cuRobo itself is short; a negative
`dz` in `FK(meas) − FK(cmd)` with the biggest per-joint errors on the
gravity-loaded joints means PD tracking error; `FK(meas) − USD panda_hand` must
be ~0 or the FK chain assumption below is wrong and everything else needs
re-reading.

## Probe 2: decaying transient, or steady state?

Samples over ~5s of physics steps without blocking the app (`time.sleep()` in the
Script Editor would stall the whole app, so physics would never advance). Also
prints the applied **velocity** targets — the prime suspect — and the live gains.

```python
# ===== follow-up: decaying transient, or steady state? =====
import numpy as np, omni.usd, omni.physx
from pxr import Usd, UsdGeom, Gf
from isaacsim.core.prims import SingleArticulation

ARM = ["panda_joint%d" % i for i in range(1, 8)]
art = SingleArticulation(prim_path="/World/Franka", name="mefron_diag2", reset_xform_properties=False)
art.initialize()
names = list(art.dof_names)
idx = [names.index(n) for n in ARM]

act = art.get_applied_action()
pos_t = np.array(act.joint_positions, dtype=float)
vel_t = None if act.joint_velocities is None else np.array(act.joint_velocities, dtype=float)
kp, kd = art.get_gains()
print("applied POS target:", np.round(pos_t[idx], 5))
print("applied VEL target:", None if vel_t is None else np.round(vel_t[idx], 5))   # <-- the key line
print("kp:", np.round(np.array(kp, dtype=float)[idx], 1))
print("kd:", np.round(np.array(kd, dtype=float)[idx], 1))

stage = omni.usd.get_context().get_stage()
def _wpos(path):
    return np.array(Gf.Transform(UsdGeom.Xformable(stage.GetPrimAtPath(path))
                    .ComputeLocalToWorldTransform(Usd.TimeCode.Default())).GetTranslation())
tgt = _wpos("/World/target")

_rows, _state = [], {"n": 0, "done": False}

def _on_step(dt):
    if _state["done"]:
        return
    n = _state["n"]
    _state["n"] += 1
    if n % 15 == 0:
        v = np.array(art.get_joint_velocities(), dtype=float)[idx]
        e = np.degrees(np.array(art.get_joint_positions(), dtype=float)[idx] - pos_t[idx])
        d = _wpos("/World/Franka/panda_hand") - tgt
        _rows.append((n, float(np.max(np.abs(v))), float(np.max(np.abs(e))),
                      d[2] * 1e3, float(np.linalg.norm(d)) * 1e3))
    if n >= 300:
        _state["done"] = True
        print("  step   max|vel|   max|err|deg    ee dz(mm)    |ee-tgt|(mm)")
        for r in _rows:
            print("  %4d  %9.4f  %12.3f  %+10.2f  %12.2f" % r)
        print("  (release with:  _MEFRON_DIAG_SUB = None )")

_MEFRON_DIAG_SUB = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)
```

## Verified offline FK chain

Reproduces `mefron.usd`'s baked `/World/target` retract seed to 1e-5, so it can
be trusted for offline checks without launching Isaac. Per joint: `(child xyz,
parent-frame X rotation)`, then `Rz(q)`.

```
(0,0,0.333) 0 | (0,0,0) -pi/2 | (0,-0.316,0) +pi/2 | (0.0825,0,0) +pi/2
(-0.0825,0.384,0) -pi/2 | (0,0,0) +pi/2 | (0.088,0,0) +pi/2
then panda_joint8 xyz (0,0,0.107), then panda_hand Rz(-0.785398163397)
retract_config = [0, -1.3, 0, -2.5, 0, 1.0, 0]   (franka.yml, minus the finger joints)
```

## Reading `mefron.usd` read-only from a shell

No Isaac app needed, and it never writes — useful for checking what is actually
baked into the scene versus what the code authors at runtime:

```bash
cd /isaac-sim/kit/python/bin && \
PYTHONPATH="/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311:/isaac-sim/extscache/omni.kit.pip_archive-0.0.0+69cbf6ad.lx64.cp311/pip_prebundle" \
LD_LIBRARY_PATH="/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311/bin" ./python3 - <<'PY'
from pxr import Usd, UsdGeom, UsdPhysics, Gf
s = Usd.Stage.Open('/workspace/isaac-cobot/assets/mefron/factory floor/mefron.usd')
PY
```

## Side finding: the target's yaw is diagonal, and always was

Raised in the same session ("the yaw is cross as opposed to being parallel or
perpendicular") and parked, but the cause is settled — it is **not** a
regression and nothing the ATC did:

- `franka.yml`'s `ee_link` is `panda_hand`, and `franka_panda.urdf`'s
  `panda_hand_joint` twists it **rpy `0 0 -0.785398`** (−45° about Z) from
  `panda_link8`, with `xyz="0 0 0"`. So the target frame — and with it the
  finger-open axis — is inherently 45° off the world axes.
- `mefron.usd`'s baked `/World/target` seed is exactly the retract-config FK:
  translate `(2.735841, -4.701985, 1.400045)`, Euler XYZ ≈ `(-171.8, 8.1, +45.6)`.
  Jogging in 90° steps from a 45.6° seed therefore stays diagonal forever.
- `MOUNT_ORIENTATION_WXYZ` has always been identity, on `atc` and `assembly`
  both (`git log -S` finds only the original commit), so no mount change caused it.
- If it is ever revisited: axis-align the target seed (or add a
  snap-yaw-to-nearest-90° key); or yaw the mount +45° so the arm's natural frame
  is world-aligned; or leave it, since J/B already snap to part-relative Grasp
  Editor poses that are square to the part by construction.

## Side finding: the gripper's dock frame is exact

Checked because a 45° dock error was a candidate for the yaw complaint. It is
not — the dock geometry is right by construction:

- `/World/gripper_tool_visual_only/panda_hand`'s local xform is **identity**
  relative to the tool root, because `robot._HAND_ONLY_URDF_TEMPLATE`'s
  `panda_hand_joint` is `rpy="0 0 0" xyz="0 0 0"` (unlike the real arm's).
- The dock joint is `panda_link8` → `{tool}/panda_hand` with `localRot1` =
  `TOOL_CHANGER_GRIPPER_HAND_JOINT_LOCAL_ORIENTATION_WXYZ` (+45° Z). Both joint
  frames must coincide, so `W_link8·I = W_toolhand·R(+45°)` ⟹
  `W_toolhand = W_link8·R(−45°)` — exactly the arm's own `panda_hand` frame.
- Because that local xform is identity, it makes no difference whether PhysX
  binds body1 to `panda_hand` itself or walks up to the tool root's runtime
  `RigidBodyAPI`. (In `mefron.usd` only the two fingers carry `RigidBodyAPI`;
  the root's is applied at runtime by `spawn_dockable_tool()`.)
- Baked world positions, for reference: gripper tool `(2.19284, -4.29841, 1.19474)`,
  `tool_rack (2.45501, -4.01482, 0.90376)`, suction `(2.54501, -4.21625, 1.20376)`,
  screwdriver `(2.36501, -4.21625, 1.20376)`, `screw_presenter (2.82676, -4.19091, 0.89376)`,
  `main_holder (3.14315, -4.78217, 0.99968)`.
- `mefron.usd` also still carries stale importer bake noise: `/World/target2`,
  `/World/target3` (unresolved references to `/World/Franka2|3/panda_hand/visuals`)
  and a `/panda` prim — the side effect CLAUDE.md documents.

## Next steps

1. Run [Probe 2](#probe-2-decaying-transient-or-steady-state); record the
   `applied VEL target` line and the decay table.
2. Apply the matching row of the fix table. Most likely a combination of holding
   the final waypoint at zero velocity and tightening the arrival gate — both in
   `scripts/mefron_lib/teleop.py` (`_step_arm()`'s `cmd_plan` block and the
   `robot_static` computation), with the threshold in `config.py`.
3. Re-run [Probe 1](#probe-1-target-vs-achieved-ee) and confirm
   `FK(meas) − FK(cmd)` collapses. The pass/fail number is `|ee − target|`:
   **3.09mm** today, measured at both a tucked and an extended pose (an extended
   pose is the harder case, since gravity torque scales with reach).
4. Worth promoting both probes to `scripts/mefron_arrival_probe.py`, alongside
   the existing `mefron_gripper_probe.py` / `mefron_screen_approach_probe.py`.
