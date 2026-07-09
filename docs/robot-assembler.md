# Robot assembler: snap-to-pose research

Research/design notes only — nothing here is implemented yet. Written in
response to a real limitation: the teleop loop in `scripts/mefron.py` can
carry `finger_print_scanner` physically close to its assembled pose on
`main_holder`, but manual dragging can't hit the exact target pose (from
`docs/grasp-and-assembly-offsets.md`'s T_H_S) — assembly needs a mechanism
that "snaps" the part into the exact pose once it's within some position/
orientation tolerance of the target. Framed as a general capability since
`screen` and `backpanel_support` will need the same treatment later, not
just `finger_print_scanner`.

## Candidate approaches

### 1. Runtime PhysX joint, created once within tolerance (recommended)

Create a `UsdPhysics.FixedJoint` (or a D6 joint for more control) between
`finger_print_scanner` and `main_holder` the moment the live pose is
within tolerance of the target. The joint's local frames should be built
from the **target** pose, not the part's current (slightly-off) pose —
this is the actual "snap": PhysX corrects a joint's bodies to satisfy its
frames on the next physics step, so building the joint from the target
pose (rather than wherever the part happens to be) is what pulls the part
the rest of the way in, rather than freezing it at its imprecise
approach pose.

A D6 joint's drive stiffness/damping can spring that residual correction
in smoothly instead of an instant teleport-like pop, if that reads better
than a hard snap.

**Real gotcha to design around**: this joint would exist alongside cuRobo/
the gripper's own active drives holding the part in the gripper. Two
rigid constraints on the same body fight each other (a closed kinematic
loop) — the gripper's grasp needs to be released or softened once the
assembly joint takes over, not run simultaneously at full stiffness.

### 2. Isaac Sim's `isaacsim.robot_setup.assembler` ("Robot Assembler")

A real, current Isaac Sim 5.1.0 extension (Tools > Robotics > Asset
Editors > Robot Assembler) that joins two assets with a PhysX fixed
joint via a `RobotAssembler` Python API
(`begin_assembly()`/`assemble()`/`finish_assemble()`). **Not a fit as-is**:
it's authoring-time only — parts must already be manually nudged into
place (only 90°-increment rotation helpers, no fine auto-alignment) while
the timeline is stopped, and it's meant to permanently rigidize a pair
(e.g. welding a tool onto an end-effector), not to be re-triggered
automatically at runtime as a carried part drifts into tolerance. Worth
knowing about as adjacent prior art / a possible source of API patterns
(it does exercise the same `FixedJoint`-creation path as approach 1), not
as a drop-in solution.

### 3. Plain kinematic teleport

Once within tolerance, just call `set_world_pose()` to snap the part
directly onto the target transform, no physics joint involved. Simplest
to implement, but the least physically grounded — the part would visibly
"pop" with no simulated seating motion, and it wouldn't stay put under
gravity without also fixing it to `main_holder` some other way afterward
(so it likely still needs approach 1 layered on top, just deferring the
question of *how* the pose gets corrected).

### 4. cuRobo `high_precision` mode (complementary, not a replacement)

cuRobo has no dedicated snap/seat primitive, but v0.7.1+ added a
`high_precision` mode to `MotionGenConfig`/the IK solver with sub-1mm
median convergence (per NVLabs' own changelog). Using this for the final
approach move would shrink the residual error any snap mechanism has to
correct — worth combining with approach 1 rather than treating as an
alternative to it, since it's still open-loop planning, not a compliant
"servo until seated" behavior on its own.

## Open questions

- How to define and check the position/orientation tolerance — reuse the
  existing `compute_relative_pose()`/T_H_S infrastructure
  (`docs/grasp-and-assembly-offsets.md`) to get "current relative pose vs.
  target relative pose," then threshold on translation distance and
  rotation angle (e.g. via quaternion dot product)?
- Instant snap vs. spring-smoothed correction via D6 drive tuning — which
  reads better for what this sim is for (VLA training data, per other
  docs in this repo) — a smooth correction may look more physically
  plausible than a pop.
- Exact sequencing: does the assembly joint engage before, after, or
  simultaneously with releasing the gripper's grasp? Simultaneous risks
  the closed-loop fight described above; sequencing them adds latency and
  its own risk of the part falling in between.
- Scope: implement this generically (any two named parts + a stored
  relative pose) now, or hard-code it for
  `finger_print_scanner`/`main_holder` first and generalize once it works?

## Recommendation

Prototype approach 1 (runtime `FixedJoint`/D6 joint built from the target
pose, gripper released once it engages) as the primary mechanism, using
approach 4 (cuRobo `high_precision` mode) on the final approach move to
minimize how large a correction the joint has to make. Treat approaches 2
and 3 as reference points, not implementation paths.

## Sources

- Isaac Sim 5.1.0 Robot Assembler docs:
  `docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/assemble_robots.html`
- NVLabs/curobo `CHANGELOG.md` (`high_precision` mode, v0.7.1) and GitHub
  discussion #218.
- IsaacLab community GitHub discussions #4189, #4088, #2472 (runtime
  `UsdPhysics.FixedJoint` creation patterns and the live-pose-at-creation
  gotcha) — community-sourced, not official docs; verify against current
  IsaacLab/Isaac Sim behavior before relying on the exact API calls.
- NVIDIA Developer Forums thread "Snap-fit simulation: Object appears too
  soft" — confirms snap-fit is a known simulated scenario, framed as a
  physical-compliance problem rather than a scripted snap.
