# Per-part conveyor feeders

How `scripts/mefron_lib/feeder.py` works, and the measurements behind its
constants. Current state and keys live in `CLAUDE.md`.

## What it does

`mefron.usd` gives each sub-part its own 1 m belt (`ConveyorBelt_A06_02…06`),
with GUI-placed copies queued behind the one at the pick spot. Each belt is a
feeder: 5 s after its part leaves the pick spot, the belt runs until a
light-beam photo-eye says the next copy has arrived, then ramps to a stop.

`ConveyorBelt_A24` (the jig + `main_holder`) keeps its manual `1` toggle and is
not a feeder. `ConveyorBelt_A06_01` — wide, empty, end-to-end upstream of A24 —
is untouched.

Nothing keys off the arm. The 5 s countdown starts when the **beam clears**,
which is how a real feeder works and keeps `feeder.py` decoupled from the
gripper, exactly like `conveyor.py`. It also gives one behaviour for free: while
the tool is standing in the station it breaks the beam too, so the belt will not
run under the gripper.

## Belt geometry, measured

- All A06 belts run along world Y, from y≈−5.14 (front, robot side) to y≈−6.14.
  Belt surface top z = 0.8938. `<belt>/Belt` is a kinematic `RigidBodyAPI` with a
  μ=0.9 physics material — identical on all five (and on A24).
- **Belt-local +X maps to world −Y**, the opposite of A24 (whose +X → +Y). So
  feeding *toward* the robot is local −X: `FEEDER_LOCAL_VELOCITY_DIRECTION =
  [1,0,0]` with a **negative** `FEEDER_SPEED`. Confirmed live.
- `FEEDER_SPEED` is **belt-local, not m/s**. These belts carry a 0.5 scale on the
  travel axis, so −0.15 is ≈0.075 m/s in world (measured: 0.0248 m/s came out of
  a −0.05 command). `feeder.belt_travel_scale()` reads that factor off the belt's
  own transform, so `FEEDER_MAX_TRAVEL` stays in real metres.
- The parked parts sit only **2.8–5.0 cm** from the belt's front edge
  (`backpanel_support` is tightest). That is the entire overshoot budget, and it
  is why the belt runs slowly and ramps rather than stepping its velocity.

## Two live-confirmed traps

**1. `PhysxSurfaceVelocityAPI` must exist before Play.** The `IsaacConveyor` node
applies the API itself when it first computes, but PhysX may never resync a rigid
body it has already created — the attribute reads correctly in USD while the belt
drives nothing. This was **nondeterministic**: two identical runs disagreed, and
within one run 3 of 5 belts worked. `feeder.prime_surface_velocity()` applies the
API (enabled, zeroed) at setup, before the timeline ever plays, so the body is
created with it. Suspect this first if a belt silently stops driving.

**2. A resting part is asleep, and a kinematic belt does not wake it.** A part
that has sat still for a second is asleep to PhysX. Turning a kinematic belt's
surface velocity on is not a contact event, so the part just sits there while the
belt runs under it. `feeder._wake_bodies()` calls
`get_physx_simulation_interface().wake_up()` on the belt's queue every frame
while feeding and stopping — not just at the start, since a part that stalls
against the one ahead falls asleep again. This is the same class of bug as
CLAUDE.md's "a held object hanging in mid-air may just be a sleeping body".

## The photo-eye

One `IsaacLightBeamSensor` per belt, under the script-owned
`/World/feeder_sensors` scope, rebuilt every run. All of its geometry is derived
from live bboxes, so re-placing a belt or a part in the GUI needs no code change:

```
mount_y   = belt_front_face_y + FEEDER_BEAM_MOUNT_STANDOFF
trip_y    = front_part_bbox_max_y          # the GUI-authored parked leading face
min_range = mount_y - trip_y - FEEDER_BEAM_PRETRIP
max_range = min_range + FEEDER_BEAM_DEPTH
```

On this scene that gives 3.6–5.2 cm windows. It is mounted on the belt's front
frame aiming **back down the belt**, not across it. Along-belt is what makes the
short range work: there is no far side rail in the ray's path to false-trigger
on, and the shallow window keeps the queued parts behind the station invisible,
so an empty station reads *no hit* with no background calibration.

**Stop on depth, not on the trip.** A bare `beam_hit.any()` fires a whole
`FEEDER_BEAM_DEPTH` early, which lands the part up to 8 cm short — and
`main_holder_back_cover`'s parked pose is already 0.826 m from the mount against
the Panda's ~0.855 m envelope, so short means *unreachable*. Since the sensor
clamps any reading closer than `minRange` to `minRange`, "depth is at minRange"
means "the leading face is at the pick spot". That is ordinary
background-suppression photo-eye behaviour, and it lands every belt within 9 mm.

**The curtain is fine on purpose.** Measured leading faces sit anywhere from 3 mm
(`PCB_Assembly`'s board) to 33 mm (the back cover) above the belt. 5 rays over
30 mm let the PCB's board slip *between* rays, so the belt overshot 8 cm and
pushed it off the front edge; 24 rays over 40 mm from 1 mm up (1.7 mm apart)
catches all five. Rays run **up from the origin**, not centred on it — a centred
curtain would put half of them under the belt and trip permanently, which does
not happen.

**Pose fail-safe.** `backpanel_support` and `finger_print_scanner` have a notch at
the ray line, so their nearest material sits a few mm behind the leading face and
the depth test can miss. `PartFeeder._overshot_station()` stops the belt when the
arriving part's **live pose** reaches the authored pick spot, and logs a WARNING
naming the part. It reads the pose rather than dead-reckoned travel because parts
slip on the belt by ~20% — a dead-reckoned cut-off stopped `backpanel_support`
5 cm short. The photo-eye stays the primary sensor; this only catches what its
rays miss. `FEEDER_MAX_TRAVEL` remains as the empty-belt cut-off.

**Visuals.** The sensor's own `drawLines`/`drawPoints` debug draw is a few-cm line
1 mm above the belt, under the part and behind the end roller — invisible in
practice, and only drawn while playing. `feeder._build_beam_visual()` adds a
barrel housing and a beam rod along the real ray. Both are **collider-free**, or
the beam's own raycast would hit them.

## Queue discovery and the instance resolver

A part's copies are `/World` children matching `^<base>(_\d+)?$` (Ctrl+D's own
naming) that carry a `RigidBodyAPI`. The regex, not a bare prefix, is required:
`main_holder` prefixes both `main_holder_back_cover` and `main_holder_jig`.
`belt_queue()` keeps the ones whose live pose is inside the belt's XY footprint
and within `FEEDER_QUEUE_HEIGHT_TOLERANCE` of its surface, front (largest Y)
first. It is recomputed from live poses, so a lifted or assembled part drops out
by itself — no "consumed" bookkeeping, the same principle as
`release_assembly_weld()`'s "the joint prim's presence IS the state".

Because a pick can now land on any copy, every **live pose read** resolves
through `feeder.py`:

- `resolve_for_pick(base)` — the latched copy if the arm is holding one, else
  whatever is at the station. `latch()` fires on a grasp/suction approach key and
  pins the whole grasp → place → weld cycle to that copy, which is necessary
  because a lifted part leaves `belt_queue()` and nothing else could still name
  it. Identity for any path with no feeder, which is what keeps each call site a
  one-liner.
- `resolve_assembled(base)` — the copy already built into the assembly. Distinct
  from the pick resolution and both are needed at once: once
  `main_holder_back_cover_01` is fitted, the screw holes and any mount pose must
  read `_01` while `K` retargets to the *next* copy on the belt.

The rule at the call sites: **`part_prim_path` resolves for pick,
`mount_prim_path` resolves assembled** — except the `suction_gripper_approach_on_*`
entries where `mount_prim_path == part_prim_path` on purpose, which use the pick
resolution for both. `grasp.relationship_pose_prim_paths()` is the one place that
decides this. Config-to-config lookups that compare base paths are untouched.

A welded copy is excluded from `belt_queue()`, and `release_assembly_weld()` calls
`forget_assembled()` to put it back — otherwise an un-welded part would stay
excluded forever and no key could ever pick it up again.

One consequence of the queue: a grasp key no longer reaches an **already
assembled** copy, because it resolves to the next copy on the belt. Taking an
assembled part back off is not a keyed action any more.

## Testing

`scripts/test_mefron_feeder_headless.py --headless` (no arm, no cuRobo,
`--belt=<part name>` for one belt). Per belt it asserts the graph and photo-eye
come up, the beam sees the parked part and clears when it is removed, the resolver
splits pick vs assembled correctly, and that after an advance the next copy lands
within `_PICK_SPOT_TOLERANCE` of the authored pick spot and is still on the belt.
Belts with only one copy are logged and fall through to the empty-belt cut-off
rather than silently passing.

One harness-only wrinkle: it fakes a pick by making the part kinematic and
teleporting it. `SingleRigidPrim.set_world_pose()` writes the PhysX/Fabric pose,
and nothing writes a *kinematic* body's pose back to USD — which is what
`belt_queue()` reads — so the test writes the USD xform too. A real gripper lift
moves a dynamic body, whose simulated transform does get written back.
