# OmniGraph adoption for mefron: GUI-editable tunables, native-node keyboard dispatch, and grasp/place behavior cleanup

> Planning document only — nothing described below has been implemented yet.
> This captures the outcome of an architecture investigation into whether
> Isaac Sim's OmniGraph can reduce the hardcoding/boilerplate in
> `scripts/mefron_lib/`, and what specifically is and isn't worth doing.

## Context

The user flagged that `scripts/mefron_lib/config.py` (~150 lines of flat Python
constants) and the keyboard-control boilerplate in `teleop.py`/`conveyor.py`
make the mefron scripts "scripted and hardcoded, not scalable or practical,"
and asked whether Isaac Sim's OmniGraph is the right tool to fix that. A
follow-up question asked specifically whether "grasp object"/"place object"
are common enough behaviors to become reusable boilerplate — "like a behavior
tree" — instead of ad hoc per-object handlers; that investigation (section
near the end of this plan) turned up a real, separate, small win plus a
significant **correction to this project's own documentation**.

After investigating (two rounds of research + two independent Plan-agent
designs, one deliberately arguing the maximal "push everything into the
graph" case), the concrete goal the user confirmed is **not** "replace the
Python control loop with a graph" — it's:

1. Minimize hardcoding by making tunable values (gripper speed, assembly lift
   height, conveyor speed, etc.) **visible and editable in the GUI** instead
   of buried in `config.py`, requiring a code edit + rerun to change.
2. Audit which of the current code's duplicated boilerplate (the keyboard-
   subscription pattern repeated 5x) can be **wrapped as reusable OmniGraph
   nodes** instead of hand-rolled Python, using cuRobo-in-its-own-node as the
   kind of thing they had in mind.

A hard fact grounds everything below: **cuRobo motion planning and the
grasp/assembly pose math in `grasp.py` have zero OmniGraph node support in
this Isaac Sim 5.1.0 install** — confirmed by an exhaustive `.ogn` file search
across the entire install plus a grep for "curobo" in every `.py`/`.ogn` file
(zero hits; cuRobo exists only as a plain pip package). Wrapping `plan_single()`
itself in a Script Node or a custom compiled node was evaluated in depth and
explicitly rejected: it would relocate identical Python code behind new
indirection (a Script Node's `compute()` still just calls the same functions)
while adding a genuinely untested failure surface (a long-lived CUDA/torch
context inside a node lifecycle, no prior art anywhere in this install) on top
of an environment that's already fragile (broken `pip`, hand-patched `ninja`
for cuRobo's CUDA kernels). That conclusion is unchanged by the user's
clarified goal — but the clarified goal turns out to be achievable much more
cheaply, via a mechanism this repo already uses for something else.

**The existing (uncommitted, in-progress) `conveyor.py` migration is the
reference pattern.** It builds `isaacsim.asset.gen.conveyor`'s `IsaacConveyor`
node once via `omni.kit.commands.execute("CreateConveyorBelt", ...)`, then
drives it at runtime purely from Python by writing into a **graph variable**:
`stage.GetAttributeAtPath(f"{graph_path}.graph:variable:Velocity").Set(...)`.
That `graph:variable:*` mechanism — confirmed by reading `omni.graph`'s and
`omni.graph.window.core`'s own source — is exactly, and cheaply, the thing
that makes a value GUI-visible/editable: any `graph:variable:<Name>` attribute
on a graph's root prim is auto-populated into the Action Graph editor's
dedicated **Variables** panel, and (independently) is also just an ordinary
USD attribute editable through the plain Property window if you select the
prim. Reading one from Python (`.Get()`) is exactly as easy as writing one —
conveyor.py just never needed to read one back. This is the mechanism both
phases below build on.

## What stays Python, permanently (with reasons)

- **cuRobo (`teleop.setup_motion_gen()`, `_step_arm()`'s planning/debounce
  block, `motion_gen.plan_single()`/`warmup()`/`get_full_js()`)** — no node
  equivalent exists; relocating the call site changes nothing about its
  constraints (blocking warmup, base-link-frame poses, wall-clock-paced
  playback) and adds risk for zero benefit.
- **`grasp.py`'s pose math** (`compute_relative_pose`, `compute_dependent_world_pose`,
  `compute_grasp_approach_pose_from_file`, etc.) — no YAML-reading or
  arbitrary-reference-frame composition node exists; this logic runs once per
  keypress, not per-frame, so there's no cadence problem a graph would solve.
- **`config.py`'s provenance-coupled constants** — must not become casually
  GUI-editable number fields, because they're derived from a specific live-
  measurement procedure and silently editing them without redoing that
  procedure reintroduces exactly the bugs their own comments warn about:
  - `ASSEMBLY_RELATIONSHIPS` (all 5 entries) — each has a comment describing
    the specific hand-jogging/measurement session that produced it.
  - `GRASP_TARGETS[*]["yaml_path"]`/`["grasp_name"]`/`["part_prim_path"]` —
    tied to specific NVIDIA Grasp Editor YAML exports; a freeform GUI field
    has no way to validate the path/name pair still matches.
  - `SURFACE_GRIPPER_APPROACH_CLEARANCE` — the file states outright that
    changing it alone does nothing without re-baking a relationship's z.
  - `MOUNT_POSITION`/`MOUNT_ORIENTATION_WXYZ` (+ `_2`/`_3` variants) — dual-
    consumed by both the physical mount (`robot.mount_franka()`) and cuRobo's
    kinematic base frame; editing one without the other desyncs them silently.
  - All structural identifiers (`*_PRIM_PATH`/`*_PRIM_NAME`/`*_USD`,
    `GRIPPER_JOINT_NAMES`, `OBSTACLE_PRIM_PATHS`, `FULL_EXPERIENCE_EXTRA_EXTENSIONS`,
    etc.) — not tuning knobs, not meaningfully graph-variable material.
- **Drive/friction physics constants** (`FRANKA_DRIVE_STRENGTH/DAMPING`,
  `GRIPPER_DRIVE_STIFFNESS/DAMPING`, `GRIPPER_STATIC/DYNAMIC_FRICTION`) —
  no OmniGraph work needed at all: once `robot.py` authors these onto the
  joint/material prim's `UsdPhysics.DriveAPI`/`PhysxSchema.PhysxMaterialAPI`,
  they're already ordinary, already GUI-editable USD attributes that PhysX
  reads live. `config.py`'s constants here are just the initial build-time
  value.

## What moves to OmniGraph

### Phase 0 (already done, reference pattern) — `conveyor.py`

No action. This is what every phase below imitates: author a graph variable
once via `og.Controller`/kit commands, drive/read it via a plain
`stage.GetAttributeAtPath(...)` call, treat "who owns writing a given
attribute and how often" as a single deliberate responsibility (the documented
lesson from this file's own two past failure modes: a node silently defaulting
to disabled, and per-frame reassertion halting motion).

### Phase 1 — GUI-editable tunables graph

New module `scripts/mefron_lib/tunables.py`:

```python
TUNABLES_GRAPH_PATH = "/World/TeleopTunables"

def setup_tunables_graph() -> None:
    """One-time. Bare prim holding graph:variable:* attributes -- no OnTick/
    exec wiring needed, unlike conveyor.py's graph: the consumer here is
    Python pulling on its own schedule, not a native per-tick node."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": TUNABLES_GRAPH_PATH, "evaluator_name": "execution"},
        {keys.CREATE_VARIABLES: [
            ("GripperCloseSpeed", "float", config.GRIPPER_CLOSE_SPEED),
            ("GripperOpenPosition", "float", config.GRIPPER_OPEN_POSITION),
            ("GripperClosedPosition", "float", config.GRIPPER_CLOSED_POSITION),
            ("AssemblyLiftHeight", "float", config.ASSEMBLY_LIFT_HEIGHT),
            ("TeleopVelocityScale", "float", config._TELEOP_VELOCITY_SCALE),
            ("TeleopAccelerationScale", "float", config._TELEOP_ACCELERATION_SCALE),
            ("TeleopTimeDilationFactor", "float", config._TELEOP_TIME_DILATION_FACTOR),
            ("ConveyorSpeed", "float", config.CONVEYOR_SPEED),
            ("ConveyorJigForwardY", "float", config.CONVEYOR_JIG_FORWARD_Y),
            ("ConveyorJigBackwardY", "float", config.CONVEYOR_JIG_BACKWARD_Y),
        ]},
    )

_tunable_cache: dict[str, "Usd.Attribute"] = {}

def read_tunable(name: str, default: float) -> float:
    """Falls back to `default` (pass the matching config.py constant) if the
    variable is missing/invalid, so config.py stays the documented reference
    for what each knob defaults to even after this migration."""
    attr = _tunable_cache.get(name)
    if attr is None:
        stage = omni.usd.get_context().get_stage()
        attr = stage.GetAttributeAtPath(f"{TUNABLES_GRAPH_PATH}.graph:variable:{name}")
        if not attr or not attr.IsValid():
            return default
        _tunable_cache[name] = attr
    return float(attr.Get())
```

Wire into the exact existing per-frame/per-keypress read sites (nothing else
about their logic changes):
- `teleop.py` gripper ramp block (~L665, uses `GRIPPER_CLOSE_SPEED`) and the
  ramp target selection (`GRIPPER_OPEN_POSITION`/`GRIPPER_CLOSED_POSITION`).
- `teleop.py`'s P-key assembly-lift waypoint snap (~L431, uses
  `ASSEMBLY_LIFT_HEIGHT`) — already re-read fresh on every keypress, trivial
  substitution.
- `teleop.py`'s per-arm `MotionGenPlanConfig`/`MotionGenConfig` construction
  for `_TELEOP_TIME_DILATION_FACTOR` (genuinely hot — cheap object, re-read
  per `plan_single()` call is safe) vs `_TELEOP_VELOCITY_SCALE`/
  `_TELEOP_ACCELERATION_SCALE` (baked into `MotionGenConfig` at construction;
  changing these still requires that arm's `MotionGen` to be rebuilt +
  rewarmed — comment this distinction explicitly in code so it's not assumed
  live when it isn't).
- `conveyor.py`'s `ConveyorControl.step()`/`request_toggle()` (`CONVEYOR_SPEED`,
  `CONVEYOR_JIG_FORWARD_Y`, `CONVEYOR_JIG_BACKWARD_Y`).

Call `tunables.setup_tunables_graph()` from `mefron.py` alongside the existing
`conveyor.setup_conveyor_belt_graph()` call.

**First validation step, before extending further:** confirm a
`graph:variable:*` attribute is actually visible/editable through the plain
Property window in this project's real launch mode (`SimulationApp`, not a
full GUI-authored session) — the devcontainer docs mark GUI/X11 reachability
"needs verification," and this whole phase's payoff depends on the GUI being
reachable at all. Do this on a scratch copy before wiring in all ten
variables.

### Phase 2 — native-node keyboard dispatch (the actual "wrap boilerplate as nodes" win)

The 5x-duplicated block (`teleop.py`'s `GripperKeyboardControl`/
`SuctionApproachControl`/`AssemblyPlacementControl`/`SurfaceGripperKeyboardControl`
builders + `conveyor.py`'s `build_conveyor_control`, all doing identical
`get_keyboard()` → `acquire_input_interface()` → `subscribe_to_keyboard_events()`)
is the strongest genuine candidate in the whole audit — and Isaac Sim ships a
compiled, NVIDIA-maintained node for exactly this, with a working shipped
example to copy (`isaacsim.examples.interactive`'s `omnigraph_keyboard.py`
builds a keyboard-driven graph with per-instance `ReadKeyboardState` nodes,
zero custom node code).

Design: one `ReadKeyboardState` node instance per existing key (J, B, K, P, C,
O, N, V, L, KEY_1), each with its own `inputs:key` set independently (a
dropdown in the GUI Property panel — **this is the concrete realization of
"change it in the GUI": rebinding an action to a different physical key
becomes editing that node instance's `inputs:key`, not a `config.py` edit**),
feeding a `WriteVariable` node into its own `graph:variable:<Name>Pressed`
bool. Python's existing per-frame loop reads that bool via the same
`read_tunable`-style mechanism from Phase 1 and edge-detects (compare this
frame's value to last frame's — the same shape of state machine
`ConveyorControl` already implements) to fire the exact same handler body each
control class already has (`request_grasp_approach_from_file`,
`request_assembly_target`, `set_closed`, `request_toggle`, etc.) — only the
"was this key just pressed" signal changes source, not what happens next.

This is deliberately **not** a Script Node wrapping the `carb.input` call
(that would reinvent plumbing NVIDIA already ships as a compiled node, and
creates a new "reach a live Python object from inside a node" problem) and
**not** a plain shared Python helper function (that dedupes code but doesn't
make key bindings GUI-visible/editable, which is the user's actual ask). Do
this only after Phase 1 has validated the graph-variable-read mechanism live.

## Grasp/place behavior generalization ("like a behavior tree")

This is orthogonal to the OmniGraph work above — it's about Python-level
behavioral reuse, not graph nodes — but addresses a real follow-up question:
are "grasp an object" and "place an object" common enough to become one
reusable, parameterized behavior instead of per-object handlers?

**Correction to this project's own documentation, found while investigating
this:** `CLAUDE.md` and `docs/grasp-and-assembly-offsets.md` currently state
that the P (place) key is hardcoded to one relationship
(`finger_print_scanner_on_main_holder`) and "not yet generalized to
`backpanel_support`." **This is stale.** Reading the current code
(`teleop.py`) shows P is already generic for arm 1: `GripperKeyboardControl`
tracks `self.last_grasped_object` (set on every J/B/K press), and the P
handler in `_step_arm()` (~L512-541) does a reverse lookup —
`config.GRASP_TARGETS[last_grasped_object]["part_prim_path"]` matched against
`config.ASSEMBLY_RELATIONSHIPS` entries by `part_prim_path` — to find the
right relationship, then routes through the one shared
`_snap_target_to_assembly_lift_waypoint()` function regardless of which object
was grasped. This already covers all 3 of arm 1's `GRASP_TARGETS` entries
(`finger_print_scanner`, `backpanel_support`, `pcb_assembly`) with zero
per-object branching — it's already the exact "reusable parameterized
behavior" the question was asking about. `grasp.py`'s
`relationship_name: str = "finger_print_scanner_on_main_holder"` default
arguments (lines 78, 90, 101) are dead code left over from before this
generalization — every real call site passes an explicit value.

**What's actually left (small, concrete):**

1. **Update `CLAUDE.md` and `docs/grasp-and-assembly-offsets.md`** to
   describe the `last_grasped_object` reverse-lookup mechanism accurately, so
   this doesn't get re-litigated in a future session. Remove the dead default
   arguments in `grasp.py`.
2. **Optional dedup**: the two P-handling code paths in `_step_arm()` — arm
   1's `gripper_control` branch (~L503-556) and arm 2's `assembly_control`
   branch (~L574-589, statically wired to `"screen_on_main_holder"` in
   `mefron.py` L191, since arm 2's suction "grasp" has no multi-object dict to
   reverse-lookup from) — are structurally parallel but separately coded, and
   this already caused the *same* "don't fire on stale grip state" bug to be
   fixed twice in two separate commits (`a19b672`, `abbe511`). Worth
   extracting a shared `_maybe_handle_placement_request(state, target,
   ee_link_prim_path, relationship_name, is_holding)` helper both branches
   call into, with each branch only computing its own `is_holding` (arm 1:
   `gripper_control.closed and last_grasped_object is not None`; arm 2: live
   `surface_gripper_control.is_closed()`). Nice-to-have, not urgent.
3. **Forward-looking, not needed now**: if arm 2 or arm 3 ever need to
   grasp/place more than one object, replicate arm 1's existing
   `GRASP_TARGETS`-style dict + reverse-lookup pattern for that gripper type —
   no new framework needed, just the same proven shape.
4. **Sequencing with the open `ASSEMBLY_LIFT_HEIGHT` bug** (CLAUDE.md's
   documented fixed-world-Z placement bug): no conflict — `_snap_target_to_
   assembly_lift_waypoint()` is already the single shared function every
   object/arm routes through, so fixing the lift-height math there (making
   clearance relative to current/final Z instead of an absolute world
   constant, per CLAUDE.md's own "next attempt" note) is already a one-place
   fix covering every relationship simultaneously. Fix that first (it's a
   real, well-diagnosed, high-value correctness bug blocking every placement
   attempt today); items 1-2 above can happen before, after, or in parallel
   since they touch disjoint code.

**Declined: NVIDIA's Cortex framework (`isaacsim.cortex.framework`) as a
behavior-tree-style engine for this.** Investigated concretely — its
decider-network API (`DfDecider`/`DfNetwork`/`DfState`/RLDS priority chains,
at `/isaac-sim/exts/isaacsim.cortex.framework/`) and the shipped
`franka_cortex`/`block_stacking_behavior.py` example do validate "one
parameterized pick/place behavior over many objects" as a real, working
pattern in the abstract — but it's the wrong shape for this project,
concretely:
- Its motion integration is built entirely on `isaacsim.robot_motion.motion_generation`'s
  `MotionPolicy` contract (`compute_joint_targets()`, called every physics
  tick) with `RmpFlow` as the shipped example's policy — there is no cuRobo
  hook anywhere in Cortex. Wiring `MotionGen.plan_single()` into that
  per-tick callback shape would mean re-implementing `_step_arm()`'s
  plan/debounce/waypoint-stepping logic underneath a different, framework-
  owned lifecycle — the same category of untested-integration risk as the
  already-rejected "host cuRobo in a Script Node" idea, for no functional gain.
- Cortex's whole value proposition is a decider network that **continuously,
  autonomously re-decides** what to do every tick from live world state. This
  project's interaction model is the opposite: a human drags a target or
  presses a key, and the existing debounce/plan/apply pipeline in
  `_step_arm()` reacts generically regardless of why the target moved — there
  is no "decide what to do next" problem here for Cortex to solve; the human
  already decided.
- It would add a second nontrivial, currently-unused framework on top of an
  already fragile stack (broken `pip`, hand-patched `ninja` for cuRobo's CUDA
  kernels) to solve a reuse problem the existing `GRASP_TARGETS`/
  `ASSEMBLY_RELATIONSHIPS` dict-driven pattern already solves for free.
- Separately confirmed: OmniGraph itself has no native behavior-tree/state-
  machine/decider node type anywhere in this install (only generic flow-
  control primitives — `Sequence`, `FlipFlop`, `Multigate` — with no
  precondition/blackboard/retry concept), so this isn't a "use the graph
  instead" option either. Moot regardless, since cuRobo has no graph hook.

## Custom `GrabObject`/`PlaceObject` nodes — investigated concretely, declined as pictured, with a concrete alternative

The user's next question was more specific still: build an actual `GrabObject`
node whose output is an object's coordinates, wired into a `PlaceObject` node
that consumes them — a real, visible graph topology, not a Python dict
lookup. This deserved its own investigation rather than folding it into the
Cortex/OmniGraph verdicts above, because it's a genuinely different, smaller-
scoped idea: unlike hosting `MotionGen` in a node (rejected for a real,
CUDA-lifecycle reason), a node that just reads a pose, composes a cheap
stateless offset, and writes a pose back carries none of that risk. Confirmed
concretely:

- **"Coordinates of object" needs zero custom code.** `omni.graph.nodes`
  already ships the full native chain: `GetPrimLocalToWorldTransform` (prim
  path in, `matrixd[4]` out) → `GetMatrix4Translation`/`GetMatrix4Quaternion`
  (position/orientation out). `teleop.build_teleop_target()`'s target prims
  (`/World/target`/`target2`/`target3`) are bare `Xform`s whose
  `xformOp:translate`/`xformOp:orient` *are* their world pose, so
  `WritePrimAttribute` is a faithful native "write coordinates" node too — no
  gap on the write side either.
- **Script Nodes can define real, named, typed input/output ports** —
  confirmed via `omni.graph.scriptnode`'s own test suite
  (`og.Controller.create_attribute(node, name, type, og.AttributePortType.INPUT)`,
  or the bulk `CREATE_ATTRIBUTES` form), callable purely from Python — the
  same `og.Controller.edit()` authoring pattern `conveyor.py` already uses,
  not a new paradigm.
- **A pure-Python custom `.ogn` node type carries none of the CUDA/pip/ninja
  risk** that made hosting cuRobo a bad idea — its code generator
  (`omni.graph.tools`) is pure Python with no subprocess/compiler step. The
  real cost is different: this repo has never authored or registered its own
  Kit extension (`extension.toml`) — every extension used today is an
  NVIDIA-shipped one enabled by name. Getting Kit to discover a first, local,
  custom extension from this project's standalone `SimulationApp` launch is
  realistic but genuinely untested, first-time packaging work.
- **The actual blocker: the pictured data flow doesn't match the real code.**
  `grasp.py`'s `compute_grasp_approach_pose_from_file()` and
  `compute_assembly_grasp_target()` take **no pose input** — each calls
  `SingleXFormPrim(...).get_world_pose()` internally, fresh, on every call
  (that's the explicit point of their live-recompute design). So there is no
  real producer→consumer relationship between "grab" and "place" today for a
  wire to represent — building the two nodes as pictured would mean either a
  decorative wire that doesn't drive the actual result (misrepresenting the
  real dependency, defeating the point), or first refactoring
  `compute_assembly_grasp_target()` to optionally accept a supplied grasp
  offset instead of always re-measuring it live.

**Decision: decline building the two nodes for now**, and do the parts that
are genuinely valuable on their own instead:

1. **Refactor `compute_assembly_grasp_target()`/`compute_part_target_pose()`
   in `grasp.py`** to accept an optional pre-measured grasp offset parameter
   instead of always re-measuring live. This is worth doing independent of
   OmniGraph entirely — it makes the grab→place relationship an explicit,
   named composition in Python, and is a prerequisite if node-wrapping is
   ever revisited.
2. **Drop a native `GetPrimLocalToWorldTransform` chain** on any part prim
   the user wants visible in the graph (e.g. `/World/finger_print_scanner`,
   `/World/screen`) — zero custom code, satisfies "see the object's
   coordinates in the graph" today.
3. Rely on Phase 1's graph-variable tunables (above) so the *parameters* of
   the grab/place relationship are GUI-visible/editable even though the
   control flow stays in Python.
4. **If, after step 1's refactor, real node-based wiring is still wanted**:
   build it as Script-Node-with-custom-ports (via `CREATE_ATTRIBUTES`), one
   instance per `GRASP_TARGETS`/`ASSEMBLY_RELATIONSHIPS` entry, `compute()`
   calling straight into the refactored `grasp.py` functions, using
   `matrixd[4]` as the shared port type (matches `GetPrimLocalToWorldTransform`'s
   own output, no custom struct needed) — not a standalone `.ogn` extension,
   given this repo's zero prior art registering its own extension. Treat this
   as optional follow-on decoration, not a required deliverable of this plan.

## Explicitly declined (with reasons, so nothing is silently dropped)

- Script-Node-hosted `MotionGen`/`plan_single()`, or any custom compiled
  `.ogn` node for planning or grasp math — same Python code behind new
  indirection, no prior art, adds risk to an already-fragile CUDA/JIT setup,
  no benefit Phase 1's graph-variable reads don't already capture more cheaply.
- `IsaacArticulationController` swapped in for the raw `apply_action()` calls
  in `_step_arm()` — evaluated in an earlier design pass as a way to get
  automatic Play/Stop handle-rebuild "for free" (it does have that, via
  `BaseResetNode`), but it answers a different question (Play/Stop robustness)
  than the one this plan is scoped to (hardcoding + GUI-editability +
  boilerplate). Noted here as a genuine, separate, optional follow-up if ever
  wanted, not part of this plan.
- Reviving `configs/scene/mefron_layout.yaml` as a unified config source —
  the user explicitly chose to track this as a separate follow-up, since it
  has its own unrelated blocking dependency (needs a fresh anonymous stage,
  conflicting with `mefron.py`'s direct-`open_stage()` requirement for
  grasp-offset derivation).

## Files touched

- New: `scripts/mefron_lib/tunables.py`
- Edit: `scripts/mefron_lib/config.py` (constants stay as documented defaults;
  add a short module-level note pointing at `tunables.py`), `scripts/mefron_lib/teleop.py`
  (swap the specific read sites listed above; Phase 2 replaces the 4 keyboard-
  control builders' subscribe blocks; optionally extract
  `_maybe_handle_placement_request()` per the behavior-generalization section),
  `scripts/mefron_lib/conveyor.py` (swap `CONVEYOR_*` read sites; Phase 2
  replaces `build_conveyor_control`'s subscribe block), `scripts/mefron.py`
  (call `tunables.setup_tunables_graph()` next to the existing
  `conveyor.setup_conveyor_belt_graph()` call, same ordering constraints —
  after `enable_full_experience_extensions()`), `scripts/mefron_lib/grasp.py`
  (drop the dead `relationship_name` default arguments), `CLAUDE.md` and
  `docs/grasp-and-assembly-offsets.md` (correct the stale "P not generalized"
  description).

## Verification

Per this repo's existing convention: test against a **scratch copy** of
`assets/mefron/factory floor/`, never the real file — a post-run diff on the
scratch `mefron.usd` is expected importer noise, not a regression signal.

- **Phase 1**: run interactively, confirm `GRIPPER_CLOSE_SPEED`/
  `ASSEMBLY_LIFT_HEIGHT`/conveyor constants behave identically to today with
  the graph variables present; delete/rename one variable and confirm the
  `read_tunable()` fallback to `config.py`'s default kicks in cleanly; edit a
  variable's value via the Property window while the sim is playing and
  confirm the next read picks up the new value with no restart.
- **Phase 2**: exercise every key (J/B/K/P/C/O/N/V/L/1) and confirm identical
  behavior to pre-migration; change one node's `inputs:key` to a different key
  in the GUI and confirm the action rebinds with zero Python edits.
- **Grasp/place generalization**: after fixing `ASSEMBLY_LIFT_HEIGHT`, run
  through J→P, B→P, and K→P (i.e. grasp `finger_print_scanner`/
  `backpanel_support`/`pcb_assembly` in turn, then press P each time) and
  confirm each correctly resolves its own `ASSEMBLY_RELATIONSHIPS` entry via
  `last_grasped_object` and places cleanly at the now-relative lift height. If
  the optional `_maybe_handle_placement_request()` dedup is done, confirm
  arm 1 and arm 2's P behavior is unchanged before/after the extraction.
