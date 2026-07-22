# Migrating mefron's grasp/place sequencing to real BehaviorTree.CPP + Groot2

> Implemented 2026-07-22 on the `behaviour-tree` branch. Scope: only the
> grasp→lift→align→descend→place sequence (arm 1's `gripper_control` P
> handling and arm 2's `assembly_control` P handling in `_step_arm()`). The
> per-frame drag-follow cuRobo plan/apply loop, obstacle rescans, gripper
> ramp, and keyboard dispatch are all untouched.

## Why

`docs/omnigraph-migration-plan.md` already investigated "should grasp/place
become a reusable behavior instead of ad hoc per-object handlers" for
OmniGraph specifically, found the Python-level reuse problem already solved
(arm 1's `last_grasped_object` reverse-lookup), and declined NVIDIA's Cortex
framework as the wrong shape for this project. It never evaluated an actual
external behavior-tree *engine* — that's this migration: replace the
procedural one-shot P-handling `if`-chain with a real **BehaviorTree.CPP**
tree, so the sequence becomes an explicit, visual, Groot2-editable structure
instead of nested conditionals, without changing what it actually does.

## Two facts that shaped the design

- **BehaviorTree.CPP has no official Python bindings.** Real BT.CPP (not a
  Python lookalike) plus real Groot2 compatibility means a small custom
  pybind11 bridge — this repo's first compiled Python extension.
- **Groot2 talks to a running tree over ZMQ** (two consecutive ports —
  confirmed live: `Groot2Publisher(tree, server_port)` binds `server_port`
  *and* `server_port + 1`), not X11/GUI forwarding. `docker/docker-compose.yaml`
  already runs every service with `network_mode: host`, so Groot2 running on
  your own desktop connects to `localhost:<port>` directly with no
  dependency on X11 forwarding at all. Groot2's free "Basic" tier monitors up
  to 20 nodes, comfortably covering this tree (3 nodes). That said, X11/GUI
  forwarding into the devcontainer is itself confirmed working
  (`docs/docker-and-devcontainer.md`), so Groot2 can alternatively run
  **inside** the devcontainer instead — see this file's own "Groot2 in the
  devcontainer" section below.

## Architecture

**`bt_bridge/`** — a new top-level directory, *not* baked into the Docker
image (it's part of the live, bind-mounted repo, same as every other
`scripts/`-adjacent thing — see `docker-compose.yaml`'s own comment on why
nothing is `COPY`'d in ahead of the bind mount). Built once per container by
running `scripts/build_bt_bridge.sh`, which drops the resulting `.so`
straight into `scripts/mefron_lib/`.

- `bt_bridge/CMakeLists.txt` + `bt_bridge/src/bt_bridge.cpp`: a pybind11
  extension exposing `BTExecutor` (wraps `BT::BehaviorTreeFactory` +
  `BT::Tree` + `BT::Groot2Publisher`, loads a tree from an XML file so Groot2
  can open/edit the tree directly) and `register_callback(name, fn)` (a
  process-global registry from an XML leaf's `callback_name` attribute to a
  Python callable). One generic `PyActionNode` C++ type — registered under
  both `PyAction` and `PyCondition` XML tags — looks up and calls whichever
  Python callable its `callback_name` names, mapping the returned
  `'SUCCESS'`/`'FAILURE'`/`'RUNNING'` string to a `BT::NodeStatus`. This is
  the same principle `docs/omnigraph-migration-plan.md` already used for
  cuRobo: the bridge owns control-flow sequencing only, never the actual
  grasp/place math, which stays in `grasp.py`.
- **Module name is `mefron_bt_bridge`, not `bt_bridge`.** Confirmed live:
  naming the compiled module `bt_bridge` — the same name as its own source
  directory — silently broke on import. `bt_bridge/` has no `__init__.py`,
  so once the repo root is on `sys.path` (Kit adds CWD), Python resolves it
  as an implicit namespace package *first*, and `import bt_bridge` "succeeds"
  against that empty package instead of the real extension —
  `AttributeError: module 'bt_bridge' has no attribute 'BTExecutor'`, no
  import error at all to signal the problem. A distinct module name
  sidesteps this regardless of `sys.path` ordering.
- **`bt_bridge/trees/assembly_placement.xml`**: the one canonical,
  Groot2-editable tree — a `Sequence` of `IsHoldingSomething` (condition),
  `SnapToLiftWaypoint` (action), `WaitForSequenceIdle` (action).
  **Only two steps actually needed migrating in.** The original code's
  "descend to the real final pose once the lift plan finishes" step was
  never part of P-handling at all — it's `_step_arm()`'s own per-frame
  plan/apply loop auto-applying `state["pending_final_pose"]` whenever any
  `cmd_plan` finishes, deliberately untouched by this migration. So
  `WaitForSequenceIdle`'s `SUCCESS` condition is "both the lift flight *and*
  that auto-triggered descend flight have finished" (`cmd_plan is None and
  pending_final_pose is None`), not just the first plan. An earlier draft of
  this design had a fourth node (`SnapToFinalPose`) that would have
  re-applied `pending_final_pose` a second time, redundantly — caught before
  it shipped by tracing exactly where the original code consumed it.
- **`scripts/mefron_lib/behavior_tree.py`**: `AssemblyPlacementBehaviorTree`,
  one instance per arm (constructed in `mefron.py`, stored as
  `arm["assembly_bt"]`). `start()` arms it with that arm's live
  `state`/`target`/`ee_link_prim_path`/`relationship_name`/`is_holding`
  callable; `tick()` is called unconditionally every frame from
  `_step_arm()` (a no-op, returning `None`, until `start()` has armed it).
  Both arms share the one static XML and the same four callback names —
  safe because `_step_arm()` only ever ticks one arm at a time
  (`run_teleop_loop()`'s `for arm in arms` loop is plain sequential, never
  concurrent): each instance's `tick()` re-registers its own closures under
  those names immediately before calling `tick_once()`, so there's never a
  moment where the wrong arm's callback is registered when a tick actually
  fires. `reset()` (called on every fresh Play, same as
  `GripperKeyboardControl.reset()`) halts the tree and clears the armed
  state, so a sequence armed just before a Stop doesn't stay "in flight"
  against a `state` dict `_fresh_arm_state()` has since replaced.
- **Single-threaded, synchronous ticking, by design.** `BTExecutor.tick_once()`
  is called directly from Python's existing per-frame loop — never from a
  separate C++ thread. This carries none of the "background thread touching
  CUDA/PhysX" risk `docs/omnigraph-migration-plan.md` already rejected for
  Script-Node-hosted cuRobo. (`Groot2Publisher` does run its own background
  ZMQ thread internally, but it only reads tree/blackboard state — it never
  calls back into Python.)

## Docker

`docker/Dockerfile.curobo` installs `libzmq3-dev`/`libboost-dev`/
`libsqlite3-dev`/`pybind11-dev` via apt and builds BehaviorTree.CPP (pinned
tag in `docker/.env.curobo`'s `BT_CPP_COMMIT`, currently `4.9.1`) from
source at image build time — same "compile once at build time, never live"
rule already enforced for cuRobo's CUDA kernels/`ninja`. `pybind11-dev` via
apt, not pip: pybind11 is header-only, and this keeps everything BT-related
on the same "apt, not pip" footing as `ninja-build`.

`bt_bridge/`'s own extension is **not** built at image-build time — it's
part of the live repo (see above), so `scripts/build_bt_bridge.sh` builds it
against Isaac Sim's own bundled Python (`/isaac-sim/kit/python/`, Python
3.11 as of Isaac Sim 5.1.0) once per container, after entering it. Building
against system Python instead is a real, confirmed-live failure mode:
`find_package(Python3 REQUIRED COMPONENTS Development)` **without**
`Interpreter` in the component list ignores the `Python3_EXECUTABLE` hint
entirely and silently resolves `Development` against whatever system Python
it finds first (here: system Python 3.12, not Isaac Sim's 3.11) — the same
wrong-interpreter trap `Dockerfile.curobo` already routes around for torch.
Fixed by requesting `COMPONENTS Interpreter Development`.

## Using Groot2

Two ways to run it, both valid:

1. **On your own desktop** (no devcontainer changes needed): "Monitor" mode
   pointed at `localhost:1667` (arm 1) or `localhost:1669` (arm 2) while
   `mefron.py` runs in the container — `network_mode: host` makes this a
   direct connection, no forwarding needed.
2. **Inside the devcontainer**, now that this project's X11/GUI forwarding
   is confirmed working end-to-end (`docs/docker-and-devcontainer.md`):
   `Dockerfile.curobo` installs Groot2 (pinned version in
   `docker/.env.curobo`'s `GROOT2_VERSION`) as an **extracted** AppImage
   (`--appimage-extract` at build time into `/opt/groot2`, symlinked as
   `/usr/local/bin/groot2`) — not run directly as a `.AppImage`, since a
   plain AppImage needs FUSE to mount itself and a container has no
   `/dev/fuse` by default. Just run `groot2` once inside the container and
   point it at `localhost:1667`/`1669` the same way.

Either way, the same XML can also be opened in Groot2's editor mode
independent of a live connection.

**Confirmed live** (this sandbox happened to already have a working X11
display): downloaded the real v1.9.0 AppImage from its actual current host
(`pub-32cef6782a9e411e82222dee82af193e.r2.dev` — their old S3 bucket URL
from search results returned `AccessDenied`; fetching their real download
page directly turned up the current one), extracted it, and launched it
against a live `DISPLAY` — it started cleanly (`Window shown, entering event
loop`) and stayed running, confirming both the download URL and the
extract-instead-of-run approach work in practice, not just in theory.

## Verification performed

- **Phase 0 spike**: a standalone dummy 2-node tree (no Isaac Sim, no
  cuRobo) confirmed BT.CPP + pybind11 build/link/tick correctly against
  Isaac Sim's exact bundled Python 3.11, and that `Groot2Publisher` opens
  its ZMQ port and accepts a real TCP connection.
- **`bt_bridge` extension, standalone**: exercised the real
  `assembly_placement.xml` tree with mock Python callbacks — multi-frame
  `RUNNING` handling, `SUCCESS` completion, and `FAILURE` short-circuit at
  the condition node (downstream actions never called) all confirmed
  correct.
- **Full integration, real scene, real cuRobo planning**: a standalone
  repro script built the exact `mefron.py`/`test_mefron_assembly_headless.py`
  arm dict (real `setup_motion_gen()`, real `build_teleop_target()`, a real
  `AssemblyPlacementBehaviorTree` with its `BTExecutor`/`Groot2Publisher`)
  against the real scene, then called `run_teleop_loop()` **twice in a
  row** (mirroring the fresh-Play `assembly_bt.reset()` path every phase
  transition hits) — first a real J grasp-approach request, then a real P
  assembly-target request, 200 iterations each. Both completed cleanly:
  `SnapToLiftWaypoint`/`WaitForSequenceIdle` drove real cuRobo plans, and
  the second call's `reset()` (halting the tree mid-construction of a fresh
  `_fresh_arm_state()`) didn't break anything on the next arm.
- **Not yet completed: the full, unmodified `test_mefron_assembly_headless.py`**
  (900 iterations × 3 phases: J → P → K). Three attempts each stalled for
  20–30+ minutes with zero new output, past cuRobo's own internal mesh-cache/
  warmup logging and before any of this migration's code even runs
  (`AssemblyPlacementBehaviorTree` isn't constructed until after
  `setup_motion_gen()` returns). Isolating further: `setup_motion_gen()`
  alone, `setup_motion_gen()` + `build_teleop_target()` +
  `AssemblyPlacementBehaviorTree` construction alone, and the two-call
  200-iteration repro above (with real planning) all completed quickly and
  correctly every time they were tried in isolation — only the full
  900-iteration × 3-phase run reproduced the stall, 3/3 attempts, including
  one with nothing else running concurrently in the sandbox. Given every
  narrower slice of the exact same code succeeded, this looks like a
  pre-existing performance characteristic of this particular sandbox's
  GPU/PhysX under sustained iteration counts (CPU was actively busy, not
  blocked, when checked mid-stall) rather than a bug in this migration --
  but that's an inference, not a confirmed root cause. Recommend running
  the full headless test in a real devcontainer before treating this
  migration as fully verified end-to-end.
