# CLAUDE.md

Project-specific context for **isaac-cobot**.

## What this repo is

An NVIDIA Isaac Sim project that builds a simulated factory cell: a real
factory-floor backdrop (vendored from NVIDIA's USD Explorer Sample Assets
Pack — factory shell, Kuka arm, car lift, safety gates, part racks; not one
of Isaac Sim's bundled warehouse environments), two work surfaces for
holding assembly parts, and a Dobot CR5 6-DOF cobot mounted between them.
The work surfaces aren't a synthetic table — they're two copies of the
vendored `ErgoTable` desk prop already present in the factory backdrop
(see `configs/scene/table_layout.yaml`'s `ergo_tables`); an earlier
synthetic gray-cuboid L-table was tried first and dropped for not reading
as "a table" visually.

**There is no physical CR5 hardware.** Everything here targets Isaac Sim
only. Since real drag-teach hardware isn't available, waypoint teaching is
done in-sim instead: the CR5 is imported via URDF, cuRobo provides
collision-aware IK / motion generation, and joint-space waypoints are
recorded and played back through `motion_gen.plan_single_js()`. Treat all
sim behavior (contact dynamics, motion timing, gripper interaction) as
illustrative, not validated against real hardware.

## Repo layout

### Done

- `robots/cr5/` — vendored CR5 URDF + meshes (MIT license, from
  `Dobot-Arm/TCP-IP-ROS-6AXis`; provenance in `robots/cr5/SOURCE.md`). Mesh
  URIs were rewritten from `package://dobot_description/...` to relative
  `../meshes/...` paths so the URDF resolves standalone.
- `docker/.env.base` — Isaac Sim 5.1.0 image + path env vars.
- `docker/.env.curobo` — pinned cuRobo commit hash.
- `docker/container.py` — container management CLI (build/start/enter/stop).
- `docker/utils/` — Isaac Lab BSD-3-Clause container tooling (vendored from
  `tolasing/groot`): `ContainerInterface`, `StateFile`, `x11_utils`. Renamed
  the hardcoded `isaac-lab-*` image/container naming to `isaac-cobot-*`
  since this project has no Isaac Lab framework at all.
- `docker/Dockerfile.base`, `docker/Dockerfile.curobo`, `docker/docker-compose.yaml` —
  two-profile Docker setup (`base`, `curobo`), templated from `tolasing/groot`'s
  Docker layer but without any Isaac Lab framework install — the repo is
  bind-mounted live rather than baked into the image. **Verified**: both
  images build and run against a live RTX PRO 4000 Blackwell GPU (torch +
  cuRobo import, CUDA available, a real matmul on `cuda:0`). Two real bugs
  found and fixed: Isaac Sim's pre-bundled `torch` under
  `omni.isaac.ml_archive/pip_prebundle/` shadows a freshly pip-installed
  one on `python.sh`'s sys.path and must be `rm -rf`'d first; and
  `TORCH_CUDA_ARCH_LIST` is now `12.0+PTX` (Blackwell/sm_120, not the
  Ampere `8.0` originally guessed).
- `.devcontainer/base/` and `.devcontainer/curobo/` — VS Code devcontainer
  configs matching the two Docker profiles. **Verified**: both bring up
  correctly via `docker compose ... up -d` with the repo mounted at
  `/workspace/isaac-cobot` and (for `curobo`) GPU/cuRobo working inside.
  Each compose file sets an explicit top-level `name:` — without it, the
  inferred project name is just the directory's basename (`base`/`curobo`),
  which collided with an unrelated `groot` checkout on this same machine
  that uses the same devcontainer folder names. **Bug found and fixed**:
  the initial devcontainer compose files had no X11 setup at all, so a GUI
  (non-`--headless`) Isaac Sim launch hung indefinitely inside
  `omni.kit.renderer.core` startup (Vulkan/XCB surface creation waiting on
  a display connection that was never authenticated) — a bare `XOpenDisplay`
  succeeds over the socket VS Code forwards automatically, but that
  forwarding doesn't carry a working X11 auth cookie or, likely, DRI3/GLX
  capabilities. Fixed by mirroring the same X11 forwarding pattern already
  used in another project on this machine (`groot`): each `build-images.sh`
  now also generates a magic-cookie xauth file at `/tmp/.docker.xauth` on
  the host (via `xauth nlist "$DISPLAY" | sed ... | xauth -f ... nmerge -`,
  best-effort, non-fatal if there's no host X session), and each
  `docker-compose.devcontainer.yaml` bind-mounts `/tmp/.X11-unix` and that
  xauth file (to `/root/.Xauthority`) and sets `DISPLAY`/`XAUTHORITY`/
  `QT_X11_NO_MITSHM` env vars. Requires `xauth` installed on the **host**
  (not the container) and a full devcontainer rebuild (not just reopen) to
  pick up the new mounts. Not yet re-verified end-to-end with a live GUI
  launch after this fix — do that before relying on it.
- `assets/factory/` — vendored factory-floor scene (NVIDIA USD Explorer
  Sample Assets Pack; NVIDIA Omniverse License Agreement, not open source).
  The ~404MB `Factory.usd` + `SubUSDs/` payload is gitignored — only
  `assets/factory/SOURCE.md` (provenance + re-fetch instructions) is
  tracked.
- `configs/scene/table_layout.yaml` — factory backdrop path + pruning
  rules, `ergo_tables` (the two reused work-surface copies), and
  `cr5_mount` (robot pose/scale + its reused pedestal). **Verified**:
  `scripts/build_scene.py` builds this end-to-end against a live Isaac Sim
  5.1.0 install (real GPU) — `/World/Factory` composes with 8 children,
  both `ErgoTable_1`/`ErgoTable_2` copies render with real geometry,
  `/World/CR5` imports with 18 children (currently the temporary Franka —
  see `cr5_mount.robot_override` below), and the reused `RobotPedestal`
  keeps its geometry. Three real, non-obvious findings baked into this
  config, each with its own inline comment at the point of use:
    - `factory.prune_name_startswith`/`prune_exact_paths` don't just
      remove unwanted *static* dressing (the welding line's rail, its
      duplicate pedestals, a leftover Kuka arm, ErgoTable's monitor/
      keyboard) — they also had to freeze **animated** content that
      wasn't touched by name-based pruning at all: pressing Play in the
      GUI advances the USD timeline, which drives baked keyframe
      animation independently of physics. Found by scanning the whole
      factory subtree for attributes whose value actually changes across
      time samples (not just "has a timestamp," which includes harmless
      single-keyframe export artifacts) — turned up a second, entirely
      separate KUKA arm (`RobotController`), a car-body carrier fixture
      faking motion via toggled visibility (`sledge`/`sledge_I1`), and an
      animated roof component (`Roof_I10`).
    - `/World/Factory` carries an implicit **×100 scale** (it directly
      references the vendored `Factory.usd`, almost certainly authored in
      centimeters, reconciled into this stage's meters convention).
      Anything positioned via `set_world_pose()` under it (the
      `ergo_tables`) needs *world* position in meters; the Property
      panel's local Translate for the same prim reads 100x that value.
      Confirmed empirically: setting world position to (226.912, -328.71)
      produced a Property-panel Translate of (22691.2, -32871.0). This
      does NOT apply to `cr5_mount.pedestal`, which uses a genuinely
      different mechanism (see next point).
    - `cr5_mount.pedestal`'s `local_translation`/`local_orientation_wxyz`
      are LOCAL values (read directly off the Property panel, applied via
      `SingleXFormPrim.set_local_pose()`), not world pose — the reused
      `RobotPedestal` prim's own parent chain has a large native offset
      baked into the vendored asset (e.g. a sibling prim, `Rail`, sits at
      local Translate X=-11000), unrelated to the `/World/Factory` ×100
      scale above. Mixing up local vs. world here silently sends a prim to
      the wrong place — get this distinction right per-prim rather than
      assuming one convention applies stage-wide.
  Also carries a **TEMPORARY** `cr5_mount.robot_override` block that swaps
  in cuRobo's own bundled, well-tuned Franka Panda (URDF + cuRobo config)
  in place of the CR5, to validate the whole pipeline (mount pose,
  pedestal, cuRobo `MotionGen`) before trusting the CR5's own
  not-yet-fully-verified kinematics config (see `configs/curobo/cr5.yml`
  below). Set `enabled: false` (or delete the block) to revert to the CR5.
  Also carries `teleop_target` (the interactive cuRobo teleop target's
  prim path, pose, and debounce thresholds) — see `scripts/build_scene.py`'s
  entry above for what was verified and the bugs found positioning it.
- `configs/curobo/cr5.yml`, `cr5_collision_spheres.yml` — cuRobo robot
  config for the CR5. **Partially verified**: config *loading* (via
  `MotionGenConfig.load_from_robot_config()`) now works against the pinned
  cuRobo commit, and two real bugs were found and fixed in the process —
  see the file's own module comment for both. `MotionGen.warmup()` itself
  has only been confirmed for the Franka case (`build_scene.py`'s current
  default); the CR5 fallback branch that also lives in
  `setup_curobo_motion_gen()` is written the same way but hasn't actually
  been exercised, since `robot_override.enabled: true` means it's dead
  code until that override is turned off.
- `configs/rmpflow/` — deferred by design (cuRobo is the primary
  IK/motion-gen path); contains only a README explaining why.
- `scripts/build_scene.py` — **verified** (see above). Also warms up a
  cuRobo `MotionGen` matching whichever robot is mounted
  (`setup_curobo_motion_gen()`) — best-effort, skipped with a printed
  message if cuRobo isn't installed (the `base` Docker profile). Real bug
  found and fixed: this script (like every other standalone script here)
  creates a `SimulationApp` at import time; it originally did this
  unconditionally, which segfaults instead of raising if something else
  imports it as a library after already starting one — fixed by guarding
  that line behind `if __name__ == "__main__":`, same pattern as
  `import_cr5.py`.
  Also builds an interactive cuRobo teleop target (`teleop_target` in
  `table_layout.yaml`): drag it in the GUI viewport and the mounted robot
  follows via `MotionGen.plan_single()`, a from-scratch adaptation of
  `examples/curobo_reference/motion_gen_reacher.py`'s pattern into this
  repo's own config-driven, robot-agnostic scene (not a copy of that
  file — see its own do-not-modify note below). The target itself
  (`build_teleop_target()`) is a detached `CopyPrim` of the robot's own
  end-effector visual mesh, not a plain marker, so it shows exactly what
  will arrive at that pose. **Verified headlessly** with real evidence, not
  just "no crash": obstacle scan scoped to just the ergo tables + pedestal
  (81 mesh objects, 0.18s — not the thousands a whole-factory scan would
  hit), `motion_gen.update_world()` succeeds in <0.01s, and a scripted
  fake-drag (`target.get_world_pose()` monkeypatched to simulate a real
  mid-loop mouse-drag) produces `plan_single success=True` and a real
  ~1.2 rad joint-position change — proof the whole chain (debounce → plan →
  interpolate → apply via the articulation controller) actually drives the
  robot. Four more real, non-obvious bugs found and fixed in the process:
    - `MotionGenConfig.load_from_robot_config()` leaves
      `motion_gen.world_coll_checker` as `None` unless a real, *non-empty*
      world is passed in at construction time — passing none at all makes
      `update_world()` later crash with `AttributeError: 'NoneType' object
      has no attribute 'load_collision_model'`, and passing an *empty*
      `WorldConfig()` at construction still makes `warmup()` itself fail
      (`"Primitive Collision has no obstacles"` — the MESH collision
      checker needs at least one real obstacle the first time it traces).
      Fixed by calling `get_teleop_obstacles()` (the same scoped scan used
      for later rescans) *before* constructing `MotionGenConfig` and
      passing its result as `world_model`.
    - `isaacsim.core.prims.SingleArticulation.initialize()` needs an actual
      PhysX simulation view, which only gets created once a `PhysicsScene`
      prim exists on the stage — `import_cr5()` imports with
      `create_physics_scene=False` (it only authors joint/drive/collider
      schemas), so nothing here had created one, and `.initialize()` failed
      deep inside `isaacsim.core.prims` even after `timeline.play()`.
      Fixed with a one-line `UsdPhysics.Scene.Define(stage, "/physicsScene")`
      at the top of `run_teleop_loop()` — well short of introducing
      `isaacsim.core.api.World`'s heavier machinery just for this.
    - cuRobo's kinematics/IK/trajopt all operate in the **robot's own
      base-link frame**, not USD world space. `examples/curobo_reference/
      motion_gen_reacher.py`'s robot happens to sit at the world origin, so
      passing a dragged cuboid's raw world pose straight into
      `plan_single()` works there by coincidence — the two frames are
      numerically identical. This repo's robot is mounted away from the
      origin (`cr5_mount.position`), so the same code reliably failed with
      `MotionGenStatus.IK_FAIL` on every attempt. Fixed by transforming the
      target's world pose into the robot's base frame via
      `Pose.compute_local_pose()` before planning (see `run_teleop_loop()`
      — `robot_base_pose` is built once from `cr5_mount.position`/
      `orientation_wxyz`, since the mount is static).
    - `teleop_target.position`'s original placeholder ([1.45, -3.34, 1.2],
      "same x,y as cr5_mount, 1.2m up") mapped to base-frame `(0, 0, 0.43)`
      — almost directly above the robot's own base at low height, which is
      kinematically unreachable for this arm (confirmed via
      `MotionGenStatus.IK_FAIL` even with a valid, reachable orientation).
      Replaced with the robot's own retract-config end-effector pose
      converted to world frame, which is reachable by construction (it's a
      trivial identity plan) and starts the ghost target exactly coincident
      with the real end-effector.
    - A fifth bug, found only once actually run non-headless: the
      `/physicsScene` fix above is necessary but not sufficient --
      `SingleArticulation.initialize()` also needs physics to have actually
      *stepped* at least once (`timeline.play()` plus a few
      `simulation_app.update()` calls), which confirmed live doesn't happen
      until the user clicks Play. An earlier version of `run_teleop_loop()`
      called `robot.initialize()` unconditionally before its own
      `while`/`is_playing()` loop even started, so it crashed with the same
      `AttributeError` immediately on launch, before the user got a chance
      to press Play. Separately, its `step_index` counter (gating the
      init/settle-frame phases and used as the obstacle-rescan cadence)
      incremented on *every* frame including while waiting for Play, so a
      user who took more than an instant to click Play would blow past
      `_TELEOP_INIT_FRAMES`/`_TELEOP_SETTLE_FRAMES` before physics ever
      started. Fixed to match the reference example's own structure:
      `robot.initialize()` is deferred until inside the "is playing"
      branch (once, via an `idx_list is None` check), and `step_index`
      only advances on frames where the timeline is actually playing (a
      separate `not_playing_frames` counter drives the "Click Play to
      start" print instead). Verified with a headless test that fakes
      `timeline.is_playing()` returning `False` for the loop's first 50
      calls before flipping to the real state -- confirms no crash while
      "waiting for Play" and a successful `plan_single` once it starts.
- `scripts/import_cr5.py` — **verified**, both standalone and imported as
  a library. Real bug found and fixed: Isaac Sim 5.1.0's
  `isaacsim.asset.importer.urdf` doesn't export a directly-constructible
  `URDFImporterConfig` class (contradicts this file's own former
  Conventions entry, now corrected below) — the only way to get a
  properly-initialized import config is
  `omni.kit.commands.execute("URDFCreateImportConfig")[1]`.
- `examples/curobo_reference/` — `motion_gen_reacher.py` + `helper.py`,
  fetched verbatim from cuRobo's own GitHub repo at the exact pinned
  commit (`docker/.env.curobo`). A pristine reference copy of cuRobo's
  official interactive teleop demo (drag a target cuboid, robot follows
  via `MotionGen`) — **do not modify these two files**; if a CR5-specific
  variant is needed, write a separate script instead (see "Needs
  verification" below). **Verified it runs** on this install with two
  environment fixes: (1) the prebuilt `kinematics_fused_cu` kernel has a
  torch ABI mismatch here, and cuRobo's JIT-compile fallback needs `ninja`,
  which `Dockerfile.curobo` doesn't install — worth adding; (2) `pip` is
  itself broken in this Isaac Sim install
  (`ModuleNotFoundError: No module named 'pip._vendor.packaging._structures'`),
  so `ninja` had to be fetched as a static binary instead of
  `pip install ninja` — anything relying on pip inside the container is
  currently dead and worth fixing separately.
- `assembly_parts` in `table_layout.yaml` + `build_assembly_parts()` in
  `build_scene.py` — **verified**. References external assembly-part USD
  files (the `mantra scanner/` CAD, converted via the CAD Converter
  extension and color-corrected -- see `fix_cad_import_colors.py` below)
  onto a work surface, via `add_reference_to_stage` (like `build_factory`),
  not `CopyPrim` (like `build_ergo_tables`) -- these are standalone external
  files, not prims already living on this stage. **Real bug found and
  fixed, caught by verifying instead of trusting an assumption**: the first
  version of this config set `scale: [0.001, 0.001, 0.001]` on the
  `pcb_assembly` instance, reasoning that USD/`add_reference_to_stage()`
  would never reconcile a `metersPerUnit` mismatch between the referenced
  file (mm-native, `metersPerUnit=0.001`) and this meters-native scene
  (`metersPerUnit=1.0`) -- the same reasoning already correctly documented
  for `factory.backdrop_usd`'s own analogous cm-vs-m mismatch. That
  assumption was wrong for this specific case: confirmed live via
  `UsdGeom.BBoxCache` that `add_reference_to_stage()` **already** yields
  the exact correct real-world size (0.1055 x 0.12655 x 0.0186, matching
  the true ~105mm x 126mm PCB board) with **no** scale applied at all --
  `SingleXFormPrim.get_local_scale()` reports `[0.001, 0.001, 0.001]` is
  already being applied automatically in this case. Adding another manual
  0.001 on top shrank the part 1000x too small instead of 1000x too large.
  Fixed by setting `scale: [1.0, 1.0, 1.0]` — confirmed via the same
  `BBoxCache` check that the part now sits at the correct size, flush on
  `ErgoTable_1`'s top surface (both bboxes' z values match exactly:
  `1.2600000187754627`), centered on the table (within ~3mm, from the
  part's own local bbox not being perfectly centered on its origin).
  **Open question, not yet resolved**: exactly *why* `add_reference_to_
  stage()` auto-reconciles `metersPerUnit` here but `CopyPrim`-based
  `build_ergo_tables()`/`mount_cr5_pedestal()` don't get the same
  treatment for `factory.backdrop_usd`'s cm-vs-m mismatch (that one still
  needs its own manual scale, confirmed still correct) -- the working
  theory is that `add_reference_to_stage()` references a *standalone* file
  with its own root-layer `metersPerUnit`, while `CopyPrim` duplicates a
  prim that's already composed *inside* `/World/Factory`'s own already-
  established hierarchy and scale, which is a structurally different
  situation despite both superficially being "cm/mm-vs-m mismatches" — not
  confirmed against Kit/USD source or docs, just consistent with what was
  observed. Verify empirically again (don't trust this theory blindly
  either) before assuming it generalizes to a third differently-scaled
  asset.
- `scripts/fix_cad_import_colors.py` — **verified**, both the bug and the
  fix. Isaac Sim's bundled CAD Converter extension (`omni.kit.converter.cad`,
  HOOPS Exchange-based -- not otherwise part of this repo's own pipeline,
  but needed for importing vendor/SolidWorks-authored assembly-part CAD
  files as scene props) has a real color-space bug: STEP's `COLOUR_RGB`
  entities are sRGB (display-referred) values, but the converter writes
  them verbatim into the resulting USD material's `diffuseColor`/
  `emissiveColor` inputs, which USD/Hydra convention treats as *linear*
  (scene-referred) color for PBR rendering -- skipping the sRGB->linear
  decode. Confirmed by diffing a converted file's `diffuseColor` values
  against the raw `COLOUR_RGB` entities in its source STEP file: they
  matched bit-for-bit (mod float32/float64 precision), proving zero
  colorspace conversion happens. Symptom: colors read washed-out/lighter
  than the source CAD tool's own viewport, most visible on dark colors
  (a near-black 0.102 gray renders as a visibly light gray; corrected, it's
  0.0103 -- an order of magnitude darker, matching the source's intent).
  Pure endpoint colors (0.0 or 1.0 per channel) are unaffected, since sRGB
  gamma is a fixed point at both ends. The fix applies the standard sRGB
  EOTF (IEC 61966-2-1) per channel to every `diffuseColor`/`emissiveColor`
  on a `UsdPreviewSurface` in a given USD file, writing to a new
  `*_color_fixed.usd` by default (never overwrites the input) so the
  result can be reviewed before replacing anything; `--in-place` overwrites
  the input directly once confirmed. Not yet confirmed whether this bug is
  STEP-input-specific or affects every format the CAD Converter handles --
  treat as a general post-import fixup until proven otherwise. Matters more
  than a cosmetic nice-to-have here: scene renders are intended as VLA
  training data, where color accuracy affects vision-language grounding
  and sim-to-real transfer, not just visual polish.
- `scripts/` (remaining) — `setup_curobo.py`, `waypoints.py`,
  `teach_waypoint.py`, `playback_waypoints.py`.
- `data/waypoints/` — recorded waypoint JSON (joint-space, not Cartesian);
  see its README for the schema.
- `README.md`, `pyproject.toml`, `.github/workflows/lint.yml`, `tests/` —
  project meta files. No top-level `LICENSE` yet (decided against for
  isaac-cobot's own code for now; the vendored CR5 and `docker/utils/`
  licenses are unaffected).

### Needs verification

`groot` (this repo's Docker/devcontainer template) has no equivalent for
raw Isaac Sim + cuRobo scripts — it uses Isaac Lab's higher-level scene
API instead. `scripts/build_scene.py`, `configs/scene/table_layout.yaml`,
`scripts/import_cr5.py`, and `examples/curobo_reference/` have since been
run end-to-end against a live Isaac Sim 5.1.0 install (see their entries
above). Still open:

- **Revert the temporary Franka swap.** `cr5_mount.robot_override` mounts
  cuRobo's bundled Franka instead of the CR5 to validate the pipeline
  first. Turn it off (`enabled: false`) and confirm the CR5 branch of both
  `mount_cr5()` and `setup_curobo_motion_gen()` in `build_scene.py` still
  works — the CR5 branch of the latter in particular has never actually
  been exercised (see `configs/curobo/cr5.yml`'s entry above). The teleop
  target's current `position`/`orientation_wxyz` in `table_layout.yaml`
  were derived from the *Franka's* retract-config end-effector pose (see
  `scripts/build_scene.py`'s teleop entry above) — re-derive them for the
  CR5's own retract config/reach envelope once the swap is reverted, the
  same way (a value that happens to work for one robot's geometry has no
  reason to be reachable for another's).
- **`scripts/setup_curobo.py`** — still first-draft/unverified, and now
  known (not just guessed) to be broken as written: it passes
  `configs/curobo/cr5.yml`'s path straight to
  `MotionGenConfig.load_from_robot_config()` without the absolute-path
  patching that turned out to be required (see the yml's own module
  comment) — will fail the same way the unpatched version did during this
  investigation.
- **Interactive teleop's real-time/GUI behavior isn't verified** — only
  what a headless run can prove (see `scripts/build_scene.py`'s entry
  above: obstacle scan scope/timing, `update_world()`, and a scripted
  fake-drag all confirmed with the temporary Franka). Still needs at least
  one manual GUI smoke test to confirm: real-time mouse-drag feel/
  responsiveness (the headless test simulates a drag by monkeypatching
  `get_world_pose()`, not an actual mouse), whether the ghost end-effector
  copy visually reads as intended next to the real robot, and the
  `timeline.is_playing()` Press-Play-to-start branch (no Play button exists
  without a display).
- `scripts/teach_waypoint.py`, `playback_waypoints.py` — each flags this in
  its own module docstring. (`scripts/waypoints.py` is plain Python with no
  Isaac Sim dependency and is covered by `tests/test_waypoints.py`.)
- `configs/curobo/cr5_collision_spheres.yml` — placeholder spheres
  proportioned from URDF joint offsets, not fit to the actual meshes.
- `robot_pedestal`/`ergo_tables` positions in `table_layout.yaml` were
  dialed in interactively in the GUI (see their own comments for the
  local/world-pose and ×100-scale gotchas) — visually reasonable, not
  measured against real hardware dimensions.
- The devcontainer X11/GUI-forwarding fix (see its own "Done" entry above)
  is still unconfirmed end-to-end with a live GUI launch.

## Conventions

- USD hierarchy: `/World/CR5` is a **sibling** of `/World/Factory`, not a
  child — this keeps the robot's transform independent of any scale applied
  to factory dressing.
- CR5 URDF quirk: every joint has `effort="0" velocity="0"` (an artifact of
  the SolidWorks exporter). Override drive strength at import time,
  otherwise the articulation won't hold a pose. **Correction**: this used
  to say `URDFImporterConfig(default_drive_strength=1e5)` — that class
  isn't directly constructible in Isaac Sim 5.1.0's
  `isaacsim.asset.importer.urdf`. Get the config object via
  `omni.kit.commands.execute("URDFCreateImportConfig")[1]` instead (see
  `scripts/import_cr5.py`), then set `default_drive_strength`/
  `default_position_drive_damping` on it.
- Waypoints are joint-space (`Waypoint.joint_positions`, radians, 6 values
  for joint1..joint6), not Cartesian poses.
- Pinned versions: Isaac Sim `5.1.0`, cuRobo commit
  `ebb71702f3f70e767f40fd8e050674af0288abe8`, torch `2.11.0+cu128` (CUDA
  12.8, installed fresh in `Dockerfile.curobo` after removing Isaac Sim's
  pre-bundled copy — see the Docker/devcontainer entry above).
- Dev GPU: RTX PRO 4000 Blackwell (sm_120) — `TORCH_CUDA_ARCH_LIST` in
  `Dockerfile.curobo` is tuned to this. Update it first if building for
  different hardware.
- Default to the `curobo` devcontainer/profile, not `base`. `curobo` is
  built `FROM isaac-cobot-base`, so it's a strict superset (scene
  building, URDF import, *and* cuRobo motion-gen). Only reach for `base`
  if deliberately avoiding cuRobo's extra build time/image size (23.5GB vs
  53.9GB) for scene/URDF-only work.
- cuRobo config files (`configs/curobo/*.yml`) can't use repo-relative
  paths directly for `urdf_path`/`asset_root_path`/`collision_spheres` —
  cuRobo's own loader always resolves those against its *own* bundled
  install directories unless the caller patches them to absolute paths
  first. See `configs/curobo/cr5.yml`'s module comment and
  `scripts/build_scene.py`'s `setup_curobo_motion_gen()` for the pattern.
- `ninja` isn't installed in this Isaac Sim/cuRobo environment by default,
  and `pip install` doesn't work here at all
  (`ModuleNotFoundError: No module named 'pip._vendor.packaging._structures'`)
  — cuRobo's CUDA kernels fall back to a JIT compile (needs `ninja`) when
  the prebuilt `.so` has a torch ABI mismatch, which happened on this
  install. Fetch `ninja` as a static binary
  (`ninja-build/ninja` GitHub releases) or via `apt-get install
  ninja-build` instead of pip. Worth fixing at the image level
  (`Dockerfile.curobo`) rather than working around it every time.
- Positioning a prim relative to `/World/Factory` needs care about which
  frame a number is in — see `configs/scene/table_layout.yaml`'s
  `ergo_tables`/`cr5_mount.pedestal` comments for two different, easy-to-
  confuse gotchas (`/World/Factory`'s implicit ×100 scale for world vs.
  local Translate; a reused prim's own large native local-space offset
  baked into the vendored asset). When in doubt, verify by reading back
  `get_world_pose()`/`get_local_pose()` rather than assuming.
- cuRobo's `MotionGen` (kinematics/IK/trajopt, `compute_kinematics()`,
  `plan_single()`) operates entirely in the **robot's own base-link
  frame**, never USD world space — any USD world pose (e.g. a dragged
  teleop target) must be transformed into that frame first via
  `robot_base_pose.compute_local_pose(world_pose)` (both
  `curobo.types.math.Pose` objects), where `robot_base_pose` comes from
  wherever the robot was actually mounted (`cr5_mount.position`/
  `orientation_wxyz`), not assumed to be the origin.
- `isaacsim.core.prims.SingleArticulation.initialize()` (and anything else
  that needs a PhysX simulation view) silently does nothing useful without
  an actual `PhysicsScene` prim on the stage — `import_cr5()` doesn't
  create one (`create_physics_scene=False`), and neither does anything
  else in this repo's scripts. `isaacsim.core.api.World()` would create one
  automatically, but this repo deliberately avoids `World` for scripts that
  don't otherwise need it (see `run_teleop_loop()`'s own module comment) —
  where physics *is* needed, define one explicitly and minimally:
  `UsdPhysics.Scene.Define(stage, "/physicsScene")`.

## Provenance / licensing

- CR5 URDF + meshes: MIT, vendored verbatim except for the mesh URI rewrite
  noted above. See `robots/cr5/LICENSE-cr5-upstream` and
  `robots/cr5/SOURCE.md`.
- `docker/utils/`, `docker/container.py`, and the devcontainer scaffolding
  are adapted from `tolasing/groot`, which itself follows Isaac Lab's
  BSD-3-Clause container tooling pattern.
- `assets/factory/`: NVIDIA Omniverse License Agreement (content-pack
  terms, not open source). See `assets/factory/SOURCE.md`.
- `examples/curobo_reference/`: fetched from NVLabs/curobo's GitHub repo at
  the pinned commit (`docker/.env.curobo`). The overall cuRobo project is
  Apache-2.0, but these two files' own header comments say "NVIDIA
  CORPORATION... strictly prohibited" (proprietary-looking boilerplate
  that doesn't obviously match the repo-level license) — not resolved
  here; treat as internal reference/testing use only until that's
  clarified, and don't redistribute beyond this repo without checking.
