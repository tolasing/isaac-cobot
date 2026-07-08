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
  torch ABI mismatch here, and cuRobo's JIT-compile fallback needs `ninja`
  — **now fixed** via `apt-get install ninja-build` in `Dockerfile.curobo`
  (see the Conventions entry above); (2) `pip` is itself broken in this
  Isaac Sim install
  (`ModuleNotFoundError: No module named 'pip._vendor.packaging._structures'`),
  so `ninja` had to be fetched as a static binary instead of
  `pip install ninja` — anything relying on pip inside the container is
  currently dead and worth fixing separately.
- `scripts/test_teleop_headless.py` — **verified**. Standalone headless
  fake-drag verification for `build_scene.py`'s interactive teleop loop:
  since `build_scene.py`'s own `main()` only calls `run_teleop_loop()` in
  the non-`--headless` path, there was previously no way to exercise the
  drag-follow logic without a live display. This script builds the full
  scene via `build_scene.py`'s own functions (reused as a library, not
  duplicated), then monkeypatches `target.get_world_pose()` to hold the
  target's real starting pose long enough to clear
  `_TELEOP_INIT_FRAMES`/`_TELEOP_SETTLE_FRAMES`, then jump to a second,
  nearby pose — indistinguishable to `run_teleop_loop()` from a real drag.
  **Verified headlessly**: `teleop plan_single success=True` and a real
  `0.5630` rad joint-position delta. Two real bugs found and fixed:
    - `run_teleop_loop()` (like `build_scene.main()`) references a
      module-level `simulation_app` global inside `build_scene`'s own
      namespace, only ever set there under `if __name__ == "__main__"` —
      since this script is the one creating the real `SimulationApp` when
      run standalone, it must assign `build_scene.simulation_app`
      explicitly before calling `run_teleop_loop()`, or that name lookup
      fails with `NameError`.
    - Calling `timeline.play()` before `run_teleop_loop()` has been
      *called* (which is where it defines `/physicsScene`, at its own
      top) plays physics with no `PhysicsScene` prim on the stage yet —
      confirmed live this corrupts PhysX's tensor simulationView, and
      `run_teleop_loop()`'s own later `SingleArticulation(...)`
      construction then crashes with `AttributeError: 'NoneType' object
      has no attribute 'link_names'`, even though that's the *first*
      `SingleArticulation` built in the whole process (ruled out via a
      separate repro: holding a second one before calling
      `run_teleop_loop()`, then `del`ing it first, still crashes the
      same way — so it isn't specifically about *count*, just about
      physics stepping with no scene at all). Fixed by defining
      `/physicsScene` in this script too, before calling
      `timeline.play()`, matching the order `run_teleop_loop()`'s own
      internal guard already assumes.
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
- `assets/mefron/` — a second, separate hand-authored scene (its own
  factory floor, two `packing_table`/`packing_table_01` copies, and a
  scanner-assembly CAD mockup — `finger_print_scanner`, `main_holder`,
  `screen`, `backpanel_support`), built directly in the Isaac Sim GUI
  (not via this repo's own `table_layout.yaml` pipeline) as a second
  target for the same Franka-teleop capability. `factory floor/mefron.usd`
  is the top-level stage file (`defaultPrim=/World`, `metersPerUnit=1.0`
  — no cm/m mismatch to work around, unlike `assets/factory/Factory.usd`).
  Untracked in git (like `assets/factory/`). **Real, non-obvious finding**:
  importing a robot into this file *while it's opened directly*
  (`omni.usd.get_context().open_stage()`, not referenced in) makes Isaac
  Sim's URDF importer write a disk-persisted, multi-layer "Robot
  Description" structure under `factory floor/configuration/` (a multi-MB
  `mefron_base.usd` plus smaller `mefron_physics.usd`/`mefron_sensor.usd`/
  `mefron_robot.usd` sublayers) — confirmed via the importer's own log
  line ("Creating Asset in an in-memory stage, will not create layered
  structure") that this behavior is conditional on the stage having a
  real, file-backed root layer; it never happens for `build_scene.py`'s
  own anonymous, in-memory stage. This write happens automatically, with
  no save prompt, every time the import runs (script or manual GUI import)
  — confirmed via file mtimes. Now gitignored (`assets/mefron/*` in
  `.gitignore`, same treatment as `assets/factory/`) — it's a large
  (~23MB), hand-authored, vendor-origin asset pack with no established
  redistribution terms of its own, same reasoning as `assets/factory/`.
  **Correction to an earlier claim in this file**: `mefron.usd`'s own
  *saved* root layer was believed still pristine (`stage.Save()` is never
  called by any script here) based on an earlier session's
  `Sdf.Layer.FindOrOpen()` check — but a `/panda` prim spec is confirmed
  **still present** in the file's saved root layer as of the session that
  did `scripts/mefron.py`'s T_H_S/T_S_G/Step-6 work (below): every fresh
  `open_stage()` of this file, from a brand-new process, immediately emits
  `Could not open asset .../configuration/mefron_*.usd for payload
  introduced by .../mefron.usd</panda{...}>` warnings — before any script
  code runs, so this can only be coming from the file itself, not
  something a script wrote in-memory this run. Not a live bug (these are
  non-fatal warnings, not the "layer already exists" crash a *corrupted*
  configuration file causes — see `clear_stale_robot_configuration()`
  below for that distinct issue), but the file does need a manual GUI
  fix (open `mefron.usd`, delete `/panda`, save) to actually clean up —
  no script here calls `stage.Save()`, so nothing here can fix this from
  code.
- `scripts/mefron.py` — a standalone script (does not import
  `build_scene.py`) that mounts cuRobo's bundled Franka Panda onto
  `assets/mefron/`'s pre-authored SEKTION-cabinet mount plate
  (`/World/sektion_cabinet_instanceable`, world position
  `[2.74097, -4.782, 0.7924]` — same value `mefron_layout.yaml`'s
  `cr5_mount.position` already uses; **corrected** from an earlier,
  now-stale mount at `/World/Factory/Stage/Pedestal_plates/Cube_05` /
  `(2.2025, -4.5025, 1.0018)`, which no longer exists after the
  pedestal-to-SEKTION-table remount documented under
  `configs/scene/mefron_layout.yaml` below), runs the same drag-follow
  teleop loop as `build_scene.py`, and (new) provides a scalable
  pick-and-place assembly capability — pressing **G**/**P** snaps the
  teleop target to a live-computed grasp-approach or assembly-placement
  pose for `finger_print_scanner`/`main_holder`, recomputed from their
  *current* world poses every time, not baked in once. Opens `mefron.usd`
  directly via `open_stage()`. **Superseded by `scripts/build_scene_mefron.py`
  below** for the base teleop capability (see that entry for why), but
  kept working and documented since it's the only script that can host
  `mefron.usd` reparenting operations `build_scene_mefron.py`'s
  referenced-stage session can't (see the T_H_S finding below) — has its
  own real, confirmed findings:
    - `build_teleop_target()`'s usual `CopyPrim`-based ghost target
      (copied from `build_scene.py`) produced a prim with a **completely
      empty bounding box** here (confirmed via `UsdGeom.BBoxCache`) — a
      real break, not the harmless instance-proxy quirk `build_scene.py`'s
      own version documents. Root cause: the `configuration/`
      multi-layer structure (see `assets/mefron/`'s entry above) means the
      source prim's visuals are composed across several layers, and
      `CopyPrim`'s shallow, spec-level copy can't correctly re-resolve a
      same-layer reference once relocated to a new prim path outside that
      layer stack. Confirmed this isn't fixable via
      `stage.SetEditTarget()` beforehand — the importer resets the edit
      target itself regardless of what it was set to. Fixed by replacing
      `CopyPrim` with an **internal USD reference**
      (`prim.GetReferences().AddInternalReference()`) — a live pointer at
      the already-*composed* result rather than a copy of raw specs, so
      it renders correctly no matter how many layers underlie the source.
      Confirmed live: non-empty bbox with real, plausible gripper-mesh
      extents.
    - `mefron.usd` already has its own hand-authored `/PhysicsScene`
      (capitalized, at stage root). `run_teleop_loop()`'s ported-from-
      `build_scene.py` physics-scene check only ever looks for the exact
      lowercase path `/physicsScene` and creates one unconditionally if
      that's missing — it has no way to know about the differently-named
      one already on the stage, so it creates a second, redundant scene.
      Confirmed live that having both active simultaneously breaks the
      robot's PhysX articulation view entirely:
      `isaacsim.core.prims.impl.articulation` logs "Physics Simulation
      View is not created yet" forever, `get_joints_state()` never
      returns non-`None`, and the robot never responds to the target no
      matter how long you wait. Fixed here by deactivating (not deleting
      — reversible, in-memory only, never touches the file on disk) the
      pre-existing `/PhysicsScene` right after opening the stage, so
      `run_teleop_loop()`'s own check finds neither path valid and
      creates exactly one canonical `/physicsScene` itself.
    - Calling `timeline.play()` before cuRobo's ~30s blocking
      `motion_gen.warmup()` (which calls no `simulation_app.update()` of
      its own) leaves physics "playing" across a long unpumped real-time
      gap — confirmed live this corrupts PhysX's tensor simulationView by
      the time the drag loop's own `SingleArticulation` gets constructed,
      crashing with the same `AttributeError: 'NoneType' object has no
      attribute 'link_names'` as `test_teleop_headless.py`'s analogous
      bug above. Fixed by never calling `timeline.play()` in this script
      at all — matching `build_scene.py`'s own convention, a human clicks
      Play in the GUI, and `run_teleop_loop()` already waits for
      `is_playing()` itself.
    - Clicking **Stop** in the GUI tears down PhysX's simulation view
      entirely — confirmed live that a `SingleArticulation` built before
      the Stop is left pointing at that now-destroyed view, and reusing
      it after a later Play produces the same endless "Physics Simulation
      View is not created yet" symptom, permanently (the robot never
      responds again for the rest of that process). The original
      `run_teleop_loop()` pattern (ported from `build_scene.py`) only
      ever builds its `SingleArticulation` once, gated by `idx_list is
      None`, checked just on the very first Play — it has no path to
      rebuild it on a *later* Play. This is a latent bug in
      `build_scene.py`'s own `run_teleop_loop()` too (not fixed there,
      since it was never exercised there via a real Stop→Play GUI cycle —
      see "Needs verification" below), just discovered here first. Fixed
      in this file by tracking not-playing→playing transitions and
      rebuilding `robot`/`idx_list`/`articulation_controller` (and
      resetting all other per-session state) on *every* fresh Play, not
      just the first. **Verified live**, not just "doesn't crash": a
      scripted test drove a fake drag (`plan_single success=True`,
      `0.29m` real end-effector movement), then called
      `timeline.stop()`/`timeline.play()` again in the same process and
      drove a second fake drag — `plan_single success=True` again, `0.34m`
      movement — proving the rebuild-on-replay logic actually works, not
      just that it avoids a crash.
    - **A corrupted-`configuration/`-file crash was found and fixed at the
      source.** A previous crash left truncated
      `configuration/mefron_{base,physics,robot,sensor}.usd` stub files on
      disk (see `assets/mefron/`'s own entry above for why the URDF
      importer writes these for a file-backed stage at all) — these broke
      every subsequent import into `mefron.usd` the same way, forever,
      since USD caches `Sdf.Layer` objects by identifier and a truncated
      file makes `open_stage()` itself crash with "a layer already
      exists" while resolving `/panda`'s broken payload references,
      before any script code even runs. `clear_stale_robot_configuration()`
      deletes any pre-existing files under that directory — but it must
      run **before** `open_stage()`, not after: the first version of this
      fix ran it after and still crashed, since USD had already cached the
      broken `Sdf.Layer` objects while opening the stage, and deleting the
      files off disk afterward doesn't invalidate that cache.
    - **Grasp-physics and trajectory-pacing parity, ported from
      `build_scene_mefron.py`.** This file had none of
      `build_scene_mefron.py`'s already-fixed grip/motion bugs — added
      `GripperKeyboardControl` (C/O keys), a gripper friction material
      (`GRIPPER_STATIC_FRICTION=0.9`/`GRIPPER_DYNAMIC_FRICTION=0.8` on
      `/World/finger_print_scanner`), a stiffened finger drive
      (`GRIPPER_DRIVE_STIFFNESS=10000.0`), and `interpolation_dt`-gated
      trajectory playback (real elapsed time via `time.time()`, not one
      waypoint per render frame — see `build_scene_mefron.py`'s own entry
      for the FPS-vs-`interpolation_dt` mechanism this fixes). Ported as
      duplicated logic, not shared code, matching this file's own
      established convention of not importing `build_scene_mefron.py`
      (different stage types).
    - **`robot.initialize()` crashed with `AttributeError: 'NoneType'
      object has no attribute 'create_articulation_view'`, even after a
      settle-frame delay and a forced post-warmup `timeline.stop()`.**
      Root cause, confirmed by reading `isaacsim.core.simulation_manager`'s
      actual source rather than guessing further:
      `SingleArticulation.initialize()` depends on
      `SimulationManager.get_physics_sim_view()`, which is only ever set
      via one specific chain — timeline PLAY event → `_warm_start()` →
      gated behind the carb setting `/app/player/playSimulations` → if
      true, `initialize_physics()` → dispatches `PHYSICS_WARMUP` →
      `_create_simulation_view()` actually sets the view. That setting is
      a real, user-facing toggle in the Play button's own toolbar dropdown
      (alongside Play Animations/Audio/Computegraph) — if it's off,
      `timeline.is_playing()` still correctly returns `True`, but the
      simulation view never gets created, and no amount of Play-timing or
      settle frames fixes it. Fixed by forcing it on explicitly at the top
      of `main()`:
      `carb.settings.get_settings().set_bool("/app/player/playSimulations", True)`.
      Confirmed via a new headless regression test (see
      `scripts/test_mefron_teleop_headless.py` below).
    - **T_H_S (`finger_print_scanner`'s pose relative to `main_holder` at
      the correctly assembled position) was derived live, in this
      script's own session, by script — not by hand.** This file opening
      `mefron.usd` directly (rather than referencing it in, like
      `build_scene_mefron.py` does) is what makes this possible at all:
      temporarily reparenting `finger_print_scanner` under `main_holder`
      in the Stage tree to dial in exact visual alignment hits a "Cannot
      move/rename ancestral prim" restriction in
      `build_scene_mefron.py`'s referenced-stage session, confirmed live,
      but works natively here since `mefron.usd` is the real edit target.
      Once aligned by hand in the GUI, the relative transform was computed
      from both prims' resulting **world poses** via a new
      `compute_relative_pose()` helper (uses
      `isaacsim.core.utils.numpy.rotations.quats_to_rot_matrices`/
      `rot_matrices_to_quats` — confirmed via direct source read to be
      **scalar-first, wxyz**), not hand Euler-angle conversion — hand
      conversion is what produced a confirmed-wrong rotation earlier in
      this same investigation, for an unrelated pose. Result, now a
      module constant:
      `ASSEMBLY_RELATIONSHIPS["finger_print_scanner_on_main_holder"]` =
      `local_position=[-0.05765023, 0.02069006, 0.01875005]`,
      `local_orientation_wxyz=[0.999973595, -0.00618904850,
      0.000842160478, -0.00371422408]`.
    - **T_H_S was re-derived a second time in a later session — the value
      above visibly placed the scanner wrong on the mount ("rederiving it
      is off from the pos it is suppose to be at").** Same technique as
      before (manually re-aligned `finger_print_scanner` under
      `main_holder` in the GUI, then `compute_relative_pose()` on the two
      prims' resulting world poses), not a hand-tweak of the old numbers.
      New value, now what `mefron.py`'s `ASSEMBLY_RELATIONSHIPS` actually
      holds, superseding the one immediately above: `local_position=
      [-0.05765001316747483, 0.02068996147910942, 0.01500000425999065]`,
      `local_orientation_wxyz=[1.0, 0.0, 0.0, 0.0]`. X/Y moved by well
      under a millimeter, but Z dropped from `0.01875` to `0.01500` (the
      part had been sitting ~3.75mm too high) and the orientation
      simplified from a small residual rotation to a clean identity
      quaternion — consistent with a more carefully-aligned re-measurement
      rather than measurement noise.
    - **The official `isaacsim.robot_setup.grasp_editor` tool (`GraspSpec`)
      was tried first for T_S_G (the gripper's grasp pose relative to
      `finger_print_scanner`) and found fundamentally unusable for this
      exact Franka+`mefron.usd` combination — abandoned, not worked
      around.** Its "Select Frames of Reference" dropdown came back
      permanently empty, and its separate Joint Settings panel crashed
      outright with `AttributeError: 'NoneType' object has no attribute
      'is_active'`. Two wrong hypotheses were ruled out live first: not a
      UI refresh-timing issue (retyping the filter field didn't help), and
      not a dual-`SingleArticulation` ownership conflict with this
      script's own running teleop loop (built a separate scene with no
      teleop loop or cuRobo running at all, `scripts/
      mefron_grasp_editor_scene.py` below — dropdown was still empty).
      **Actual confirmed root cause**, found via a direct diagnostic
      script that bypassed the Grasp Editor UI entirely: the Franka's own
      articulation/DOF resolution works fine (`dof_names` populates
      correctly, matching that the SEKTION cabinet's identical-mechanism
      articulation also works) but `Usd.PrimRange(art.prim)` — which the
      Grasp Editor's own dropdown-population code uses — finds **zero**
      Xformable descendants under the Franka. Traced to the URDF
      importer's file-backed-stage "layered Robot Description" mechanism
      (see `assets/mefron/`'s own entry above) producing genuinely broken
      internal cross-references for this specific Franka in this specific
      file, confirmed via persistent "Could not open asset"/"Unresolved
      reference prim path" warnings on every fresh, freshly-cleared
      import — not just after a crash.
    - **T_S_G was derived the same way as T_H_S instead** — via
      `compute_relative_pose()` on the Franka's `ee_link` and
      `finger_print_scanner`'s live world poses at a manually-jogged,
      visually-confirmed good grasp, not via the Grasp Editor. Confirmed
      the result is a real, physically-sensible transform, not a
      derivation error: its near-1 component landed in the *last* slot
      (`w≈0.99999`) rather than the first, initially looking suspicious
      next to T_H_S's own result — double-checked directly against
      `isaacsim/core/utils/numpy/rotations.py`'s own source (not assumed)
      and confirmed `rot_matrices_to_quats` really is scalar-first,
      confirming this is a legitimate ~180-degree rotation about the
      scanner's own local Z axis (the gripper approaches from above; the
      scanner's CAD-authored local frame has its own flipped axis
      convention relative to that approach direction), not a bug. Result,
      now module constants: `GRASP_OFFSET_POSITION=[0.01277519,
      -0.02169126, -0.02863107]`,
      `GRASP_OFFSET_ORIENTATION_WXYZ=[-0.000518294608, -0.00348700255,
      0.000751325308, 0.999993504]`. `scripts/franka_grasp_editor_scene.py`/
      `scripts/mefron_grasp_editor_scene.py` (below) remain in the repo as
      working diagnostic artifacts for future parts, in case the Grasp
      Editor is worth retrying against a from-scratch stage for a
      robot/asset combination that doesn't hit this same layered-import
      bug.
    - **T_S_G was also re-derived in a later session**, same technique
      (manually jog the gripper to a fresh visually-confirmed good grasp,
      then `compute_relative_pose()` on the live poses), not a hand-tweak.
      New value, now what `mefron.py` actually holds, superseding the one
      immediately above: `GRASP_OFFSET_POSITION=
      [0.00027002069774515104, -0.021693730387954874,
      -0.1271989186209571]`, `GRASP_OFFSET_ORIENTATION_WXYZ=
      [-2.1523912431273915e-05, -8.089888886539503e-06,
      5.762411090611313e-06, 0.9999999997190347]` — a near-identity
      rotation (`w≈1`) rather than the earlier ~180-degree-about-Z one,
      reflecting a different jog approach angle this time, not a
      convention change. **Still an open, unresolved problem even with
      this re-derivation**: placement lands close on X, visibly off on Y.
      Diagnosed (not yet fixed) as a grasp-*centering* issue, not a T_S_G
      accuracy problem or a joint-configuration asymmetry — see
      "Needs verification" below.
    - **Step 6: wired T_H_S/T_S_G into two new pose functions and two new
      keybindings, table-position-independent by construction.**
      `compute_grasp_approach_pose()`/`compute_assembly_grasp_target()`
      each re-read the live world pose of `finger_print_scanner`/
      `main_holder` on every call and compose it with the fixed relative
      transforms above via a new `compute_dependent_world_pose()` helper
      (the forward direction of `compute_relative_pose()`), so neither
      function depends on where the parts happened to be sitting when
      T_H_S/T_S_G were derived. `GripperKeyboardControl` gained two
      one-shot request/consume method pairs
      (`request_grasp_approach()`/`consume_grasp_approach_request()`, and
      the `_assembly_target` equivalents) wired to new **G**/**P** keys in
      `build_gripper_keyboard_control()`. **Real bug found and fixed,
      caught by a headless regression test rather than assumed working**:
      the first version placed the G/P snap-consumption block *before*
      `run_teleop_loop()`'s own `past_pose`/`target_pose is None`
      bootstrap block. On the very first eligible frame of a call where a
      request was already pending (the exact scenario a caller pre-arming
      a request before the loop even starts produces), the snap fired
      first, so `target.get_world_pose()` read back the *already-snapped*
      pose, and `target_pose` got bootstrapped from that same post-snap
      value — making the debounce's `norm(cube_position - target_pose)`
      distance check exactly zero, forever, for that entire call. The snap
      itself worked (the target prim really did move), but
      `motion_gen.plan_single()` was never even called — confirmed via a
      headless test (`scripts/test_mefron_assembly_headless.py` below)
      whose pose-sanity checks passed (grasp-approach/assembly-target
      poses both landed a plausible ~4-5cm from their reference objects)
      while its full run produced **zero** occurrences of the
      `"plan_single"` log line across ~280 frames per phase — comfortably
      enough for the debounce to have fired if the condition could ever
      become true. Three independent agents adversarially re-derived this
      exact root cause from the live code (not from this write-up) before
      the fix was applied, and all three converged on the same diagnosis
      and fix. Fixed by moving the bootstrap block to run first (seeding
      the baseline from the true pre-snap pose), then applying the snap
      and reassigning the local `cube_position`/`cube_orientation` to the
      post-snap values so the rest of that frame's logic (the debounce
      check, and the trailing `past_pose`/`past_orientation` update) sees
      the fresh pose. **Verified live** after the fix: `plan_single
      success=True` for both phases, with real joint-position deltas
      (`1.8159` rad for the grasp-approach move, `0.6809` rad more for the
      subsequent assembly-placement move).
    - **A later session tried moving robot/friction ownership into
      `mefron.usd` itself (`scripts/mefron2.py`, see its own entry below),
      then reverted back to this file.** The user deleted the manually-
      placed Franka and its gripper-friction material back out of
      `mefron.usd` in the GUI and asked to return to this script's own
      code-driven `mount_franka()`/`apply_gripper_friction()`/
      `stiffen_gripper_drive()` pipeline ("lets go back to mefron.py i
      deleted the franka from mefron.usd and gripper friction do this
      reimport and physciacs from script itself"). This file is the active
      script again; the fixes below all landed here, not in `mefron2.py`.
    - **Motion was only uniformly slow in *relative* terms — far targets
      still moved fast enough to disturb a carried load — fixed via
      `velocity_scale`/`acceleration_scale`, not `time_dilation_factor`.**
      `_TELEOP_TIME_DILATION_FACTOR` (`0.3`, a `MotionGenPlanConfig`-level
      setting) only uniformly re-times an *already-planned* trajectory
      after the fact — confirmed via cuRobo's own source that this can't
      change the plan's relative speed profile, only stretch it, so a
      naturally-fast long move and a naturally-slow short move both just
      get stretched by the same factor and the long move still ends up
      faster in absolute terms. Fixed by capping the velocity/acceleration
      *limits the trajectory optimizer plans within* instead, via
      `MotionGenConfig.load_from_robot_config()`'s own `velocity_scale`/
      `acceleration_scale` kwargs (set once, at `MotionGen` construction
      time, in `setup_motion_gen()`). New module constants
      `_TELEOP_VELOCITY_SCALE = 0.2` / `_TELEOP_ACCELERATION_SCALE = 0.2`
      — deliberately kept in the `0.1`–`0.25` band: confirmed via direct
      source read that cuRobo treats `scale <= 0.25` as a first-class
      case, automatically swapping in a dedicated
      `finetune_trajopt_slow.yml` tuning file made for slow trajectories
      and increasing the trajectory's own time budget
      (`maximum_trajectory_dt`) to compensate, whereas going below `0.1`
      would additionally require setting `maximum_trajectory_dt` by hand.
      Scaling both velocity **and** acceleration (not just velocity) is
      deliberate: a low acceleration limit specifically damps sudden
      starts/stops/direction changes, which is what actually shakes a
      friction-held carried object loose, not just top speed.
    - **Gripper was closing too fast even with drive stiffness already
      tuned up for grip strength** ("the fingers are closing too quickly
      need to slow that also down"). Root cause: the gripper block in
      `run_teleop_loop()` commanded `GRIPPER_OPEN_POSITION`/
      `GRIPPER_CLOSED_POSITION` as an instant position-target jump every
      frame — with `stiffen_gripper_drive()`'s already-high stiffness/
      damping, a stiff position drive tracks a sudden setpoint change
      aggressively, snapping the fingers shut almost immediately. Fixed by
      ramping the *commanded setpoint* itself gradually instead of jumping
      straight to the target: new `gripper_setpoint`/`last_gripper_time`
      loop-local state, advanced toward the target by at most
      `GRIPPER_CLOSE_SPEED` (new module constant, `0.02` m/s — the full
      `0.04`m open↔closed travel takes about 2 seconds) times real elapsed
      wall-clock time (`time.time()`, same non-frame-based pacing pattern
      already used for trajectory playback) each frame. The drive itself
      is unchanged, so the same strong holding force applies once fully
      closed — only the approach to that setpoint is gradual now.
    - **Confirmed via source but not yet wired in: cuRobo has no awareness
      that the robot is carrying `finger_print_scanner` once grasped**,
      which is at least part of why a planned move crashes the carried
      part into `main_holder` instead of avoiding it ("why is cuRobo not
      avoiding the main holder side instead crashing into it?"). Confirmed
      via direct source read: `MotionGen` exposes
      `attach_objects_to_robot()`/`detach_object_from_robot()` specifically
      for this (treating a grasped object as rigidly attached to the
      robot's own collision body for the rest of planning, until
      detached), and `franka.yml` already has a pre-built `attached_object`
      link with 4 spare collision spheres sized for exactly this purpose,
      currently unused. The natural wiring point is the existing C/O
      gripper keybindings in `run_teleop_loop()` (call
      `attach_objects_to_robot()` on close, `detach_object_from_robot()` on
      open), but this is **not implemented** — paused mid-decision to
      check how placement accuracy behaves first ("wait first let me check
      how precisely this placement works"), and the session moved on to
      the grasp-centering problem below before returning to it.
    - **Still open, not yet fixed: a grasp-centering problem, confirmed to
      NOT be a per-finger joint/drive asymmetry.** Reviewing a screen
      recording of a grasp-close (5 frames extracted from the session's
      own `.webm` capture for review — see the repo's gitignored
      screencast files), the object visibly shifted sideways as the
      fingers closed. A per-finger drive/mimic-joint asymmetry between
      `panda_finger_joint1`/`panda_finger_joint2` looked plausible and was
      about to be investigated as the cause. **The user corrected this
      diagnosis directly**: "i wouldnt say fixed since the central mount is
      closer to the right finger joint it reached first and then the left
      joint comes" — i.e. `finger_print_scanner` isn't equidistant from
      both fingertips at the moment closing begins (a grasp-pose centering
      issue), so one finger contacts and starts pushing the object before
      the other one arrives, rather than both sides closing onto it
      symmetrically. This remains unresolved — no fix has been attempted.
      Two directions were discussed but neither started: re-derive
      `GRASP_OFFSET_POSITION` checking explicitly that it's equidistant
      from both fingertips at grasp time, or derive it from the gripper's
      own finger-midpoint frame instead of `ee_link` directly. The
      per-finger joint-asymmetry hypothesis above was explicitly rejected
      by the user — don't re-investigate it without new evidence.
- `scripts/test_mefron_teleop_headless.py` — headless regression test for
  `mefron.py`'s `run_teleop_loop()`, mirroring `test_teleop_headless.py`'s
  established pattern for `build_scene.py` (reuses `mefron.py`'s own
  functions as a library, fakes a target drag via monkeypatching
  `target.get_world_pose()`). Used to confirm the
  `/app/player/playSimulations` fix above. **Verified**: `plan_single
  success=True`, real joint-position deltas.
- `scripts/test_mefron_assembly_headless.py` — headless regression test
  for Step 6's `compute_grasp_approach_pose()`/
  `compute_assembly_grasp_target()` and the G/P one-shot snap requests,
  simulating a keypress by calling
  `gripper_control.request_grasp_approach()`/`request_assembly_target()`
  directly rather than a real keyboard event (indistinguishable to
  `run_teleop_loop()`, which only ever reads the request through its own
  `consume_*_request()` methods). Runs both phases in sequence in one
  process. **Verified**: caught the debounce-ordering bug above via its
  own failure (sane pose math, zero `plan_single` calls) before the fix,
  then passed cleanly after.
- `scripts/franka_grasp_editor_scene.py`, `scripts/mefron_grasp_editor_scene.py`
  — diagnostic scenes built while chasing the Grasp Editor bug documented
  above; kept as working artifacts for future Grasp Editor use on a
  different robot/asset combination, not currently part of any regular
  workflow since T_S_G was derived via `compute_relative_pose()` instead.
- `configs/scene/mefron_layout.yaml` + `scripts/build_scene_mefron.py` —
  originally written as the **preferred** approach for the mefron scene,
  in place of `scripts/mefron.py` above. Same overall goal (mount the
  Franka, run cuRobo teleop) but built the way `build_scene.py` itself is:
  a fresh, anonymous `SimulationApp` stage with `mefron.usd` brought in via
  `add_reference_to_stage()` (under `/World/Factory`), not opened
  directly. This one architectural difference avoids essentially every
  bug found in `scripts/mefron.py` above *by construction*, confirmed
  live (see below) — **but in practice, all of this session's active
  interactive work (grasp/assembly tuning, speed/gripper fixes, pose
  re-derivations) happened directly in `scripts/mefron.py`, not this
  file**, because deriving T_H_S required temporarily reparenting
  `finger_print_scanner` under `main_holder` in the Stage tree, which
  only works against `mefron.usd` opened directly — `build_scene_mefron.py`'s
  own referenced-stage session hits the "Cannot move/rename ancestral
  prim" restriction for that (see the T_H_S entry under `scripts/mefron.py`
  above). Treat `build_scene_mefron.py` as verified-and-working but
  currently dormant, and `scripts/mefron.py` as the actually-active script,
  until/unless something forces a switch back:
    - Since the stage's root layer stays anonymous/in-memory (same as
      `build_scene.py`'s own stage always has), the URDF importer never
      triggers the file-backed "Robot Description" multi-layer write (see
      `assets/mefron/`'s entry above) — `build_teleop_target()`'s
      original, unmodified `CopyPrim` approach (verbatim from
      `build_scene.py`, no internal-reference workaround needed) produces
      a target with real geometry on the first try.
    - `mefron.usd`'s own `/PhysicsScene` lives at `/PhysicsScene`
      (a sibling of `/World`, not nested inside it) in the source file —
      `add_reference_to_stage()` only brings in the referenced prim's own
      subtree (mefron's `/World` and everything under it), so this
      sibling prim is never pulled onto the new stage at all. No
      duplicate-scene conflict to work around; `run_teleop_loop()`'s
      unmodified `/physicsScene` check just creates the one and only
      scene, same as it does for `build_scene.py`'s own main scene.
    - Referencing `mefron.usd` under `/World/Factory` nests its own
      content one level deeper than opening it directly would: mefron's
      own `/World/Factory` (its internal factory floor) becomes
      `/World/Factory/Factory` here, and its `/World` siblings
      (`packing_table`, `finger_print_scanner`, etc.) become
      `/World/Factory/packing_table` etc. Confirmed empirically (not
      assumed) via a live reference-and-inspect script — world positions
      of nested content are unaffected (both stages are meters-native, no
      scale reconciliation needed), only prim *paths* shift.
    - The Stop→Play stale-`SingleArticulation` fix (see `mefron.py`'s
      entry above) is ported here too and **re-verified independently** in
      this file's own architecture: first-play fake-drag `0.2886` rad
      end-effector movement, then a real `timeline.stop()`/`play()` cycle
      in-process, then a second fake-drag `0.3378` rad movement — both
      `plan_single success=True`.
  Also loads `SimulationApp` with the **full** `isaacsim.exp.full.kit`
  experience (same one `isaac-sim.sh` itself launches) instead of
  `SimulationApp`'s own default minimal `isaacsim.exp.base.python.kit`,
  for interactive (non-`--headless`) runs only — the base experience is
  missing most UI extensions, including the Physics debug-visualization
  menu needed to view collision meshes. **Real bug found and fixed**:
  switching to the full experience broke cuRobo's own `from packaging
  import version` (inside `curobo/util/torch_utils.py`) with
  `FileNotFoundError: .../omni.services.pip_archive-.../pip_prebundle/
  packaging/_structures.py` — a *different* extension bundles its own
  incomplete internal `packaging` copy (missing `_structures.py`, an
  older `packaging` release than the real one) that somehow takes
  priority under the full experience. Confirmed this is **not** the
  same kind of `sys.path`-ordering shadow already documented for `torch`
  below: a full `sys.path` dump under the full experience never contains
  any path under that extension at all, yet
  `importlib.util.find_spec("packaging")` still resolves there — some
  other, non-path-based resolution (almost certainly a custom
  `sys.meta_path` finder the extension system registers) is responsible,
  and it turned out to intercept `packaging.version` specifically by
  name too, ignoring the parent module's own `__path__` even after
  pre-registering a correct `packaging` in `sys.modules`. Fixed by
  explicitly pre-loading *both* `packaging` and `packaging.version` from
  their real `site-packages` location and setting the latter as a plain
  attribute on the former, so `from packaging import version` resolves
  via attribute lookup alone, with no further import-machinery
  involvement for either name — confirmed live this survives the full
  experience and reaches `curobo motion_gen: READY` same as before.
  Applied to both this file and `scripts/mefron.py` for consistency.
    - **Mount remount: pedestal → SEKTION table.** The old
      `Pedestal_plates/Cube_05` mount plate was removed from `mefron.usd` in
      the GUI and replaced with a pre-authored SEKTION cabinet table
      (`/World/sektion_cabinet_instanceable`, a `/World` sibling of
      `Factory` in mefron.usd's own raw hierarchy — becomes
      `/World/Factory/sektion_cabinet_instanceable` here after this file's
      own one-level nesting, see above). `cr5_mount.position`
      (`[2.74097, -4.782, 0.7924]`) was read directly off a manually-placed
      Franka copy's Property-panel Translate/Orient in the GUI, not
      independently re-derived via a `get_world_pose()`/`BBoxCache` script
      like most other poses in this project — worth re-checking first if
      the robot ends up floating/clipping through the table.
      `cr5_mount.pedestal` was renamed to **`cr5_mount.mount_surface`** in
      `mefron_layout.yaml` (`get_teleop_obstacles()` and `main()`'s
      status-print list both use the new key). `teleop_target.position` was
      carried forward **algebraically** (old target minus old mount,
      applied to the new mount position) rather than re-derived from
      scratch — valid only because `cr5_mount.orientation_wxyz` is
      unchanged (still identity); recompute properly, don't just shift,
      if the mount orientation ever changes.
    - **Grasp-physics fixes: `apply_gripper_friction()` /
      `stiffen_gripper_drive()`.** Read-only inspection of `mefron.usd`
      found **zero** `PhysxMaterialAPI` authored anywhere (not on
      `finger_print_scanner`/`main_holder`/`screen`, not a usable one on
      `backpanel_support`'s pure-render `Black_Paint_01` material, no
      `PhysicsScene`-level default), and confirmed the Franka side has none
      either (`franka_panda.urdf` has no friction tags; `import_cr5.py`
      sets none) — both sides of every grasp contact were relying on
      PhysX's un-overridden engine default friction, which is what was
      causing `finger_print_scanner` to slip out of the gripper regardless
      of its mass. `apply_gripper_friction()` creates one shared material
      at `/World/GripperFrictionMaterial`
      (`GRIPPER_STATIC_FRICTION=0.9`/`GRIPPER_DYNAMIC_FRICTION=0.8`,
      restitution 0.0) via the real
      `omni.physx.scripts.utils.addRigidBodyMaterial()`/
      `physicsUtils.add_physics_material_to_prim()` helpers, bound to both
      Franka fingertip links and everything listed in the new
      `high_friction_prim_paths` config key (currently just
      `finger_print_scanner`). Separately, a headless inspection of the
      actual imported joint prims
      (`/World/CR5/joints/panda_finger_joint1|2`, `UsdPhysics.DriveAPI`
      type `"linear"`) found `stiffness=625.0`/`damping=10.0` — not the
      configured `default_drive_strength=1047.2`/
      `default_position_drive_damping=52.36` (the URDF importer derives a
      different effective value for prismatic joints, and the URDF's own
      `<dynamics damping="10.0"/>` on these two joints wins over the
      importer's default damping), leaving most of the fingers' real
      `effort="20"` N ceiling unused for a typical 1-2cm grasp position
      error (~6-12N reached). `stiffen_gripper_drive()` raises both to
      `GRIPPER_DRIVE_STIFFNESS=10000.0`/`GRIPPER_DRIVE_DAMPING=200.0` via
      `UsdPhysics.DriveAPI` on both finger joints. Both fixes are
      **runtime-only, not persisted** to `mefron.usd` — deliberate, matches
      how every other robot-tied property in this scene works. Headless
      read-back confirmed the material's friction values and the joints'
      drive values land exactly as configured; **not yet live-tested**
      whether the combination actually produces a firm, non-slipping grip
      in the GUI.
    - **Keyboard gripper control.** `GripperKeyboardControl` +
      `build_gripper_keyboard_control()` subscribe to real `carb.input`
      keyboard events (**C** closes, **O** opens), confirmed against this
      install's own stubs (`carb/input.pyi`,
      `omni/appwindow/_appwindow.pyi`) rather than assumed from memory.
      `GRIPPER_OPEN_POSITION=0.04`/`GRIPPER_CLOSED_POSITION=0.0` come from
      `franka_panda.urdf`'s actual joint limits (`panda_finger_joint1/2`,
      prismatic, `lower="0.0" upper="0.04"`). Wired into `run_teleop_loop()`
      via an optional `gripper_control` param, applied every playing frame
      *after* the arm's own `cmd_plan` block so it always wins that frame's
      write to the finger joints (`get_full_js()` re-applies cuRobo's own
      `lock_joints`-locked-open value on every planned frame otherwise).
      Chosen over the `isaacsim.robot_setup.grasp_editor` tool because
      `franka.yml` already excludes the two finger joints from IK/trajopt
      entirely, so finger actuation was always going to be orthogonal to
      cuRobo regardless — Grasp Editor only pays off once something
      computes a grasp target pose autonomously, which nothing here did at
      the time this decision was made. **Verified headlessly** (joint-drive
      mechanism only, via the `set_closed()` test hook the class exposes
      for exactly this purpose); **not yet verified** with a real
      interactive keypress in the GUI, and no permanent headless regression
      test exists yet for plain open/close specifically
      (`test_mefron_assembly_headless.py` exercises this file's G/P
      snap-to-pose requests instead — a different code path from `C`/`O`).
    - **Trajectory playback was running too fast with an arrival
      oscillation — root-caused to a frame-vs-time mismatch, not a
      stiffness/damping problem.** `get_interpolated_plan()` spaces
      waypoints `interpolation_dt` seconds apart (`0.02s`), but this loop's
      own render rate (confirmed live at ~119 FPS, far above the 50Hz the
      plan assumes) has nothing to do with that — applying one waypoint per
      render *frame* instead of one per `interpolation_dt` played the whole
      trajectory back at roughly 2.4x its intended speed and cut its final
      deceleration-to-zero-velocity ramp short, leaving real residual
      velocity for the position-hold drive to absorb once `cmd_plan` ran
      out — the actual cause of both the too-fast motion and the arrival
      oscillation. Fixed by gating playback on real elapsed time
      (`time.time()`) against each plan's own `result.interpolation_dt` (a
      `MotionGenResult`-level value — `MotionGen` itself exposes no such
      attribute) instead of one waypoint per render frame.
- `scripts/mefron2.py` — a simplified sibling of `scripts/mefron.py` built
  for a "everything already baked into `mefron.usd`, no code-driven
  import" approach: assumes the Franka and its gripper-friction material
  are already saved directly into `mefron.usd` (via Isaac Sim's own GUI
  robot-asset import — NVIDIA's bundled Nucleus Franka Panda asset, not
  this repo's URDF-import pipeline), so this script does no import and no
  friction/drive-stiffness authoring at all — only cuRobo setup, the
  draggable teleop target, and the G/P/C/O controls, all ported from
  `mefron.py`. Two real, confirmed-live differences from `mefron.py`'s
  equivalents were needed:
    - `build_teleop_target()`'s first attempt — a plain `CopyPrim` from
      `panda_hand/geometry` — produced an **empty bounding box**, confirmed
      via `UsdGeom.BBoxCache`. Root cause: `geometry` is only
      `instanceable=True` metadata pointing at an instance; `CopyPrim`'s
      shallow, spec-level copy carries the instanceable flag but not the
      composition arc needed to resolve it, leaving a hollow shell. Fixed
      by resolving the actual instance-proxy `Mesh` prim underneath
      `geometry` first (via `Usd.TraverseInstanceProxies()`), then
      `CopyPrim`-ing from *that* already-resolved path — confirmed live
      this produces a correct non-empty bbox, and (separately, tested via a
      diagnostic script, not just assumed) `check_ancestral()==False` and a
      real `MovePrim` reparent succeeds, unlike the `AddInternalReference()`
      approach `mefron.py` uses for its own (differently-sourced) Franka.
    - NVIDIA's bundled Franka asset bakes real `UsdPhysics.CollisionAPI`
      onto that same mesh prim (visuals and collision aren't split into
      separate prims the way a from-scratch URDF import keeps them), so
      the copied target inherited a real, live collider — confirmed via a
      real PhysX overlap query that it was already geometrically
      overlapping the actual robot's own nearby links. Since the resolved
      copy (unlike a plain instance proxy) is a genuine, editable prim,
      `RemoveAPI(UsdPhysics.CollisionAPI)` works directly on it with no
      further workaround needed.
    - `MOUNT_POSITION`/`MOUNT_ORIENTATION_WXYZ` aren't hardcoded here since
      there's no mount step — `get_robot_base_pose()` reads the
      already-placed robot's real live world pose off the stage once at
      startup instead, correct regardless of exactly where the robot was
      manually placed when it was saved into `mefron.usd`.
  **Verified working** at the time it was built: headless run reaches
  `curobo motion_gen: READY`, all status paths `OK`, and a fake-drag test
  gives `plan_single success=True` with a non-empty target bbox and no
  articulation errors. **Now superseded**, not actively used: the user
  later deleted the manually-placed Franka and its gripper-friction
  material back out of `mefron.usd` in the GUI and asked to return to
  `scripts/mefron.py`'s own code-driven pipeline instead (see that file's
  own entry above for the revert). This file is kept as a working artifact
  for its CopyPrim/instance-proxy-resolution technique, but as of that
  revert it no longer matches what's actually saved in `mefron.usd` (no
  Franka, no friction material) — it would need a fresh Franka re-added to
  `mefron.usd` by hand (from `robots/franka_panda/`, below) before it could
  run again; treat it as a reference, not as ready-to-run.
- `robots/franka_panda/` — a local, Content-Browser-"Collect Asset"-vendored
  copy of NVIDIA's own Nucleus-hosted Franka Panda asset (public,
  unauthenticated S3 bucket; see the directory's own `SOURCE.md` for the
  exact URL), built specifically to unblock `scripts/mefron2.py`: the
  Nucleus-hosted original's link geometry is `instanceable=True`, and USD
  refuses to author anything — including `SetInstanceable(False)` — onto a
  *read-only, Nucleus-backed* instance proxy. Collecting the asset locally
  (which also pulls in every file it references, unlike a plain `curl` of
  `franka.usd` alone) makes it a real, locally-editable file instead.
  ~39MB, gitignored like every other vendored asset pack in this repo
  (`robots/franka_panda/*` / `!robots/franka_panda/SOURCE.md`, added to
  `.gitignore` this session) — same NVIDIA Omniverse License Agreement
  content-pack terms as `assets/factory/`/`assets/mefron/`. Now effectively
  dormant along with `mefron2.py` itself, kept only because that script
  still references it.
- **`main_holder` convex-decomposition collision tuning — researched and
  confirmed against this Isaac Sim install's actual schema, not yet
  applied.** Switching `main_holder`'s collider from Convex Hull to Convex
  Decomposition (via GUI, needed for the same reason as
  `finger_print_scanner`'s own collider — see 3b under `scripts/mefron.py`'s
  grasp-physics findings above) made the part sink slightly into the table
  and lose its small mounting studs. A research **Workflow** (3 parallel
  research agents + 3 adversarial verify agents, every claim grounded
  against this install's real schema files, not memory) confirmed:
    - Schema: `PhysxSchema.PhysxConvexDecompositionCollisionAPI`
      (single-apply), applied alongside `UsdPhysics.MeshCollisionAPI` with
      `approximation="convexDecomposition"`. Real schema defaults:
      `hullVertexLimit=64`, `maxConvexHulls=32`, `minThickness=0.001`,
      `voxelResolution=500000`, `errorPercentage=10`, `shrinkWrap=False`.
    - Mechanism (VHACD-family: voxelize → cluster → convex-hull-per-cluster
      → optional shrink-wrap re-projection): **sinking** happens because
      `shrinkWrap` defaults to `False`, so nothing re-projects the
      voxel-quantized hull back onto the true surface. **Small-feature
      loss** happens because `voxelResolution` is a budget spread over the
      *whole part's bounding box*, not per-feature — mm-scale studs on a
      much larger flat part can fail to rasterize at all, or get merged
      away during the volume-error-driven clustering step.
    - `main_holder`'s actual collider prim (confirmed via headless
      inspection, not assumed by analogy):
      `/World/Factory/main_holder/tn__mainholder_kA` — `approximation`
      is `convexHull` on-disk in `mefron.usd` as of this check (any live
      GUI edit to `convexDecomposition` is session-local until saved).
    - Recommended values, given to the user as a GUI walkthrough, **not**
      implemented in code — deliberately: the user pushed back on
      hardcoding per-part collision tuning as not scalable, and this is an
      asset-intrinsic property of `mefron.usd` itself, unlike the
      robot-tied friction/stiffness fixes above which are legitimately
      runtime concerns: **Shrink Wrap → ON** (fixes sinking), **Voxel
      Resolution → ~3,000,000–5,000,000** (fixes stud loss; hard ceiling is
      5,000,000), **Max Convex Hulls → ~128** (secondary, budget for small
      features), **Error Percentage → ~1–2** (secondary), Hull Vertex Limit
      and Min Thickness left at defaults — then **save `mefron.usd`**
      (`Ctrl+S`), the one fix in this investigation meant to persist into
      the asset file directly rather than be reproduced by code. **Not yet
      applied/tested** as of the last check (`mefron.usd` isn't tracked in
      git, so this can't be re-verified from the repo alone — check live
      before assuming it's still pending).
- `scenes/cell_scene.usda` — a small, hand-written USD stage
  (`defaultPrim=/World`, `metersPerUnit=1`) that references
  `assets/factory/Factory.usd` under `/World/Factory` and nothing else — no
  CR5/Franka, no ergo tables, no pruning. Not referenced by any script in
  this repo (confirmed via search); origin and intended use are unclear as
  of this check — likely a manual scratch/exploration stage from GUI work,
  not a pipeline artifact. Investigate before building on it, in case it's
  in-progress work rather than dead weight.
- `solidworks_transform_extraction.md` — notes on an alternative,
  SolidWorks-side method (Coordinate Systems + Measure, or direct mate
  values) for extracting the `finger_print_scanner`→`main_holder` relative
  transform (`T_part_target`) at the CAD-authoring stage, instead of
  deriving it live in Isaac Sim. **Superseded in practice**: the actual
  `ASSEMBLY_RELATIONSHIPS["finger_print_scanner_on_main_holder"]` value
  used in `scripts/mefron.py` was instead derived live via
  `compute_relative_pose()` on both parts' Isaac Sim world poses at a
  manually-aligned position (the T_H_S finding, see `scripts/mefron.py`'s
  entry above) — kept as a reference for a CAD-side alternative, not part
  of the executed pipeline.
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
- **Interactive teleop's real-time/GUI behavior isn't verified for
  `build_scene.py`/`table_layout.yaml` specifically** — only what a
  headless run can prove (see `scripts/build_scene.py`'s and
  `scripts/test_teleop_headless.py`'s entries above: obstacle scan
  scope/timing, `update_world()`, and a scripted fake-drag all confirmed
  with the temporary Franka). The *same* `run_teleop_loop()` pattern
  (identical debounce/plan/apply logic) has since been GUI-verified for
  real, in a sibling scene — see `scripts/mefron.py`/
  `scripts/build_scene_mefron.py`'s entries above, which confirmed real
  mouse-drag responsiveness, the ghost end-effector copy reading
  correctly next to the real robot, and the Press-Play-to-start branch,
  all live — which gives good confidence the same logic works here too,
  but `build_scene.py`/`table_layout.yaml` itself still hasn't had that
  exact manual GUI smoke test. **One real, confirmed bug from that GUI
  testing is still latent and unfixed here specifically**: `run_teleop_loop()`
  only ever builds its `SingleArticulation` once, on the very first Play
  (gated by `idx_list is None`) — clicking **Stop** in the GUI tears down
  PhysX's simulation view entirely, and reusing that now-stale
  `SingleArticulation` after a later Play leaves the robot permanently
  unresponsive (endless "Physics Simulation View is not created yet" in
  the log) for the rest of that process. Confirmed and fixed in the
  mefron scripts (track not-playing→playing transitions, rebuild
  `robot`/`idx_list`/`articulation_controller` and reset per-session
  state on *every* fresh Play) but that fix has not been ported into this
  file.
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
- **`build_scene_mefron.py`'s grasp-physics fixes** (`apply_gripper_friction()`,
  `stiffen_gripper_drive()`) — headless read-back confirmed the friction
  material and drive values land exactly as configured, but whether the
  combination actually produces a firm, non-slipping, non-dangling grasp on
  `finger_print_scanner` has not been tested live in the GUI.
- **Keyboard gripper open/close (`C`/`O`) in `build_scene_mefron.py`** —
  the underlying joint-drive mechanism is headlessly verified (via the
  `GripperKeyboardControl.set_closed()` test hook), but a real interactive
  keypress in the GUI hasn't been tried, and there's no permanent headless
  regression test for plain open/close specifically (only the separate G/P
  snap-to-pose request path in `mefron.py` has one, via
  `test_mefron_assembly_headless.py`).
- **`main_holder`'s convex-decomposition collision tuning** (Shrink Wrap
  on, Voxel Resolution ~3-5M, Max Convex Hulls ~128, Error Percentage ~1-2
  — see its own "Done" entry above for the full research) is a
  recommendation only — apply it via the GUI, confirm live that it fixes
  the sinking/stud-loss symptoms, and save `mefron.usd`.
- **Grasp-centering problem in `scripts/mefron.py` — open, unresolved.**
  `finger_print_scanner` isn't equidistant from both fingertips at the
  moment the gripper starts closing, so one finger contacts and pushes the
  object before the other arrives, shifting it sideways instead of
  squeezing it symmetrically (see that file's own entry above for the full
  diagnosis and the explicitly-rejected joint-asymmetry hypothesis). No
  fix attempted yet — re-derive `GRASP_OFFSET_POSITION` checking
  equidistance from both fingertips, or derive it from the gripper's
  finger-midpoint frame instead of `ee_link`, are the two directions
  discussed but not started.
- **`attach_objects_to_robot()`/`detach_object_from_robot()` wiring in
  `scripts/mefron.py`** — confirmed via cuRobo source that this is the
  right mechanism for making planning aware of a carried
  `finger_print_scanner` (see that file's own entry above), and
  `franka.yml` already has a pre-built `attached_object` link ready for it,
  but it's not wired into the C/O gripper keybindings yet. Paused pending
  the user's own check of "how precisely this placement works" — confirm
  that's resolved before implementing, in case it changes the approach.
- **`_TELEOP_VELOCITY_SCALE`/`_TELEOP_ACCELERATION_SCALE = 0.2` and
  `GRIPPER_CLOSE_SPEED = 0.02`** (both in `scripts/mefron.py`, see that
  file's own entry above) were applied in direct response to the user
  reporting fast-target motion disturbing a carried load and the gripper
  snapping shut too quickly — the mechanism for both is confirmed correct
  against cuRobo/PhysX source, but neither was independently re-confirmed
  live afterward against the *original* complaints specifically (the
  session moved on to the separate grasp-centering investigation above
  before circling back). Worth a quick live re-check that both actually
  feel right before assuming these constants are final.
- **The re-derived `GRASP_OFFSET_POSITION`/`GRASP_OFFSET_ORIENTATION_WXYZ`
  and `ASSEMBLY_RELATIONSHIPS["finger_print_scanner_on_main_holder"]`
  values** (`scripts/mefron.py`, re-derivation documented inline in that
  file's own entry above) fixed the T_H_S/T_S_G values being visibly off,
  but placement is still described as off on the Y axis — likely the same
  grasp-centering problem above rather than a T_S_G derivation error, but
  not explicitly confirmed as the same root cause versus a second,
  independent issue.

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
  install. **Fixed**: `ninja-build` is now installed via `apt-get` in
  `Dockerfile.curobo` (not `pip install ninja`, since pip itself is
  broken here) — confirmed live that `build_scene.py --headless` now
  JIT-compiles all five of cuRobo's CUDA kernels
  (`kinematics_fused_cu`, `geom_cu`, `tensor_step_cu`, `lbfgs_step_cu`,
  `line_search_cu`) cleanly and reaches `curobo motion_gen: READY`,
  where it previously crashed with `undefined symbol:
  _ZN3c104cuda29c10_cuda_check_implementationEiPKcS2_ib` (the torch ABI
  mismatch) immediately followed by `RuntimeError: Ninja is required`.
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
- Calling `timeline.play()` before physics has a real chance to settle
  corrupts PhysX's tensor simulationView — confirmed live in two distinct
  ways (see `scripts/test_teleop_headless.py`'s and `scripts/mefron.py`'s
  entries above): playing before `/physicsScene` even exists on the
  stage, and playing before a long blocking call (cuRobo's
  `motion_gen.warmup()`, ~30s, which calls no `simulation_app.update()`
  of its own) that leaves physics "playing" across an unpumped real-time
  gap. Both produce the identical downstream symptom: a later
  `SingleArticulation(...)` construction crashes with `AttributeError:
  'NoneType' object has no attribute 'link_names'`. Any script driving
  `timeline.play()` itself (rather than leaving it to a human clicking
  Play in the GUI, this repo's usual pattern) needs to do so only *after*
  both the physics scene exists and any blocking warmup work is done.
- A `SingleArticulation` object is only valid for the specific PhysX
  simulation view that existed when it was constructed — clicking **Stop**
  in the GUI tears that view down entirely, and reusing a
  `SingleArticulation` built before the Stop after a later Play leaves it
  permanently broken (`get_joints_state()` never returns non-`None` again;
  `isaacsim.core.prims.impl.articulation` logs "Physics Simulation View is
  not created yet" forever). Any interactive loop that only builds its
  `SingleArticulation` once (gated by e.g. `idx_list is None`, checked
  just on the first Play) needs to instead track not-playing→playing
  *transitions* and rebuild it (plus reset any other per-session state)
  on every fresh Play, not just the first — see `scripts/mefron.py`'s
  and `scripts/build_scene_mefron.py`'s `run_teleop_loop()` for the
  pattern; `scripts/build_scene.py`'s own copy still has this bug (see
  "Needs verification" above).
- Isaac Sim's URDF importer behaves differently depending on whether the
  target stage's root layer is a real, file-backed USD file
  (`omni.usd.get_context().open_stage()`) or anonymous/in-memory (the
  default for a fresh `SimulationApp`, or content brought in via
  `add_reference_to_stage()` into such a stage). Confirmed live (see
  `assets/mefron/`'s and `scripts/mefron.py`'s entries above): only the
  file-backed case writes a disk-persisted, multi-layer "Robot
  Description" structure (a `configuration/` folder of sublayer `.usd`
  files) as a side effect of import, with no save prompt — and that
  extra layering breaks `CopyPrim`-based prim duplication (a shallow,
  spec-level copy that can't correctly re-resolve a same-layer reference
  once relocated across the resulting more complex layer stack). Prefer
  building scenes the way `build_scene.py` itself does — a fresh
  anonymous stage with external content brought in via
  `add_reference_to_stage()` — over opening an existing authored `.usd`
  file directly, when the script needs to import a robot into it; this
  sidesteps the whole class of bug rather than working around it (an
  internal USD reference, `prim.GetReferences().AddInternalReference()`,
  is the workaround if opening the file directly is unavoidable — see
  `scripts/mefron.py`'s `build_teleop_target()`).
- `SimulationApp`'s default experience (`isaacsim.exp.base.python.kit`)
  is missing most UI extensions, including the Physics debug-
  visualization menu (needed to view collision meshes in the viewport).
  Pass `experience=f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'`
  (the same experience `isaac-sim.sh` itself launches) to get the full
  menu bar for interactive runs — see `scripts/mefron.py`'s/
  `scripts/build_scene_mefron.py`'s `SimulationApp(...)` construction.
  **Gotcha, confirmed live**: doing this breaks cuRobo's own `from
  packaging import version` (inside `curobo/util/torch_utils.py`) —
  the full experience's extra extensions make `packaging`/
  `packaging.version` resolve to a different, incomplete internal
  pip-bootstrap bundle instead of the real `site-packages` install, and
  confirmed this is *not* a `sys.path`-ordering issue like the `torch`
  shadow above (that bundle's path never appears in `sys.path` at all,
  and pre-registering a correct `sys.modules["packaging"]` alone still
  didn't stop `packaging.version` specifically from resolving wrong —
  some other, non-path-based resolution, almost certainly a custom
  `sys.meta_path` finder the extension system registers, intercepts the
  submodule by name regardless of the parent module's own `__path__`).
  Fixed by explicitly pre-loading both `packaging` and `packaging.version`
  from their real `site-packages` files and setting the latter as a
  plain attribute on the former, so `from packaging import version`
  resolves via attribute lookup alone — see the top of
  `scripts/mefron.py`/`scripts/build_scene_mefron.py` for the pattern.

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
