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
- `docker/.env.base` — Isaac Sim image + path env vars. Pin bumped to
  `6.0.1` (from `5.1.0`) to pick up Isaac Sim's new Newton physics backend.
  **Verified**: rebuilt side-by-side (not overwriting the working `:latest`
  5.1.0 images) via `docker/container.py build curobo --suffix 601`,
  producing `isaac-cobot-base-601`/`isaac-cobot-curobo-601`; GPU/torch/CUDA
  confirmed live inside the new container (`NVIDIA RTX PRO 4000 Blackwell`,
  torch `2.11.0+cu128`, real `cuda:0` matmul), and Isaac Sim's own
  `/isaac-sim/VERSION` confirms `6.0.1-rc.7`. Bonus finding: the
  `pip._vendor.packaging._structures` bug that broke `pip` inside the
  5.1.0 image (see the `ninja`/pip Conventions bullet) appears **fixed
  upstream** in 6.0.1 — `python.sh -m pip --version` now reports cleanly.
- `docker/.env.curobo` — pinned cuRobo commit hash.
- `docker/.env.newton` — pin for *standalone* NVIDIA Newton
  (`newton-physics/newton` on GitHub; GPU physics built on Warp, no Isaac
  Sim dependency) — a fast way to smoke-test Newton/Warp/CUDA on this GPU
  without waiting on a full Isaac Sim image rebuild. Recorded for
  visibility only; nothing under `docker/` actually consumes this file
  (Newton isn't installed in any Docker image). Not to be confused with
  Isaac Sim 6.0+'s own *bundled* Newton physics backend (a separate
  integration, see `scripts/newton_backend.py` below), which ships inside
  the `isaac-cobot-*` images themselves and is pinned via
  `docker/.env.base`'s `ISAACSIM_VERSION` instead.
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
  pick up the new mounts. **Now verified end-to-end with a live GUI
  launch**: opened the `cuRobo` devcontainer through VS Code's picker and
  got a real Isaac Sim 6.0.1 window showing the built scene (factory,
  ergo tables, mounted robot) — this X11 forwarding genuinely works, not
  just configured-but-untested.

  **Second bug found and fixed the same session, unrelated to X11**: both
  `build-images.sh`/`docker-compose.devcontainer.yaml` pairs originally
  built/referenced bare image tags (`isaac-cobot-base`, `isaac-cobot-curobo`)
  — but Docker image tags are global on this machine, **not git-branch-
  scoped**. Since `newton` pins a different Isaac Sim version than `main`
  (`docker/.env.base`: `6.0.1` vs. `main`'s `5.1.0`) but both branches'
  devcontainer files used the same bare tag names, opening one branch's
  devcontainer after the other's could silently reuse or overwrite the
  wrong Isaac Sim version's image — confirmed as a real, not hypothetical,
  risk (it happened this session: promoting `newton`'s rebuilt images to
  `:latest` for convenience briefly meant `main`'s own devcontainer would
  have silently picked up 6.0.1 instead of its pinned 5.1.0). Fixed by
  suffixing every image tag this branch's devcontainer files build/reference
  with the Isaac Sim version pin, dots stripped (`-601`, derived from
  `ISAACSIM_VERSION` in `build-images.sh`, hardcoded to match in the
  compose files since compose has no easy access to that shell variable at
  the point VS Code evaluates it) — `main`'s devcontainer files are
  untouched and still build/reference the bare tags, so the two branches
  can no longer collide. Keep the compose files' hardcoded suffix in sync
  with `docker/.env.base` if `ISAACSIM_VERSION` is bumped again on this
  branch.
- `assets/factory/` — vendored factory-floor scene (NVIDIA USD Explorer
  Sample Assets Pack; NVIDIA Omniverse License Agreement, not open source).
  The ~404MB `Factory.usd` + `SubUSDs/` payload is gitignored — only
  `assets/factory/SOURCE.md` (provenance + re-fetch instructions) is
  tracked.
- `configs/scene/table_layout.yaml` — factory backdrop path + pruning
  rules, `ergo_tables` (the two reused work-surface copies), `cr5_mount`
  (robot pose/scale + its reused pedestal), and `physics_backend` (Newton
  toggle, see `scripts/newton_backend.py` below). **Verified**:
  `scripts/build_scene.py` builds this end-to-end against a live Isaac Sim
  6.0.1 install (real GPU), Newton physics backend enabled by default —
  `/World/Factory` composes with 8 children, both `ErgoTable_1`/
  `ErgoTable_2` copies render with real geometry, `/World/CR5` imports
  (currently the temporary Franka — see `cr5_mount.robot_override` below),
  and the reused `RobotPedestal` keeps its geometry. Previously verified
  against 5.1.0, where `/World/CR5` reported 18 children directly —
  re-verifying against 6.0.1 found this is now 4 (`Geometry`/`Physics`/
  `Materials`/`VisualMaterials` scopes), a deliberate reorganization in
  Isaac Sim 6.0.1's redesigned URDF importer (see `scripts/import_cr5.py`
  below), not a lost-content regression: the full subtree still has 54
  prims and all 6 joints, confirmed by walking it with `Usd.PrimRange`.
  Three real, non-obvious findings baked into this config, each with its
  own inline comment at the point of use:
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
- `scripts/build_scene.py` — **verified** (see above), including against
  6.0.1 with Newton enabled by default. Also warms up a cuRobo `MotionGen`
  matching whichever robot is mounted (`setup_curobo_motion_gen()`) —
  best-effort, skipped with a printed message if cuRobo isn't installed
  (the `base` Docker profile). Two real bugs found and fixed: (1) this
  script (like every other standalone script here) creates a
  `SimulationApp` at import time; it originally did this unconditionally,
  which segfaults instead of raising if something else imports it as a
  library after already starting one — fixed by guarding that line behind
  `if __name__ == "__main__":`, same pattern as `import_cr5.py`; (2) under
  Isaac Sim 6.0.1 with Newton enabled, `world.reset()`/physics
  initialization crashed outright (`cannot access local variable
  'cmp_i_diag'`, inside Isaac Sim's own bundled
  `isaacsim.pip.newton/pip_prebundle/newton/_src/sim/builder.py`) as soon
  as the mounted robot's articulation was actually stepped — root-caused
  to `import_cr5.py`'s `link_density` fix, see that entry below.
- `scripts/import_cr5.py` — **verified** against a live Isaac Sim 6.0.1
  install, both standalone and imported as a library. Previously verified
  against 5.1.0. Two real bugs found and fixed while re-verifying against
  6.0.1:
    - Isaac Sim 6.0.1 no longer registers the `"URDFCreateImportConfig"`/
      `"URDFParseAndImportFile"` kit commands this file used to rely on at
      all (`Can't execute command... it wasn't registered`) — they now live
      behind the `isaacsim.asset.importer.urdf.ui` extension and are
      themselves deprecated in favor of a directly-constructible
      `isaacsim.asset.importer.urdf.URDFImporterConfig` dataclass +
      `URDFImporter` class (confirmed by reading
      `/isaac-sim/exts/isaacsim.asset.importer.urdf/` inside the
      container). That importer now converts the URDF to a standalone USD
      *file* on disk (`URDFImporter.import_urdf()`) rather than importing
      directly into the current stage, so a separate
      `add_reference_to_stage()` call is now needed too. Several config
      fields were also renamed or removed outright: `self_collision` →
      `allow_self_collision`; `default_drive_strength`/
      `default_position_drive_damping` → `override_joint_stiffness`/
      `override_joint_damping` (same Nm/rad units, just converted
      internally to USD's Nm/deg drive convention — not a weaker value);
      `distance_scale` and `import_inertia_tensor` removed entirely. This
      is the second time Isaac Sim's URDF-import API has changed between
      versions in this repo's own history — don't assume it's stable going
      forward either.
    - A **Newton-specific crash**, found while re-verifying
      `build_scene.py` end-to-end (not caught by `import_cr5.py`'s own
      isolated check, which never steps physics): cuRobo's bundled Franka
      Panda URDF (`robot/franka_description/franka_panda.urdf`) has a
      physically invalid inertia tensor on `panda_link3` (off-diagonal
      terms larger than the diagonal — a known real-world URDF-quality
      defect in that widely-used file, not something specific to this
      repo's own assets). Isaac Sim 6.0.1's Newton backend detects this
      (`authored diagonal inertia contains negative values. Falling back
      to mass-computer result.`) but its own fallback path has a bug of
      its own (`cmp_i_diag` referenced before assignment,
      `isaacsim.pip.newton/pip_prebundle/newton/_src/sim/builder.py:2601`)
      that crashes physics initialization outright instead — PhysX
      tolerates the same authored tensor silently. Reproduced 2/2 times
      without a fix, 2/2 successes with: `import_cr5()` now passes
      `link_density=1000.0` (kg/m³, an arbitrary-but-reasonable
      placeholder, not measured — consistent with this repo's existing
      "illustrative" stance on sim physics) to force geometry-based
      inertia computation instead of trusting authored URDF values. Applied
      unconditionally (CR5 included, not just the Franka override) since
      `physics_backend` now defaults to Newton and the CR5's own URDF has
      no more claim to trustworthy authored inertia than the Franka's
      (same SolidWorks-exporter provenance concerns already noted for its
      degenerate joints, below) — untested either way, since
      `robot_override.enabled: true` means the CR5 branch itself still
      hasn't actually been exercised (see "Needs verification").
- `scripts/newton_backend.py` — enables Isaac Sim 6.0+'s bundled Newton
  physics backend (`isaacsim.physics.newton`) as an alternative to the
  default PhysX backend, gated by `configs/scene/table_layout.yaml`'s
  `physics_backend.enabled` (defaults to `true`). **Verified** against a
  live Isaac Sim 6.0.1 install two ways: (1) an isolated Tier-1 check (this
  file's own `__main__`) — a lone dynamic cube over a ground plane, no CR5,
  no vendored factory assets — actually falls under gravity (z: 2.0 →
  1.715 after 120 steps) confirming Newton really steps physics on this
  GPU; (2) the full `build_scene.py` scene (factory + ergo tables + mounted
  Franka + cuRobo `MotionGen`) builds and steps cleanly with Newton active,
  after the `link_density` fix above — joint positions confirmed finite
  after 180 steps. Real bug found and fixed (via independent research
  agents cross-checking Isaac Sim's actual 6.0.1 source before the live
  test, not just docs): `SimulationManager.switch_physics_engine("newton")`
  returns `bool` and never raises — the first draft discarded that return
  value, so a failed switch could have silently reported success while
  still running on PhysX. Also confirmed live: `isaacsim.physics.newton`
  auto-switches the active engine to Newton on its own extension startup
  (logged as `Auto-switched to newton on startup via SimulationManager`),
  making the explicit `switch_physics_engine()` call somewhat redundant in
  practice but still correct defensive belt-and-suspenders given that
  auto-switch behavior isn't documented as guaranteed.
- `scenes/cell_scene.usda` — a minimal GUI-safe entry-point stage (just a
  reference to `assets/factory/Factory.usd` at `/World/Factory`) for
  interactively adding new assets without ever hand-editing the vendored
  factory file. Exists because of a real incident this session: a "Collect
  As"/flattened-save operation was accidentally aimed at
  `assets/factory/Factory.usd` itself, wiping every composition arc except
  whatever was loaded at that moment — recovered by re-fetching only the
  small root file from the vendor zip (`SubUSDs/`, ~404MB, was untouched and
  didn't need re-fetching, since the flattened save only replaced the small
  root layer). This file is the safe alternative going forward: reference
  content into it, position things, save — `Factory.usd` itself is never
  the file being edited/saved, so it can't be overwritten this way again.
- `assets/mefron/` — hand-assembled-in-GUI scanner-cell assets (CAD parts
  converted from the vendored `mantra scanner/` STEP files via
  `omni.kit.converter.cad`, e.g. `scanner assembly/finger print
  scanner.usd`, `backpanel support.usd`), plus `packing_table.usd`. Not
  built by `build_scene.py` — added interactively via the GUI workflow
  above. **Real, confirmed findings from debugging why Newton wouldn't let
  a Rigid Body + Collider added to these parts fall under gravity**,
  reproduced headlessly via `isaac-sim.newton.sh --exec` (the GUI's own
  `carb.log_error` truncates real tracebacks to `str(exception)`, which is
  why this needed a standalone repro to actually root-cause):
    - **Instancing.** Every part imported with "Instanceable References"
      (the CAD Converter dialog's default) wraps its actual mesh in an
      instanceable prim. Newton's rigid-body/collider USD parser lacks the
      prototype-to-instance-proxy remapping its visual-shape-only loader
      has (`_load_visual_shapes_impl` in the bundled `newton` package
      explicitly skips anything with `RigidBodyAPI`/a collider, so that
      loader's instance-remap logic never runs for physics-enabled prims
      at all). Fix: `Usd.Prim.SetInstanceable(False)` on the wrapper prim
      before adding physics. Since each of these parts appears once in the
      scene, instancing buys nothing here anyway.
    - **Units.** These CAD-converted files carry their own `metersPerUnit`
      (`0.001`, i.e. millimeters — the STEP source's native unit), distinct
      from the consuming stage's default (`1.0`, meters). USD does **not**
      auto-convert for this mismatch across a `reference`/`payload` — only
      a stage's *own* `metersPerUnit` applies when it's the actively-opened
      root. `isaacsim.core.utils.stage.add_reference_to_stage()` already
      authors a corrective `xformOp:scale:unitsResolve` op (alongside a
      normal, identity `scale` op) to compensate for exactly this — but
      only if left alone. Clearing xformOpOrder before authoring your own
      transform (e.g. `Xformable.ClearXformOpOrder()`) silently discards
      this correction (confirmed: world bbox came out ~61×135×29 units —
      absurd for a fingerprint-scanner-sized part), and manually adding
      *another* scale op on top of an existing `unitsResolve` double-
      corrects (confirmed: bbox shrank to ~1,000,000x too small). Lesson:
      leave `add_reference_to_stage()`'s authored xformOps alone; add a
      *child* prim for your own positioning instead of re-authoring the
      reference prim's own transform.
  Also confirmed — not project-specific, genuine gaps in Isaac Sim 6.0.1's
  bundled `isaacsim.physics.newton`/`newton` packages themselves (checked
  GitHub for both `isaac-sim/IsaacSim` and `newton-physics/newton`; nothing
  filed for any of these):
    - **Root cause of the original `[Errno 13] Permission denied: ''`
      mystery.** Newton's default solver is MuJoCo
      (`isaacsim.physics.newton.impl.newton_config.NewtonConfig.solver_cfg`
      defaults to `MuJoCoSolverConfig`, not XPBD). The bundled `mujoco`
      Python package unconditionally imports its OpenGL rendering submodule
      even for pure headless physics; that submodule imports `glfw`, whose
      library-version-check code runs `subprocess.Popen([sys.executable,
      ...])` — and `sys.executable` is `''` whenever Python is embedded
      inside Kit's own C++ process (true for *any* `kit/kit <app>.kit`
      launch, GUI or headless `--exec`, not specific to this repo).
      Confirmed by temporarily patching `newton_stage.py`'s exception
      handler (which normally only logs `str(e)`, swallowing the real
      traceback) with `traceback.print_exc()` — reverted immediately after,
      since that file is an installed package, not part of this repo.
      Workaround: patch `sys.executable` to a real interpreter path
      (`{ISAACSIM_ROOT_PATH}/kit/python/bin/python3`) before anything
      imports Newton/mujoco, in any script launched via the raw `kit`
      executable rather than `python.sh`.
    - Past that: MuJoCo's solver requires at least one joint to convert a
      model at all (`solver_mujoco.py::_convert_to_mjc` — "The model must
      have at least one joint to be able to convert it to MuJoCo"). Newton
      is supposed to auto-assign an implicit free joint to floating rigid
      bodies for MuJoCo, but skips this for bodies it considers massless —
      and a collider alone (mass left to be auto-computed at runtime, no
      authored `UsdPhysics.MassAPI`) wasn't enough to avoid that in Isaac
      Sim 6.0.1: this part still hit the joint error despite having real
      collision geometry. **Fix, confirmed working, no solver change
      needed**: `UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(<mass>)`
      — once an explicit mass is authored, Newton's auto-free-joint
      assignment kicks in correctly and MuJoCo works with zero other
      changes (no manually-authored joint prim needed). Forcing
      `XPBDSolverConfig` via
      `isaacsim.physics.newton.impl.extension.acquire_stage()`'s singleton
      also works (confirmed both ways) and remains a valid alternative if
      MuJoCo isn't otherwise needed — XPBD (maximal-coordinate) never had
      a joint requirement to begin with — but explicit mass is the better
      fix if the scene needs to stay on MuJoCo (e.g. alongside other
      articulated robots).
    - Only relevant if forcing XPBD instead of authoring mass: that path
      exposes a related inconsistency where Newton's USD parser always
      includes `SchemaResolverMjc` in its resolver stack regardless of
      solver type, but only calls
      `SolverMuJoCo.register_custom_attributes()` (which is what satisfies
      that resolver's validation) when `solver_type == "mujoco"`. Switching
      to XPBD without any MuJoCo-tagged schema in the scene needs
      `SchemaResolverMjc.validate_custom_attributes` patched to a no-op.
    - `isaacsim.core.api.World`/`SimulationContext.step()` (the deprecated
      core API, already flagged as a suspect by this repo's own
      `newton_backend.py` docstring) crashes under Newton with
      `AttributeError: 'NoneType' object has no attribute '_step'`
      (`self._physics_context` is never populated for Newton). Worked
      around by driving `omni.timeline.get_timeline_interface().play()` +
      `omni.kit.app.get_app().update()` directly instead of `world.step()`.
    - `isaacsim.core.prims.SingleRigidPrim.get_world_pose()` (also the
      deprecated core API) silently falls back to a stale, unsimulated raw
      USD attribute read whenever its internal `_physics_view` is `None` —
      which is only bound by calling `.initialize()` explicitly, not by
      construction or by `world.reset()` alone.
    - Separately, once the physics view *was* correctly bound and the
      object was genuinely falling (confirmed via the physics tensor view,
      matching a known-good control cube's fall trajectory closely), both
      the Fabric-backed (`usdrt`) and plain-USD reads of the same prim's
      world transform stayed frozen at its initial authored value for the
      whole run — Newton's `update_fabric` write-back did not appear to
      sync for a body authored via `Usd.Stage.DefinePrim()` + reference +
      `setRigidBody()`, as opposed to one added via `world.scene.add()`.
      Real, but non-blocking for physics correctness — matters for whether
      the *viewport* visibly shows the motion, not whether the simulation
      itself is right. Not root-caused further this session.
  Reproduced end-to-end in `scratchpad/newton_fingerprint_repro.py`
  (diagnostic only, not committed — not one of this project's own
  documented/verified pipeline scripts): confirmed `finger_print_scanner`
  with a Rigid Body + Collider added actually falls under Newton/XPBD
  gravity (z: 0.498 → 0.006 over 120 steps) once all of the above are
  applied together, run headlessly via `/isaac-sim/isaac-sim.newton.sh
  --no-window --/app/fastShutdown=true --exec <script>` — not `python.sh`;
  that launcher just execs `kit/kit apps/isaacsim.exp.full.newton.kit
  "$@"`, so headless mode and script injection come from Kit's own
  `--no-window`/`--exec` flags, not a separate standalone-script path, and
  a script run this way must not create its own `SimulationApp` (Kit's app
  already exists) or rely on this repo's own `newton_backend.py` (its
  module-level `from isaacsim import SimulationApp` import fails in this
  context — inline the same `enable_newton_physics()` logic instead).
  **Since confirmed working end-to-end in the actual interactive GUI too**
  (not just headlessly), for both `finger_print_scanner` and
  `backpanel_support`, once *all* of the following are true together (any
  one missing reproduces some form of the failure — either the original
  crash, silently frozen/not-falling, or falling straight through the
  ground): (1) `sys.executable` patched (see
  `scripts/launch_isaac_sim_newton.sh` below); (2) an explicit
  `UsdPhysics.MassAPI` mass authored on the body (needed for Newton's
  default MuJoCo solver specifically — its auto-free-joint assignment for
  floating bodies skips ones it considers massless, and a collider alone
  with mass left to auto-compute at runtime wasn't enough); (3)
  `Instanceable` off on the CAD wrapper prim; (4) Rigid Body applied to a
  translate-only parent Xform, never the scaled reference prim itself; (5)
  exactly one ground collider in the scene (a duplicate — e.g. one bundled
  inside an `Environment`/`FlatGrid` preset *and* a separately-added
  Ground Plane both providing collision — produces a distinct error,
  `"The number of geoms in the MuJoCo model does not match the number of
  colliding shapes in the Newton model"`, confirmed via
  `solver_mujoco.py`'s own geom-count assertion).
- `scripts/newton_sys_executable_patch.py` +
  `scripts/launch_isaac_sim_newton.sh` — automates fix (1) above so it
  doesn't need to be pasted into the Script Editor by hand every session.
  The wrapper shell script calls the real, **unmodified**
  `/isaac-sim/isaac-sim.newton.sh` with an added `--exec
  scripts/newton_sys_executable_patch.py` flag — deliberately not a copy
  of that launcher (its `$(dirname ${BASH_SOURCE})`-relative paths would
  break if duplicated into this repo) and deliberately not editing it
  in place either (same "don't fork vendor files" stance already applied
  to `examples/curobo_reference/`) — nothing under `/isaac-sim/` is
  touched. The patch script itself deliberately does **not** call
  `os._exit()` the way this repo's diagnostic scratchpad scripts do; it's
  meant to leave an interactive GUI session running normally afterward,
  just with the patch already applied before you touch anything. **Verified**:
  confirmed headlessly that Kit accepts multiple `--exec` flags in
  sequence (this wrapper's own + an additional one appended by the
  caller), running each in order. Use this in place of directly invoking
  `/isaac-sim/isaac-sim.newton.sh`, for both GUI and headless
  (`--no-window`) use — same other args accepted, forwarded through
  unchanged. Only covers fix (1); explicit mass (2), instancing (3), body
  hierarchy (4), and duplicate ground colliders (5) above are still
  per-scene/per-asset choices, not something a launcher wrapper can fix
  for you.
- `scripts/launch_isaac_sim.sh` — same pattern, for the **base**
  `/isaac-sim/isaac-sim.sh` (launches `isaacsim.exp.full.kit`, not the
  `.newton` variant). Deliberately does **not** inject the
  `sys.executable` patch — this app config doesn't load Newton by
  default (no `isaacsim.physics.newton*` extensions in its own `.kit`
  file, unlike `isaacsim.exp.full.newton.kit`), so the MuJoCo/glfw bug
  that patch works around isn't in play here. Exists purely for
  convenience/consistency with `launch_isaac_sim_newton.sh` — a thin,
  unmodified pass-through to the real launcher. **Verified** headlessly
  (`--exec` smoke test, exit code 0).
- `examples/curobo_reference/` — `motion_gen_reacher.py` + `helper.py`,
  fetched verbatim from cuRobo's own GitHub repo at the exact pinned
  commit (`docker/.env.curobo`). A pristine reference copy of cuRobo's
  official interactive teleop demo (drag a target cuboid, robot follows
  via `MotionGen`) — **do not modify these two files**; if a CR5-specific
  variant is needed, write a separate script instead (see "Needs
  verification" below). **Verified it runs against Isaac Sim 5.1.0** with
  two environment fixes: (1) the prebuilt `kinematics_fused_cu` kernel has
  a torch ABI mismatch here, and cuRobo's JIT-compile fallback needs
  `ninja`, which `Dockerfile.curobo` didn't install at the time — now fixed
  at the image level (see `docker/Dockerfile.curobo`'s own comment); (2)
  `pip` was itself broken in the 5.1.0 Isaac Sim install
  (`ModuleNotFoundError: No module named 'pip._vendor.packaging._structures'`),
  so `ninja` had to be fetched as a static binary instead of
  `pip install ninja` at the time — confirmed fixed upstream in 6.0.1 (see
  `docker/.env.base`'s entry above). **BROKEN against Isaac Sim 6.0.1,
  not re-verified further — see "Needs verification".**
- `scripts/motion_gen_teleop.py` — a from-scratch, CR5-cell-specific
  interactive drag-target teleop demo, written to replace
  `examples/curobo_reference/motion_gen_reacher.py` for actual use now that
  the latter is broken under 6.0.1 (see its own entry above) — this is the
  "CR5-specific interactive teleop script" previously tracked as attempted
  but never completed (see "Needs verification"'s old entry, now resolved).
  Builds this repo's real scene via `build_scene.py`'s own functions
  (factory + ergo tables + mounted robot + pedestal) rather than a
  synthetic world, and uses `isaacsim.core.api`/`isaacsim.core.utils`
  instead of the removed `omni.isaac.*` namespace. **Verified** against a
  live Isaac Sim 6.0.1 install (real GPU, Newton enabled, Franka via
  `cr5_mount.robot_override`) via `--headless` mode (which also drives the
  target cuboid programmatically once, for automated testing, since
  there's no GUI to drag it in): builds the full scene, warms up cuRobo
  against 74 real obstacles synced from the two ergo tables, and
  successfully plans and executes a reach to the moved target — confirmed
  via a real `MotionGenResult.success`, not just "no exception raised."
  Four real, non-obvious bugs found and fixed along the way, each also
  documented in the script's own module docstring:
    - cuRobo's collision-world sync (`UsdHelper.get_obstacles_from_stage()`)
      was first tried scoped to the entire `/World` tree, matching the
      pristine script's intent to reflect "the real scene" — this pulled
      in the whole ~13,500-object factory backdrop and made cuRobo's
      warmup take minutes, unusable for an interactive loop. Rescoped to
      just the two ergo tables (`only_paths=[...]`), which is both fast
      (74 objects) and actually relevant to this cell's real workspace.
    - The reused `RobotPedestal` (`cr5_mount.pedestal`) must be excluded
      from the obstacle sync explicitly, even though it's already outside
      the ergo-tables-only scope above — omitting this makes cuRobo see
      the robot colliding with its own mounting stand and refuse to plan
      at all (`MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION`).
    - Under Newton, `SimulationManager` forces articulation state onto a
      torch backend. Two consequences: `robot.get_joints_state()`'s
      `.positions`/`.velocities` come back as CUDA torch tensors (need
      `.cpu().numpy()` before any `np.*` call); and
      `robot.set_joint_positions()`'s own wrapper
      (`isaacsim.core.prims.impl.single_articulation.SingleArticulation`)
      silently coerces a torch tensor back to numpy via its own stale
      `self._backend_utils.expand_dims()` before handing off to
      `self._articulation_view`, which then crashes on a mismatched
      backend (`'numpy.ndarray' object has no attribute 'to'`) — fixed by
      calling `robot._articulation_view.set_joint_positions()` directly,
      bypassing the wrapper (the pristine reference script already reaches
      into `_articulation_view` for other calls, so this isn't an
      unprecedented pattern). Unclear whether this is fixed in a later
      Isaac Sim point release or a standing Newton/deprecated-Core-API
      interaction — worth rechecking on a future version bump.
    - cuRobo's `WorldMeshCollision` calls `wp.torch.device_from_torch()`,
      an older nested-submodule warp API path absent from the actually-
      installed `warp-lang` 1.14.0 (pip auto-resolved the newest release
      when `Dockerfile.curobo` installed cuRobo from source — confirmed
      this is the *only* `wp.torch.*` call site in the whole installed
      cuRobo package). Shimmed a fake `warp.torch` namespace at module load
      time rather than re-pinning `warp-lang` (which risks reopening
      Newton's own warp-version questions) or patching cuRobo's vendored
      source.
  **NOT verified against the CR5 itself** (only Franka, since
  `robot_override.enabled` is still `true`).

  **Since extended with three more features, GUI-tested this session** (X11
  forwarding now works — see the devcontainer entry above):
    - **Launches Isaac Sim's full experience** (`isaacsim.exp.full.kit`,
      same as `isaac-sim.sh`/`scripts/launch_isaac_sim.sh`) instead of the
      `SimulationApp` default (`isaacsim.exp.base.kit`, confirmed by
      reading `isaacsim.simulation_app.SimulationApp`'s own resolution
      order) — the base experience was why panels like Joint Inspector
      were missing. Only for the interactive path; `--headless` stays on
      the base experience, untouched, so the automated smoke test isn't
      put at risk. Deliberately `isaacsim.exp.full.kit`, not
      `.full.newton.kit` — the Newton-only variant disables PhysX
      entirely, which would break this project's own
      `physics_backend.enabled: false` fallback.
    - **Gripper open/close via keyboard** (**O** open, **C** close),
      polled per-frame via `carb.input`/`omni.appwindow` (confirmed this is
      the current, non-deprecated pattern by reading a real usage,
      `isaacsim.replicator.experimental.mobility_gen`'s `KeyboardDriver`).
      Drives `panda_finger_joint1`/`panda_finger_joint2` via
      `robot._articulation_view.set_joint_position_targets()` (the
      *targets* variant — smooth, physically-driven, matching this
      script's own established `_articulation_view` workaround pattern for
      the arm joints, not an instant kinematic snap) — fully independent
      of cuRobo's arm planning, since the fingers aren't in its cspace
      joint list.
    - **The drag target itself is a "ghost gripper"**: instead of a plain
      cube, a *detached copy* of the robot's own end-effector visual mesh —
      following `main`'s own `build_teleop_target()` (from that branch's
      "interactive cuRobo teleop" commit) as the reference template, one
      unified prim serves as both the visual and the functional target, no
      separate cube. `main`'s version (Isaac Sim 5.1.0's URDF importer)
      copies a clean `f"{ee_link}/visuals"` sub-scope directly; this
      branch's redesigned 6.0.1 importer doesn't produce one equivalent
      prim — confirmed live by walking the actual imported hierarchy that
      `panda_hand`'s visual content is split across `panda_hand/hand`,
      `panda_hand/hand_1` (clean) and
      `panda_hand/panda_leftfinger/finger(+finger_1)`,
      `panda_hand/panda_rightfinger/finger(+finger_1)` (also clean — the
      `RigidBodyAPI` actually lives one level up, on
      `panda_leftfinger`/`panda_rightfinger` themselves). Rather than
      hand-assembling six sub-copies at two nesting depths, this
      `CopyPrim`s the *whole* `panda_hand` (found by runtime name search,
      not a hardcoded path — must keep working for the CR5 too, once
      `robot_override.enabled` flips back) and strips
      `RigidBodyAPI`/`CollisionAPI`/`ArticulationRootAPI` from the copy's
      entire subtree afterward. **Real bug found and fixed on the way
      here**: stripping those APIs from just the copy's *root* prim
      (`panda_hand` itself) was not enough —
      `panda_leftfinger`/`panda_rightfinger` (its children) came through
      still carrying real `RigidBodyAPI`, confirmed via a live diagnostic
      scan, giving Newton two phantom, unconstrained "rigid bodies" with no
      joint attaching them to anything. That's what was actually behind a
      GUI-only crash on pressing Play
      (`[Newton] ... isaac sim has returned NAN joint position values`) —
      reproducible only interactively, never headlessly, since the
      headless smoke test's own scripted target-move happens too late
      (`step_index > 60`) to exercise the same code path the same way.
      Fixed by walking the *entire* copied subtree (`Usd.PrimRange`), not
      just its root, when stripping APIs. Also tried (and abandoned) an
      `AddInternalReference` clone before landing on `CopyPrim` to match
      `main`'s exact technique — a live reference arc left an unresolved
      doubt about whether its own composed transform could stack with the
      pose set on it afterward; `CopyPrim` is a fully independent,
      flattened prim spec, no composition ambiguity possible. Known
      simplification (confirmed acceptable via AskUserQuestion): the
      copy's fingers are frozen at whatever pose they were in at copy
      time — only the ghost's overall position/orientation is draggable,
      not a live mirror of the real gripper's open/close state.
    - **Headless smoke test passes clean after all of the above** (exit 0,
      `PASS: headless smoke test completed, plan succeeded, no NaNs`) —
      but the NaN crash and the ghost-gripper feature are both
      interactive-only concerns by nature (the headless path never
      exercises human dragging, real Play-button timing, or the full
      experience), so **GUI confirmation from the user is still the actual
      verification for these three**, same as the original cube-drag
      teleop loop always was.
- `scripts/` (remaining) — `setup_curobo.py`, `waypoints.py`,
  `teach_waypoint.py`, `playback_waypoints.py`.
- `scripts/newton_standalone_smoketest.py` — standalone smoke test for
  NVIDIA's standalone Newton physics engine (`newton-physics/newton`,
  independent of Isaac Sim entirely — see `docker/.env.newton`).
  **Verified**: in a throwaway conda env (`newton==1.3.0`,
  `warp-lang==1.14.0`; no `python3.12-venv` package and no passwordless
  `sudo` were available on this host to use a plain venv instead), Warp
  correctly detects and JIT-compiles for this machine's RTX PRO 4000
  Blackwell GPU, and a dynamic box dropped above a ground plane (SolverXPBD)
  actually falls under gravity and settles (z: 2.0 → 0.250) — real physics
  stepping on the GPU, not just a clean import.
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
`scripts/import_cr5.py`, and `scripts/newton_backend.py` have since been
run end-to-end against a live Isaac Sim 6.0.1 install (see their entries
above). Still open:

- **`examples/curobo_reference/motion_gen_reacher.py`/`helper.py` are
  broken under Isaac Sim 6.0.1 and were deliberately left that way.**
  These two files are pristine, vendored verbatim from cuRobo's GitHub
  repo — this repo's own convention is "do not modify them." Re-verifying
  against 6.0.1 found their hard-coded `from omni.isaac.kit import
  SimulationApp` / `from omni.isaac.core import ...` imports (no fallback
  to `isaacsim.*`) fail immediately: confirmed live that the entire
  `omni.isaac` namespace no longer exists in 6.0.1 at all
  (`ModuleNotFoundError: No module named 'omni.isaac'`), not just renamed
  with a compatibility shim. Per explicit instruction, this was left
  untouched rather than patched or worked around — needs a decision on how
  to proceed (patch despite the "pristine" convention and note the
  deviation, find/vendor an updated upstream reference if cuRobo has one
  for newer Isaac Sim versions, or accept this directory as
  5.1.0-only/reference-only going forward and say so explicitly).
  `scripts/motion_gen_teleop.py` (see its own "Done" entry) now covers the
  *practical* need this reference script served — an interactive
  drag-target teleop demo that actually runs on 6.0.1 — so this item is
  about the vendored reference copy's own status, not a blocker on having
  a working teleop demo at all.
- **Revert the temporary Franka swap.** `cr5_mount.robot_override` mounts
  cuRobo's bundled Franka instead of the CR5 to validate the pipeline
  first. Turn it off (`enabled: false`) and confirm the CR5 branch of both
  `mount_cr5()` and `setup_curobo_motion_gen()` in `build_scene.py` still
  works — the CR5 branch of the latter in particular has never actually
  been exercised (see `configs/curobo/cr5.yml`'s entry above). Now also
  untested for the Newton inertia-crash workaround (`link_density` in
  `import_cr5.py` — see its "Done" entry): applied unconditionally on the
  assumption the CR5's own URDF has no more claim to trustworthy authored
  inertia than the Franka's, but this has never actually been checked
  against the CR5's real inertia values.
- **`scripts/setup_curobo.py`** — still first-draft/unverified, and now
  known (not just guessed) to be broken as written: it passes
  `configs/curobo/cr5.yml`'s path straight to
  `MotionGenConfig.load_from_robot_config()` without the absolute-path
  patching that turned out to be required (see the yml's own module
  comment) — will fail the same way the unpatched version did during this
  investigation.
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
  otherwise the articulation won't hold a pose. **Correction (second time
  this API has changed)**: this used to say
  `omni.kit.commands.execute("URDFCreateImportConfig")[1]` — that kit
  command no longer exists in Isaac Sim 6.0.1 at all. Use the directly-
  constructible `isaacsim.asset.importer.urdf.URDFImporterConfig` dataclass
  + `URDFImporter` class instead (see `scripts/import_cr5.py`), setting
  `override_joint_stiffness`/`override_joint_damping` (renamed from
  `default_drive_strength`/`default_position_drive_damping`, same Nm/rad
  units) — and pass `link_density` too, working around a real Newton
  inertia-crash bug (see `import_cr5.py`'s own "Done" entry). Note also
  that `URDFImporter.import_urdf()` now writes a USD *file* to disk rather
  than importing directly into the current stage — a separate
  `add_reference_to_stage()` call is required afterward.
- `physics_backend` toggle (`configs/scene/table_layout.yaml`, consumed by
  `scripts/newton_backend.py`): same `enabled: <bool>` shape as
  `cr5_mount.robot_override` above, but inverted intent — `robot_override`
  defaults **on** to de-risk an untrusted path (the CR5's own kinematics)
  by substituting a known-good stand-in (Franka); `physics_backend`
  defaults **on** too, but here PhysX is the known-good path and Newton is
  the experimental substitute being exercised by default per explicit
  project decision, not because it's already trusted — NVIDIA's own docs
  call this integration "experimental." Set `enabled: false` to fall back
  to PhysX if Newton causes problems with this scene's assets that aren't
  worth chasing down.
- Waypoints are joint-space (`Waypoint.joint_positions`, radians, 6 values
  for joint1..joint6), not Cartesian poses.
- Pinned versions: Isaac Sim `6.0.1` (bumped from `5.1.0`, rebuilt and
  re-verified — see `docker/.env.base`'s "Done" entry), cuRobo commit
  `ebb71702f3f70e767f40fd8e050674af0288abe8`, torch `2.11.0+cu128` (CUDA
  12.8, installed fresh in `Dockerfile.curobo` after removing Isaac Sim's
  pre-bundled copy — see the Docker/devcontainer entry above). Standalone
  Newton (`docker/.env.newton`) is pinned separately — `newton==1.3.0`,
  `warp-lang==1.14.0` — and has no relationship to the Isaac Sim version;
  Isaac Sim 6.0+'s own *bundled* Newton physics backend has no separate
  version pin of its own beyond `ISAACSIM_VERSION` (it ships inside the
  Isaac Sim image itself — this session observed `isaacsim.physics.newton`
  extension version `0.8.1` and Warp `1.13.0` bundled specifically inside
  Isaac Sim 6.0.1, both slightly older than the standalone pins above,
  which pull whatever's newest on PyPI — not expected to matter for this
  repo's purposes, but don't assume they're identical if debugging a
  Newton-specific discrepancy between the two contexts).
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
- `ninja`: cuRobo's CUDA kernels fall back to a JIT compile (needs `ninja`)
  when the prebuilt `.so` has a torch ABI mismatch, which happened on this
  install. **Fixed at the image level**: `Dockerfile.curobo` now installs
  `ninja-build` via `apt-get` alongside the CUDA toolkit, instead of the
  original one-off manual/static-binary workaround per-container. `pip
  install` inside the Isaac Sim container was separately broken on 5.1.0
  (`ModuleNotFoundError: No module named 'pip._vendor.packaging._structures'`)
  — confirmed **fixed upstream in 6.0.1** (`python.sh -m pip --version`
  now reports cleanly); if working against an older 5.1.0-based image,
  this bug and its `apt`/static-binary workaround still apply.
- Positioning a prim relative to `/World/Factory` needs care about which
  frame a number is in — see `configs/scene/table_layout.yaml`'s
  `ergo_tables`/`cr5_mount.pedestal` comments for two different, easy-to-
  confuse gotchas (`/World/Factory`'s implicit ×100 scale for world vs.
  local Translate; a reused prim's own large native local-space offset
  baked into the vendored asset). When in doubt, verify by reading back
  `get_world_pose()`/`get_local_pose()` rather than assuming.

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
