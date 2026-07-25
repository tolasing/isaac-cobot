# isaac-cobot

A simulated factory cell in NVIDIA Isaac Sim: a real factory-floor backdrop
(vendored from NVIDIA's USD Explorer Sample Assets Pack — not a generic
warehouse), two reused work-surface desks for holding assembly parts, and a
Dobot CR5 6-DOF cobot mounted between them.

There is no physical CR5 hardware behind this project — everything targets
Isaac Sim only. The CR5 is imported from its vendored URDF, cuRobo provides
collision-aware IK / motion generation, and joint-space waypoints are taught
in-sim and played back through `motion_gen.plan_single_js()`. See
[CLAUDE.md](CLAUDE.md) for the full set of project conventions.

**Status**: both Docker images (`base`, `curobo`) and both devcontainers
build and run against a live RTX PRO 4000 Blackwell GPU — torch, cuRobo,
and `scripts/build_scene.py` (factory + work surfaces + robot mount + a
warmed-up cuRobo `MotionGen`, plus an interactive teleop target you drag in
the GUI to command the robot via `MotionGen.plan_single()`) all verified
end-to-end, including a scripted fake-drag that produces a real,
successful plan and robot motion. `build_scene.py` currently mounts
cuRobo's own bundled Franka Panda instead of the CR5, as a temporary
stand-in to validate the pipeline (see `cr5_mount.robot_override` in
`configs/scene/table_layout.yaml`) — the CR5 config itself
(`configs/curobo/cr5.yml`) is only partially verified, and the remaining
CR5-specific scripts are still a first draft not yet run against a live
install. Live GUI mouse-drag feel and the ghost end-effector target's
visual appearance next to the real robot still need a manual smoke test.
See CLAUDE.md's "Needs verification" section for the current, specific
list.

## Prerequisites

- Docker with the NVIDIA Container Toolkit, and an NVIDIA GPU
- (Optional) [VS Code](https://code.visualstudio.com/) with the
  [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

## Quickstart

Two Docker profiles are available: `base` (plain Isaac Sim) and `curobo`
(adds cuRobo, pinned to the commit in `docker/.env.curobo`).

```bash
# Build + start a container, then attach a shell
python docker/container.py start base      # or: curobo
python docker/container.py enter base      # or: curobo

# Inside the container
python scripts/build_scene.py
python scripts/import_cr5.py
python scripts/teach_waypoint.py --name home
python scripts/playback_waypoints.py

# From the host, when done
python docker/container.py stop base       # or: curobo
```

Or open this repo in VS Code and use "Reopen in Container" with either
`.devcontainer/base` or `.devcontainer/curobo`.

## Running the mefron scanner-assembly demo

The above `build_scene.py`/CR5 pipeline is the original scaffolding; the
actively-worked-on task is `scripts/mefron.py` (see
[CLAUDE.md](CLAUDE.md)'s "What this repo is") — three cuRobo-driven Franka
Pandas, teleoperated by dragging targets in the Isaac Sim GUI, picking and
placing a scanner-assembly mockup on `assets/mefron/`.

From inside the `curobo` container (`python` is aliased there to
`${ISAACSIM_ROOT_PATH}/python.sh`):

```bash
python scripts/mefron.py
```

This opens `assets/mefron/factory floor/mefron.usd` directly, mounts all
three arms, and warms up cuRobo's `MotionGen` for each (~30s per arm — the
viewport looks frozen/black during this). Once the console prints
`[mefron] click Play in the GUI to start teleop.`, click **Play** in the
Isaac Sim viewport.

Each arm is teleoperated by dragging its own target cube in the viewport
(`/World/target`, `/World/target2`, `/World/target3`) — cuRobo re-plans a
collision-aware path to wherever you drop it.

### Keyboard controls

| Key | Arm | Action |
|---|---|---|
| `J` | 1 | Snap target to `finger_print_scanner`'s grasp-approach pose |
| `B` | 1 | Snap target to `backpanel_support`'s grasp-approach pose |
| `C` | 1 | Close gripper |
| `O` | 1 | Open gripper |
| `P` | 1 & 2 | Snap target to the assembly-placement pose for whichever object was last grasped/approached |
| `N` | 2 | Snap target to `screen`'s suction-approach pose |
| `M` | 2 | Snap target to `PCB_Assembly_color_fixed`'s suction-approach pose |
| `V` | 2 | Suction on (attach) |
| `L` | 2 | Suction off (release) — **see CLAUDE.md's "Currently open issues": this doesn't actually let go yet** without also manually unchecking "Joint Enabled" on `SurfaceGripperJoint` (under `panda_hand` in the Stage panel) |
| `1` | conveyor | Send `main_holder_jig` forward; press again to send it back |

Arm 3 (screwdriver end effector) has no keyboard controls yet — drag its
target only.

## Workspace layout

| Path | What it is |
|---|---|
| `robots/cr5/` | Vendored CR5 URDF + meshes (MIT, see `robots/cr5/SOURCE.md`) |
| `assets/factory/` | Vendored factory backdrop (not in git — see `assets/factory/SOURCE.md`) |
| `docker/` | Container profiles (`base`, `curobo`) and the `container.py` CLI |
| `.devcontainer/` | VS Code devcontainer configs matching the two Docker profiles |
| `configs/scene/` | Factory pruning rules, work-surface placement, and robot mount |
| `configs/curobo/` | cuRobo robot config + collision spheres for the CR5 |
| `configs/rmpflow/` | Deferred — cuRobo is the primary IK/motion-gen path |
| `scripts/` | Scene builder, URDF importer, cuRobo setup, waypoint teach/playback |
| `data/waypoints/` | Recorded waypoints (joint-space JSON) |
| `tests/` | Pure-Python checks for configs/waypoints (no Isaac Sim required) |

## Development

```bash
pip install .[dev]
ruff check .
ruff format --check .
pytest
```
