# isaac-cobot

A simulated factory cell in NVIDIA Isaac Sim: a real factory-floor backdrop
(vendored from NVIDIA's USD Explorer Sample Assets Pack — not a generic
warehouse), packing tables, and a scanner-assembly CAD mockup that a cuRobo-
driven Franka Panda picks and places.

There is no physical robot hardware behind this project — everything targets
Isaac Sim only. cuRobo provides collision-aware motion generation, and the arm
is teleoperated by dragging a target in the GUI. See [CLAUDE.md](CLAUDE.md) for
the full set of project conventions and current state.

**Status**: both Docker images (`base`, `curobo`) and both devcontainers build
and run against a live RTX PRO 4000 Blackwell GPU. The `atc` branch fits the
single Franka with an automatic tool changer — a permanent male coupler on the
wrist, with gripper, suction and screwdriver tools parked in a rack until a
tool-change key docks one. See CLAUDE.md's "Currently open issues" for what is
and isn't validated.

This branch (`atc-laptop`) binds those three tools to **Y/U/I** instead of the
numpad, so the demo can be driven from a laptop keyboard.

## Prerequisites

- Docker with the NVIDIA Container Toolkit, and an NVIDIA GPU
- (Optional) [VS Code](https://code.visualstudio.com/) with the
  [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

## Quickstart

Two Docker profiles are available: `base` (plain Isaac Sim) and `curobo`
(adds cuRobo, pinned to the commit in `docker/.env.curobo`).

```bash
# Build + start a container, then attach a shell
python docker/container.py start curobo    # or: base
python docker/container.py enter curobo

# Inside the container ("python" is aliased to ${ISAACSIM_ROOT_PATH}/python.sh)
python scripts/mefron.py

# From the host, when done
python docker/container.py stop curobo
```

Or open this repo in VS Code and use "Reopen in Container" with either
`.devcontainer/base` or `.devcontainer/curobo`.

## Running the mefron scanner-assembly demo

`scripts/mefron.py` opens `assets/mefron/factory floor/mefron.usd` directly,
mounts the Franka, and warms up cuRobo's `MotionGen` (~30s — the viewport looks
frozen/black during this). Once the console prints
`[mefron] click Play in the GUI to start teleop.`, click **Play** in the
viewport.

The arm follows `/World/target`: drag it anywhere and cuRobo re-plans a
collision-aware path to it. The keys below snap that target to a computed pose
instead of requiring a hand-drag.

### Keyboard controls

| Key | Action |
|---|---|
| `Y` / `U` / `I` | Dock the gripper / suction / screwdriver tool |
| `J` / `B` / `K` | Gripper: approach `finger_print_scanner` / `backpanel_support` / `main_holder_back_cover` |
| `C` / `O` | Gripper: close / open — releasing near the assembly pose welds the part there |
| `N` / `M` | Suction: approach `screen` / `PCB_Assembly_color_fixed` |
| `V` / `L` | Suction: attach / release (same weld behavior as `O`) |
| `5` / `6` | Screwdriver: pick the presented screw / place it in the next hole |
| `P` | Place whichever object was last grasped or approached |
| `1` (number row) | Send `main_holder_jig` forward; press again to send it back |

Each group only works once the matching tool is docked. Headless regression
harnesses for the same mechanics live in `scripts/test_mefron_*_headless.py`.

## Workspace layout

| Path | What it is |
|---|---|
| `assets/mefron/` | The hand-authored scanner-assembly scene (`mefron.usd`) |
| `assets/factory/` | Vendored factory backdrop (not in git — see `assets/factory/SOURCE.md`) |
| `robots/accessories/` | The dockable tools' CAD (suction, screwdriver, rack) |
| `robots/cr5/`, `robots/franka_panda/` | Vendored URDFs + meshes (see each `SOURCE.md`) |
| `docker/` | Container profiles (`base`, `curobo`) and the `container.py` CLI |
| `.devcontainer/` | VS Code devcontainer configs matching the two Docker profiles |
| `configs/` | Legacy CR5 cuRobo/scene configs, kept for reference |
| `scripts/` | `mefron.py` (the entry point) + the headless regression harnesses |
| `scripts/mefron_lib/` | Everything `mefron.py` is built from — see CLAUDE.md |
| `docs/` | Full history, derivations, and the ATC design |

## Development

```bash
pip install .[dev]
ruff check .
ruff format --check .
```
