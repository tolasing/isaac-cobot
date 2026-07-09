# Docker & devcontainer setup

Generic Isaac Sim + cuRobo environment infrastructure — not specific to any
one scene or robot, so it applies to running `scripts/mefron.py` /
`scripts/build_scene_mefron.py` the same as it did to the earlier CR5-based
pipeline. Moved out of `CLAUDE.md` to keep that file focused on the sim
itself; see `CLAUDE.md`'s "Must-know gotchas" for the one-line pointer that
sends you here.

## Done

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
  Ampere `8.0` originally guessed). This Dockerfile is also where
  `ninja-build` gets installed via `apt-get` (see `CLAUDE.md`'s cuRobo
  conventions for why cuRobo needs it, and why `pip install ninja` doesn't
  work in this environment).
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

## Needs verification

- The devcontainer X11/GUI-forwarding fix above is still unconfirmed
  end-to-end with a live GUI launch.

## Provenance / licensing

- `docker/utils/`, `docker/container.py`, and the devcontainer scaffolding
  are adapted from `tolasing/groot`, which itself follows Isaac Lab's
  BSD-3-Clause container tooling pattern.
