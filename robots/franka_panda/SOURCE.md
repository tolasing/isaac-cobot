# Franka Panda Asset Provenance

`franka.usd` and its `Props/`, `DetailedProps/`, `Materials/`, `configuration/`,
`Robotiq/` subfolders in this directory are vendored from **NVIDIA's own
bundled Isaac Robot Asset** for the Franka Panda — the same asset
`scripts/mefron2.py`'s `/World/Franka` originally referenced directly from
Nucleus, before being collected locally here.

- **Source**: `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd`
  (public, unauthenticated S3 bucket — same content Isaac Sim's own Content
  Browser "Isaac Sim" bookmark tree browses under `Robots/FrankaRobotics/FrankaPanda/`)
- **License**: NVIDIA Omniverse License Agreement (content-pack terms), not
  open source — same category as `assets/factory/` and `assets/mefron/`.

## Why this was vendored locally

The Nucleus-hosted asset's link geometry (e.g. `panda_hand/geometry`) is
`instanceable=True`. Referencing it directly makes every mesh under it an
"instance proxy" — USD refuses to author anything onto an instance proxy
directly (`Cannot move/rename ancestral prim`, and binding a physics
material to it fails with "Failed to bind material... they are instance
proxies"). Collecting the asset locally (via Isaac Sim's Content Browser
→ right-click `franka.usd` → **Collect Asset**, which also pulls in every
file it references) makes it possible to open the local copy directly and
uncheck **Instanceable** on the `geometry` prims, which isn't possible
against the read-only Nucleus original.

## Current status

This was built for `scripts/mefron2.py`'s "everything already baked into
mefron.usd, no code-driven import" approach. That approach was later
deprioritized in favor of going back to `scripts/mefron.py`'s own
code-driven `mount_franka()` (which imports a *different* Franka —
cuRobo's own bundled URDF via `import_cr5()`, not this Nucleus asset at
all) — see `CLAUDE.md`'s own history for why. This directory is kept
because `mefron2.py` still references it and still works; it's just not
the currently-preferred script.

## Not committed to git

This directory (~39MB, mostly binary USD) is gitignored (see the repo's
top-level `.gitignore`) — same treatment as `assets/factory/`/`assets/mefron/`.
To restore it after a fresh clone, use Isaac Sim's Content Browser as
described above (a plain `curl` of the URL above only gets `franka.usd`
itself, not the `Props/`/`DetailedProps/`/etc. files it references — the
Collect Asset tool is what resolves the whole dependency graph).
