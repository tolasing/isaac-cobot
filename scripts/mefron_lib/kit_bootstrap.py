"""Kit-process bootstrapping needed before any omni/curobo import. Stdlib-only by design (see
__init__.py) so it's always safe to import first, even before SimulationApp exists.
"""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

_REAL_PACKAGING_DIR = "/isaac-sim/kit/python/lib/python3.11/site-packages/packaging"


def _preload_real_submodule(pkg_module, name: str) -> None:
    spec = importlib.util.spec_from_file_location(f"packaging.{name}", f"{_REAL_PACKAGING_DIR}/{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[f"packaging.{name}"] = module
    spec.loader.exec_module(module)
    setattr(pkg_module, name, module)


def preload_real_packaging() -> None:
    """Pre-loads real `packaging`/`packaging.version` from site-packages before cuRobo imports them --
    the full SimulationApp experience (isaacsim.exp.full.kit) shadows packaging.version with a broken
    bundle (see docs/mefron-history.md). Call this before importing anything that transitively imports
    cuRobo. Safe to call more than once; a no-op once `packaging` is already in sys.modules."""
    if "packaging" in sys.modules or not os.path.isdir(_REAL_PACKAGING_DIR):
        return
    spec = importlib.util.spec_from_file_location(
        "packaging", f"{_REAL_PACKAGING_DIR}/__init__.py", submodule_search_locations=[_REAL_PACKAGING_DIR]
    )
    packaging_module = importlib.util.module_from_spec(spec)
    sys.modules["packaging"] = packaging_module
    spec.loader.exec_module(packaging_module)
    _preload_real_submodule(packaging_module, "version")


def clear_stale_robot_configuration(configuration_dir: Path) -> None:
    """Deletes any pre-existing files under configuration_dir before the URDF importer writes fresh
    ones. Must run BEFORE open_stage(): mefron.usd has a persisted, broken /panda prim reference, and
    resolving it against stale files caches an Sdf.Layer that later crashes the next URDF import."""
    if not configuration_dir.is_dir():
        return
    for stale_file in configuration_dir.glob("*.usd"):
        stale_file.unlink()
        print(f"[mefron_lib] cleared stale robot-description file: {stale_file}", flush=True)
