"""Builds /World/Factory (backdrop), two reused ErgoTable desks near the
robot, imports+mounts the CR5 cobot (or, temporarily, a Franka Panda --
see cr5_mount.robot_override) between them, and warms up a matching cuRobo
MotionGen (best-effort -- skipped if cuRobo isn't installed, e.g. the
`base` Docker profile).

Verified against a live Isaac Sim 6.0.1 install (isaac-cobot-curobo
container, real GPU), Newton physics backend enabled by default (see
newton_backend.py). Previously verified against 5.1.0. The factory backdrop
asset loads asynchronously -- main() pumps a bounded number of frames after
building so a one-shot --headless run sees it fully resolved before
pruning/mounting/printing.

Only creates its own SimulationApp when run standalone (`__main__`), same
reasoning as import_cr5.py -- safe to import as a library from a script
that already has one running (confirmed the hard way: importing this
module after a second SimulationApp already exists segfaults instead of
raising).

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/build_scene.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import yaml
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": _headless})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import Usd  # noqa: E402

from import_cr5 import import_cr5  # noqa: E402
from newton_backend import enable_newton_physics  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = REPO_ROOT / "configs" / "scene" / "table_layout.yaml"


def load_config() -> dict:
    return yaml.safe_load(CONFIG_PATH.read_text())


def build_factory(cfg: dict) -> None:
    factory_cfg = cfg["factory"]
    backdrop_usd = REPO_ROOT / factory_cfg["backdrop_usd"]
    if not backdrop_usd.is_file():
        raise FileNotFoundError(f"{backdrop_usd} not found -- see assets/factory/SOURCE.md for how to fetch it.")
    add_reference_to_stage(usd_path=str(backdrop_usd), prim_path=factory_cfg["prim_path"])


def build_ergo_tables(cfg: dict) -> None:
    """Copies the vendored ErgoTable desk prop to two positions near the
    robot, for holding assembly parts.

    CopyPrim (not MovePrim -- see mount_cr5_pedestal's docstring for why)
    duplicates the source's composition arcs cleanly, so each copy renders
    with full geometry independent of the original.
    """
    ergo_cfg = cfg["ergo_tables"]
    source_path = ergo_cfg["source_prim_path"]
    for instance in ergo_cfg["instances"]:
        prim_path = instance["prim_path"]
        omni.kit.commands.execute("CopyPrim", path_from=source_path, path_to=prim_path)
        x, y = instance["position_xy"]
        xform = SingleXFormPrim(prim_path=prim_path)
        xform.set_world_pose(position=np.array([x, y, 0.0]), orientation=np.array(instance["orientation_wxyz"]))
        xform.set_local_scale(np.array(ergo_cfg["scale"]))


def mount_cr5(cfg: dict) -> None:
    mount_cfg = cfg["cr5_mount"]
    override = mount_cfg.get("robot_override")
    if override and override.get("enabled"):
        # Lazy import: build_scene.py otherwise has no cuRobo dependency
        # and must keep working in the `base` profile (no cuRobo installed)
        # when this temporary override isn't enabled.
        from curobo.util_file import get_assets_path, join_path

        urdf_path = Path(join_path(get_assets_path(), override["urdf_relative_path"]))
        import_cr5(
            urdf_path=urdf_path,
            prim_path=mount_cfg["prim_path"],
            default_drive_strength=override["default_drive_strength"],
            default_position_drive_damping=override["default_position_drive_damping"],
        )
    else:
        import_cr5(prim_path=mount_cfg["prim_path"])
    xform = SingleXFormPrim(prim_path=mount_cfg["prim_path"])
    xform.set_world_pose(
        position=np.array(mount_cfg["position"]),
        orientation=np.array(mount_cfg["orientation_wxyz"]),
    )
    xform.set_local_scale(np.array(mount_cfg["scale"]))


def mount_cr5_pedestal(cfg: dict) -> None:
    """Repositions the reused RobotPedestal prim (see
    factory.prune_name_startswith's comment in table_layout.yaml) so the
    robot isn't left floating.

    Overrides pose in place rather than moving/renaming the prim out of the
    welding line's hierarchy: RobotPedestal's mesh comes from nested
    `reference` arcs several levels deep in the vendored asset, and
    MovePrim on a prim like that leaves an empty shell behind (0 children).

    Uses set_local_pose(), not set_world_pose(): pedestal.local_translation/
    local_orientation_wxyz (configs/scene/table_layout.yaml) are LOCAL
    values read directly from the GUI's Property panel, since
    RobotPedestal's parent chain has a large offset baked into the vendored
    asset -- set_world_pose() would instead compute a different local
    transform needed to reach that number as a *world* position, which is
    not what these values represent.
    """
    pedestal_cfg = cfg["cr5_mount"]["pedestal"]
    xform = SingleXFormPrim(prim_path=pedestal_cfg["prim_path"])
    xform.set_local_pose(
        translation=np.array(pedestal_cfg["local_translation"]),
        orientation=np.array(pedestal_cfg["local_orientation_wxyz"]),
    )
    xform.set_local_scale(np.array(pedestal_cfg["scale"]))


def setup_curobo_motion_gen(cfg: dict, world_cfg=None):
    """Builds and warms up a cuRobo MotionGen for whichever robot is
    actually mounted at cr5_mount.

    Returns None (printing why) if cuRobo isn't installed -- build_scene.py
    must keep working in the `base` profile, which has no cuRobo, so this
    step is best-effort rather than a hard dependency.

    `world_cfg` (a curobo.geom.types.WorldConfig, or None) is passed
    through as MotionGenConfig.load_from_robot_config()'s `world_model` --
    left at its default None here (build_scene.py itself has no need for
    obstacle-aware collision checking, just confirming the config loads and
    warms up), which leaves `MotionGen.world_coll_checker` unset. Callers
    that need collision checking against a real world (e.g.
    scripts/motion_gen_teleop.py, syncing against the actual stage) must
    pass one.
    """
    try:
        from curobo.types.base import TensorDeviceType
        from curobo.util_file import get_robot_configs_path, join_path, load_yaml
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
    except ImportError:
        print("[build_scene] cuRobo not installed -- skipping MotionGen setup.", flush=True)
        return None

    mount_cfg = cfg["cr5_mount"]
    override = mount_cfg.get("robot_override")
    if override and override.get("enabled"):
        robot_cfg = join_path(get_robot_configs_path(), override["motion_gen_robot_cfg"])
    else:
        # See configs/curobo/cr5.yml's module comment: urdf_path/
        # asset_root_path/collision_spheres are repo-root-relative for
        # readability, but cuRobo always resolves them against its own
        # bundled assets/config dirs unless patched to absolute paths here.
        cr5_yml = REPO_ROOT / "configs" / "curobo" / "cr5.yml"
        robot_cfg = load_yaml(str(cr5_yml))
        k = robot_cfg["robot_cfg"]["kinematics"]
        k["urdf_path"] = str(REPO_ROOT / k["urdf_path"])
        k["asset_root_path"] = str(REPO_ROOT / k["asset_root_path"])
        k["collision_spheres"] = str(cr5_yml.parent / k["collision_spheres"])

    motion_gen_config = MotionGenConfig.load_from_robot_config(robot_cfg, world_cfg, tensor_args=TensorDeviceType())
    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup()
    return motion_gen


def prune_factory_dressing(cfg: dict) -> list[str]:
    """Deactivates the welding line's sliding rail and robot pedestals
    under /World/Factory, leaving every other prim (fences, feeders,
    process nodes, roof racks, robot controllers/arms, ErgoTable, etc.)
    untouched.

    Two matching modes, both against `factory` (configs/scene/table_layout.yaml):
      - `prune_name_startswith`: case-insensitive *prefix* (not substring)
        match against a prim's name, applied anywhere under /World/Factory
        -- e.g. "rail" matches `Rail`/`Rail_U20__U23_7` but not `Handrail`
        or `GuardRail`, since those don't start with it.
      - `prune_exact_paths`: exact full prim paths, for names too generic
        to safely prefix-match anywhere in the tree (e.g. "Link1", which
        also names our own CR5's first arm link).
    Verified against a live install: see CLAUDE.md.

    Deactivation (Prim.SetActive(False)), not deletion: reversible, and
    never touches the vendored Factory.usd file on disk.
    """
    factory_cfg = cfg["factory"]
    prefixes = [p.lower() for p in factory_cfg.get("prune_name_startswith", [])]
    exact_paths = factory_cfg.get("prune_exact_paths", [])

    stage = omni.usd.get_context().get_stage()
    pruned = []

    if prefixes:
        root = stage.GetPrimAtPath(factory_cfg["prim_path"])
        it = iter(Usd.PrimRange(root))
        for prim in it:
            if any(prim.GetName().lower().startswith(p) for p in prefixes):
                prim.SetActive(False)
                pruned.append(str(prim.GetPath()))
                it.PruneChildren()

    for path in exact_paths:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            prim.SetActive(False)
            pruned.append(path)

    return pruned


def main() -> None:
    cfg = load_config()

    # Documented-recommended ordering (see newton_backend.py) -- first thing
    # in main(), before build_factory().
    physics_backend_cfg = cfg.get("physics_backend", {})
    if physics_backend_cfg.get("enabled", False):
        newton_ok = enable_newton_physics()
        print(f"[build_scene] Newton physics backend: {'ENABLED' if newton_ok else 'FAILED TO ENABLE'}", flush=True)
        if not newton_ok:
            # Fail loud, don't silently keep building under PhysX: the
            # config's own opt-out (`physics_backend.enabled: false`) is the
            # sanctioned way to fall back, not an unexpected extension/engine
            # failure going unnoticed while the rest of the scene builds.
            raise RuntimeError("physics_backend.enabled is true but Newton could not be enabled -- see logs above")
    else:
        print("[build_scene] Newton physics backend: disabled (using default PhysX)", flush=True)

    build_factory(cfg)

    # The factory backdrop is a large USD reference and resolves
    # asynchronously -- give it a bounded number of frames to load before
    # pruning/copying/mounting/printing below (build_ergo_tables() copies a
    # prim that lives inside this reference, so it must come after this).
    for _ in range(120):
        simulation_app.update()

    pruned = prune_factory_dressing(cfg)
    print(f"[build_scene] pruned {len(pruned)} factory prim(s): {pruned}", flush=True)

    # After pruning so both copies inherit the deactivated Monitor/Keyboard.
    build_ergo_tables(cfg)

    mount_cr5(cfg)
    mount_cr5_pedestal(cfg)

    # motion_gen.warmup() blocks the main thread with real GPU work (kernel
    # compilation/loading, pre-tracing batched IK/trajopt solves) and calls
    # no simulation_app.update() of its own -- the viewport will go black
    # and look frozen for however long this takes (seconds to a couple
    # minutes depending on kernel caching). That's expected, not a hang.
    print("[build_scene] warming up cuRobo motion_gen (viewport will look frozen/black until this finishes)...", flush=True)
    motion_gen = setup_curobo_motion_gen(cfg)
    print(f"[build_scene] curobo motion_gen: {'READY' if motion_gen else 'SKIPPED'}", flush=True)

    stage = omni.usd.get_context().get_stage()
    pedestal_prim_path = cfg["cr5_mount"]["pedestal"]["prim_path"]
    ergo_table_paths = [instance["prim_path"] for instance in cfg["ergo_tables"]["instances"]]
    for prim_path in (
        cfg["factory"]["prim_path"],
        *ergo_table_paths,
        cfg["cr5_mount"]["prim_path"],
        pedestal_prim_path,
    ):
        prim = stage.GetPrimAtPath(prim_path)
        num_children = len(prim.GetChildren()) if prim.IsValid() else 0
        status = "OK" if prim.IsValid() else "MISSING"
        print(f"[build_scene] {prim_path}: {status} ({num_children} children)", flush=True)

    if _headless:
        simulation_app.close()
        return

    while simulation_app.is_running():
        simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
