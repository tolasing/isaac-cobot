"""Imports the vendored CR5 URDF into /World/CR5 as a sibling of /World/Factory. Only creates its
own SimulationApp when run standalone; when imported (e.g. by build_scene.py), reuses the
caller's already-running Kit process instead of starting a second one.
Run standalone: ${ISAACSIM_ROOT_PATH}/python.sh scripts/import_cr5.py"""

from __future__ import annotations

from pathlib import Path

from isaacsim import SimulationApp

if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": False})

import omni.kit.commands  # noqa: E402

URDF_PATH = Path(__file__).resolve().parent.parent / "robots" / "cr5" / "urdf" / "cr5_robot.urdf"
CR5_PRIM_PATH = "/World/CR5"


def import_cr5(
    urdf_path: Path = URDF_PATH,
    prim_path: str = CR5_PRIM_PATH,
    default_drive_strength: float = 1e5,
    default_position_drive_damping: float = 1e4,
) -> str:
    """Imports a URDF (the CR5 by default) via URDFParseAndImportFile. default_drive_strength/
    default_position_drive_damping default to the CR5's own tuning -- a workaround for its URDF's
    degenerate effort="0" velocity="0" joints (see robots/cr5/SOURCE.md), not generally correct
    for any robot. Callers importing a different, properly-specified URDF should override both."""
    # isaacsim.asset.importer.urdf doesn't export a directly-constructible
    # config class -- the URDFCreateImportConfig command is the only way to
    # get a properly-initialized isaacsim.asset.importer.urdf._urdf.ImportConfig.
    import_config = omni.kit.commands.execute("URDFCreateImportConfig")[1]
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.import_inertia_tensor = True
    import_config.self_collision = False
    import_config.distance_scale = 1.0
    import_config.default_drive_strength = default_drive_strength
    import_config.default_position_drive_damping = default_position_drive_damping

    status, imported_prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
    )
    if not status:
        raise RuntimeError(f"URDF import failed for {urdf_path}")

    if imported_prim_path != prim_path:
        omni.kit.commands.execute("MovePrim", path_from=imported_prim_path, path_to=prim_path)

    return prim_path


def main() -> None:
    import_cr5()
    while simulation_app.is_running():
        simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
