"""Imports the vendored CR5 URDF into /World/CR5 as a sibling of /World/Factory.

Verified against a live Isaac Sim 6.0.1 install (real GPU), both standalone
and imported as a library by build_scene.py. Previously verified against
5.1.0; re-verifying against 6.0.1 found a real API break, fixed below --
Isaac Sim 6.0.1 no longer registers the "URDFCreateImportConfig"/
"URDFParseAndImportFile" kit commands this file used to rely on (they now
live behind the isaacsim.asset.importer.urdf.ui extension and are
themselves deprecated in favor of a directly-constructible
isaacsim.asset.importer.urdf.URDFImporterConfig dataclass + URDFImporter
class, confirmed by reading /isaac-sim/exts/isaacsim.asset.importer.urdf/
inside the container). That importer now converts the URDF to a standalone
USD *file* on disk (URDFImporter.import_urdf()) rather than importing
directly into the current stage, so a separate add_reference_to_stage()
call is needed to actually bring it in -- this two-step split, plus the
config field renames (self_collision -> allow_self_collision,
default_drive_strength/default_position_drive_damping ->
override_joint_stiffness/override_joint_damping, distance_scale and
import_inertia_tensor removed entirely), is new in 6.0.1 and not just a
cosmetic rename.

Only creates its own SimulationApp when run standalone (`__main__`); when
imported (e.g. by build_scene.py, which already has one running),
import_cr5() reuses the caller's Kit process instead of starting a second
one -- the isaacsim/omni imports below just need *some* Kit app to already
be up, not specifically the one this module would create.

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/import_cr5.py
"""

from __future__ import annotations

import tempfile
from pathlib import Path

from isaacsim import SimulationApp

if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": False})

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402

URDF_PATH = Path(__file__).resolve().parent.parent / "robots" / "cr5" / "urdf" / "cr5_robot.urdf"
CR5_PRIM_PATH = "/World/CR5"


def import_cr5(
    urdf_path: Path = URDF_PATH,
    prim_path: str = CR5_PRIM_PATH,
    default_drive_strength: float = 1e5,
    default_position_drive_damping: float = 1e4,
    link_density: float = 1000.0,
) -> str:
    """Imports a URDF (the CR5 by default) via URDFImporter, then references
    the resulting USD file into the current stage at `prim_path`.

    `default_drive_strength`/`default_position_drive_damping` (Nm/rad and
    Nm*s/rad respectively -- URDFImporterConfig's own documented units,
    converted internally to USD's Nm/deg drive convention, not a weaker
    value) default to the CR5's own tuning -- a workaround for its URDF's
    degenerate effort="0" velocity="0" joints (see robots/cr5/SOURCE.md),
    not a generally-correct value for any robot. Callers importing a
    different, properly-specified URDF (e.g. build_scene.py's temporary
    Franka swap, which passes cuRobo's own tuned 1047.19751 / 52.35988)
    should override both.

    `link_density` (kg/m^3, an arbitrary-but-reasonable placeholder, not a
    measured value -- consistent with this repo's existing "illustrative,
    not validated against real hardware" stance on sim physics) forces
    URDFImporterConfig to compute link mass/inertia from geometry instead
    of trusting the URDF's own authored inertia tensor. This works around a
    real, reproducible bug (confirmed via 4 controlled trials against a
    live Isaac Sim 6.0.1 install): cuRobo's bundled Franka Panda URDF
    (robot/franka_description/franka_panda.urdf, used by
    cr5_mount.robot_override) has a physically invalid inertia tensor on
    panda_link3 (off-diagonal terms larger than the diagonal -- a known
    real-world URDF-quality defect, not something specific to this repo's
    own assets). Isaac Sim 6.0.1's Newton backend detects this (logs
    "authored diagonal inertia contains negative values. Falling back to
    mass-computer result.") but its own fallback path has a bug of its own
    (`cmp_i_diag` referenced before assignment,
    isaacsim.pip.newton/pip_prebundle/newton/_src/sim/builder.py:2601) that
    crashes physics initialization outright -- PhysX tolerates the same
    authored tensor without complaint. Without link_density set, this
    reproduced 2/2 times; with it set, 2/2 successes (world.reset() and
    stepping both succeed). Since configs/scene/table_layout.yaml's
    physics_backend now defaults to Newton, this fix is needed for normal
    operation, not just as an opt-in workaround -- applied unconditionally
    (including for the CR5's own URDF, which has its own already-documented
    exporter quirks and no more claim to trustworthy authored inertia than
    the Franka's).
    """
    enable_extension("isaacsim.asset.importer.urdf")

    # usd_path deliberately NOT left at its default (which would write the
    # converted USD into urdf_path's own directory, i.e. back into the
    # vendored robots/cr5/urdf/ tree) -- a throwaway temp dir instead, so
    # repeated imports don't dirty vendored source with generated output.
    import_config = URDFImporterConfig(
        urdf_path=str(urdf_path),
        usd_path=tempfile.mkdtemp(prefix="cr5_urdf_import_"),
        merge_fixed_joints=False,
        fix_base=True,
        allow_self_collision=False,
        override_joint_stiffness=default_drive_strength,
        override_joint_damping=default_position_drive_damping,
        link_density=link_density,
    )
    usd_file_path = URDFImporter(import_config).import_urdf()

    add_reference_to_stage(usd_path=usd_file_path, prim_path=prim_path)
    return prim_path


def main() -> None:
    import_cr5()
    while simulation_app.is_running():
        simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
