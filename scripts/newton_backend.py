"""Enables NVIDIA's Newton physics backend (isaacsim.physics.newton) as an
alternative to Isaac Sim's default PhysX backend.

Newton ships bundled with Isaac Sim 6.0+ but is NOT enabled by default
outside NVIDIA's own `isaac-sim.newton.sh` launcher -- a documented,
upstream-acknowledged gap (isaac-sim/IsaacSim#558), not a missing feature.
NVIDIA calls this integration "experimental" and explicitly not a drop-in
PhysX replacement (docs.isaacsim.omniverse.nvidia.com/6.0.0/physics/newton_physics.html).

Controlled by configs/scene/table_layout.yaml's `physics_backend.enabled`
(defaults to true in this repo) -- build_scene.py calls
enable_newton_physics() as the very first thing in main(), before any stage
content is created. This isn't a hard API requirement (switching later just
triggers an invalidate-and-reload internally) but it's NVIDIA's documented
recommended flow: create stage -> switch engine -> author scene content ->
play.

NOT yet verified against a live Isaac Sim 6.0.1 install -- written from
NVIDIA's documented API surface (cross-checked against the actual
isaac-sim/IsaacSim source at the v6.0.1 tag) while the 6.0.1 image rebuild
was still in progress. See CLAUDE.md's "Needs verification" until this note
is removed. Note also that `isaacsim.core.api`/`isaacsim.core.utils.extensions`
(used below) now live under that repo's `source/deprecated/` tree as of
6.0.0 -- still shipped and functional, but NVIDIA's own docs are steering
new code toward `isaacsim.core.experimental.*` instead. If run_tier1_cube_drop()
fails in a way that doesn't obviously look like a physics problem, that's
worth suspecting first.

Only creates its own SimulationApp when run standalone (`__main__`), same
convention as import_cr5.py/build_scene.py.

Run standalone for an isolated Tier-1 smoke test (no CR5, no vendored
factory assets -- just a dynamic cube dropped onto a ground plane, to prove
Newton is really stepping physics on this GPU before trusting it against
this repo's larger, quirkier scene):

    ${ISAACSIM_ROOT_PATH}/python.sh scripts/newton_backend.py
"""

from __future__ import annotations

from isaacsim import SimulationApp

if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": True})


def enable_newton_physics() -> bool:
    """Enables Newton's three required extensions and switches the active
    physics engine to Newton. Returns True only if every extension reported
    success.
    """
    from isaacsim.core.utils.extensions import enable_extension

    results = {
        "isaacsim.physics.newton": enable_extension("isaacsim.physics.newton"),
        "isaacsim.physics.newton.tensors": enable_extension("isaacsim.physics.newton.tensors"),
        "isaacsim.core.simulation_manager": enable_extension("isaacsim.core.simulation_manager"),
    }
    for name, ok in results.items():
        print(f"[newton_backend] enable_extension({name}): {'OK' if ok else 'FAILED'}", flush=True)
    if not all(results.values()):
        return False

    from isaacsim.core.simulation_manager import SimulationManager

    # switch_physics_engine() returns bool and never raises -- e.g. if the
    # extensions above didn't actually register the "newton" engine id, it
    # just carb.log_errors and returns False, silently leaving PhysX active.
    # Must check this, not assume success from a lack of exception.
    switched = SimulationManager.switch_physics_engine("newton", verbose=True)
    print(f"[newton_backend] switch_physics_engine(newton): {'OK' if switched else 'FAILED'}", flush=True)
    return switched


def run_tier1_cube_drop() -> tuple[float, float]:
    """Drops a single dynamic cube onto a ground plane, isolated from the
    CR5/vendored factory assets (which have their own known quirks --
    implicit x100 scale, animated content, stray duplicate rigs). Returns
    (z_before, z_after) so the caller can confirm the cube actually fell
    under gravity, not just that no exception was raised.
    """
    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid

    world = World()
    world.scene.add_default_ground_plane()
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/NewtonSmokeTestCube",
            name="newton_smoke_test_cube",
            position=np.array([0.0, 0.0, 2.0]),
            size=0.5,
        )
    )
    world.reset()

    z_before = float(cube.get_world_pose()[0][2])
    for _ in range(120):
        world.step(render=False)
    z_after = float(cube.get_world_pose()[0][2])

    return z_before, z_after


def main() -> None:
    newton_ok = enable_newton_physics()
    print(f"[newton_backend] enable_newton_physics(): {'OK' if newton_ok else 'FAILED'}", flush=True)
    if not newton_ok:
        raise RuntimeError("FAIL: could not enable Newton physics backend")

    z_before, z_after = run_tier1_cube_drop()
    print(f"[newton_backend] cube z before={z_before:.4f} after={z_after:.4f}", flush=True)

    if z_after != z_after:  # NaN check
        raise RuntimeError("FAIL: cube position is NaN after stepping")
    if not (z_after < z_before - 0.2):
        raise RuntimeError(f"FAIL: cube did not fall as expected ({z_before} -> {z_after})")

    print(f"[newton_backend] PASS: cube fell from z={z_before:.3f} to z={z_after:.3f} under Newton physics", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
