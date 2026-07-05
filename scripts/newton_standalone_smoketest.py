"""Standalone smoke test for NVIDIA's Newton physics engine (newton-physics/newton
on GitHub, GPU physics built on Warp) -- completely independent of Isaac Sim.

This is NOT an Isaac Sim script: it has no `isaacsim`/`omni` imports, never
creates a `SimulationApp`, and must never be run with `python.sh`. Run it
with a plain `python3` against its own environment:

    conda create -n newton-smoke python=3.12
    conda activate newton-smoke
    pip install "newton[examples]"
    python3 scripts/newton_standalone_smoketest.py

(A venv works too, but this repo's host had no `python3.12-venv` package and
no passwordless sudo to install one -- conda was used instead. Either is
fine; the point is a throwaway env, not the Isaac Sim container's Python,
whose own pip is separately broken for unrelated reasons -- see CLAUDE.md.)

Verified against a live RTX PRO 4000 Blackwell GPU: newton 1.3.0 / warp-lang
1.14.0 correctly detect and JIT-compile for cuda:0, and a single dynamic box
dropped above a ground plane (SolverXPBD) actually falls under gravity and
comes to rest on the ground -- proof this is really stepping physics on the
GPU, not just importing cleanly.
"""

from __future__ import annotations

import warp as wp

import newton

DROP_HEIGHT = 2.0
BOX_HALF_EXTENT = 0.25
FPS = 60
SUBSTEPS = 10
NUM_FRAMES = 120


def run_box_drop(device: wp.context.Device) -> tuple[float, float]:
    """Drops a box onto a ground plane and returns (z_before, z_after)."""
    builder = newton.ModelBuilder()
    builder.add_ground_plane()
    body = builder.add_body(
        xform=wp.transform(p=wp.vec3(0.0, 0.0, DROP_HEIGHT), q=wp.quat_identity()),
        label="box",
    )
    builder.add_shape_box(body, hx=BOX_HALF_EXTENT, hy=BOX_HALF_EXTENT, hz=BOX_HALF_EXTENT)

    model = builder.finalize(device=device)
    solver = newton.solvers.SolverXPBD(model, iterations=10)

    state_0 = model.state()
    state_1 = model.state()
    control = model.control()
    contacts = model.contacts()

    dt = (1.0 / FPS) / SUBSTEPS
    z_before = float(state_0.body_q.numpy()[0][2])

    for _ in range(NUM_FRAMES):
        for _ in range(SUBSTEPS):
            state_0.clear_forces()
            model.collide(state_0, contacts)
            solver.step(state_0, state_1, control, contacts, dt)
            state_0, state_1 = state_1, state_0

    z_after = float(state_0.body_q.numpy()[0][2])
    return z_before, z_after


def main() -> None:
    wp.init()
    if not wp.is_cuda_available():
        raise RuntimeError("No CUDA device visible to Warp -- this smoke test requires a live GPU")
    device = wp.get_device("cuda:0")
    print(f"device: {device}")

    z_before, z_after = run_box_drop(device)
    print(f"z before: {z_before}")
    print(f"z after: {z_after}")

    if not (z_after == z_after):  # NaN check
        raise RuntimeError("FAIL: box position is NaN after stepping")
    if not (z_after < z_before - 0.5):
        raise RuntimeError(f"FAIL: box did not fall as expected ({z_before} -> {z_after})")

    print(f"PASS: box fell from z={z_before} and settled at z={z_after:.3f} on {device}")


if __name__ == "__main__":
    main()
