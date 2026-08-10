"""Headless regression test for the per-part conveyor feeders: asserts each belt's graph and
photo-eye come up, and that removing the parked part makes the belt advance the next one and stop
it on the belt. No arm, no cuRobo. Run with --headless."""

from __future__ import annotations

import sys

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # Base experience, like test_mefron_assembly_weld_headless.py -- feeder.setup_part_feeders()
    # enables the two extensions it needs itself.
    simulation_app = SimulationApp({"headless": _headless})

# Must run before any omni/curobo import -- see mefron_lib/kit_bootstrap.py's docstring.
from mefron_lib.kit_bootstrap import clear_stale_robot_configuration, preload_real_packaging  # noqa: E402

preload_real_packaging()

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleRigidPrim, SingleXFormPrim  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402
from mefron_lib import config, conveyor, feeder  # noqa: E402

# --belt=<part name> runs one config.PART_FEEDERS entry (e.g. --belt=screen); default is all of them.
_BELT_FILTER = next((arg.split("=", 1)[1] for arg in sys.argv if arg.startswith("--belt=")), None)
# Frames to let PhysX/the sensor settle after Play before reading any beam.
_SETTLE_FRAMES = 60
# Cap on frames spent waiting for one belt's cycle. 0.05m/s over a ~0.25m gap is ~5s = ~300 frames at
# 60Hz, so this is generous without hanging a broken run forever.
_CYCLE_FRAME_BUDGET = 1200
# How far the "picked" part is teleported, and the kinematic flip that keeps it there against gravity.
_PICK_LIFT = 0.5
# A one-part belt has nothing to advance, so its cut-off is shortened from config's 0.85m rather than
# spending ~17s of sim time proving the same branch.
_EMPTY_BELT_MAX_TRAVEL = 0.05
_ORIGINAL_MAX_TRAVEL = config.FEEDER_MAX_TRAVEL
# How far the arriving copy may land from where the original was parked. main_holder_back_cover's
# parked pose is already 0.826m from the mount against a ~0.855m envelope, so this is a reach budget.
_PICK_SPOT_TOLERANCE = 0.02


def movers_pose(prim_path: str, label: str) -> float:
    return float(SingleRigidPrim(prim_path=prim_path, name=f"mefron_feeder_test_pose_{label}").get_world_pose()[0][1])

_failures: list[str] = []


def _check(label: str, condition: bool) -> None:
    print(f"[test_mefron_feeder_headless] {'PASS' if condition else 'FAIL'}: {label}", flush=True)
    if not condition:
        _failures.append(label)


def _note(message: str) -> None:
    print(f"[test_mefron_feeder_headless] NOTE: {message}", flush=True)


def _set_kinematic(prim_path: str, kinematic: bool) -> None:
    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(kinematic)


def _check_setup(part_feeder: feeder.PartFeeder, graph_path: str) -> None:
    stage = omni.usd.get_context().get_stage()
    name = part_feeder.base_part_prim_path.rsplit("/", 1)[-1]

    graph_prim = stage.GetPrimAtPath(graph_path)
    _check(f"{name}: conveyor graph authored at the predicted path", graph_prim.IsValid())
    # inputs:enabled silently False is one of the two confirmed failure modes this whole route was
    # rebuilt around -- see docs/mefron-history.md.
    node_prims = [child for child in graph_prim.GetChildren() if child.GetAttribute("inputs:direction").IsValid()]
    _check(f"{name}: the graph has an IsaacConveyor node", bool(node_prims))
    if node_prims:
        _check(f"{name}: the node's inputs:enabled is True", node_prims[0].GetAttribute("inputs:enabled").Get() is True)
    _check(
        f"{name}: the graph's Velocity variable exists",
        stage.GetAttributeAtPath(conveyor.velocity_attr_path(graph_path)).IsValid(),
    )
    sensor_prim = stage.GetPrimAtPath(part_feeder.sensor_prim_path)
    _check(f"{name}: photo-eye authored at {part_feeder.sensor_prim_path}", sensor_prim.IsValid())
    if sensor_prim.IsValid():
        min_range = float(sensor_prim.GetAttribute("minRange").Get())
        max_range = float(sensor_prim.GetAttribute("maxRange").Get())
        _check(f"{name}: photo-eye ranges bracket the parked face ({min_range:.4f} < {max_range:.4f})", 0.0 < min_range < max_range)
    # The visible housing/beam. They must stay collider-free, or the beam's own raycast hits them.
    for suffix in ("_housing", "_beam"):
        visual_prim = stage.GetPrimAtPath(f"{part_feeder.sensor_prim_path}{suffix}")
        _check(f"{name}: photo-eye{suffix} is visible geometry", visual_prim.IsValid())
        if visual_prim.IsValid():
            _check(
                f"{name}: photo-eye{suffix} carries no collider",
                not any(prim.HasAPI(UsdPhysics.CollisionAPI) for prim in Usd.PrimRange(visual_prim)),
            )


def _check_resolver(base_part_prim_path: str) -> None:
    name = base_part_prim_path.rsplit("/", 1)[-1]
    queue = feeder.belt_queue(base_part_prim_path)
    _check(f"{name}: belt_queue() finds it on its belt", bool(queue))
    if not queue:
        return
    feeder.clear_instance_state()
    _check(f"{name}: resolve_for_pick() returns the front of the queue", feeder.resolve_for_pick(base_part_prim_path) == queue[0])
    _check(f"{name}: base_part_prim_path_of() maps the copy back to config's base", feeder.base_part_prim_path_of(queue[0]) == base_part_prim_path)
    _check(f"{name}: latch() pins resolve_for_pick() to that copy", feeder.latch(base_part_prim_path) == queue[0])
    feeder.record_assembled(base_part_prim_path, queue[0])
    _check(f"{name}: resolve_assembled() names the welded copy", feeder.resolve_assembled(base_part_prim_path) == queue[0])
    if len(queue) > 1:
        # The whole point of the split: the assembled copy and the next pick are different prims,
        # even though the "welded" copy here never physically left the belt.
        _check(
            f"{name}: after the weld, resolve_for_pick() moves on to the next copy",
            feeder.resolve_for_pick(base_part_prim_path) == queue[1],
        )
    feeder.forget_assembled(queue[0])
    _check(f"{name}: forgetting the weld puts that copy back at the front of the queue", feeder.belt_queue(base_part_prim_path) == queue)
    feeder.clear_instance_state()


def _run_cycle(control: feeder.FeederControl, part_feeder: feeder.PartFeeder, graph_path: str) -> None:
    """Removes the parked part, then runs the loop the way run_teleop_loop() does and watches the
    state machine take the belt through advance -> trip -> stop."""
    stage = omni.usd.get_context().get_stage()
    name = part_feeder.base_part_prim_path.rsplit("/", 1)[-1]
    queue = feeder.belt_queue(part_feeder.base_part_prim_path)
    if not queue:
        _note(f"{name}: nothing on its belt -- skipping the cycle checks.")
        return

    _check(f"{name}: photo-eye sees the parked part", part_feeder.beam_occupied() is True)
    # The authored pick spot, in the frame the grasp poses are actually computed from.
    parked_origin_y = float(SingleRigidPrim(prim_path=queue[0], name=f"mefron_feeder_test_parked_{name}").get_world_pose()[0][1])

    has_next = len(queue) > 1
    if not has_next:
        _note(
            f"{name}: only one part on its belt, so this run checks the empty-belt cut-off instead of "
            "an arrival. Add copies in the GUI (see CLAUDE.md) to cover the queue path."
        )
        config.FEEDER_MAX_TRAVEL = _EMPTY_BELT_MAX_TRAVEL

    # "Picked": kinematic so gravity can't drop it back onto the belt and re-trip the beam.
    picked = SingleRigidPrim(prim_path=queue[0], name=f"mefron_feeder_test_picked_{name}")
    picked_trans, picked_quat = picked.get_world_pose()
    _set_kinematic(queue[0], True)
    lifted = np.array(picked_trans) + np.array([0.0, 0.0, _PICK_LIFT])
    picked.set_world_pose(position=lifted, orientation=picked_quat)
    # The USD xform too: set_world_pose() on a SingleRigidPrim writes the PhysX/Fabric pose, and for
    # a KINEMATIC body nothing writes it back to USD -- which is what feeder.belt_queue() reads. A
    # real gripper lift moves a DYNAMIC body, whose simulated transform does get written back.
    SingleXFormPrim(prim_path=queue[0], reset_xform_properties=False).set_world_pose(
        position=lifted, orientation=picked_quat
    )
    for _ in range(_SETTLE_FRAMES):
        simulation_app.update()
    _check(f"{name}: photo-eye clears once the part is gone", part_feeder.beam_occupied() is False)
    lifted_pose, _ = picked.get_world_pose()
    _note(f"{name}: after the lift, USD says z={float(lifted_pose[2]):.4f}, belt_queue={feeder.belt_queue(part_feeder.base_part_prim_path)}")
    _check(
        f"{name}: the lifted part left the belt queue",
        queue[0] not in feeder.belt_queue(part_feeder.base_part_prim_path),
    )

    velocity_attr = stage.GetAttributeAtPath(conveyor.velocity_attr_path(graph_path))
    states_seen = set()
    fed_velocity = 0.0
    for frame in range(_CYCLE_FRAME_BUDGET):
        # config.FEEDER_ADVANCE_DELAY_SECONDS is exercised by the wait itself; the key path is used
        # here only so five belts don't cost 25s of sim time on top of their travel.
        if part_feeder.state == "waiting":
            control.request_advance()
        simulation_app.update()
        control.step()
        states_seen.add(part_feeder.state)
        if part_feeder.state == "feeding":
            fed_velocity = float(velocity_attr.Get() or 0.0)
        if part_feeder.state == "empty" or (has_next and part_feeder.state == "occupied" and "stopping" in states_seen):
            break

    _check(f"{name}: the cleared beam started a wait", "waiting" in states_seen)
    _check(f"{name}: the belt ran", "feeding" in states_seen)
    _check(
        f"{name}: it ran toward the robot (Velocity {fed_velocity:+.4f} matches config's {config.FEEDER_SPEED:+.4f})",
        fed_velocity != 0.0 and np.sign(fed_velocity) == np.sign(config.FEEDER_SPEED),
    )
    if has_next:
        _check(f"{name}: the next copy tripped the photo-eye and the belt stopped", "stopping" in states_seen)
        _check(f"{name}: the feeder is back to occupied", part_feeder.state == "occupied")
        _check(f"{name}: the belt's Velocity is back to zero", float(velocity_attr.Get() or 0.0) == 0.0)
        arrived_bound = feeder._world_bound(queue[1])
        belt_bound = feeder._world_bound(part_feeder.belt_prim_path)
        if arrived_bound is not None and belt_bound is not None:
            overshoot = float(arrived_bound.GetMax()[1]) - float(belt_bound.GetMax()[1])
            # The whole point of the ramped stop: the part must not be hanging off the front edge.
            _check(f"{name}: the arrived part is still on the belt (leading face {overshoot:+.4f}m vs the front edge)", overshoot < 0.0)
        # The check that decides whether the arm can still reach it: grasp poses are computed from
        # the part's own live frame, so the copy must land where the original was parked.
        arrived_origin_y = float(movers_pose(queue[1], name))
        error = arrived_origin_y - parked_origin_y
        _check(
            f"{name}: the copy landed on the authored pick spot (off by {error:+.4f}m)",
            abs(error) <= _PICK_SPOT_TOLERANCE,
        )
    else:
        _check(f"{name}: an empty belt stops itself at the travel cut-off", part_feeder.state == "empty")
        _check(f"{name}: the belt's Velocity is back to zero", float(velocity_attr.Get() or 0.0) == 0.0)

    config.FEEDER_MAX_TRAVEL = _ORIGINAL_MAX_TRAVEL
    if not has_next:
        # Only safe to put back when nothing advanced into its place -- restoring it on top of an
        # arrived copy would leave two interpenetrating bodies to explode.
        picked.set_world_pose(position=picked_trans, orientation=picked_quat)
        _set_kinematic(queue[0], False)


def main() -> None:
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)
    carb.settings.get_settings().set_bool("/app/player/useFixedTimeStepping", True)

    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)
    stage_context = omni.usd.get_context()
    stage_context.open_stage(str(config.MEFRON_USD))
    for _ in range(120):
        simulation_app.update()

    stage = stage_context.get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    feeders = feeder.setup_part_feeders()
    if _BELT_FILTER is not None:
        feeders = [f for f in feeders if f["base_part_prim_path"].rsplit("/", 1)[-1] == _BELT_FILTER]
        if not feeders:
            raise SystemExit(f"[test_mefron_feeder_headless] no armed feeder matches --belt={_BELT_FILTER}")
    _check(
        f"setup_part_feeders() armed every config.PART_FEEDERS belt ({len(feeders)}/{len(config.PART_FEEDERS)})",
        _BELT_FILTER is not None or len(feeders) == len(config.PART_FEEDERS),
    )

    part_feeders = [feeder.PartFeeder(**entry) for entry in feeders]
    for part_feeder, entry in zip(part_feeders, feeders):
        _check_setup(part_feeder, entry["graph_path"])
        _check_resolver(part_feeder.base_part_prim_path)

    control = feeder.FeederControl(part_feeders)
    _check("a feeder reads no beam data before Play", all(f.beam_occupied() is None for f in part_feeders))

    omni.timeline.get_timeline_interface().play()
    for _ in range(_SETTLE_FRAMES):
        simulation_app.update()
    control.reset()

    for part_feeder, entry in zip(part_feeders, feeders):
        _run_cycle(control, part_feeder, entry["graph_path"])

    if _failures:
        print(f"[test_mefron_feeder_headless] {len(_failures)} CHECK(S) FAILED: {_failures}", flush=True)
    else:
        print("[test_mefron_feeder_headless] ALL CHECKS PASSED", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
