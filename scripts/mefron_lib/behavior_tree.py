"""Wraps bt_bridge's real BehaviorTree.CPP + Groot2Publisher for the assembly-placement sequence
(grasp -> lift -> align -> descend -> place). See docs/behavior-tree-migration.md for the full
design; this module replaces the inline P-handling `if`-chain _step_arm() used to have (both arm
1's gripper_control branch and arm 2's assembly_control branch) with calls into one
AssemblyPlacementBehaviorTree instance per arm.

Only two behaviors actually needed migrating into the tree -- deciding whether to start (gated on
"is something actually held"), and doing the initial lift-waypoint snap. The subsequent "descend to
the real final pose once the lift plan finishes" step is NOT a tree node: it was, and remains,
handled automatically by _step_arm()'s own per-frame plan/apply loop (the `if state["pending_final_
pose"] is not None` check at the tail of that loop, wholly unrelated to P-handling and deliberately
left untouched by this migration -- see the plan's own "only the grasp/place sequencing" scope).
So WaitForSequenceIdle's SUCCESS condition is "both the lift flight AND the auto-triggered descend
flight have finished" (cmd_plan is None AND pending_final_pose is None), not just the first plan.

Both arms share the one canonical, Groot2-editable XML (bt_bridge/trees/assembly_placement.xml) and
the same four callback_name strings -- safe because bt_bridge's callback registry is process-global
but _step_arm() only ever ticks one arm at a time (run_teleop_loop()'s `for arm in arms` loop is
plain sequential, never concurrent): each instance's tick() re-registers its own closures under
those names immediately before calling tick_once(), so there is never a moment where the "wrong"
arm's callback is registered when a tick actually runs.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

import numpy as np

from . import config
from .grasp import compute_assembly_grasp_target

_TREE_XML_PATH = Path(__file__).resolve().parent.parent.parent / "bt_bridge" / "trees" / "assembly_placement.xml"

# Groot2Publisher binds two consecutive ports per instance (server_port, server_port + 1) --
# confirmed live during the Phase 0 spike (a second instance one port above the first's still
# collided). Spaced by 2 so arm1 and arm2's trees can run -- and be monitored in Groot2 -- at the
# same time.
GROOT2_PORT_ARM1 = 1667
GROOT2_PORT_ARM2 = 1669


def resolve_relationship_name_for_grasped_object(object_name: str) -> str:
    """Reverse lookup used by arm 1 (multi-object): config.GRASP_TARGETS[object_name]["part_prim_path"]
    -> the matching config.ASSEMBLY_RELATIONSHIPS entry -- the same lookup _step_arm() did inline
    before this migration. Not every object mounts onto main_holder, so this matches by
    part_prim_path rather than assuming a "{object_name}_on_main_holder" key."""
    part_prim_path = config.GRASP_TARGETS[object_name]["part_prim_path"]
    return next(
        name
        for name, relationship in config.ASSEMBLY_RELATIONSHIPS.items()
        if relationship["part_prim_path"] == part_prim_path
    )


class AssemblyPlacementBehaviorTree:
    """One instance per arm (see mefron.py). start() arms it with that arm's live state/target/
    relationship for one placement request; tick() must be called every frame regardless (a no-op,
    returning None, until start() has been called) -- same shape as the gripper/assembly keyboard
    controls' own one-shot-request pattern."""

    def __init__(self, name: str, groot2_port: int) -> None:
        # scripts/build_bt_bridge.sh builds this into scripts/mefron_lib/, so it's an ordinary
        # submodule of the mefron_lib package -- a relative import, not `import mefron_bt_bridge`
        # (scripts/mefron_lib/ itself isn't on sys.path, only scripts/ is; confirmed live). Named
        # mefron_bt_bridge, not bt_bridge, to avoid colliding with the bt_bridge/ source directory
        # (see bt_bridge/src/bt_bridge.cpp's PYBIND11_MODULE comment for the concrete failure that
        # caused: an implicit Python namespace package silently shadowed the real extension).
        from . import mefron_bt_bridge as bt_bridge

        self._bt_bridge = bt_bridge
        self._name = name
        self._executor = bt_bridge.BTExecutor(str(_TREE_XML_PATH), groot2_port)
        self._is_holding: Callable[[], bool] | None = None
        self._state: dict | None = None
        self._target = None
        self._ee_link_prim_path: str | None = None
        self._relationship_name: str | None = None

    def start(
        self,
        *,
        state: dict,
        target,
        ee_link_prim_path: str,
        relationship_name: str,
        is_holding: Callable[[], bool],
    ) -> None:
        """Arms this instance for one placement request -- called once, right when a P press has
        passed every existing gate (has_pending_assembly_target_request()/cmd_plan is None/etc.,
        unchanged, still in teleop.py). `is_holding` is evaluated fresh on every tick, not cached
        here, matching the original inline code's live `gripper_control.closed` re-check."""
        self._state = state
        self._target = target
        self._ee_link_prim_path = ee_link_prim_path
        self._relationship_name = relationship_name
        self._is_holding = is_holding

    def reset(self) -> None:
        """Called on every fresh Play (see run_teleop_loop()), same reasoning as
        GripperKeyboardControl.reset(): without this, a placement sequence armed just before a Stop
        would still be "in flight" on the next Play, ticking against a state dict/target that
        _fresh_arm_state() has since replaced."""
        self._executor.halt()
        self._is_holding = None

    def tick(self) -> str | None:
        """Ticks this arm's tree once. Returns None (and does nothing) if start() hasn't armed a
        request since the last time this returned a non-RUNNING status; else 'RUNNING'/'SUCCESS'/
        'FAILURE' from the underlying BTExecutor."""
        if self._is_holding is None:
            return None
        self._bt_bridge.register_callback("is_holding_something", self._is_holding_something)
        self._bt_bridge.register_callback("snap_to_lift_waypoint", self._snap_to_lift_waypoint)
        self._bt_bridge.register_callback("wait_for_sequence_idle", self._wait_for_sequence_idle)
        status = self._executor.tick_once()
        if status != "RUNNING":
            self._is_holding = None
        return status

    def _is_holding_something(self) -> str:
        return "SUCCESS" if self._is_holding() else "FAILURE"

    def _snap_to_lift_waypoint(self) -> str:
        """Same computation as the old _snap_target_to_assembly_lift_waypoint(): stages the real
        final pose in state["pending_final_pose"] and snaps `target` to an intermediate waypoint --
        final X/Y and final orientation, held at the constant ASSEMBLY_LIFT_HEIGHT."""
        final_position, final_orientation = compute_assembly_grasp_target(self._ee_link_prim_path, self._relationship_name)
        self._state["pending_final_pose"] = (final_position, final_orientation)
        lift_position = np.array([final_position[0], final_position[1], config.ASSEMBLY_LIFT_HEIGHT])
        self._target.set_world_pose(position=lift_position, orientation=final_orientation)
        return "SUCCESS"

    def _wait_for_sequence_idle(self) -> str:
        if self._state["cmd_plan"] is not None or self._state["pending_final_pose"] is not None:
            return "RUNNING"
        return "SUCCESS"
