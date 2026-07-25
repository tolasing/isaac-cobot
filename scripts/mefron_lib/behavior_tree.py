"""Wraps bt_bridge's real BehaviorTree.CPP + Groot2Publisher for the grasp/approach and
assembly-placement sequences (grasp/approach -> wait -> engage; lift -> wait -> auto-descend ->
place). See docs/behavior-tree-migration.md for the full design.

Two shared, generated, Groot2-editable XML trees -- bt_bridge/trees/generated/grasp_main.xml and
placement_main.xml, regenerated from config.py on every GraspObjectBehaviorTree/
PlaceObjectBehaviorTree construction -- replace both the old per-object Python `if`-chains AND the
narrower single-purpose trees an earlier pass of this migration built. Each generated file is one
`MainTree` = Fallback of per-object branches (one per config.GRASP_TARGETS/APPROACH_TARGETS or
"kind": "placement" config.ASSEMBLY_RELATIONSHIPS entry), each branch gated on a PyCondition
comparing its own literal object_name/relationship_name port against a `requested_object`/
`requested_relationship` blackboard value, then a `<SubTree>` instantiation of one shared,
hand-written-once implementation tree (GraspObjectImpl / PlaceObjectImpl) parameterized entirely by
real BT.CPP ports -- not Python closures. Per-object data (which yaml, which tolerances, which end
effector) is real tree/blackboard data now, visible and editable in Groot2's XML; only the *live*
per-arm bindings that can't be serialized as ports (the draggable `target` Xform, `ee_link_prim_path`,
gripper_control/surface_gripper_control, the `state` dict `_step_arm()` mutates) stay Python-side,
bound once per arm at construction/start() time -- see each class's own docstring.

Both arms share the SAME generated files (arm 1's 3 GRASP_TARGETS + arm 2's 1 APPROACH_TARGETS
entry all live in one grasp_main.xml Fallback; every "kind": "placement" ASSEMBLY_RELATIONSHIPS
entry lives in one placement_main.xml Fallback) -- which branch actually does anything is entirely
determined by whichever object/relationship that arm's own instance last wrote to its OWN tree's
blackboard, so nothing about sharing the file lets one arm's request affect the other's. bt_bridge's
callback registry is process-global, but _step_arm() only ever ticks one arm at a time
(run_teleop_loop()'s `for arm in arms` loop is plain sequential, never concurrent): each instance's
tick() re-registers its own closures under the fixed callback names immediately before calling
tick_once(), so there is never a moment where the "wrong" arm's callback is registered when a tick
actually runs -- same safety argument as before, now additionally reinforced by each BTExecutor
having its own isolated blackboard (a second, independent source of per-instance separation).
"""

from __future__ import annotations

import xml.sax.saxutils
from pathlib import Path
from typing import Callable

import numpy as np

from . import config

# grasp.py's own top-level `from isaacsim.core.prims import SingleXFormPrim` requires a running
# SimulationApp -- deferred into the callback methods that actually need it (same pattern grasp.py
# itself already uses for its *internal* omni imports) rather than imported here at module level,
# so generate_grasp_tree_xml()/generate_placement_tree_xml() (pure config.py -> XML string, no omni
# dependency at all) stay callable standalone -- e.g. for a quick sanity check of the generated
# XML without booting Isaac Sim first.

_GENERATED_TREES_DIR = Path(__file__).resolve().parent.parent.parent / "bt_bridge" / "trees" / "generated"

# Groot2Publisher binds two consecutive ports per instance (server_port, server_port + 1) --
# confirmed live during the Phase 0 spike (a second instance one port above the first's still
# collided). Spaced by 2 so all four instances (2 arms x {grasp, place}) can run -- and be
# monitored in Groot2 -- at the same time.
GROOT2_PORT_ARM1_PLACE = 1667
GROOT2_PORT_ARM2_PLACE = 1669
GROOT2_PORT_ARM1_GRASP = 1671
GROOT2_PORT_ARM2_GRASP = 1673


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


def _xml_attr(value) -> str:
    return xml.sax.saxutils.quoteattr(str(value))


# Hand-written once, shared by every object -- reused via <SubTree> instantiation, never copied.
# Every {port} here is GraspObjectImpl's OWN blackboard scope, populated by whichever <SubTree
# ID="GraspObjectImpl" .../> instantiation is currently executing (see _grasp_dispatch_branch()).
_GRASP_OBJECT_IMPL_XML = """\
  <BehaviorTree ID="GraspObjectImpl">
    <Sequence name="GraspObject">
      <PyAction name="SnapToObjectPose" callback_name="snap_to_object_pose"
                pose_source="{pose_source}" yaml_path="{yaml_path}" grasp_name="{grasp_name}"
                part_prim_path="{part_prim_path}" relationship_name="{relationship_name}"/>
      <PyAction name="WaitForHandAtTarget" callback_name="wait_for_hand_at_target"
                position_tolerance="{position_tolerance}" orientation_tolerance="{orientation_tolerance}"/>
      <PyAction name="WaitBeforeEngaging" callback_name="wait_before_engaging"
                delay_seconds="{delay_seconds}"/>
      <PyAction name="EngageEndEffector" callback_name="engage_end_effector"
                end_effector_kind="{end_effector_kind}"/>
    </Sequence>
  </BehaviorTree>
"""

_PLACEMENT_OBJECT_IMPL_XML = """\
  <BehaviorTree ID="PlaceObjectImpl">
    <Sequence name="PlaceObject">
      <PyCondition name="IsHoldingSomething" callback_name="is_holding_something"/>
      <PyAction name="SnapToLiftWaypoint" callback_name="snap_to_lift_waypoint"
                relationship_name="{relationship_name}"/>
      <PyAction name="WaitForSequenceIdle" callback_name="wait_for_sequence_idle"/>
    </Sequence>
  </BehaviorTree>
"""


def _node_models_xml_fragment() -> str:
    """The `<TreeNodesModel>...</TreeNodesModel>` block describing PyAction/PyCondition's real
    ports, straight from BT.CPP's own `writeTreeNodesModelXML()` (via bt_bridge.node_models_xml(),
    which wraps it in its own throwaway `<root>` -- only the inner block is wanted here, this
    generator's own `<root>` already wraps everything). Embedded into every generated file:
    without it, a tool parsing the file standalone (Groot2's Editor mode, no live connection to
    this process) has no way to know PyAction/PyCondition's ports and rejects any it doesn't
    recognize -- confirmed live: "Node 'WaitForHandAtTarget' contains port
    'orientation_tolerance' in the XML that doesn't match any of the ports in the model"."""
    from . import mefron_bt_bridge as bt_bridge

    wrapped = bt_bridge.node_models_xml()
    start = wrapped.index("<TreeNodesModel>")
    end = wrapped.index("</TreeNodesModel>") + len("</TreeNodesModel>")
    return wrapped[start:end]


def _grasp_dispatch_branch(object_name: str, entry: dict) -> str:
    condition = (
        f'<PyCondition name="IsRequested_{object_name}" callback_name="is_requested_object" '
        f"object_name={_xml_attr(object_name)} requested_object=\"{{requested_object}}\"/>"
    )
    attrs = " ".join(
        f"{key}={_xml_attr(value)}" for key, value in entry.items() if key != "key"
    )
    subtree = f'<SubTree ID="GraspObjectImpl" object_name={_xml_attr(object_name)} {attrs}/>'
    return f"<Sequence>{condition}{subtree}</Sequence>"


def generate_grasp_tree_xml() -> Path:
    """Regenerates bt_bridge/trees/generated/grasp_main.xml from config.GRASP_TARGETS +
    config.APPROACH_TARGETS -- one Fallback branch per entry, dispatched at runtime by whichever
    object_name the currently-ticking GraspObjectBehaviorTree instance last wrote to its own
    tree's "requested_object" blackboard key (see GraspObjectBehaviorTree.start()). Adding an
    object is purely a new config.py entry; this function is the only thing that turns it into
    tree structure, so there's nothing else to hand-edit."""
    entries = {**config.GRASP_TARGETS, **config.APPROACH_TARGETS}
    branches = "\n      ".join(_grasp_dispatch_branch(name, entry) for name, entry in entries.items())
    xml_text = (
        '<?xml version="1.0"?>\n'
        '<root BTCPP_format="4" main_tree_to_execute="MainTree">\n'
        f"{_node_models_xml_fragment()}\n"
        f"{_GRASP_OBJECT_IMPL_XML}"
        '  <BehaviorTree ID="MainTree">\n'
        '    <Fallback name="GraspDispatch">\n'
        f"      {branches}\n"
        "    </Fallback>\n"
        "  </BehaviorTree>\n"
        "</root>\n"
    )
    _GENERATED_TREES_DIR.mkdir(parents=True, exist_ok=True)
    path = _GENERATED_TREES_DIR / "grasp_main.xml"
    path.write_text(xml_text)
    return path


def _placement_dispatch_branch(relationship_name: str) -> str:
    condition = (
        f'<PyCondition name="IsRequested_{relationship_name}" callback_name="is_requested_relationship" '
        f"relationship_name={_xml_attr(relationship_name)} requested_relationship=\"{{requested_relationship}}\"/>"
    )
    subtree = f'<SubTree ID="PlaceObjectImpl" relationship_name={_xml_attr(relationship_name)}/>'
    return f"<Sequence>{condition}{subtree}</Sequence>"


def generate_placement_tree_xml() -> Path:
    """Regenerates bt_bridge/trees/generated/placement_main.xml from every "kind": "placement"
    config.ASSEMBLY_RELATIONSHIPS entry -- same Fallback-of-branches shape as
    generate_grasp_tree_xml(), dispatched by a "requested_relationship" blackboard key instead of
    "requested_object"."""
    names = [name for name, relationship in config.ASSEMBLY_RELATIONSHIPS.items() if relationship.get("kind") == "placement"]
    branches = "\n      ".join(_placement_dispatch_branch(name) for name in names)
    xml_text = (
        '<?xml version="1.0"?>\n'
        '<root BTCPP_format="4" main_tree_to_execute="MainTree">\n'
        f"{_node_models_xml_fragment()}\n"
        f"{_PLACEMENT_OBJECT_IMPL_XML}"
        '  <BehaviorTree ID="MainTree">\n'
        '    <Fallback name="PlacementDispatch">\n'
        f"      {branches}\n"
        "    </Fallback>\n"
        "  </BehaviorTree>\n"
        "</root>\n"
    )
    _GENERATED_TREES_DIR.mkdir(parents=True, exist_ok=True)
    path = _GENERATED_TREES_DIR / "placement_main.xml"
    path.write_text(xml_text)
    return path


class GraspObjectBehaviorTree:
    """One instance per arm with a grasp/approach flow -- arm 1 (parallel-jaw, J/B/K) and arm 2
    (suction, N) both use this same class and the same generated grasp_main.xml; only which
    object_name gets written to this instance's OWN blackboard (start()) and which of
    gripper_control/surface_gripper_control was bound at construction differ. tick() must be
    called every frame regardless (a no-op, returning None, until start() has armed a request
    since the last non-RUNNING result)."""

    def __init__(
        self,
        name: str,
        groot2_port: int,
        *,
        gripper_control=None,
        surface_gripper_control=None,
    ) -> None:
        # scripts/build_bt_bridge.sh builds this into scripts/mefron_lib/, so it's an ordinary
        # submodule of the mefron_lib package -- a relative import, not `import mefron_bt_bridge`
        # (scripts/mefron_lib/ itself isn't on sys.path, only scripts/ is; confirmed live). Named
        # mefron_bt_bridge, not bt_bridge, to avoid colliding with the bt_bridge/ source directory
        # (see bt_bridge/src/bt_bridge.cpp's PYBIND11_MODULE comment for the concrete failure that
        # caused: an implicit Python namespace package silently shadowed the real extension).
        from . import mefron_bt_bridge as bt_bridge

        self._bt_bridge = bt_bridge
        self._name = name
        self._executor = bt_bridge.BTExecutor(str(generate_grasp_tree_xml()), groot2_port)
        self._gripper_control = gripper_control
        self._surface_gripper_control = surface_gripper_control
        self._target = None
        self._ee_link_prim_path: str | None = None
        self._armed = False
        self._wait_start_time: float | None = None

    def start(self, *, object_name: str, target, ee_link_prim_path: str) -> None:
        """Arms this instance for one grasp/approach request -- called right when a J/B/K/N press
        is consumed. All per-object data (which yaml, which tolerances, ...) comes from the
        generated tree's own config-derived ports, not from arguments here -- this only carries
        the live, per-arm bindings that can't be a port."""
        self._target = target
        self._ee_link_prim_path = ee_link_prim_path
        self._wait_start_time = None
        self._armed = True
        self._executor.set_blackboard("requested_object", object_name)

    def reset(self) -> None:
        """Called on every fresh Play (see run_teleop_loop()): without this, a request armed just
        before a Stop would still be "in flight" on the next Play, ticking against a target the
        physics-view rebuild has since invalidated."""
        self._executor.halt()
        self._armed = False
        self._wait_start_time = None

    def tick(self) -> str | None:
        if not self._armed:
            return None
        self._bt_bridge.register_callback("is_requested_object", self._is_requested_object)
        self._bt_bridge.register_callback("snap_to_object_pose", self._snap_to_object_pose)
        self._bt_bridge.register_callback("wait_for_hand_at_target", self._wait_for_hand_at_target)
        self._bt_bridge.register_callback("wait_before_engaging", self._wait_before_engaging)
        self._bt_bridge.register_callback("engage_end_effector", self._engage_end_effector)
        status = self._executor.tick_once()
        if status != "RUNNING":
            self._armed = False
        return status

    def _is_requested_object(self, ports: dict) -> str:
        return "SUCCESS" if ports.get("object_name") == ports.get("requested_object") else "FAILURE"

    def _snap_to_object_pose(self, ports: dict) -> str:
        from .grasp import compute_grasp_approach_pose_from_file, compute_grasp_finger_widths_from_file, compute_part_target_pose

        pose_source = ports["pose_source"]
        if pose_source == "grasp_yaml":
            position, orientation = compute_grasp_approach_pose_from_file(
                ports["yaml_path"], ports["grasp_name"], part_prim_path=ports["part_prim_path"]
            )
            if self._gripper_control is not None:
                open_position, closed_position = compute_grasp_finger_widths_from_file(
                    ports["yaml_path"], ports["grasp_name"]
                )
                self._gripper_control.set_grasp_widths(open_position, closed_position)
                self._gripper_control.set_closed(False)
        elif pose_source == "relationship":
            position, orientation = compute_part_target_pose(ports["relationship_name"])
        else:
            raise ValueError(f"unknown pose_source {pose_source!r}")
        self._target.set_world_pose(position=position, orientation=orientation)
        return "SUCCESS"

    def _wait_for_hand_at_target(self, ports: dict) -> str:
        from isaacsim.core.prims import SingleXFormPrim

        hand_position, hand_orientation = SingleXFormPrim(
            prim_path=self._ee_link_prim_path, reset_xform_properties=False
        ).get_world_pose()
        target_position, target_orientation = self._target.get_world_pose()
        position_delta = np.linalg.norm(hand_position - target_position)
        orientation_delta = np.linalg.norm(hand_orientation - target_orientation)
        if position_delta <= float(ports["position_tolerance"]) and orientation_delta <= float(
            ports["orientation_tolerance"]
        ):
            return "SUCCESS"
        return "RUNNING"

    def _wait_before_engaging(self, ports: dict) -> str:
        import time

        now = time.monotonic()
        if self._wait_start_time is None:
            self._wait_start_time = now
        if now - self._wait_start_time >= float(ports["delay_seconds"]):
            return "SUCCESS"
        return "RUNNING"

    def _engage_end_effector(self, ports: dict) -> str:
        kind = ports["end_effector_kind"]
        if kind == "parallel_gripper":
            self._gripper_control.set_closed(True)
        elif kind == "suction":
            self._surface_gripper_control.close()
        else:
            raise ValueError(f"unknown end_effector_kind {kind!r}")
        return "SUCCESS"


class PlaceObjectBehaviorTree:
    """One instance per arm (arm 1 and arm 2), both sharing the same generated
    placement_main.xml. start() arms it with that arm's live state/target/relationship/is_holding
    for one placement request; tick() must be called every frame regardless (a no-op, returning
    None, until start() has been called).

    Only two behaviors actually needed migrating into the tree -- deciding whether to start (gated
    on "is something actually held"), and doing the initial lift-waypoint snap. The subsequent
    "descend to the real final pose once the lift plan finishes" step is NOT a tree node: it was,
    and remains, handled automatically by _step_arm()'s own per-frame plan/apply loop (the
    state["pending_final_pose"] check at the tail of that loop, wholly unrelated to P-handling and
    deliberately left untouched by this migration). So WaitForSequenceIdle's SUCCESS condition is
    "both the lift flight AND that auto-triggered descend flight have finished" (cmd_plan is None
    AND pending_final_pose is None), not just the first plan."""

    def __init__(self, name: str, groot2_port: int) -> None:
        from . import mefron_bt_bridge as bt_bridge

        self._bt_bridge = bt_bridge
        self._name = name
        self._executor = bt_bridge.BTExecutor(str(generate_placement_tree_xml()), groot2_port)
        self._is_holding: Callable[[], bool] | None = None
        self._state: dict | None = None
        self._target = None
        self._ee_link_prim_path: str | None = None

    def start(
        self,
        *,
        state: dict,
        target,
        ee_link_prim_path: str,
        relationship_name: str,
        is_holding: Callable[[], bool],
    ) -> None:
        """`is_holding` is evaluated fresh on every tick, not cached here, matching the original
        inline code's live `gripper_control.closed` re-check."""
        self._state = state
        self._target = target
        self._ee_link_prim_path = ee_link_prim_path
        self._is_holding = is_holding
        self._executor.set_blackboard("requested_relationship", relationship_name)

    def reset(self) -> None:
        self._executor.halt()
        self._is_holding = None

    def tick(self) -> str | None:
        if self._is_holding is None:
            return None
        self._bt_bridge.register_callback("is_requested_relationship", self._is_requested_relationship)
        self._bt_bridge.register_callback("is_holding_something", self._is_holding_something)
        self._bt_bridge.register_callback("snap_to_lift_waypoint", self._snap_to_lift_waypoint)
        self._bt_bridge.register_callback("wait_for_sequence_idle", self._wait_for_sequence_idle)
        status = self._executor.tick_once()
        if status != "RUNNING":
            self._is_holding = None
        return status

    def _is_requested_relationship(self, ports: dict) -> str:
        return "SUCCESS" if ports.get("relationship_name") == ports.get("requested_relationship") else "FAILURE"

    def _is_holding_something(self, ports: dict) -> str:
        return "SUCCESS" if self._is_holding() else "FAILURE"

    def _snap_to_lift_waypoint(self, ports: dict) -> str:
        """Same computation as the old _snap_target_to_assembly_lift_waypoint(): stages the real
        final pose in state["pending_final_pose"] and snaps `target` to an intermediate waypoint --
        final X/Y and final orientation, held at the constant ASSEMBLY_LIFT_HEIGHT."""
        from .grasp import compute_assembly_grasp_target

        final_position, final_orientation = compute_assembly_grasp_target(
            self._ee_link_prim_path, ports["relationship_name"]
        )
        self._state["pending_final_pose"] = (final_position, final_orientation)
        lift_position = np.array([final_position[0], final_position[1], config.ASSEMBLY_LIFT_HEIGHT])
        self._target.set_world_pose(position=lift_position, orientation=final_orientation)
        return "SUCCESS"

    def _wait_for_sequence_idle(self, ports: dict) -> str:
        if self._state["cmd_plan"] is not None or self._state["pending_final_pose"] is not None:
            return "RUNNING"
        return "SUCCESS"
