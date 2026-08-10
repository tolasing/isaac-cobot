"""Belt-graph setup + ConveyorBelt_A24's keyboard control, driven through the
isaacsim.asset.gen.conveyor OmniGraph node, not a hand-authored PhysX write: docs/mefron-history.md.
The setup/velocity helpers here are shared with feeder.py's per-part belts."""

from __future__ import annotations

import omni.usd
from isaacsim.core.prims import SingleXFormPrim

from . import config


def graph_prim_path(belt_prim_path: str, graph_prim_name: str) -> str:
    """Where CreateConveyorBelt puts its graph: a sibling of the belt body, named graph_prim_name.
    Mirrors the command's own base_path logic so callers can predict the path."""
    from pxr import Sdf

    return Sdf.Path(belt_prim_path).GetParentPath().AppendChild(graph_prim_name).pathString


def velocity_attr_path(graph_path: str) -> str:
    return f"{graph_path}.graph:variable:{config.CONVEYOR_VELOCITY_VARIABLE_NAME}"


def setup_conveyor_belt_graph(
    belt_prim_path: str = config.CONVEYOR_BELT_PRIM_PATH,
    graph_prim_name: str = config.CONVEYOR_ACTION_GRAPH_PRIM_NAME,
    local_velocity_direction=config.CONVEYOR_LOCAL_VELOCITY_DIRECTION,
) -> str | None:
    """Builds one belt's graph via the CreateConveyorBelt kit command, returning its path. Must run
    AFTER kit_experience.enable_full_experience_extensions(); rationale: docs/mefron-history.md."""
    import omni.kit.app
    import omni.kit.commands
    from pxr import Gf, PhysxSchema

    stage = omni.usd.get_context().get_stage()
    graph_path = graph_prim_path(belt_prim_path, graph_prim_name)

    stray_graph_prim = stage.GetPrimAtPath(graph_path)
    if stray_graph_prim.IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[graph_path])
        # Same post-DeletePrims pump as mount_franka()'s: without it CreateConveyorBelt can see the
        # delete as in-flight and uniquify to *_01, breaking the deterministic path.
        omni.kit.app.get_app().update()

    belt_prim = stage.GetPrimAtPath(belt_prim_path)
    if not belt_prim.IsValid():
        print(
            f"[mefron_lib] WARNING: conveyor belt {belt_prim_path} not found -- skipping its graph "
            "setup, so that belt will not move.",
            flush=True,
        )
        return None
    if belt_prim.HasAPI(PhysxSchema.PhysxSurfaceVelocityAPI):
        PhysxSchema.PhysxSurfaceVelocityAPI(belt_prim).GetSurfaceVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    # Must be the Usd.Prim itself, not its path string -- the command's do() calls .GetPath() on it,
    # which raises on a str and makes the whole command fail silently (logged, not raised).
    success, _ = omni.kit.commands.execute(
        "CreateConveyorBelt",
        prim_name=graph_prim_name,
        conveyor_prim=belt_prim,
    )
    if not success:
        print(
            f"[mefron_lib] WARNING: CreateConveyorBelt failed for {belt_prim_path} -- that belt will "
            "not move.",
            flush=True,
        )
        return None

    graph_prim = stage.GetPrimAtPath(graph_path)
    if not graph_prim.IsValid():
        print(
            f"[mefron_lib] WARNING: CreateConveyorBelt did not create {graph_path} as expected -- "
            f"{belt_prim_path} will not move.",
            flush=True,
        )
        return None

    # Keyed off inputs:direction, not inputs:conveyorPrim -- the latter is a "target"-typed input,
    # which USD stores as a relationship, so GetAttribute() on it is always invalid.
    node_prim = None
    for child in graph_prim.GetChildren():
        if child.GetAttribute("inputs:direction").IsValid():
            node_prim = child
            break
    if node_prim is None:
        print(
            f"[mefron_lib] WARNING: could not find the IsaacConveyor node under {graph_path} -- "
            f"{belt_prim_path} will not move.",
            flush=True,
        )
        return None

    node_prim.GetAttribute("inputs:direction").Set(Gf.Vec3f(*(float(v) for v in local_velocity_direction)))

    enabled_attr = node_prim.GetAttribute("inputs:enabled")
    if not enabled_attr.IsValid():
        print(
            f"[mefron_lib] WARNING: {node_prim.GetPath()} has no inputs:enabled attribute -- "
            "conveyor may not move.",
            flush=True,
        )
        enabled_readback = None
    else:
        enabled_attr.Set(True)
        enabled_readback = enabled_attr.Get()
    if enabled_readback is not True:
        print(
            f"[mefron_lib] WARNING: {node_prim.GetPath()}'s inputs:enabled read back as "
            f"{enabled_readback!r} after being explicitly set True -- conveyor will not move.",
            flush=True,
        )

    print(
        f"[mefron_lib] conveyor graph ready: node={node_prim.GetPath()} "
        f"enabled={enabled_readback} velocity_attr={velocity_attr_path(graph_path)}",
        flush=True,
    )
    return graph_path


class BeltVelocity:
    """One belt graph's Velocity variable. Must be this variable, not the node's inputs:velocity --
    the ReadVariable node overwrites that every tick. Shared by ConveyorControl and feeder.py."""

    def __init__(self, graph_path: str, label: str) -> None:
        self._attr_path = velocity_attr_path(graph_path)
        self._label = label
        self._attr = None
        self._warned = False

    def _resolve(self):
        if self._attr is not None:
            return self._attr
        attr = omni.usd.get_context().get_stage().GetAttributeAtPath(self._attr_path)
        if attr is None or not attr.IsValid():
            if not self._warned:
                print(
                    f"[mefron_lib] WARNING: conveyor graph variable {self._attr_path} not found -- "
                    f"{self._label} will do nothing.",
                    flush=True,
                )
                self._warned = True
            return None
        self._attr = attr
        return attr

    def set(self, value: float) -> None:
        attr = self._resolve()
        if attr is None:
            return
        attr.Set(float(value))


class ConveyorControl:
    """Toggled by config.CONVEYOR_TOGGLE_KEY: carries main_holder_jig CONVEYOR_TRAVEL_DISTANCE each
    press, forward then back. State machine + mid-transit-press rationale: docs/mefron-history.md."""

    def __init__(self, graph_path: str = config.CONVEYOR_ACTION_GRAPH_PATH) -> None:
        self._state = "back"  # "back" | "moving_forward" | "front" | "moving_backward"
        self._toggle_requested = False
        self._velocity = BeltVelocity(graph_path, f"key {config.CONVEYOR_TOGGLE_KEY}")
        self._transit_target_y = None

    def reset(self) -> None:
        """Called on every fresh Play, or a press queued before a Stop fires the instant Play starts.
        Stop reverts the jig to its start position, so state="back" matches. docs/mefron-history.md."""
        self._state = "back"
        self._toggle_requested = False
        self._transit_target_y = None
        self._set_velocity(0.0)

    def request_toggle(self) -> None:
        self._toggle_requested = True

    def _set_velocity(self, value: float) -> None:
        self._velocity.set(value)

    def _jig_world_y(self) -> float:
        # reset_xform_properties=False is required -- the default strips main_holder_jig's
        # unitsResolve xformOp every frame, the confirmed root cause of conveyor vibration.
        xform = SingleXFormPrim(prim_path=config.MAIN_HOLDER_JIG_PRIM_PATH, reset_xform_properties=False)
        position, _ = xform.get_world_pose()
        return float(position[1])

    def step(self) -> None:
        if self._toggle_requested:
            self._toggle_requested = False
            if self._state == "back":
                self._transit_target_y = self._jig_world_y() + config.CONVEYOR_TRAVEL_DISTANCE
                self._set_velocity(config.CONVEYOR_SPEED)
                self._state = "moving_forward"
                print("[mefron] conveyor: moving main_holder_jig forward.", flush=True)
            elif self._state == "front":
                self._transit_target_y = self._jig_world_y() - config.CONVEYOR_TRAVEL_DISTANCE
                self._set_velocity(-config.CONVEYOR_SPEED)
                self._state = "moving_backward"
                print("[mefron] conveyor: moving main_holder_jig backward.", flush=True)
            # Mid-transit presses are ignored -- see class docstring.

        if self._state == "moving_forward" and self._jig_world_y() >= self._transit_target_y:
            self._set_velocity(0.0)
            self._state = "front"
            print("[mefron] conveyor: main_holder_jig reached forward end.", flush=True)
        elif self._state == "moving_backward" and self._jig_world_y() <= self._transit_target_y:
            self._set_velocity(0.0)
            self._state = "back"
            print("[mefron] conveyor: main_holder_jig reached backward end.", flush=True)


def build_conveyor_control(key: str = config.CONVEYOR_TOGGLE_KEY) -> ConveyorControl:
    import carb.input
    import omni.appwindow

    control = ConveyorControl()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()
    toggle_input = getattr(carb.input.KeyboardInput, key)

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input == toggle_input:
            control.request_toggle()
        return True

    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control
