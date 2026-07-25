"""ConveyorBelt_A24 setup + keyboard control, driven through Isaac Sim's own
isaacsim.asset.gen.conveyor OmniGraph node rather than a hand-authored PhysX attribute write --
see docs/mefron-history.md for why. setup_conveyor_belt_graph() is one-time-per-script-run setup
(after kit_experience.enable_full_experience_extensions()); ConveyorControl is the per-frame piece."""

from __future__ import annotations

import omni.usd
from isaacsim.core.prims import SingleXFormPrim

from . import config


def setup_conveyor_belt_graph() -> None:
    """Builds config.CONVEYOR_ACTION_GRAPH_PATH via the CreateConveyorBelt kit command -- the
    supported entry point, not a hand-authored graph. Must run after
    kit_experience.enable_full_experience_extensions(). Deletes any stray prior graph, re-zeros
    surfaceVelocity, and forces+reads back inputs:enabled -- see docs/mefron-history.md for why."""
    import omni.kit.app
    import omni.kit.commands
    from pxr import Gf, PhysxSchema

    stage = omni.usd.get_context().get_stage()

    stray_graph_prim = stage.GetPrimAtPath(config.CONVEYOR_ACTION_GRAPH_PATH)
    if stray_graph_prim.IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[config.CONVEYOR_ACTION_GRAPH_PATH])
        # Same reasoning as mount_franka()'s own post-DeletePrims pump: without this, CreateConveyorBelt
        # below can see the deletion as still in-flight and uniquify to *_01 instead of reusing this
        # exact path, breaking the deterministic-path assumption ConveyorControl relies on.
        omni.kit.app.get_app().update()

    belt_prim = stage.GetPrimAtPath(config.CONVEYOR_BELT_PRIM_PATH)
    if not belt_prim.IsValid():
        print(
            f"[mefron_lib] WARNING: conveyor belt {config.CONVEYOR_BELT_PRIM_PATH} not found -- "
            f"skipping conveyor graph setup, key {config.CONVEYOR_TOGGLE_KEY} will do nothing.",
            flush=True,
        )
        return
    if belt_prim.HasAPI(PhysxSchema.PhysxSurfaceVelocityAPI):
        PhysxSchema.PhysxSurfaceVelocityAPI(belt_prim).GetSurfaceVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    # Must be the Usd.Prim itself, not its path string -- the command's own do() calls
    # self._conveyor_prim.GetPath(), which raises AttributeError on a str and makes the whole
    # command fail (logged, not raised -- confirmed live via a throwaway headless repro against an
    # empty stage/scratch prim before finding this).
    success, _ = omni.kit.commands.execute(
        "CreateConveyorBelt",
        prim_name=config.CONVEYOR_ACTION_GRAPH_PRIM_NAME,
        conveyor_prim=belt_prim,
    )
    if not success:
        print(
            "[mefron_lib] WARNING: CreateConveyorBelt command failed -- "
            f"key {config.CONVEYOR_TOGGLE_KEY} will do nothing.",
            flush=True,
        )
        return

    graph_prim = stage.GetPrimAtPath(config.CONVEYOR_ACTION_GRAPH_PATH)
    if not graph_prim.IsValid():
        print(
            f"[mefron_lib] WARNING: CreateConveyorBelt did not create "
            f"{config.CONVEYOR_ACTION_GRAPH_PATH} as expected -- key {config.CONVEYOR_TOGGLE_KEY} "
            "will do nothing.",
            flush=True,
        )
        return

    # Keyed off inputs:direction, not inputs:conveyorPrim -- the latter is a "target"-typed input,
    # which USD represents as a relationship, not an attribute, so GetAttribute() on it is always
    # invalid regardless of which child this is.
    node_prim = None
    for child in graph_prim.GetChildren():
        if child.GetAttribute("inputs:direction").IsValid():
            node_prim = child
            break
    if node_prim is None:
        print(
            f"[mefron_lib] WARNING: could not find the IsaacConveyor node under "
            f"{config.CONVEYOR_ACTION_GRAPH_PATH} -- key {config.CONVEYOR_TOGGLE_KEY} will do nothing.",
            flush=True,
        )
        return

    node_prim.GetAttribute("inputs:direction").Set(Gf.Vec3f(*(float(v) for v in config.CONVEYOR_LOCAL_VELOCITY_DIRECTION)))

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

    velocity_attr_path = f"{config.CONVEYOR_ACTION_GRAPH_PATH}.graph:variable:{config.CONVEYOR_VELOCITY_VARIABLE_NAME}"
    print(
        f"[mefron_lib] conveyor graph ready: node={node_prim.GetPath()} "
        f"enabled={enabled_readback} velocity_attr={velocity_attr_path}",
        flush=True,
    )


class ConveyorControl:
    """Toggled by config.CONVEYOR_TOGGLE_KEY: drives the ConveyorBeltGraph's "Velocity" variable to
    carry main_holder_jig config.CONVEYOR_TRAVEL_DISTANCE each press, forward then back. step()
    applies a pending toggle each teleop frame and zeros velocity once the jig reaches the target.
    See docs/mefron-history.md for the state-machine/mid-transit-press rationale."""

    def __init__(self) -> None:
        self._state = "back"  # "back" | "moving_forward" | "front" | "moving_backward"
        self._toggle_requested = False
        self._velocity_attr = None
        self._warned_missing_velocity_attr = False
        self._transit_target_y = None

    def reset(self) -> None:
        """Called on every fresh Play. Without this, a '1' press queued before a Stop would fire the
        instant the next Play starts with no new keypress, and once "moving" it silently ignores
        further presses until reaching an end. Stop reverts the stage (and the jig) to its start
        position, so state="back" matches -- see docs/mefron-history.md."""
        self._state = "back"
        self._toggle_requested = False
        self._transit_target_y = None
        self._set_velocity(0.0)

    def request_toggle(self) -> None:
        self._toggle_requested = True

    def _resolve_velocity_attr(self):
        if self._velocity_attr is not None:
            return self._velocity_attr
        stage = omni.usd.get_context().get_stage()
        attr = stage.GetAttributeAtPath(
            f"{config.CONVEYOR_ACTION_GRAPH_PATH}.graph:variable:{config.CONVEYOR_VELOCITY_VARIABLE_NAME}"
        )
        if attr is None or not attr.IsValid():
            if not self._warned_missing_velocity_attr:
                print(
                    f"[mefron_lib] WARNING: conveyor graph variable "
                    f"{config.CONVEYOR_ACTION_GRAPH_PATH}.graph:variable:{config.CONVEYOR_VELOCITY_VARIABLE_NAME} "
                    f"not found -- key {config.CONVEYOR_TOGGLE_KEY} will do nothing.",
                    flush=True,
                )
                self._warned_missing_velocity_attr = True
            return None
        self._velocity_attr = attr
        return attr

    def _set_velocity(self, value: float) -> None:
        attr = self._resolve_velocity_attr()
        if attr is None:
            return
        attr.Set(float(value))

    def _jig_world_y(self) -> float:
        # reset_xform_properties=False is required -- the default silently strips main_holder_jig's
        # unitsResolve xformOp every frame, which was the confirmed root cause of conveyor
        # vibration. See docs/mefron-history.md.
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
