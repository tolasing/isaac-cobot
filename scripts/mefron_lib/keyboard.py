"""The five keyboard control objects the teleop loop consumes: gripper, suction approach, surface
gripper, tool changer, screws. Each owns its own carb.input subscription and one-shot requests."""

from __future__ import annotations

from . import config


class GripperKeyboardControl:
    """Open/closed request for the docked gripper tool, plus one-shot P and J/B/K snap requests.
    Each grasp key overwrites the widths, so C/O ramp toward whichever object was last selected."""

    def __init__(self) -> None:
        self.closed = False
        self.open_position = config.GRIPPER_OPEN_POSITION
        self.closed_position = config.GRIPPER_CLOSED_POSITION
        self._assembly_target_requested = False
        self._grasp_approach_object_requested: str | None = None
        self._release_requested = False
        # Last config.GRASP_TARGETS key requested -- lets P look up the matching
        # config.ASSEMBLY_RELATIONSHIPS entry instead of a single hardcoded object.
        self.last_grasped_object: str | None = None

    def set_closed(self, closed: bool) -> None:
        self.closed = closed

    def request_release(self) -> None:
        """Set by the open key ONLY when the gripper was actually closed -- the teleop loop turns
        this into an assembly weld (see assembly.weld_part_at_assembly_pose())."""
        self._release_requested = True

    def consume_release_request(self) -> bool:
        requested = self._release_requested
        self._release_requested = False
        return requested

    def set_grasp_widths(self, open_position: float, closed_position: float) -> None:
        self.open_position = open_position
        self.closed_position = closed_position

    def request_assembly_target(self) -> None:
        self._assembly_target_requested = True

    def has_pending_assembly_target_request(self) -> bool:
        """Peek without consuming -- the loop holds a P request open until the robot is idle, since
        snapping /World/target mid-plan would be silently discarded."""
        return self._assembly_target_requested

    def consume_assembly_target_request(self) -> bool:
        requested = self._assembly_target_requested
        self._assembly_target_requested = False
        return requested

    def request_grasp_approach_from_file(self, object_name: str) -> None:
        self._grasp_approach_object_requested = object_name
        self.last_grasped_object = object_name

    def consume_grasp_approach_from_file_request(self) -> str | None:
        requested = self._grasp_approach_object_requested
        self._grasp_approach_object_requested = None
        return requested

    def reset(self) -> None:
        """Called on every fresh Play -- this object outlives the per-Play state rebuild, so without
        this a stale P request or last_grasped_object survives a Stop."""
        self.closed = False
        self.last_grasped_object = None
        self._assembly_target_requested = False
        self._grasp_approach_object_requested = None
        self._release_requested = False


def build_gripper_keyboard_control(close_key: str = "C", open_key: str = "O") -> GripperKeyboardControl:
    """Subscribes close_key/open_key, one key per config.GRASP_TARGETS (J/B/K), and P for the
    assembly-placement snap."""
    import carb.input
    import omni.appwindow

    control = GripperKeyboardControl()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()

    grasp_key_bindings = {
        getattr(carb.input.KeyboardInput, target["key"]): object_name
        for object_name, target in config.GRASP_TARGETS.items()
    }
    close_input = getattr(carb.input.KeyboardInput, close_key)
    open_input = getattr(carb.input.KeyboardInput, open_key)

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == close_input:
                control.set_closed(True)
            elif event.input == open_input:
                # Only a close->open transition is a real release: J/B/K also open the gripper (to
                # pregrasp width), and re-pressing O when already open must not fire a weld.
                if control.closed:
                    control.request_release()
                control.set_closed(False)
            elif event.input == carb.input.KeyboardInput.P:
                control.request_assembly_target()
            elif event.input in grasp_key_bindings:
                control.request_grasp_approach_from_file(grasp_key_bindings[event.input])
        return True

    # Kept alive on the control object so the subscription isn't garbage-collected.
    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control


class SuctionApproachControl:
    """One-shot 'snap target to an object's suction-approach pose' request, one key per
    config.SUCTION_TARGETS entry. Only acted on while the suction tool is docked."""

    def __init__(self) -> None:
        self._requested_object: str | None = None
        # Mirrors GripperKeyboardControl.last_grasped_object -- lets P look up the matching
        # assembly relationship instead of a single hardcoded object.
        self.last_approached_object: str | None = None

    def request_approach(self, object_name: str) -> None:
        self._requested_object = object_name
        self.last_approached_object = object_name

    def consume_approach_request(self) -> str | None:
        requested = self._requested_object
        self._requested_object = None
        return requested

    def reset(self) -> None:
        """Called on every fresh Play, same reasoning as GripperKeyboardControl.reset() -- a stale
        last_approached_object would route P to something nothing is attached to."""
        self._requested_object = None
        self.last_approached_object = None


def build_suction_approach_keyboard_control(
    key_bindings: dict[str, str] | None = None,
) -> SuctionApproachControl:
    """key_bindings defaults to config.SUCTION_TARGETS (N for screen, M for pcb_assembly)."""
    import carb.input
    import omni.appwindow

    if key_bindings is None:
        key_bindings = {target["key"]: object_name for object_name, target in config.SUCTION_TARGETS.items()}

    control = SuctionApproachControl()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()
    request_bindings = {
        getattr(carb.input.KeyboardInput, key): object_name for key, object_name in key_bindings.items()
    }

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input in request_bindings:
            control.request_approach(request_bindings[event.input])
        return True

    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control


class SurfaceGripperKeyboardControl:
    """Requests close/open on the real Surface Gripper runtime once per keypress -- its C++ manager
    owns the state machine, and is_closed() reads that manager-owned state back."""

    def __init__(self, gripper_prim_path: str) -> None:
        import isaacsim.robot.surface_gripper._surface_gripper as surface_gripper

        self.gripper_prim_path = gripper_prim_path
        self._interface = surface_gripper.acquire_surface_gripper_interface()
        self._release_requested = False
        self._attach_requested = False

    def close(self) -> None:
        self._attach_requested = True
        self._interface.close_gripper(self.gripper_prim_path)

    def open(self) -> None:
        # Flagged only when something was actually held, mirroring GripperKeyboardControl's own
        # close->open gate -- the teleop loop turns this into an assembly weld.
        if self.is_closed():
            self._release_requested = True
        self._interface.open_gripper(self.gripper_prim_path)

    def consume_release_request(self) -> bool:
        requested = self._release_requested
        self._release_requested = False
        return requested

    def consume_attach_request(self) -> bool:
        """Lets the teleop loop check, a few frames later, whether the grab actually took -- a
        failed one leaves the wrist welded to the world. See docs/fr5-migration.md."""
        requested = self._attach_requested
        self._attach_requested = False
        return requested

    def is_closed(self) -> bool:
        import isaacsim.robot.surface_gripper._surface_gripper as surface_gripper

        status = self._interface.get_gripper_status(self.gripper_prim_path)
        return surface_gripper.GripperStatus(status) == surface_gripper.GripperStatus.Closed


def build_surface_gripper_keyboard_control(
    gripper_prim_path: str,
    close_key: str = config.SUCTION_ATTACH_KEY,
    open_key: str = config.SUCTION_DETACH_KEY,
    tool_changer_control: "ToolChangerControl | None" = None,
) -> SurfaceGripperKeyboardControl:
    """tool_changer_control, if given, gates V/L on the suction tool actually being docked -- the
    attach joint rides panda_hand permanently, whichever tool is on."""
    import carb.input
    import omni.appwindow

    control = SurfaceGripperKeyboardControl(gripper_prim_path)
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()
    close_input = getattr(carb.input.KeyboardInput, close_key)
    open_input = getattr(carb.input.KeyboardInput, open_key)

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input in (close_input, open_input):
            if tool_changer_control is not None and tool_changer_control.currently_docked_tool != "suction":
                print("[mefron] suction attach/detach ignored -- the suction tool isn't currently docked.", flush=True)
                return True
            if event.input == close_input:
                control.close()
            else:
                control.open()
        return True

    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control


class ToolChangerControl:
    """One-shot 'swap to this tool' request (Y/U/I), plus which tool is currently docked so
    motion._build_tool_change_queue() knows whether a return-to-rack leg is needed first."""

    def __init__(self) -> None:
        self._requested_tool: str | None = None
        self.currently_docked_tool: str | None = None

    def request_tool(self, tool_name: str) -> None:
        self._requested_tool = tool_name

    def has_pending_request(self) -> bool:
        return self._requested_tool is not None

    def consume_request(self) -> str | None:
        requested = self._requested_tool
        self._requested_tool = None
        return requested

    def reset(self) -> None:
        """Clears only the one-shot request, NOT currently_docked_tool -- a Stop doesn't remove the
        FixedJoint from the stage, so whichever tool was docked still physically is."""
        self._requested_tool = None


def build_tool_changer_keyboard_control() -> ToolChangerControl:
    """Subscribes one key per config.TOOL_CHANGE_TARGETS entry (Y/U/I on this branch)."""
    import carb.input
    import omni.appwindow

    control = ToolChangerControl()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()
    key_bindings = {
        getattr(carb.input.KeyboardInput, target["key"]): tool_name
        for tool_name, target in config.TOOL_CHANGE_TARGETS.items()
    }

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input in key_bindings:
            control.request_tool(key_bindings[event.input])
        return True

    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control


class ScrewControl:
    """One-shot pick/place requests, only acted on while the screwdriver is docked. Two requests
    rather than one cycle, so the pick can be inspected before committing to the placement."""

    def __init__(self) -> None:
        self._pick_requested = False
        self._place_requested = False
        # Next config.SCREW_HOLES index to fill, and which screw (if any) is on the bit right now.
        self.hole_index = 0
        self.carried_screw_index: int | None = None

    def request_pick(self) -> None:
        self._pick_requested = True

    def request_place(self) -> None:
        self._place_requested = True

    def has_pending_pick_request(self) -> bool:
        return self._pick_requested

    def has_pending_place_request(self) -> bool:
        return self._place_requested

    def consume_pick_request(self) -> bool:
        requested = self._pick_requested
        self._pick_requested = False
        return requested

    def consume_place_request(self) -> bool:
        requested = self._place_requested
        self._place_requested = False
        return requested

    def reset(self) -> None:
        """Clears only the one-shot requests, not hole_index/carried_screw_index -- same reasoning
        as ToolChangerControl.reset(): already-filled holes are still filled after a Stop."""
        self._pick_requested = False
        self._place_requested = False


def build_screw_keyboard_control() -> ScrewControl:
    """Subscribes config.SCREW_PICK_KEY/SCREW_PLACE_KEY, with its own subscription like every other
    control here -- the screwdriver has no existing per-key control to piggyback onto."""
    import carb.input
    import omni.appwindow

    control = ScrewControl()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()
    pick_input = getattr(carb.input.KeyboardInput, config.SCREW_PICK_KEY)
    place_input = getattr(carb.input.KeyboardInput, config.SCREW_PLACE_KEY)

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == pick_input:
                control.request_pick()
            elif event.input == place_input:
                control.request_place()
        return True

    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control
