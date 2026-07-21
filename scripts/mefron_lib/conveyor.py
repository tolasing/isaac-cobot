"""ConveyorBelt_A24 setup + keyboard control, driven through Isaac Sim's own
isaacsim.asset.gen.conveyor OmniGraph node (CreateConveyorBelt command) rather than a hand-authored
PhysX attribute write. See config.CONVEYOR_BELT_PRIM_PATH's own comment for why this retries a route
abandoned on 2026-07-20, and what's different this time (explicit inputs:enabled assertion,
deterministic-path stray-graph cleanup + explicit surfaceVelocity re-zeroing before rebuild).

setup_conveyor_belt_graph() is one-time-per-script-run setup (call after
kit_experience.enable_full_experience_extensions(), which is what actually enables
isaacsim.asset.gen.conveyor); ConveyorControl is the per-frame runtime piece, structurally identical
to before except it now writes a signed float to the graph's own "Velocity" variable instead of a
Gf.Vec3f to PhysxSurfaceVelocityAPI directly.
"""

from __future__ import annotations

import omni.usd
from isaacsim.core.prims import SingleXFormPrim

from . import config


def setup_conveyor_belt_graph() -> None:
    """Builds config.CONVEYOR_ACTION_GRAPH_PATH via the CreateConveyorBelt kit command, driving
    config.CONVEYOR_BELT_PRIM_PATH -- the supported, NVIDIA-maintained entry point
    (isaacsim.asset.gen.conveyor), not a hand-authored graph. Must run after
    kit_experience.enable_full_experience_extensions() (which enables isaacsim.asset.gen.conveyor and
    omni.graph.bundle.action via config.FULL_EXPERIENCE_EXTRA_EXTENSIONS) -- so only ever called from
    mefron.py's non-headless branch, right before build_conveyor_control().

    Two defensive steps map 1:1 onto the two confirmed 2026-07-20 failures:
    - Deletes any prim already at CONVEYOR_ACTION_GRAPH_PATH (a stray survivor of a previous run's
      silent mefron.usd resave, or of this function re-running) and re-zeros the belt's own
      surfaceVelocity directly first -- since deleting a graph does NOT clear that attribute (it lives
      on the rigid body, not the graph), a leftover nonzero value would otherwise drive the belt
      forever with no graph left to stop it.
    - Explicitly forces the new node's inputs:enabled to True and reads it back, rather than trusting
      whatever the command/node template defaults to -- the prior attempt's node silently ended up
      unchecked and never computed at all.
    """
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
    """Toggled by config.CONVEYOR_TOGGLE_KEY (number-row '1'): drives the ConveyorBeltGraph's own
    "Velocity" graph variable (set up by setup_conveyor_belt_graph(), evaluated each tick by the
    isaacsim.asset.gen.conveyor OmniGraph node while playing) to carry config.MAIN_HOLDER_JIG_PRIM_PATH
    config.CONVEYOR_TRAVEL_DISTANCE each press, forward then back, reversing direction each time.
    step() must be called once per teleop frame (see teleop.run_teleop_loop()); it both applies a
    pending toggle request and, while in transit, checks the jig's live world Y against whichever
    end it's heading toward, zeroing the velocity once reached.

    A press mid-transit is ignored (not queued, not a reversal) -- deliberately simple: this belt's
    only measured, confirmed-safe behavior is a full back-to-front or front-to-back run, and half-way
    reversals were never asked for or tested.

    Direction is a fixed node input set once by setup_conveyor_belt_graph(), so unlike the previous
    direct-PhysX version, reversing here is just a sign flip on a scalar, not negating a vector.

    Each transit's target Y is measured live off the jig's own position at the moment it starts
    (config.CONVEYOR_TRAVEL_DISTANCE applied from there), not two fixed absolute world-Y endpoints --
    see config.CONVEYOR_TRAVEL_DISTANCE's own comment for why."""

    def __init__(self) -> None:
        self._state = "back"  # "back" | "moving_forward" | "front" | "moving_backward"
        self._toggle_requested = False
        self._velocity_attr = None
        self._warned_missing_velocity_attr = False
        self._transit_target_y = None

    def reset(self) -> None:
        """Called on every fresh Play (see teleop.run_teleop_loop()) -- without this, a '1' press
        queued before a Stop (or before mefron.py's own forced timeline.stop() during warmup) would
        sit on this object (built once, outside the per-Play arm-state rebuild) and fire the instant
        the next Play starts, with no new keypress at all; worse, once "moving" it silently ignores
        every further press until it reaches an end (see class docstring), making the belt look
        completely unresponsive to the key. Stop reverts the stage to its initial authored transform,
        so main_holder_jig is physically back at the start position too -- state="back" matches
        that. Re-zeroing the velocity variable here (not just once ever) also covers Stop reverting
        the graph's own authored default back to whatever was last saved."""
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
        # reset_xform_properties=False is required here -- main_holder_jig's xformOpOrder is
        # [translate, orient, scale, scale:unitsResolve] (that last op compensates the source
        # asset's metersPerUnit=0.001 down to this stage's meters), but SingleXFormPrim's default
        # (reset_xform_properties=True) forces every prim it wraps down to exactly
        # [translate, orient, scale] "post-init" -- silently stripping unitsResolve. Since this
        # constructs fresh every frame during transit (matching this codebase's Stop/Play-safety
        # convention of never caching PhysX/Fabric-backed handles across a Stop), leaving the
        # default on was re-stripping that op every single frame while the belt moved, violently
        # disrupting the jig's effective scale/transform each step -- confirmed live 2026-07-21 as
        # the actual cause of the conveyor vibration/rotation (a manual edit of the graph's own
        # Velocity variable, which never touches SingleXFormPrim, moved the same jig smoothly).
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
