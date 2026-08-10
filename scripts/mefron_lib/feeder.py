"""Per-part conveyor feeders: one belt per sub-part, queueing GUI-placed copies, stopped by a real
light-beam photo-eye. Also owns the base-path -> live-instance resolver. Design: docs/part-feeders.md."""

from __future__ import annotations

import re

import numpy as np
import omni.usd
from isaacsim.core.prims import SingleXFormPrim

from . import config, conveyor

_SENSOR_EXTENSION = "isaacsim.sensors.physx"
_CONVEYOR_EXTENSION = "isaacsim.asset.gen.conveyor"

# Which copy the arm is currently holding, and which copy ended up assembled, per BASE part path.
# Module-level (not on FeederControl) so grasp.py/assembly.py can resolve without threading it through.
_LATCHED: dict[str, str] = {}
_ASSEMBLED: dict[str, str] = {}
# Acquired lazily -- isaacsim.sensors.physx isn't importable until the extension is enabled.
_LIGHTBEAM_INTERFACE = None


def _wake_bodies(prim_paths: list[str]) -> None:
    """A part resting untouched for a second is asleep to PhysX, and turning a KINEMATIC belt's
    surface velocity on does not wake it -- it just sits there. Confirmed live; docs/part-feeders.md."""
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools, UsdUtils

    stage_id = UsdUtils.StageCache.Get().GetId(omni.usd.get_context().get_stage()).ToLongInt()
    simulation = get_physx_simulation_interface()
    for prim_path in prim_paths:
        simulation.wake_up(stage_id, PhysicsSchemaTools.sdfPathToInt(prim_path))


def _lightbeam_interface():
    global _LIGHTBEAM_INTERFACE
    if _LIGHTBEAM_INTERFACE is None:
        try:
            from isaacsim.sensors.physx import _range_sensor
        except ImportError:
            return None
        _LIGHTBEAM_INTERFACE = _range_sensor.acquire_lightbeam_sensor_interface()
    return _LIGHTBEAM_INTERFACE


# --- instance discovery -------------------------------------------------------------------------


def _instance_pattern(base_part_prim_path: str) -> re.Pattern:
    """`<base>` or `<base>_<digits>` (Ctrl+D's own naming). A bare prefix would be wrong -- it makes
    main_holder match main_holder_back_cover and main_holder_jig."""
    from pxr import Sdf

    return re.compile(rf"^{re.escape(Sdf.Path(base_part_prim_path).name)}(_\d+)?$")


def part_instance_prim_paths(base_part_prim_path: str) -> list[str]:
    """Every copy of one part on the stage, GUI-placed or original, ordered by name."""
    from pxr import Sdf, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    parent = stage.GetPrimAtPath(Sdf.Path(base_part_prim_path).GetParentPath())
    if not parent.IsValid():
        return []
    pattern = _instance_pattern(base_part_prim_path)
    return sorted(
        child.GetPath().pathString
        for child in parent.GetChildren()
        if pattern.match(child.GetName()) and child.HasAPI(UsdPhysics.RigidBodyAPI)
    )


def all_part_instance_prim_paths() -> list[str]:
    """Every copy of every feeder-fed part -- for the collision/friction sweeps that used to iterate
    config's base paths alone."""
    paths: list[str] = []
    for base_part_prim_path in config.PART_FEEDERS:
        paths.extend(part_instance_prim_paths(base_part_prim_path))
    return paths


def _world_bound(prim_path: str):
    from pxr import Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    bound = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return None if bound.IsEmpty() else bound


def _origin_y(prim_path: str) -> float:
    # reset_xform_properties=False -- parts carry an xformOp:scale:unitsResolve op (see grasp.py).
    position, _ = SingleXFormPrim(prim_path=prim_path, reset_xform_properties=False).get_world_pose()
    return float(position[1])


def belt_travel_scale(belt_prim_path: str) -> float:
    """How much world travel one unit of belt-local surface velocity buys. These belts carry a 0.5
    scale, so config.FEEDER_SPEED is half its value in m/s -- measured, see docs/part-feeders.md."""
    from pxr import Gf, Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(belt_prim_path)
    if not prim.IsValid():
        return 1.0
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    length = matrix.TransformDir(Gf.Vec3d(*config.FEEDER_LOCAL_VELOCITY_DIRECTION)).GetLength()
    return float(length) or 1.0


def belt_queue(base_part_prim_path: str) -> list[str]:
    """The copies still riding this part's belt, front (largest world Y, nearest the robot) first.
    Recomputed from live poses, so a lifted or assembled part leaves the queue by itself."""
    feeder = config.PART_FEEDERS.get(base_part_prim_path)
    if feeder is None:
        return []
    belt_bound = _world_bound(feeder["belt_prim_path"])
    if belt_bound is None:
        return []
    belt_min, belt_max = belt_bound.GetMin(), belt_bound.GetMax()
    margin = config.FEEDER_QUEUE_FOOTPRINT_MARGIN

    on_belt = []
    assembled = set(_ASSEMBLED.values())
    for instance_prim_path in part_instance_prim_paths(base_part_prim_path):
        # A copy already built into the assembly is never a pick candidate again, whatever its pose
        # says -- release_assembly_weld() is what puts it back in the running.
        if instance_prim_path in assembled:
            continue
        # reset_xform_properties=False -- parts carry an xformOp:scale:unitsResolve op the default
        # would silently strip (see grasp.py).
        position, _ = SingleXFormPrim(prim_path=instance_prim_path, reset_xform_properties=False).get_world_pose()
        if not belt_min[0] - margin <= position[0] <= belt_max[0] + margin:
            continue
        if not belt_min[1] - margin <= position[1] <= belt_max[1] + margin:
            continue
        if abs(float(position[2]) - belt_max[2]) > config.FEEDER_QUEUE_HEIGHT_TOLERANCE:
            continue
        on_belt.append((float(position[1]), instance_prim_path))
    return [instance_prim_path for _, instance_prim_path in sorted(on_belt, reverse=True)]


# --- base-path -> live-instance resolution -------------------------------------------------------


def resolve_for_pick(part_prim_path: str) -> str:
    """Which copy a pick/place/weld acts on: the latched one if the arm is holding one, else whatever
    is at the station. Identity for any path with no feeder, which is what keeps call sites one-line."""
    if part_prim_path not in config.PART_FEEDERS:
        return part_prim_path
    latched = _LATCHED.get(part_prim_path)
    if latched is not None:
        return latched
    queue = belt_queue(part_prim_path)
    return queue[0] if queue else part_prim_path


def resolve_assembled(part_prim_path: str) -> str:
    """Which copy is already built into the assembly -- what a MOUNT pose and the screw holes must
    read. Distinct from resolve_for_pick(), which by then points at the next copy on the belt."""
    if part_prim_path not in config.PART_FEEDERS:
        return part_prim_path
    return _ASSEMBLED.get(part_prim_path, part_prim_path)


def latch(part_prim_path: str) -> str:
    """Pins resolve_for_pick() to the station's current copy, for the whole grasp->place->weld cycle:
    a lifted part leaves belt_queue(), so nothing else could still name it."""
    resolved = resolve_for_pick(part_prim_path)
    if part_prim_path in config.PART_FEEDERS:
        _LATCHED[part_prim_path] = resolved
    return resolved


def record_assembled(part_prim_path: str, instance_prim_path: str) -> None:
    """Called by the release weld: this copy is now part of the assembly, and the latch is spent."""
    if part_prim_path not in config.PART_FEEDERS:
        return
    _ASSEMBLED[part_prim_path] = instance_prim_path
    _LATCHED.pop(part_prim_path, None)


def forget_assembled(instance_prim_path: str) -> None:
    """Exact inverse of record_assembled(), called when a weld is released -- otherwise that copy
    stays excluded from belt_queue() forever and no key could ever pick it up again."""
    for base_part_prim_path, assembled_prim_path in list(_ASSEMBLED.items()):
        if assembled_prim_path == instance_prim_path:
            del _ASSEMBLED[base_part_prim_path]


def base_part_prim_path_of(instance_prim_path: str) -> str:
    """Instance path -> its config base path, for the config lookups that must stay keyed on the
    base (e.g. ASSEMBLY_WELD_KEEP_COLLISION_PART_PRIM_PATHS). Identity if it isn't an instance."""
    for candidate in config.PART_FEEDERS:
        if _instance_pattern(candidate).match(instance_prim_path.rsplit("/", 1)[-1]):
            return candidate
    return instance_prim_path


def clear_instance_state() -> None:
    """Latches only -- _ASSEMBLED survives a Stop for the same reason ToolChangerControl.reset()
    keeps currently_docked_tool: Stop doesn't delete the weld joint, so the part is still assembled."""
    _LATCHED.clear()


# --- stage setup --------------------------------------------------------------------------------


def _ensure_extension(extension_name: str) -> bool:
    """Headless runs skip kit_experience.enable_full_experience_extensions(), so enable what this
    module needs itself rather than failing on an import."""
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled(extension_name):
        manager.set_extension_enabled_immediate(extension_name, True)
    return manager.is_extension_enabled(extension_name)


def clear_feeder_sensors() -> None:
    """Wipes the script-owned sensor scope. Unconditional: the URDF importer rewrites mefron.usd
    every run, so a stray scope can be inherited, and the create command uniquifies onto it."""
    import omni.kit.app
    import omni.kit.commands

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(config.FEEDER_SENSOR_SCOPE_PRIM_PATH).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[config.FEEDER_SENSOR_SCOPE_PRIM_PATH])
        # Same post-DeletePrims pump as the conveyor graph's: without it the create command below
        # sees the delete as in-flight and uniquifies to *_01.
        omni.kit.app.get_app().update()


def prime_surface_velocity(belt_prim_path: str) -> bool:
    """Applies PhysxSurfaceVelocityAPI (zeroed, enabled) BEFORE Play. The conveyor node applies it
    itself, but mid-session PhysX may never resync the already-created body -- docs/part-feeders.md."""
    from pxr import Gf, PhysxSchema

    prim = omni.usd.get_context().get_stage().GetPrimAtPath(belt_prim_path)
    if not prim.IsValid():
        return False
    surface_velocity_api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
    surface_velocity_api.CreateSurfaceVelocityEnabledAttr().Set(True)
    surface_velocity_api.CreateSurfaceVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    return True


def _beam_sensor_prim_path(belt_prim_path: str) -> str:
    from pxr import Sdf

    return f"{config.FEEDER_SENSOR_SCOPE_PRIM_PATH}/beam_{Sdf.Path(belt_prim_path).GetParentPath().name}"


def _build_beam_visual(sensor_prim_path: str, x: float, mount_y: float, z: float, max_range: float) -> None:
    """A visible barrel housing plus its beam. The sensor's own debug draw is a few-cm line 1mm above
    the belt, under the part and behind the end roller -- invisible in practice. No colliders here."""
    from pxr import Gf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    housing = UsdGeom.Cylinder.Define(stage, f"{sensor_prim_path}_housing")
    housing.CreateAxisAttr("Y")
    housing.CreateRadiusAttr(config.FEEDER_BEAM_HOUSING_RADIUS)
    housing.CreateHeightAttr(config.FEEDER_BEAM_HOUSING_LENGTH)
    housing.CreateDisplayColorPrimvar().Set([Gf.Vec3f(0.05, 0.05, 0.07)])
    UsdGeom.XformCommonAPI(housing).SetTranslate(
        Gf.Vec3d(x, mount_y + config.FEEDER_BEAM_HOUSING_LENGTH / 2.0, z)
    )

    beam = UsdGeom.Cylinder.Define(stage, f"{sensor_prim_path}_beam")
    beam.CreateAxisAttr("Y")
    beam.CreateRadiusAttr(config.FEEDER_BEAM_VISUAL_RADIUS)
    beam.CreateHeightAttr(max_range)
    beam.CreateDisplayColorPrimvar().Set([Gf.Vec3f(0.9, 0.05, 0.05)])
    UsdGeom.XformCommonAPI(beam).SetTranslate(Gf.Vec3d(x, mount_y - max_range / 2.0, z))


def build_beam_sensor(base_part_prim_path: str, belt_prim_path: str) -> str | None:
    """One photo-eye per belt, on the belt's front frame aiming back down it. min/max range bracket
    the parked part's leading face, so an empty station reads NO hit -- docs/part-feeders.md."""
    import omni.kit.commands
    from pxr import Gf

    belt_bound = _world_bound(belt_prim_path)
    if belt_bound is None:
        print(f"[mefron_lib] WARNING: no bounds for belt {belt_prim_path} -- skipping its photo-eye.", flush=True)
        return None
    queue = belt_queue(base_part_prim_path)
    if not queue:
        print(
            f"[mefron_lib] WARNING: nothing riding {belt_prim_path} at setup, so there is no parked "
            f"pose to aim its photo-eye at -- {base_part_prim_path}'s feeder is disabled this run.",
            flush=True,
        )
        return None
    front_bound = _world_bound(queue[0])
    if front_bound is None:
        print(f"[mefron_lib] WARNING: no bounds for {queue[0]} -- skipping {belt_prim_path}'s photo-eye.", flush=True)
        return None

    mount_y = float(belt_bound.GetMax()[1]) + config.FEEDER_BEAM_MOUNT_STANDOFF
    trip_y = float(front_bound.GetMax()[1])
    min_range = mount_y - trip_y - config.FEEDER_BEAM_PRETRIP
    if min_range <= 0.0:
        print(
            f"[mefron_lib] WARNING: {queue[0]}'s leading face is past the front of {belt_prim_path} "
            f"(min_range={min_range:.4f}m) -- move it back in the GUI; feeder disabled this run.",
            flush=True,
        )
        return None
    max_range = min_range + config.FEEDER_BEAM_DEPTH

    # The ray line: the front part's own centre X, one CURTAIN_BASE_OFFSET above the belt surface.
    ray_x = float((front_bound.GetMin()[0] + front_bound.GetMax()[0]) / 2.0)
    ray_z = float(belt_bound.GetMax()[2]) + config.FEEDER_BEAM_CURTAIN_BASE_OFFSET

    sensor_prim_path = _beam_sensor_prim_path(belt_prim_path)
    omni.usd.get_context().get_stage().DefinePrim(config.FEEDER_SENSOR_SCOPE_PRIM_PATH, "Scope")
    success, _ = omni.kit.commands.execute(
        "IsaacSensorCreateLightBeamSensor",
        path=sensor_prim_path,
        parent="",
        translation=Gf.Vec3d(ray_x, mount_y, ray_z),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        num_rays=config.FEEDER_BEAM_NUM_RAYS,
        curtain_length=config.FEEDER_BEAM_CURTAIN_LENGTH,
        # World -Y (back down the belt) and +Z (a curtain, so a 5mm screen and a 29mm scanner both
        # break it) -- the sensor prim itself is unrotated, so these are world axes.
        forward_axis=Gf.Vec3d(0.0, -1.0, 0.0),
        curtain_axis=Gf.Vec3d(0.0, 0.0, 1.0),
        min_range=min_range,
        max_range=max_range,
        draw_lines=True,
        draw_points=True,
    )
    if not success or not omni.usd.get_context().get_stage().GetPrimAtPath(sensor_prim_path).IsValid():
        print(f"[mefron_lib] WARNING: could not create the photo-eye at {sensor_prim_path}.", flush=True)
        return None
    _build_beam_visual(sensor_prim_path, ray_x, mount_y, ray_z, max_range)
    print(
        f"[mefron_lib] photo-eye {sensor_prim_path}: range [{min_range:.4f}, {max_range:.4f}]m at "
        f"y={mount_y:.4f}, watching {queue[0]} ({len(queue)} on the belt).",
        flush=True,
    )
    return sensor_prim_path


def setup_part_feeders() -> list[dict]:
    """One conveyor graph + one photo-eye per config.PART_FEEDERS entry. Same slot as
    conveyor.setup_conveyor_belt_graph(): after the full experience is enabled."""
    if not _ensure_extension(_CONVEYOR_EXTENSION) or not _ensure_extension(_SENSOR_EXTENSION):
        print(
            f"[mefron_lib] WARNING: {_CONVEYOR_EXTENSION}/{_SENSOR_EXTENSION} would not enable -- "
            "the part feeders are disabled this run.",
            flush=True,
        )
        return []

    clear_feeder_sensors()
    feeders = []
    for base_part_prim_path, feeder in config.PART_FEEDERS.items():
        belt_prim_path = feeder["belt_prim_path"]
        # Before the graph, so the body already carries the API when PhysX first creates it.
        prime_surface_velocity(belt_prim_path)
        graph_path = conveyor.setup_conveyor_belt_graph(
            belt_prim_path=belt_prim_path,
            graph_prim_name=config.FEEDER_GRAPH_PRIM_NAME,
            local_velocity_direction=config.FEEDER_LOCAL_VELOCITY_DIRECTION,
        )
        if graph_path is None:
            continue
        sensor_prim_path = build_beam_sensor(base_part_prim_path, belt_prim_path)
        if sensor_prim_path is None:
            continue
        feeders.append(
            {
                "base_part_prim_path": base_part_prim_path,
                "belt_prim_path": belt_prim_path,
                "graph_path": graph_path,
                "sensor_prim_path": sensor_prim_path,
            }
        )
    return feeders


# --- runtime ------------------------------------------------------------------------------------


class PartFeeder:
    """One belt's cycle: beam clears -> wait -> feed -> beam trips -> ramp to a stop. States are
    "occupied" | "waiting" | "feeding" | "stopping" | "empty"."""

    def __init__(self, base_part_prim_path: str, belt_prim_path: str, graph_path: str, sensor_prim_path: str) -> None:
        self.base_part_prim_path = base_part_prim_path
        self.belt_prim_path = belt_prim_path
        self.sensor_prim_path = sensor_prim_path
        self._velocity = conveyor.BeltVelocity(graph_path, f"{base_part_prim_path}'s feeder")
        self._name = base_part_prim_path.rsplit("/", 1)[-1]
        self._travel_scale = belt_travel_scale(belt_prim_path)
        # minRange is one PRETRIP short of the authored parked face, and the sensor clamps any closer
        # reading to it, so "depth is at minRange" means "the part is at the pick spot".
        min_range_attr = omni.usd.get_context().get_stage().GetAttributeAtPath(f"{sensor_prim_path}.minRange")
        self._arrival_depth = float(min_range_attr.Get()) + config.FEEDER_BEAM_ARRIVAL_EPSILON
        # The authored pick spot, read once before Play while every copy is still where the GUI put
        # it. Only used to bound how far the belt may run -- the photo-eye still decides arrival.
        self._station_y = _origin_y(queue[0]) if (queue := belt_queue(base_part_prim_path)) else None
        self.reset()

    def reset(self) -> None:
        """Every fresh Play: Stop reverts the parts to their start poses, so the cycle restarts from
        "occupied" and re-seeds from the beam on the first step that has data."""
        self.state = "occupied"
        self._commanded_velocity = 0.0
        self._waited = 0.0
        self._travel = 0.0
        self._riding = []
        self._velocity.set(0.0)

    # Ramped, not stepped: an instant surface-velocity change can tip a part, and the ramp-down is
    # what keeps the stop inside the front-edge budget. See docs/part-feeders.md.
    def _ramp_toward(self, target_velocity: float, dt: float) -> None:
        max_step = abs(config.FEEDER_SPEED) * dt / max(config.FEEDER_RAMP_SECONDS, 1.0e-6)
        if self._commanded_velocity < target_velocity:
            self._commanded_velocity = min(self._commanded_velocity + max_step, target_velocity)
        elif self._commanded_velocity > target_velocity:
            self._commanded_velocity = max(self._commanded_velocity - max_step, target_velocity)
        self._velocity.set(self._commanded_velocity)

    def beam_reading(self) -> tuple[bool, float | None]:
        """(has_data, distance to the nearest thing in the window). has_data is False before Play,
        when the sensor produces nothing at all -- distinct from a confirmed-clear beam."""
        interface = _lightbeam_interface()
        if interface is None:
            return False, None
        hits = interface.get_beam_hit_data(self.sensor_prim_path)
        if hits is None or len(hits) == 0:
            return False, None
        depths = interface.get_linear_depth_data(self.sensor_prim_path)
        hit_depths = [float(depth) for depth, hit in zip(depths, hits) if hit] if depths is not None else []
        return True, (min(hit_depths) if hit_depths else None)

    def beam_occupied(self) -> bool | None:
        """Anything in the beam's window at all. None while the sensor has no data."""
        has_data, depth = self.beam_reading()
        return (depth is not None) if has_data else None

    def _start_feeding(self) -> None:
        self._riding = belt_queue(self.base_part_prim_path)
        if not self._riding:
            print(
                f"[mefron] feeder {self._name}: nothing left on the belt -- running to the "
                f"{config.FEEDER_MAX_TRAVEL}m cut-off. Add copies in the GUI (see CLAUDE.md).",
                flush=True,
            )
        else:
            print(
                f"[mefron] feeder {self._name}: advancing {self._station_y - _origin_y(self._riding[0]):.3f}m, "
                f"{len(self._riding)} still on the belt."
                if self._station_y is not None
                else f"[mefron] feeder {self._name}: advancing, {len(self._riding)} still on the belt.",
                flush=True,
            )
        self.state = "feeding"
        self._travel = 0.0

    def _overshot_station(self) -> bool:
        """Fail-safe for a leading edge the curtain's rays miss (a notch at the ray line). Reads the
        part's LIVE pose, not dead-reckoned travel -- parts slip on the belt by ~20%."""
        if self._station_y is None or not self._riding:
            return False
        return _origin_y(self._riding[0]) >= self._station_y

    def step(self, dt: float, advance_requested: bool = False) -> None:
        has_data, depth = self.beam_reading()
        if not has_data:
            return
        occupied = depth is not None
        # A trip alone would stop the belt a whole beam-depth short of the authored pick spot, which
        # on the far belts is past the arm's reach. Depth (clamped at minRange) says "face is there".
        at_station = occupied and depth <= self._arrival_depth

        if self.state == "occupied":
            if not occupied:
                self.state = "waiting"
                self._waited = 0.0
        elif self.state == "waiting":
            if occupied:
                # A part put back, or the tool still standing in the station -- either way the belt
                # must not run. Deliberate: the beam sees the gripper too.
                self.state = "occupied"
            else:
                self._waited += dt
                if advance_requested or self._waited >= config.FEEDER_ADVANCE_DELAY_SECONDS:
                    self._start_feeding()
        elif self.state == "feeding":
            self._travel += abs(self._commanded_velocity) * self._travel_scale * dt
            if at_station:
                print(f"[mefron] feeder {self._name}: part at the pick spot ({depth:.4f}m), stopping.", flush=True)
                self.state = "stopping"
            elif self._overshot_station():
                # Loud, not silent: the photo-eye should have caught this, and which part it missed
                # is the thing to look at.
                print(
                    f"[mefron] feeder {self._name}: WARNING {self._riding[0]} reached the pick spot "
                    "without the photo-eye confirming -- stopping on its pose instead.",
                    flush=True,
                )
                self.state = "stopping"
            elif self._travel >= config.FEEDER_MAX_TRAVEL:
                print(f"[mefron] feeder {self._name}: ran {self._travel:.2f}m with no part -- stopped.", flush=True)
                self.state = "empty"
        elif self.state == "stopping":
            if self._commanded_velocity == 0.0:
                self.state = "occupied"
        elif self.state == "empty":
            if occupied:
                self.state = "occupied"

        if self.state in ("feeding", "stopping"):
            # Every frame, not just at the start: a part that stalls against the one ahead falls
            # asleep again, and only a wake-up gets it moving on the belt. See _wake_bodies().
            _wake_bodies(self._riding)
        self._ramp_toward(config.FEEDER_SPEED if self.state == "feeding" else 0.0, dt)


class FeederControl:
    """Steps every PartFeeder off timeline (sim) time, and owns config.FEEDER_ADVANCE_KEY. Duck-typed
    into run_teleop_loop() the same way ConveyorControl is -- only reset()/step() are called."""

    def __init__(self, feeders: list[PartFeeder]) -> None:
        self.feeders = feeders
        self._advance_requested = False
        self._last_time = None

    def request_advance(self) -> None:
        self._advance_requested = True

    def reset(self) -> None:
        self._advance_requested = False
        self._last_time = None
        clear_instance_state()
        for feeder in self.feeders:
            feeder.reset()

    def step(self) -> None:
        import omni.timeline

        # Timeline time, not time.time(): /app/player/useFixedTimeStepping decouples the two, and a
        # cuRobo plan stalls the wall clock for hundreds of ms mid-frame.
        now = float(omni.timeline.get_timeline_interface().get_current_time())
        if self._last_time is None:
            self._last_time = now
            return
        dt = now - self._last_time
        self._last_time = now
        if dt <= 0.0:
            return

        advance_requested = self._advance_requested
        self._advance_requested = False
        if advance_requested and not any(feeder.state == "waiting" for feeder in self.feeders):
            print(
                f"[mefron] {config.FEEDER_ADVANCE_KEY}: no feeder is waiting to advance -- it only "
                "skips the delay after a part has been picked, so a parked part can't be pushed off.",
                flush=True,
            )
        for feeder in self.feeders:
            feeder.step(dt, advance_requested=advance_requested)


def build_feeder_control(key: str = config.FEEDER_ADVANCE_KEY) -> FeederControl:
    """Builds the feeders setup_part_feeders() authored and subscribes the skip-the-delay key."""
    import carb.input
    import omni.appwindow

    control = FeederControl([PartFeeder(**feeder) for feeder in setup_part_feeders()])
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()
    advance_input = getattr(carb.input.KeyboardInput, key)

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input == advance_input:
            control.request_advance()
        return True

    # Kept alive on the control object so the subscription isn't garbage-collected.
    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control
