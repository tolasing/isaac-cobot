"""Loads cuRobo's bundled Franka Panda, a draggable teleop target, and a
warmed-up cuRobo MotionGen into assets/mefron/factory floor/mefron.usd -- a
separate, already hand-authored stage (its own factory backdrop, two
packing tables, and a scanner-assembly CAD mockup), distinct from this
repo's main table_layout.yaml-driven scene.

Standalone: does not import build_scene.py. The only other repo script
reused here is import_cr5.py, a generic URDF-import utility already
designed to be used as a library (see its own docstring). Everything else
below (cuRobo obstacle scan/MotionGen setup, teleop target, the
drag-follow loop) is implemented directly in this file rather than reused
from build_scene.py, because build_scene.py's own versions carry
assumptions from the main scene's stage (an anonymous, in-memory
SimulationApp stage, never opened from an existing file) that don't hold
for mefron.usd (opened via omni.usd.get_context().open_stage() -- a real,
file-backed stage). Three real, confirmed-live problems this caused, and
why each is fixed the way it is:

  - build_scene.py's run_teleop_loop() hardcodes a check for /physicsScene
    (lowercase) before creating one -- it has no way to know about a
    differently-named, pre-existing scene on a stage it didn't build
    itself (mefron.usd already has its own /PhysicsScene, capitalized,
    authored by hand in the GUI). Having both active at once breaks the
    robot's PhysX articulation view entirely: isaacsim.core.prims logs
    "Physics Simulation View is not created yet" on every frame, and
    get_joints_state() never returns non-None, so the robot never
    responds to the target. Fixed here by owning the physics-scene check
    directly (run_teleop_loop() below): check for a valid scene at either
    the lowercase or capitalized path and reuse whichever exists, only
    defining a new one if genuinely neither does -- never two at once.
  - build_scene.py's build_teleop_target() uses the CopyPrim command to
    duplicate the end-effector's visuals prim -- confirmed live that
    CopyPrim's shallow, spec-level copy produces a prim with a completely
    empty bounding box here (checked via UsdGeom.BBoxCache), unlike the
    main scene where the same pattern works. Root cause, also confirmed
    live: Isaac Sim's URDF importer detects that mefron.usd is a real,
    file-backed stage (unlike the main scene's anonymous in-memory one --
    see the importer's own "Creating Asset in an in-memory stage, will not
    create layered structure" log line, which only fires for the
    anonymous case) and organizes the imported robot into a separate,
    disk-persisted "configuration/mefron_*.usd" multi-layer structure --
    Kit's own designed behavior for file-backed stages, not overridable by
    calling stage.SetEditTarget() beforehand (confirmed live: the importer
    resets the edit target itself regardless of what it was set to).
    CopyPrim's raw spec copy can't correctly re-resolve a same-layer
    reference across that more complex layer stack once relocated to a
    new prim path. Fixed in build_teleop_target() below by using an
    internal USD reference instead
    (prim.GetReferences().AddInternalReference()) -- a live pointer at the
    already-*composed* result, not a copy of raw specs, so it renders
    correctly no matter how many layers underlie the source. Confirmed
    live: non-empty bbox with real, plausible extents for a gripper mesh.
  - Calling timeline.play() before cuRobo's motion_gen warmup (which
    blocks the main thread for ~30s calling no simulation_app.update() of
    its own) leaves physics "playing" across a long unpumped real-time
    gap, which corrupts PhysX's tensor simulationView by the time the drag
    loop's own SingleArticulation gets constructed -- confirmed live,
    crashes every time with `AttributeError: 'NoneType' object has no
    attribute 'link_names'`. This script never calls timeline.play() at
    all -- matching build_scene.py's own convention, a human clicks Play
    in the GUI, and run_teleop_loop() below already waits for
    is_playing() itself and prints a reminder until it does.

One more side effect worth knowing, not a bug fixed here: every URDF
import into mefron.usd (whether from this script or manual GUI use)
writes real files to disk under
"assets/mefron/factory floor/configuration/" (a multi-MB mefron_base.usd
plus smaller physics/sensor/robot sublayers) -- confirmed via file mtimes
matching exactly when these sessions ran. This is Kit's own robot-import
behavior for file-backed stages (see above), not something this script
does deliberately. Not currently gitignored.

Mount pose: world (2.74097, -4.782, 0.7924), identity orientation -- directly
on top of /World/sektion_cabinet_instanceable, the SEKTION cabinet table that
replaced the original Pedestal_plates/Cube_05 mount plate (removed from
mefron.usd; see mefron_layout.yaml's cr5_mount comment for the full story).
This script originally targeted the old pedestal and was updated to match
once that mount changed -- MOUNT_POSITION here is the same value
mefron_layout.yaml's own cr5_mount.position uses.

Drive-strength/damping/URDF-path/motion_gen_robot_cfg constants below are
copied from table_layout.yaml's own cr5_mount.robot_override block (cuRobo's
own tuned Franka values, not a guess -- see that file's comment for
provenance).

Only creates its own SimulationApp when run standalone (same convention as
build_scene.py/import_cr5.py).

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/mefron.py [--headless]
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # SimulationApp defaults to the minimal isaacsim.exp.base.python.kit
    # experience, missing most UI extensions (Physics debug visualization,
    # full menu bar, etc.) -- load the same isaacsim.exp.full.kit
    # experience isaac-sim.sh itself uses instead, for interactive runs
    # only (headless verification doesn't need it).
    experience = "" if _headless else f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    simulation_app = SimulationApp({"headless": _headless}, experience=experience)

# The full experience's extra extensions make `import packaging` (and
# specifically `packaging.version`) resolve to an incomplete internal
# pip-bootstrap bundle instead of the real site-packages install --
# confirmed live this is NOT a sys.path ordering issue (that bundle's
# path never appears in sys.path at all) and NOT fixed by only
# pre-registering `sys.modules["packaging"]` (still resolved the broken
# `packaging.version` afterward regardless -- whatever finder is doing
# this intercepts the submodule import by name, ignoring the parent
# module's own __path__). cuRobo's own torch_utils does `from packaging
# import version`; fixed by pre-loading both `packaging` and
# `packaging.version` explicitly and setting the latter as a plain
# attribute on the former, so the `from X import Y` statement resolves
# via attribute lookup alone, without any further import-machinery
# involvement for either name.
import importlib.util  # noqa: E402

_REAL_PACKAGING_DIR = "/isaac-sim/kit/python/lib/python3.11/site-packages/packaging"


def _preload_real_submodule(pkg_module, name):
    spec = importlib.util.spec_from_file_location(f"packaging.{name}", f"{_REAL_PACKAGING_DIR}/{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[f"packaging.{name}"] = module
    spec.loader.exec_module(module)
    setattr(pkg_module, name, module)


if "packaging" not in sys.modules and os.path.isdir(_REAL_PACKAGING_DIR):
    _spec = importlib.util.spec_from_file_location(
        "packaging", f"{_REAL_PACKAGING_DIR}/__init__.py", submodule_search_locations=[_REAL_PACKAGING_DIR]
    )
    _packaging_module = importlib.util.module_from_spec(_spec)
    sys.modules["packaging"] = _packaging_module
    _spec.loader.exec_module(_packaging_module)
    _preload_real_submodule(_packaging_module, "version")

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Sdf, UsdPhysics  # noqa: E402

from import_cr5 import import_cr5  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
MEFRON_USD = REPO_ROOT / "assets" / "mefron" / "factory floor" / "mefron.usd"
# Isaac Sim's URDF importer writes a disk-persisted, multi-layer "Robot
# Description" structure here on every import into this file-backed stage
# (see this module's own docstring) -- confirmed live that a crash mid-import
# can leave these as truncated, defaultPrim-less stubs (492 bytes, just a bare
# USD crate header) that break every subsequent import the same way
# ("Unresolved payload prim path... <defaultPrim>" then a null-prim abort in
# the URDF importer). Cleared at the top of main(), before any import, so a
# past crash's leftovers can never silently break the next launch.
MEFRON_CONFIGURATION_DIR = MEFRON_USD.parent / "configuration"

ROBOT_PRIM_PATH = "/World/Franka"
TARGET_PRIM_PATH = "/World/target"
# The old Pedestal_plates/Cube_05 mount plate was removed from mefron.usd and
# replaced with a SEKTION table -- see mefron_layout.yaml's cr5_mount comment
# for the full story. This script wasn't updated when that happened; these two
# constants now match mefron_layout.yaml's own cr5_mount.position (no /Factory
# prefix here, unlike that file's mount_surface.prim_path -- that prefix is a
# build_scene_mefron.py-only artifact of referencing mefron.usd in under
# /World/Factory; this script opens the file directly, so main_holder,
# finger_print_scanner, etc. all live one level shallower, directly under /World).
MOUNT_PLATE_PRIM_PATH = "/World/sektion_cabinet_instanceable"
MOUNT_POSITION = [2.74097, -4.782, 0.7924]
MOUNT_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

FRANKA_URDF_RELATIVE_PATH = "robot/franka_description/franka_panda.urdf"
FRANKA_DRIVE_STRENGTH = 1047.19751
FRANKA_DRIVE_DAMPING = 52.35988
FRANKA_MOTION_GEN_ROBOT_CFG = "franka.yml"

# Nearby scene objects actually within the Franka's reach envelope -- not
# the whole /World/Factory backdrop (thousands of small meshes that add
# nothing but scan time, mirroring build_scene.py's own get_teleop_obstacles
# scoping rationale).
OBSTACLE_PRIM_PATHS = [
    "/World/packing_table",
    "/World/packing_table_01",
    "/World/finger_print_scanner",
    "/World/main_holder",
    "/World/screen",
    "/World/backpanel_support",
    MOUNT_PLATE_PRIM_PATH,
]

# Loop-timing constants, ported verbatim from build_scene.py's own
# run_teleop_loop (itself ported from
# examples/curobo_reference/motion_gen_reacher.py).
_TELEOP_INIT_FRAMES = 10
_TELEOP_SETTLE_FRAMES = 20
_TELEOP_OBSTACLE_RESCAN_INTERVAL = 1000
_POSE_DELTA_THRESHOLD = 1.0e-3
_STATIC_JOINT_VELOCITY_THRESHOLD = 0.5

# Frames to wait, once timeline.is_playing() first turns True, before
# constructing SingleArticulation at all -- confirmed live this crashes
# intermittently without it: "AttributeError: 'NoneType' object has no
# attribute 'create_articulation_view'" inside robot.initialize(), because
# is_playing() becoming True doesn't guarantee PhysX has actually stepped
# enough to have a real simulation view ready yet (a race, not deterministic
# -- worked on an earlier run, crashed on a later one with identical code).
# Matches this same file's already-established pattern of small settle-frame
# waits (_TELEOP_INIT_FRAMES/_TELEOP_SETTLE_FRAMES) for the same underlying
# class of "physics needs a few real steps first" issue documented repeatedly
# in this project's own CLAUDE.md.
_ROBOT_INIT_SETTLE_FRAMES = 5

# Re-times the already-planned trajectory to play out slower -- confirmed via
# curobo's own source (MotionGenPlanConfig.time_dilation_factor's docstring:
# "Slow down optimized trajectory by re-timing with a dilation factor..."):
# this is a pure post-process, UNIFORM re-time of an already collision-checked,
# successfully-planned trajectory -- it cannot affect planning success, and
# crucially it does NOT change the *relative* speed profile the optimizer
# already chose: a naturally-fast long-distance move and a naturally-slow
# short move both just get stretched by the same 1/0.3 ~ 3.3x factor, so a
# far target still ends up moving faster in absolute terms than a near one --
# just proportionally less fast than before dilation. Confirmed live this
# doesn't fix "far targets move fast enough to disturb a carried load" --
# use _TELEOP_VELOCITY_SCALE/_TELEOP_ACCELERATION_SCALE below for that
# instead, since those actually cap the velocity/acceleration LIMITS the
# optimizer plans within (a uniform ceiling regardless of distance), not
# just stretch whatever profile it already produced.
_TELEOP_TIME_DILATION_FACTOR = 0.3

# Caps the actual velocity/acceleration limits used during trajectory
# optimization (set at MotionGenConfig construction time, unlike
# time_dilation_factor above) -- this is what makes motion uniformly slow
# regardless of target distance, since the optimizer can no longer plan a
# fast cruise phase for long moves in the first place. cuRobo's own source
# treats this as a first-class, supported case (not just a risky override):
# scale <= 0.25 automatically swaps in a dedicated "finetune_trajopt_slow.yml"
# tuning file made specifically for slow trajectories, and automatically
# increases the trajectory's own time budget (maximum_trajectory_dt) to
# compensate -- confirmed via direct source read of
# MotionGenConfig.load_from_robot_config(). 0.2 stays safely below that 0.25
# threshold (to get the dedicated slow-trajectory tuning) and above the 0.1
# floor below which cuRobo requires you to also set maximum_trajectory_dt
# manually. Scaling both velocity and acceleration together (not just one)
# matters for not disturbing a carried/grasped load specifically: a low
# acceleration limit avoids sudden starts/stops/direction changes (the main
# thing that shakes a friction-held object loose), not just a low top speed.
_TELEOP_VELOCITY_SCALE = 0.2
_TELEOP_ACCELERATION_SCALE = 0.2

# Grasp-physics constants, ported from build_scene_mefron.py -- confirmed there
# (not assumed here) via franka_panda.urdf's actual joint limits
# (panda_finger_joint1/2, prismatic, lower=0.0 upper=0.04) and a live
# inspection of mefron.usd finding zero PhysxMaterialAPI authored anywhere in
# the file, on either the scanner parts or the Franka's fingertips -- both
# sides of every grasp contact were relying on PhysX's un-overridden engine
# default friction, and the finger joints' effective drive stiffness/damping
# (625.0/10.0, confirmed via UsdPhysics.DriveAPI) left most of their 20N
# maxForce budget unused. See build_scene_mefron.py's apply_gripper_friction()/
# stiffen_gripper_drive() for the full original investigation; this script has
# no cfg dict (unlike build_scene_mefron.py), so high_friction_prim_paths
# becomes a plain module constant instead of a config entry.
GRIPPER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]
GRIPPER_OPEN_POSITION = 0.04
GRIPPER_CLOSED_POSITION = 0.0
# The commanded gripper position is ramped toward GRIPPER_OPEN_POSITION/
# GRIPPER_CLOSED_POSITION at this rate (meters/second) instead of being
# commanded as an instant step -- with the high drive stiffness/damping
# tuned in for grip strength (see stiffen_gripper_drive()'s own history),
# an instant open<->closed position-target jump makes the fingers snap
# shut almost immediately, since a stiff position drive tracks a sudden
# setpoint change aggressively. Ramping the SETPOINT itself gradually keeps
# that same strong holding force once closed, without the fast snap while
# getting there. 0.02 m/s means the full 0.04m open<->closed travel takes
# about 2 seconds; tune this one constant for a faster/slower feel.
GRIPPER_CLOSE_SPEED = 0.02
GRIPPER_FRICTION_MATERIAL_PATH = "/World/GripperFrictionMaterial"
GRIPPER_STATIC_FRICTION = 0.9
GRIPPER_DYNAMIC_FRICTION = 0.8
GRIPPER_FINGER_LINK_NAMES = ["panda_leftfinger", "panda_rightfinger"]
GRIPPER_DRIVE_STIFFNESS = 10000.0
GRIPPER_DRIVE_DAMPING = 200.0
HIGH_FRICTION_PRIM_PATHS = ["/World/finger_print_scanner"]

# T_S_G: the Franka's ee_link pose expressed in finger_print_scanner's own
# local frame, at a known-good grasp. Derived live via
# compute_relative_pose(scanner_pose, target_pose) against a manually-jogged
# /World/target -- confirmed via direct source inspection that
# isaacsim.core.utils.numpy.rotations.rot_matrices_to_quats returns
# scalar-first (wxyz), so this is unambiguous, not a hand-Euler-conversion
# guess. Re-derived from a fresh manual jog (superseding an earlier value
# that had a ~180-degree relative rotation about the scanner's own Z axis --
# this one's near-identity orientation, w~1 and x/y/z~0, reflects wherever
# the gripper was actually jogged to this time, not a fixed convention).
GRASP_OFFSET_POSITION = [0.00027002069774515104, -0.021693730387954874, -0.1271989186209571]
GRASP_OFFSET_ORIENTATION_WXYZ = [
    -2.1523912431273915e-05,
    -8.089888886539503e-06,
    5.762411090611313e-06,
    0.9999999997190347,
]

# T_H_S: finger_print_scanner's pose expressed in main_holder's own local
# frame, at the correctly assembled position. Derived live the same way, by
# temporarily nesting finger_print_scanner under main_holder in mefron.usd
# directly (only possible because this script opens that file directly,
# unlike build_scene_mefron.py's referenced session, where reparenting hits
# an "ancestral prim" restriction) and running compute_relative_pose() on
# both prims' resulting world poses.
ASSEMBLY_RELATIONSHIPS = {
    "finger_print_scanner_on_main_holder": {
        "part_prim_path": "/World/finger_print_scanner",
        "mount_prim_path": "/World/main_holder",
        "local_position": [-0.05765001316747483, 0.02068996147910942, 0.01500000425999065],
        "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
}


def compute_grasp_approach_pose(part_prim_path: str = HIGH_FRICTION_PRIM_PATHS[0]):
    """Returns the world pose /World/target should be set to in order to grasp
    the named part (finger_print_scanner by default) at the fixed relative
    offset GRASP_OFFSET_*, wherever that part currently sits on the table.
    Table-position-independent by construction -- recomputes from the part's
    live pose every call, not a value baked in once."""
    part_trans, part_quat = SingleXFormPrim(prim_path=part_prim_path).get_world_pose()
    return compute_dependent_world_pose(part_trans, part_quat, GRASP_OFFSET_POSITION, GRASP_OFFSET_ORIENTATION_WXYZ)


def compute_assembly_grasp_target(relationship_name: str = "finger_print_scanner_on_main_holder"):
    """Returns the world pose /World/target should be set to in order to place
    the already-grasped part at its correctly assembled position on its mount,
    wherever the mount currently sits. Composes two independently-verified
    transforms: the mount's live pose + ASSEMBLY_RELATIONSHIPS (mount -> part
    at the assembled position), then that result + GRASP_OFFSET_* (part ->
    gripper) -- same compute_dependent_world_pose() primitive used both times,
    not two different mechanisms."""
    relationship = ASSEMBLY_RELATIONSHIPS[relationship_name]
    mount_trans, mount_quat = SingleXFormPrim(prim_path=relationship["mount_prim_path"]).get_world_pose()
    part_target_trans, part_target_quat = compute_dependent_world_pose(
        mount_trans, mount_quat, relationship["local_position"], relationship["local_orientation_wxyz"]
    )
    return compute_dependent_world_pose(
        part_target_trans, part_target_quat, GRASP_OFFSET_POSITION, GRASP_OFFSET_ORIENTATION_WXYZ
    )


def mount_franka() -> None:
    from curobo.util_file import get_assets_path, join_path

    urdf_path = Path(join_path(get_assets_path(), FRANKA_URDF_RELATIVE_PATH))
    import_cr5(
        urdf_path=urdf_path,
        prim_path=ROBOT_PRIM_PATH,
        default_drive_strength=FRANKA_DRIVE_STRENGTH,
        default_position_drive_damping=FRANKA_DRIVE_DAMPING,
    )
    xform = SingleXFormPrim(prim_path=ROBOT_PRIM_PATH)
    xform.set_world_pose(
        position=np.array(MOUNT_POSITION),
        orientation=np.array(MOUNT_ORIENTATION_WXYZ),
    )


def apply_gripper_friction() -> None:
    """Authors one high-friction physics material and binds it to both the
    Franka's fingertip links and HIGH_FRICTION_PRIM_PATHS. Runtime-only, not
    persisted: this script never calls stage.Save(), so this material is
    re-authored fresh every run. Ported from build_scene_mefron.py's function
    of the same name -- see that file's own docstring for the original
    investigation (confirmed there that neither side of the grasp contact had
    any physics material authored anywhere in mefron.usd)."""
    from omni.physx.scripts import utils as physx_utils
    from omni.physx.scripts.physicsUtils import add_physics_material_to_prim

    stage = omni.usd.get_context().get_stage()
    physx_utils.addRigidBodyMaterial(
        stage,
        GRIPPER_FRICTION_MATERIAL_PATH,
        staticFriction=GRIPPER_STATIC_FRICTION,
        dynamicFriction=GRIPPER_DYNAMIC_FRICTION,
        restitution=0.0,
    )

    target_paths = [f"{ROBOT_PRIM_PATH}/{name}" for name in GRIPPER_FINGER_LINK_NAMES]
    target_paths += HIGH_FRICTION_PRIM_PATHS
    for prim_path in target_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            print(f"[mefron] WARNING: {prim_path} not found -- skipping friction bind.", flush=True)
            continue
        add_physics_material_to_prim(stage, prim, GRIPPER_FRICTION_MATERIAL_PATH)


def stiffen_gripper_drive() -> None:
    """Raises the finger joints' own position-drive stiffness/damping well
    above the whole-robot import-time default. Ported from
    build_scene_mefron.py's function of the same name -- see that file's own
    docstring for the original investigation (confirmed there that the finger
    joints land at stiffness=625.0/damping=10.0, not the configured
    FRANKA_DRIVE_STRENGTH/FRANKA_DRIVE_DAMPING, leaving most of their 20N
    maxForce budget unused)."""
    stage = omni.usd.get_context().get_stage()
    for joint_name in GRIPPER_JOINT_NAMES:
        joint_prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/joints/{joint_name}")
        if not joint_prim.IsValid():
            print(f"[mefron] WARNING: {joint_prim.GetPath()} not found -- skipping stiffen.", flush=True)
            continue
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "linear")
        drive.CreateStiffnessAttr().Set(GRIPPER_DRIVE_STIFFNESS)
        drive.CreateDampingAttr().Set(GRIPPER_DRIVE_DAMPING)


class GripperKeyboardControl:
    """Open/closed request for the Franka's gripper, read once per teleop
    frame. Real instances are driven by carb.input keyboard events (see
    build_gripper_keyboard_control()). Ported from build_scene_mefron.py's
    class of the same name.

    Also carries two one-shot snap-to-pose requests (G: grasp approach, P:
    assembly placement) -- these aren't held state like `closed` above, they're
    consumed exactly once by run_teleop_loop() via the request_*/consume_*
    pair below, so a key press doesn't keep re-snapping the target every
    frame for as long as it happens to still be pressed."""

    def __init__(self) -> None:
        self.closed = False
        self._grasp_approach_requested = False
        self._assembly_target_requested = False

    def set_closed(self, closed: bool) -> None:
        self.closed = closed

    def request_grasp_approach(self) -> None:
        self._grasp_approach_requested = True

    def consume_grasp_approach_request(self) -> bool:
        requested = self._grasp_approach_requested
        self._grasp_approach_requested = False
        return requested

    def request_assembly_target(self) -> None:
        self._assembly_target_requested = True

    def consume_assembly_target_request(self) -> bool:
        requested = self._assembly_target_requested
        self._assembly_target_requested = False
        return requested


def build_gripper_keyboard_control() -> GripperKeyboardControl:
    """Subscribes to real keyboard events: C closes the gripper, O opens it,
    G snaps /World/target to the grasp-approach pose (wherever
    finger_print_scanner currently sits), P snaps it to the assembly-placement
    pose (wherever main_holder currently sits). Ported from
    build_scene_mefron.py's function of the same name -- see that file's own
    docstring for the carb.input/omni.appwindow API confirmation against this
    install's own stubs."""
    import carb.input
    import omni.appwindow

    control = GripperKeyboardControl()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == carb.input.KeyboardInput.C:
                control.set_closed(True)
            elif event.input == carb.input.KeyboardInput.O:
                control.set_closed(False)
            elif event.input == carb.input.KeyboardInput.G:
                control.request_grasp_approach()
            elif event.input == carb.input.KeyboardInput.P:
                control.request_assembly_target()
        return True

    # Kept alive on the control object itself -- nothing else holds a
    # reference to keyboard/input_iface/the subscription id otherwise, and an
    # unreferenced subscription is liable to be garbage-collected away.
    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control


def get_obstacles():
    from curobo.util.usd_helper import UsdHelper

    usd_help = UsdHelper()
    usd_help.load_stage(omni.usd.get_context().get_stage())
    return usd_help.get_obstacles_from_stage(
        only_paths=list(OBSTACLE_PRIM_PATHS),
        reference_prim_path=ROBOT_PRIM_PATH,
        ignore_substring=[ROBOT_PRIM_PATH, TARGET_PRIM_PATH, "/curobo"],
    ).get_collision_check_world()


def setup_motion_gen():
    from curobo.types.base import TensorDeviceType
    from curobo.util_file import get_robot_configs_path, join_path, load_yaml
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

    robot_cfg = load_yaml(join_path(get_robot_configs_path(), FRANKA_MOTION_GEN_ROBOT_CFG))["robot_cfg"]
    # A real, populated world must be passed at construction time -- an
    # empty/absent one leaves motion_gen.world_coll_checker as None
    # (update_world() later crashes) and separately fails warmup() itself
    # ("Primitive Collision has no obstacles" -- confirmed in this repo's
    # own main-scene setup_curobo_motion_gen(), same underlying cuRobo
    # behavior).
    world_cfg = get_obstacles()
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        {"robot_cfg": robot_cfg},
        world_cfg,
        tensor_args=TensorDeviceType(),
        velocity_scale=_TELEOP_VELOCITY_SCALE,
        acceleration_scale=_TELEOP_ACCELERATION_SCALE,
    )
    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup()
    return motion_gen, robot_cfg


def build_teleop_target(robot_cfg: dict) -> SingleXFormPrim:
    """Creates a draggable target at the robot's own retract_config
    end-effector pose (guaranteed reachable -- it's a trivial identity
    plan), displaying an internally-referenced (not CopyPrim'd -- see this
    module's own docstring) live view of the real end-effector mesh.
    """
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose as CuroboPose

    ee_link = robot_cfg["kinematics"]["ee_link"]
    source_path = f"{ROBOT_PRIM_PATH}/{ee_link}/visuals"

    stage = omni.usd.get_context().get_stage()
    target_prim = stage.DefinePrim(TARGET_PRIM_PATH, "Xform")
    target_prim.GetReferences().AddInternalReference(Sdf.Path(source_path))

    tensor_args = TensorDeviceType()
    retract_config = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])
    q = tensor_args.to_device(retract_config).unsqueeze(0)
    local_ee_pose = motion_gen_kinematics_get_state(robot_cfg, q)

    robot_base_pose = CuroboPose(
        position=tensor_args.to_device(np.array(MOUNT_POSITION)),
        quaternion=tensor_args.to_device(np.array(MOUNT_ORIENTATION_WXYZ)),
    )
    world_ee_pose = robot_base_pose.multiply(local_ee_pose)

    xform = SingleXFormPrim(prim_path=TARGET_PRIM_PATH)
    xform.set_world_pose(
        position=world_ee_pose.position.squeeze(0).cpu().numpy(),
        orientation=world_ee_pose.quaternion.squeeze(0).cpu().numpy(),
    )
    return xform


def motion_gen_kinematics_get_state(robot_cfg, q):
    # Deferred import + tiny standalone CudaRobotModel, so build_teleop_target()
    # doesn't need a live MotionGen passed in just for forward kinematics.
    from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig

    kinematics_config = CudaRobotModelConfig.from_data_dict(robot_cfg["kinematics"])
    kinematics = CudaRobotModel(kinematics_config)
    return kinematics.get_state(q).ee_pose


def compute_relative_pose(reference_trans, reference_quat, dependent_trans, dependent_quat):
    """Given two live world poses, returns the dependent object's pose expressed in the
    reference object's own frame -- the DERIVATION direction (measure a relationship
    from two live poses), as opposed to compute_dependent_world_pose()'s CONSUMPTION
    direction (apply an already-known relationship to a live reference pose).

    Uses isaacsim.core.utils.numpy.rotations -- the same primitives
    isaacsim.robot_setup.grasp_editor's GraspSpec.compute_gripper_pose_from_rigid_body_pose
    uses internally (confirmed by reading grasp_importer.py directly), not hand Euler-angle
    math -- that's what got an earlier relative-pose derivation's rotation wrong this
    session (the position matched a script cross-check; the hand-converted rotation didn't).
    """
    from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats

    ref_rot, dep_rot = quats_to_rot_matrices(np.array([reference_quat, dependent_quat]))
    rel_rot = ref_rot.T @ dep_rot
    rel_trans = ref_rot.T @ (np.array(dependent_trans) - np.array(reference_trans))
    return rel_trans, rot_matrices_to_quats(np.array([rel_rot]))[0]


def compute_dependent_world_pose(reference_trans, reference_quat, relative_trans, relative_quat_wxyz):
    """Inverse of compute_relative_pose(): reference_* is a live world pose;
    relative_* is a fixed offset already expressed in the reference's own frame
    (e.g. a value derived once via compute_relative_pose(), or an isaac_grasp file's
    stored position/orientation). Returns the dependent object's resulting world pose.
    Same isaacsim.core.utils.numpy.rotations primitives as compute_relative_pose()."""
    from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats

    (ref_rot,) = quats_to_rot_matrices(np.array([reference_quat]))
    trans = ref_rot @ np.array(relative_trans) + np.array(reference_trans)
    rot = ref_rot @ quats_to_rot_matrices(np.array([relative_quat_wxyz]))[0]
    return trans, rot_matrices_to_quats(np.array([rot]))[0]


def run_teleop_loop(
    motion_gen,
    robot_cfg: dict,
    target: SingleXFormPrim,
    max_iterations: int | None = None,
    gripper_control: GripperKeyboardControl | None = None,
) -> None:
    """Drag `target` in the GUI viewport; the robot follows via cuRobo's
    MotionGen. Ported from build_scene.py's own run_teleop_loop() (see
    that file for the original's own detailed design notes -- the
    debounce/plan/apply logic below is unchanged) with these differences:

      - The physics-scene check reuses whichever of /physicsScene or
        /PhysicsScene already exists on mefron.usd, rather than
        unconditionally creating a new /physicsScene -- see this module's
        own docstring for why the original's version breaks here.
      - The original only ever builds its SingleArticulation once (gated
        by `idx_list is None`, checked just on the very first Play).
        Confirmed live: clicking Stop in the GUI tears down PhysX's
        simulation view entirely; on a later Play, the original code kept
        reusing the now-stale SingleArticulation, producing an endless
        "Physics Simulation View is not created yet" warning and a robot
        that never responds again for the rest of that process. This
        version tracks not-playing-to-playing transitions and rebuilds
        `robot` (and resets all per-session state) on every fresh Play,
        not just the first one.
      - Trajectory playback is gated on real elapsed time (result.interpolation_dt)
        instead of one waypoint per render frame -- ported from
        build_scene_mefron.py's identical fix. Confirmed there live at ~119
        FPS against a plan generated for 50Hz (interpolation_dt=0.02s):
        one-waypoint-per-frame played the whole trajectory back at ~2.4x its
        intended speed and cut its final deceleration-to-zero-velocity ramp
        short, leaving real residual velocity for the position-hold drive to
        absorb once cmd_plan ran out -- the actual cause of both too-fast
        motion and an arrival oscillation, not a stiffness/damping problem.
        This script's own render rate was never separately confirmed, but the
        mechanism is identical regardless of the exact FPS -- one waypoint per
        frame is only correct by coincidence if FPS happens to equal 1/interpolation_dt.
      - `gripper_control`: if given, the Franka's two finger joints are driven
        directly to GRIPPER_OPEN_POSITION/GRIPPER_CLOSED_POSITION every playing
        frame based on `gripper_control.closed`, independent of cmd_plan/cuRobo
        -- ported from build_scene_mefron.py's identical block; see
        GRIPPER_JOINT_NAMES' comment above for why fingers are locked out of
        cuRobo's own IK/trajopt (franka.yml's lock_joints).
      - plan_config now sets time_dilation_factor=_TELEOP_TIME_DILATION_FACTOR,
        not cuRobo's own default speed -- see that constant's own comment for
        why this (not velocity_scale/acceleration_scale) is the correct knob
        for "slower, more deliberate" motion.
      - `gripper_control` also gates two one-shot snap-to-pose requests (G/P
        keys, see GripperKeyboardControl): G snaps `target` to
        compute_grasp_approach_pose() (the gripper pose that grasps
        finger_print_scanner wherever it currently sits), P snaps it to
        compute_assembly_grasp_target() (the gripper pose that places an
        already-grasped scanner correctly onto main_holder wherever it
        currently sits). New in this file, not ported from
        build_scene_mefron.py -- see ASSEMBLY_RELATIONSHIPS/
        GRASP_OFFSET_POSITION's own comments for how T_H_S/T_S_G were
        derived.
    """
    import time

    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.state import JointState
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    tensor_args = TensorDeviceType()
    plan_config = MotionGenPlanConfig(time_dilation_factor=_TELEOP_TIME_DILATION_FACTOR)
    timeline = omni.timeline.get_timeline_interface()

    robot_base_pose = Pose(
        position=tensor_args.to_device(np.array(MOUNT_POSITION)),
        quaternion=tensor_args.to_device(np.array(MOUNT_ORIENTATION_WXYZ)),
    )

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])

    robot = None
    idx_list = None
    gripper_idx_list = None
    articulation_controller = None

    past_pose = None
    past_orientation = None
    target_pose = None
    target_orientation = None
    cmd_plan = None
    cmd_idx = 0
    # Real elapsed time (not frame count) since the last waypoint was applied,
    # and the plan's own intended per-waypoint duration -- see run_teleop_loop's
    # own module comment on gripper_control/trajectory pacing above.
    last_cmd_time = None
    interpolation_dt = 0.02
    obstacles = None
    step_index = 0
    not_playing_frames = 0
    was_playing = False
    # Ramped gripper setpoint state -- see GRIPPER_CLOSE_SPEED's own comment
    # for why the commanded position moves gradually instead of jumping
    # straight to GRIPPER_OPEN_POSITION/GRIPPER_CLOSED_POSITION.
    gripper_setpoint = None
    last_gripper_time = None

    while simulation_app.is_running():
        simulation_app.update()

        if not timeline.is_playing():
            was_playing = False
            not_playing_frames += 1
            if not_playing_frames % 100 == 0:
                print("[mefron] Click Play to start cuRobo teleop.", flush=True)
            continue

        if not was_playing:
            # Fresh Play (first ever, or after a Stop) -- rebuild
            # everything that was bound to the previous physics
            # view/step count.
            idx_list = None
            gripper_idx_list = None
            articulation_controller = None
            past_pose = None
            past_orientation = None
            target_pose = None
            target_orientation = None
            cmd_plan = None
            cmd_idx = 0
            last_cmd_time = None
            obstacles = None
            step_index = 0
            gripper_setpoint = None
            last_gripper_time = None
            was_playing = True

        step_index += 1
        if max_iterations is not None and step_index > max_iterations:
            return

        if idx_list is None:
            if step_index < _ROBOT_INIT_SETTLE_FRAMES:
                continue
            robot = SingleArticulation(prim_path=ROBOT_PRIM_PATH, name="mefron_teleop_robot")
            robot.initialize()
            idx_list = [robot.get_dof_index(x) for x in j_names]
            gripper_idx_list = [robot.get_dof_index(x) for x in GRIPPER_JOINT_NAMES]
            articulation_controller = robot.get_articulation_controller()

        if step_index < _TELEOP_INIT_FRAMES:
            robot.set_joint_positions(default_config, idx_list)
            continue
        if step_index < _TELEOP_SETTLE_FRAMES:
            continue

        if obstacles is None or step_index % _TELEOP_OBSTACLE_RESCAN_INTERVAL == 0:
            obstacles = get_obstacles()
            motion_gen.update_world(obstacles)

        cube_position, cube_orientation = target.get_world_pose()
        if past_pose is None:
            past_pose = cube_position
        if target_pose is None:
            target_pose = cube_position
        if target_orientation is None:
            target_orientation = cube_orientation
        if past_orientation is None:
            past_orientation = cube_orientation

        # One-shot snap requests (G/P keys) -- consumed at most once each,
        # re-armed only by another key press, not held state like
        # gripper_control.closed above. Must run AFTER the past_pose/
        # target_pose bootstrap above, not before: bootstrapping is only a
        # no-op once those are no longer None, but on the very first eligible
        # frame of a call where a request was already pending (e.g. armed by
        # the caller before run_teleop_loop() even started), doing this
        # before the bootstrap made cube_position already reflect the
        # post-snap pose, so target_pose got seeded from that same value and
        # the debounce distance was 0 forever after -- plan_single was never
        # called even though the snap itself worked. Fixed by bootstrapping
        # from the true pre-snap pose first, then applying the snap and
        # updating cube_position/cube_orientation here so the rest of this
        # frame's logic (the debounce check below, and the past_pose/
        # past_orientation update at this loop iteration's end) sees the
        # fresh post-snap pose, not the stale pre-snap one -- table-position-
        # independent, since both compute_* functions read the part/mount's
        # *live* world pose every call rather than using a value baked in
        # once.
        if gripper_control is not None:
            if gripper_control.consume_grasp_approach_request():
                cube_position, cube_orientation = compute_grasp_approach_pose()
                target.set_world_pose(position=cube_position, orientation=cube_orientation)
            elif gripper_control.consume_assembly_target_request():
                cube_position, cube_orientation = compute_assembly_grasp_target()
                target.set_world_pose(position=cube_position, orientation=cube_orientation)

        sim_js = robot.get_joints_state()
        if sim_js is None:
            continue
        sim_js_names = robot.dof_names
        cu_js = JointState(
            position=tensor_args.to_device(sim_js.positions),
            velocity=tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=sim_js_names,
        )
        cu_js = cu_js.get_ordered_joint_state(motion_gen.kinematics.joint_names)

        robot_static = bool(np.max(np.abs(sim_js.velocities)) < _STATIC_JOINT_VELOCITY_THRESHOLD)

        if (
            (
                np.linalg.norm(cube_position - target_pose) > _POSE_DELTA_THRESHOLD
                or np.linalg.norm(cube_orientation - target_orientation) > _POSE_DELTA_THRESHOLD
            )
            and np.linalg.norm(past_pose - cube_position) == 0.0
            and np.linalg.norm(past_orientation - cube_orientation) == 0.0
            and robot_static
            and cmd_plan is None
        ):
            world_target_pose = Pose(
                position=tensor_args.to_device(cube_position),
                quaternion=tensor_args.to_device(cube_orientation),
            )
            ik_goal = robot_base_pose.compute_local_pose(world_target_pose)
            result = motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, plan_config)
            print(f"[mefron] teleop plan_single success={result.success.item()}", flush=True)
            if result.success.item():
                cmd_plan = motion_gen.get_full_js(result.get_interpolated_plan())
                cmd_plan = cmd_plan.get_ordered_joint_state(sim_js_names)
                cmd_idx = 0
                # result.interpolation_dt (not a motion_gen-level attribute --
                # MotionGen itself doesn't expose one; MotionGenResult does)
                # is this specific plan's own intended per-waypoint duration.
                interpolation_dt = result.interpolation_dt
                last_cmd_time = None
            target_pose = cube_position
            target_orientation = cube_orientation

        past_pose = cube_position
        past_orientation = cube_orientation

        if cmd_plan is not None:
            # Gate on real elapsed time, not frame count -- see run_teleop_loop's
            # own module comment on trajectory pacing above.
            now = time.time()
            if last_cmd_time is None or (now - last_cmd_time) >= interpolation_dt:
                cmd_state = cmd_plan[cmd_idx]
                art_action = ArticulationAction(
                    cmd_state.position.cpu().numpy(),
                    cmd_state.velocity.cpu().numpy(),
                    joint_indices=idx_list,
                )
                articulation_controller.apply_action(art_action)
                cmd_idx += 1
                last_cmd_time = now
                if cmd_idx >= len(cmd_plan.position):
                    cmd_idx = 0
                    cmd_plan = None

        # Independent of cmd_plan/cuRobo -- applied every frame (not just while
        # a plan is interpolating), so it always wins that frame's drive-target
        # write for the finger indices even when the block above just also
        # wrote to them (get_full_js() re-applies the lock_joints value on
        # every planned frame; see GRIPPER_JOINT_NAMES' comment above).
        if gripper_control is not None:
            gripper_target = GRIPPER_CLOSED_POSITION if gripper_control.closed else GRIPPER_OPEN_POSITION
            if gripper_setpoint is None:
                gripper_setpoint = gripper_target
            now = time.time()
            if last_gripper_time is not None:
                max_step = GRIPPER_CLOSE_SPEED * (now - last_gripper_time)
                if gripper_setpoint < gripper_target:
                    gripper_setpoint = min(gripper_setpoint + max_step, gripper_target)
                elif gripper_setpoint > gripper_target:
                    gripper_setpoint = max(gripper_setpoint - max_step, gripper_target)
            last_gripper_time = now
            gripper_action = ArticulationAction(
                np.array([gripper_setpoint, gripper_setpoint]),
                joint_indices=gripper_idx_list,
            )
            articulation_controller.apply_action(gripper_action)


def clear_stale_robot_configuration() -> None:
    """Deletes any pre-existing files under MEFRON_CONFIGURATION_DIR before the URDF
    importer writes fresh ones. See that constant's own comment for why this is needed:
    a crash mid-import can leave truncated stub files behind that break every subsequent
    import into this same file-backed stage, the same way, forever, until cleared."""
    if not MEFRON_CONFIGURATION_DIR.is_dir():
        return
    for stale_file in MEFRON_CONFIGURATION_DIR.glob("*.usd"):
        stale_file.unlink()
        print(f"[mefron] cleared stale robot-description file: {stale_file}", flush=True)


def main() -> None:
    # Confirmed via isaacsim.core.simulation_manager's actual source
    # (SimulationManager._warm_start/_create_simulation_view): pressing Play
    # only creates a real PhysX simulation view through a specific chain --
    # timeline PLAY event -> _warm_start() -> checked against this exact carb
    # setting -> if true, initialize_physics() -> dispatches PHYSICS_WARMUP
    # -> _create_simulation_view() actually sets the view SingleArticulation.
    # initialize() later depends on. If this setting is off (a real, user-
    # facing toggle in the Play button's own toolbar dropdown, alongside Play
    # Animations/Audio/Computegraph), timeline.is_playing() still correctly
    # returns True, but that whole chain never fires -- the simulation view
    # stays None forever regardless of Play-timing, explaining a crash that
    # no settle-frame delay or forced Stop could ever fix. Setting this
    # explicitly removes the dependency on a GUI toggle that's easy to have
    # off without realizing it's what's blocking everything.
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    # Must run BEFORE open_stage(), not after: mefron.usd's own saved file
    # apparently has a persisted, broken /panda prim (contradicting this
    # module's own docstring assumption that nothing here ever calls
    # stage.Save() -- confirmed live that opening the file alone, before any
    # of this script's own code runs, already emits "Unresolved reference
    # prim path" warnings for /panda against these exact files). open_stage()
    # resolving those broken references registers Sdf.Layer objects for
    # these paths in USD's own in-memory layer cache -- deleting the files
    # from disk afterward doesn't clear that cache, so a later CreateNew()
    # for the same identifier during mount_franka()'s import fails with "a
    # layer already exists", cascading into an invalid-stage fatal crash.
    # Confirmed live this exact ordering bug in an earlier version of this
    # function. Clearing the files before the stage ever opens means nothing
    # ever resolves against them in the first place, so no stale layer gets
    # cached at all.
    clear_stale_robot_configuration()

    omni.usd.get_context().open_stage(str(MEFRON_USD))
    # mefron.usd's own content resolves asynchronously, same reasoning as
    # build_scene.py's own post-build_factory() frame pump.
    for _ in range(120):
        simulation_app.update()

    mount_franka()
    apply_gripper_friction()
    stiffen_gripper_drive()

    stage = omni.usd.get_context().get_stage()
    for status_path in [ROBOT_PRIM_PATH, *OBSTACLE_PRIM_PATHS]:
        prim = stage.GetPrimAtPath(status_path)
        print(f"[mefron] {status_path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

    print("[mefron] warming up cuRobo motion_gen (viewport will look frozen/black until this finishes)...", flush=True)
    motion_gen, robot_cfg = setup_motion_gen()
    print("[mefron] curobo motion_gen: READY", flush=True)
    # motion_gen.warmup() above is a ~30s blocking call that makes no
    # simulation_app.update() calls of its own -- confirmed live that physics
    # "playing" across that entire unpumped gap corrupts PhysX's simulation
    # view by the time run_teleop_loop()'s SingleArticulation gets
    # constructed, regardless of *why* the timeline was already playing
    # (autoplay-on-open, state carried over from a previous run in the same
    # long-running session, or a premature click) -- confirmed live this
    # crashes even with zero manual interaction. Forcing a stop here,
    # unconditionally, resets to a known-clean state every time; the loop
    # already correctly rebuilds everything on a fresh Play after a Stop, so
    # this guarantees whatever Play happens after this point starts from that
    # same known-good path instead of possibly inheriting corrupted state.
    omni.timeline.get_timeline_interface().stop()

    target = build_teleop_target(robot_cfg)
    target_prim = stage.GetPrimAtPath(TARGET_PRIM_PATH)
    print(f"[mefron] {TARGET_PRIM_PATH}: {'OK' if target_prim.IsValid() else 'MISSING'}", flush=True)

    if _headless:
        simulation_app.close()
        return

    gripper_control = build_gripper_keyboard_control()
    print("[mefron] Gripper: press C to close, O to open.", flush=True)
    print("[mefron] click Play in the GUI to start teleop.", flush=True)
    run_teleop_loop(motion_gen, robot_cfg, target, gripper_control=gripper_control)
    simulation_app.close()


if __name__ == "__main__":
    main()
