"""Every constant mefron.py and mefron_lib use: paths, poses, tuning, and the derived
grasp/assembly/screw offsets. Pure data -- no omni/curobo imports, safe to import anywhere."""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MEFRON_USD = REPO_ROOT / "assets" / "mefron" / "factory floor" / "mefron.usd"
# Disk-persisted "Robot Description" dir the URDF importer writes on every import; see
# kit_bootstrap.clear_stale_robot_configuration().
MEFRON_CONFIGURATION_DIR = MEFRON_USD.parent / "configuration"

ROBOT_PRIM_PATH = "/World/FR5"
TARGET_PRIM_PATH = "/World/target"
# UR10 pedestal the arm mounts on; MOUNT_POSITION is this prim's own xformOp:translate
# (unconfirmed whether that's the top mounting flange).
MOUNT_PLATE_PRIM_PATH = "/World/ur10_mount"
MOUNT_POSITION = [2.625260866887235, -4.7019853821770115, 0.8093334035921127]
# 180 deg about Z, not the Franka's identity -- CONFIRMED LIVE 2026-08-13, the FR5 faced backwards
# out of the cell without it. Same correction the CR5 needed. See docs/fr5-migration.md.
MOUNT_ORIENTATION_WXYZ = [0.0, 0.0, 0.0, 1.0]

# Repo-local, unlike the Franka's cuRobo-bundled path this replaced. Provenance: robots/fr5/SOURCE.md.
FR5_URDF_PATH = REPO_ROOT / "robots" / "fr5" / "urdf" / "fairino5_v6.urdf"
FR5_BASE_LINK = "base_link"
# Terminates the chain -- the FR5 ships no tool0/flange link. Carries visual geometry, which
# motion.build_teleop_target() requires of ee_link.
FR5_EE_LINK = "wrist3_link"
# The ISO tool flange, +99.0mm along wrist3_link's own Z -- measured from the imported geometry
# (link origin sits 53mm short of any metal) and corroborated by the FR5's 922mm published reach.
FR5_TOOL_FLANGE_OFFSET = 0.0990
# cuRobo's real ee frame: added to fr5.xrdf via modifiers/add_frame AND authored as a live child
# Xform, since grasp/screw code reads the ee's world pose off an actual prim.
FR5_EE_FRAME_NAME = "tool_flange"
FR5_EE_FRAME_PRIM_PATH = f"{ROBOT_PRIM_PATH}/{FR5_EE_LINK}/{FR5_EE_FRAME_NAME}"
FR5_JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
# Everything past base_link, in chain order -- the links cuRobo collision-checks.
FR5_MOVING_LINK_NAMES = ["shoulder_link", "upperarm_link", "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link"]
# Radians. NOT all-zero: that is fully outstretched (wrist3 0.82m out, 0.05m up) AND singular --
# measured Jacobian cond = inf vs 8.2 here, tool pointing straight down. docs/fr5-migration.md.
FR5_HOME_JOINT_POSITIONS = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]

# Accent links, painted at runtime: the URDF colors every link the same light grey and carries no
# orange at all. See robot.apply_accent_color().
FR5_ACCENT_LINK_NAMES = ["shoulder_link", "wrist2_link"]
FR5_ACCENT_COLOR_RGB = [0.937, 0.400, 0.055]
# Authored under the arm's own Looks scope, so re-importing the arm disposes of it too.
FR5_ACCENT_MATERIAL_NAME = "FR5Accent"
# MEASURED IGNORED: the URDF importer applied neither of these (joints came out at stiffness 625,
# damping 0). Real damping now comes from the URDF's own <dynamics> -- robots/fr5/SOURCE.md.
FR5_DRIVE_STRENGTH = 1047.19751
FR5_DRIVE_DAMPING = 210.0
# cuRobo robot config, as a cuMotion XRDF exported from the Lula Robot Description Editor and
# converted at load time. Its default_joint_positions ARE FR5_HOME_JOINT_POSITIONS -- keep in sync.
FR5_XRDF_PATH = REPO_ROOT / "configs" / "curobo" / "fr5.xrdf"
# Mesh root the XRDF's URDF resolves its ../meshes/ against.
FR5_URDF_ASSET_ROOT = REPO_ROOT / "robots" / "fr5" / "urdf"

# Reach-envelope obstacles only, not the whole backdrop. Currently a DEBUG value -- normally
# main_holder_jig + tool_rack_gripper; see CLAUDE.md's open issues before restoring those.
OBSTACLE_PRIM_PATHS = [
    "/World/ConveyorBelt_A06_01",
]

# Loop-timing constants for teleop.run_teleop_loop().
_TELEOP_INIT_FRAMES = 10
_TELEOP_SETTLE_FRAMES = 20
_TELEOP_OBSTACLE_RESCAN_INTERVAL = 1000
_POSE_DELTA_THRESHOLD = 1.0e-3
_STATIC_JOINT_VELOCITY_THRESHOLD = 0.5

# World-frame Z height P holds while aligning X/Y/orientation before descending -- see CLAUDE.md's
# open issues (not yet relationship-relative).
ASSEMBLY_LIFT_HEIGHT = 1.21

# Frames to wait after is_playing() first turns True before constructing SingleArticulation --
# PhysX needs a few real steps before its simulation view is ready.
_ROBOT_INIT_SETTLE_FRAMES = 5

# Re-times the already-planned trajectory to play out slower; doesn't change the optimizer's
# relative speed profile or planning success.
_TELEOP_TIME_DILATION_FACTOR = 1.0

# Caps velocity/acceleration limits used during trajectory optimization. cuRobo treats scale <=
# 0.25 as a special case (swaps in finetune_trajopt_slow.yml); 0.2 stays under that threshold.
_TELEOP_VELOCITY_SCALE = 0.6
_TELEOP_ACCELERATION_SCALE = 0.1

# Grasp-physics constants for robot.apply_gripper_friction()/stiffen_gripper_drive().
GRIPPER_JOINT_NAMES = ["pgc140_finger1_joint", "pgc140_finger2_joint"]
# INVERTED vs the Franka: on the PGC-140 the joint measures inward travel, so 0.0 is OPEN (59mm
# apart) and 0.025 is CLOSED. Derived from the URDF's opposed joint yaws; see docs/fr5-migration.md.
# Only the DEFAULT widths before any grasp key is pressed -- each grasp key overrides them.
GRIPPER_OPEN_POSITION = 0.000
GRIPPER_CLOSED_POSITION = 0.025
# Rate (m/s) the commanded gripper position ramps toward open/closed, instead of stepping
# instantly -- avoids a snap shut under the high drive stiffness.
GRIPPER_CLOSE_SPEED = 0.02
GRIPPER_FRICTION_MATERIAL_PATH = "/World/GripperFrictionMaterial"
GRIPPER_STATIC_FRICTION = 1.5
GRIPPER_DYNAMIC_FRICTION = 1.5
GRIPPER_FINGER_LINK_NAMES = ["pgc140_finger1_link", "pgc140_finger2_link"]
GRIPPER_DRIVE_STIFFNESS = 10000.0
# The PGC-140 asset's own value. 200 was the Franka's; a 14g finger at stiffness 10000 needs the
# damping, and under-damped drives ring -- see the FR5's own import bug in docs/fr5-migration.md.
GRIPPER_DRIVE_DAMPING = 1000.0
# "force" (N/m) over the assets' baked mass-normalized "acceleration", which turned STIFFNESS
# into well under a newton on a few-gram finger. See docs/mefron-history.md.
GRIPPER_DRIVE_TYPE = "force"
HIGH_FRICTION_PRIM_PATHS = ["/World/finger_print_scanner"]

# Grasp Editor-exported poses, keyed by object name and wired to a key in
# keyboard.build_gripper_keyboard_control(). Finger widths are read from the yaml live.
# STALE: all four yamls are keyed to panda_hand/panda_finger_joint1 and must be re-exported
# against the PGC-140 in the Grasp Editor (step 3). See docs/fr5-migration.md.
GRASP_TARGETS = {
    "main_holder": {
        "key": "G",
        "yaml_path": REPO_ROOT / "assets" / "main_holder.yaml",
        "grasp_name": "grasp_0",
        "part_prim_path": "/World/main_holder",
    },
    "finger_print_scanner": {
        "key": "J",
        "yaml_path": REPO_ROOT / "assets" / "finger_print_scanner.yaml",
        "grasp_name": "grasp_0",
        "part_prim_path": "/World/finger_print_scanner",
    },
    "backpanel_support": {
        "key": "B",
        "yaml_path": REPO_ROOT / "assets" / "backpanel_support2.yaml",
        "grasp_name": "grasp_0",
        "part_prim_path": "/World/backpanel_support",
    },
    "main_holder_back_cover": {
        "key": "K",
        "yaml_path": REPO_ROOT / "assets" / "main_holder_back_cover.yaml",
        "grasp_name": "grasp_0",
        "part_prim_path": "/World/main_holder_back_cover",
    },
    # pcb_assembly's K retired 2026-07-22 for the suction cup instead (redundant-branch twisting
    # on approach, see docs/mefron-history.md); the freed key now drives the back cover above.
}

# T_H_S: each part's pose expressed in its MOUNT's own local frame at the correctly assembled
# position, derived via grasp.compute_relative_pose() -- see docs/grasp-and-assembly-offsets.md.
ASSEMBLY_RELATIONSHIPS = {
    # The one entry mounting onto the belt-driven jig rather than main_holder. Given as -24mm in the
    # jig's own mm-scale frame; the jig is flipped 180 deg about Y, so that is 24mm *up* in world.
    "main_holder_on_main_holder_jig": {
        "part_prim_path": "/World/main_holder",
        "mount_prim_path": "/World/main_holder_jig",
        "local_position": [0.0, 0.0, -0.024],
        # 180 deg about Z, not identity -- cancels the jig-vs-holder frame difference, so the holder
        # seats facing the way it parks on the table. Hand-specified, see docs.
        "local_orientation_wxyz": [0.0, 0.0, 0.0, 1.0],
    },
    "finger_print_scanner_on_main_holder": {
        "part_prim_path": "/World/finger_print_scanner",
        "mount_prim_path": "/World/main_holder",
        "local_position": [-0.05765, 0.02069, 0.01565],
        "local_orientation_wxyz": [0.0, 0.0, 0.0, 1.0],
    },
    "backpanel_support_on_main_holder": {
        "part_prim_path": "/World/backpanel_support",
        "mount_prim_path": "/World/main_holder",
        "local_position": [0.023463946069672652, -0.013916167562435, 0.001499950486007643],
        "local_orientation_wxyz": [
            1.146981958298904e-07,
            0.9999999999991531,
            -5.587935447688139e-08,
            1.2951986718679054e-06,
        ],
    },
    "screen_on_main_holder": {
        "part_prim_path": "/World/screen",
        "mount_prim_path": "/World/main_holder",
        "local_position": [0.02688002586364746, -0.012380123138427736, 0.01234102249145508],
        "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    },
    # Probed live while the cover sat assembled. z is negative because main_holder's frame is
    # flipped 180 deg about X, so this is 15mm *up* in world.
    "main_holder_back_cover_on_main_holder": {
        "part_prim_path": "/World/main_holder_back_cover",
        "mount_prim_path": "/World/main_holder",
        "local_position": [0.0, 0.0, -0.015],
        "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    },
    "pcb_assembly_on_backpanel_support": {
        "part_prim_path": "/World/PCB_Assembly_color_fixed",
        "mount_prim_path": "/World/backpanel_support",
        "local_position": [-0.0015799999237060547, -0.02138996124267578, 0.008999995231628418],
        "local_orientation_wxyz": [0.7063401483274144, 0.0, 0.0, 0.7078725837753616],
    },
    # An APPROACH target in screen's own live frame, not a mount pose -- mount_prim_path ==
    # part_prim_path on purpose. Derivation and the 2026-08-06 re-derive: docs/mefron-history.md.
    "suction_gripper_approach_on_screen": {
        "part_prim_path": "/World/screen",
        "mount_prim_path": "/World/screen",
        "local_position": [0.00028, -0.00024, -0.11558],
        "local_orientation_wxyz": [
            0.9999999973427276,
            2.989584746736537e-05,
            -4.272515434410521e-05,
            5.094452340128793e-05,
        ],
    },
    # Same derivation as suction_gripper_approach_on_screen above.
    "suction_gripper_approach_on_pcb_assembly": {
        "part_prim_path": "/World/PCB_Assembly_color_fixed",
        "mount_prim_path": "/World/PCB_Assembly_color_fixed",
        "local_position": [-0.0028148316864434492, 4.480405335696105e-06, 0.11491755932216695],
        "local_orientation_wxyz": [0.011293781372283615, 0.38247333692520136, 0.9238856699972284, 0.004676089968013461],
    },
}

# Where the O/L release weld SEATS each part; P still DRIVES to ASSEMBLY_RELATIONSHIPS' pose. A
# relationship with no entry welds at that pose instead. Provenance: docs/grasp-and-assembly-offsets.md.
ASSEMBLY_WELD_POSES = {
    "finger_print_scanner_on_main_holder": {
        "local_position": [-0.05765, 0.02069, 0.01875],
        "local_orientation_wxyz": [0.0, 0.0, 0.0, 1.0],
    },
    "backpanel_support_on_main_holder": {
        "local_position": [0.02346, -0.01392, 0.01375],
        "local_orientation_wxyz": [0.0, 1.0, 0.0, 0.0],
    },
    "screen_on_main_holder": {
        "local_position": [0.02688, -0.01238, 0.01965],
        "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    },
    # Mounts on backpanel_support, itself a welded part, so this was measured against wherever
    # THAT was placed -- the two stay a set.
    "pcb_assembly_on_backpanel_support": {
        "local_position": [-0.00158, -0.02139, 0.005],
        "local_orientation_wxyz": [0.7071067811865476, 0.0, 0.0, 0.7071067811865476],
    },
}

# --- Assembly weld (O/L release) ----------------------------------------------------------------

# Script-owned scope of anchors + joints, wiped every run by assembly.clear_assembly_welds().
ASSEMBLY_WELD_SCOPE_PRIM_PATH = "/World/assembly_welds"
# How close (metres) a part must be for a release to snap+weld it. Past this, O/L is an ordinary
# release -- so aborting a grasp mid-air doesn't teleport the part onto the jig.
ASSEMBLY_WELD_MAX_DISTANCE = 0.05
# Parts whose colliders SURVIVE their weld, against the default of turning them off. Opting one
# back in re-exposes the interpenetration-shake risk that default guards against.
ASSEMBLY_WELD_KEEP_COLLISION_PART_PRIM_PATHS = ["/World/main_holder_back_cover"]
# Formality: the anchor is KINEMATIC (infinite mass to the solver), but carries no colliders for
# PhysX to derive mass/inertia from either. Same values as a screw's.
ASSEMBLY_WELD_ANCHOR_MASS = 0.002
ASSEMBLY_WELD_ANCHOR_DIAGONAL_INERTIA = [1.0e-7, 1.0e-7, 1.0e-7]

# Driven via the isaacsim.asset.gen.conveyor OmniGraph node, not a direct PhysX write -- two
# confirmed failure modes on the naive route, see docs/mefron-history.md.
CONVEYOR_BELT_PRIM_PATH = "/World/ConveyorBelt_A24/Belt"
# Kept deterministic so setup_conveyor_belt_graph() can find/delete a stray survivor first.
CONVEYOR_ACTION_GRAPH_PRIM_NAME = "ConveyorBeltGraph"
CONVEYOR_ACTION_GRAPH_PATH = "/World/ConveyorBelt_A24/ConveyorBeltGraph"
# ConveyorControl must set this graph variable, not inputs:velocity -- the ReadVariable node
# overwrites that every tick.
CONVEYOR_VELOCITY_VARIABLE_NAME = "Velocity"
# Belt-local direction (not world) -- this axis has flipped between local X/Y across graph
# rebuilds before; treat as a starting point, not a settled fact.
CONVEYOR_LOCAL_VELOCITY_DIRECTION = [1.0, 0.0, 0.0]
CONVEYOR_SPEED = 1.0
MAIN_HOLDER_JIG_PRIM_PATH = "/World/main_holder_jig"
# Relative travel distance, not fixed world-Y endpoints -- see ASSEMBLY_LIFT_HEIGHT's own issue.
CONVEYOR_TRAVEL_DISTANCE = 1.1
# Number-row "1" -- carb.input.KeyboardInput.KEY_1, not the numpad.
CONVEYOR_TOGGLE_KEY = "KEY_1"

# --- Per-part conveyor feeders (ConveyorBelt_A06_02..06) ----------------------------------------

# One 1m belt per sub-part, each queueing GUI-placed copies. Keyed by BASE part prim path -- the
# join key GRASP_TARGETS/ASSEMBLY_RELATIONSHIPS already use. Full design: docs/part-feeders.md.
PART_FEEDERS = {
    "/World/PCB_Assembly_color_fixed": {"belt_prim_path": "/World/ConveyorBelt_A06_02/Belt"},
    "/World/screen": {"belt_prim_path": "/World/ConveyorBelt_A06_03/Belt"},
    "/World/backpanel_support": {"belt_prim_path": "/World/ConveyorBelt_A06_04/Belt"},
    "/World/finger_print_scanner": {"belt_prim_path": "/World/ConveyorBelt_A06_05/Belt"},
    "/World/main_holder_back_cover": {"belt_prim_path": "/World/ConveyorBelt_A06_06/Belt"},
}
FEEDER_GRAPH_PRIM_NAME = "FeederBeltGraph"
# Belt-local, same convention as CONVEYOR_LOCAL_VELOCITY_DIRECTION. These belts' local +X maps to
# world -Y (A24's maps to +Y), so a NEGATIVE speed feeds toward the robot. Confirmed live.
FEEDER_LOCAL_VELOCITY_DIRECTION = [1.0, 0.0, 0.0]
# BELT-LOCAL, not m/s: these belts carry a 0.5 scale, so this is ~0.075 m/s in world (measured).
# feeder._belt_travel_scale() converts, so FEEDER_MAX_TRAVEL below stays in real metres.
FEEDER_SPEED = -0.15
# Ramp instead of stepping the surface velocity: the friction impulse of an instant step can tip a
# part, and the ramp-down is what keeps the stop overshoot inside the front-edge budget (~28mm).
FEEDER_RAMP_SECONDS = 0.2
FEEDER_ADVANCE_DELAY_SECONDS = 5.0
# Dead-reckoned cut-off so a belt whose queue has run out stops instead of running forever.
FEEDER_MAX_TRAVEL = 0.85

# Light-beam photo-eye per belt, all geometry derived from live bboxes at setup -- see
# feeder.build_beam_sensors(). Mounted on the belt's front frame, aimed back down the belt.
FEEDER_SENSOR_SCOPE_PRIM_PATH = "/World/feeder_sensors"
FEEDER_BEAM_MOUNT_STANDOFF = 0.01
# Trips this far BEFORE the parked leading face, and only sees FEEDER_BEAM_DEPTH past it -- the
# background suppression that keeps queued parts further back invisible.
FEEDER_BEAM_PRETRIP = 0.002
FEEDER_BEAM_DEPTH = 0.03
# How close to minRange counts as "arrived". The belt stops on DEPTH, not on the bare trip: a trip
# fires a whole FEEDER_BEAM_DEPTH early, which on the far belts lands the part outside the arm's reach.
# 0.003 is the value the harness passed with. Widening it to ~0.008 would let backpanel_support and
# finger_print_scanner confirm on the beam too (they have a notch at the ray line) -- UNVERIFIED.
FEEDER_BEAM_ARRIVAL_EPSILON = 0.003
# A vertical curtain, not a single ray, and a fine one: measured leading faces sit anywhere from
# 3mm (PCB_Assembly's board) to 33mm (the back cover) above the belt. 24 rays over 40mm = 1.7mm apart.
FEEDER_BEAM_NUM_RAYS = 24
FEEDER_BEAM_CURTAIN_LENGTH = 0.04
# Rays run UP from the sensor origin (a centred curtain would put half of them under the belt and
# trip permanently -- it doesn't, which is how we know). 1mm clears the belt without missing a board.
FEEDER_BEAM_CURTAIN_BASE_OFFSET = 0.001
# Visual-only barrel housing + beam rod, because the sensor's own debug draw is a few-cm line at belt
# height that nothing can see. No colliders, so a raycast can't hit them. feeder._build_beam_visual().
FEEDER_BEAM_HOUSING_RADIUS = 0.009
FEEDER_BEAM_HOUSING_LENGTH = 0.03
FEEDER_BEAM_VISUAL_RADIUS = 0.0015
# How far above/below the belt surface, and how far outside its XY footprint, a rigid body still
# counts as riding that belt in feeder.belt_queue().
FEEDER_QUEUE_HEIGHT_TOLERANCE = 0.1
FEEDER_QUEUE_FOOTPRINT_MARGIN = 0.02
# Number-row "2": advance every feeder now, skipping the delay. A verification key, not part of the
# automatic cycle.
FEEDER_ADVANCE_KEY = "KEY_2"


# The two CAD tools with the female coupler modeled onto the body. Both are baked into
# mefron.usd now (see TOOL_CHANGE_TARGETS), so these paths are only a record of provenance.
SUCTION_GRIPPER_USD = REPO_ROOT / "robots" / "accessories" / "suction_gripper_with_tool_female.usd"
SUCTION_GRIPPER_PRIM_NAME = "suction_gripper_with_tool_female"
SCREWDRIVER_USD = REPO_ROOT / "robots" / "accessories" / "electric_screwdriver_with_tool_female.usd"
SCREWDRIVER_PRIM_NAME = "electric_screwdriver_with_tool_female"
# Identity, not 0.001 -- the raw mesh is mm-scale but add_reference_to_stage()'s Metrics
# Assembler auto-corrects it; an explicit 0.001 double-scales to 1e-6. docs/tool-changer.md's gotcha 7.
SUCTION_GRIPPER_LOCAL_SCALE = [1.0, 1.0, 1.0]
SCREWDRIVER_LOCAL_SCALE = [1.0, 1.0, 1.0]

# --- Screw pick-and-place (screwdriver tool only) -----------------------------------------------

# Placeholder fastener, NOT real CAD -- two Cylinders whose origin is the screw's TIP, body back
# along local -Z. No colliders on purpose: every stage of its life is joint-driven.
SCREW_USD = REPO_ROOT / "assets" / "mefron" / "screw_m3.usd"
# Script-owned scope, cleared and rebuilt every run by screws.clear_screws(). Deliberately
# separate from SCREW_PRESENTER_PRIM_PATH below, which may be hand-placed and must survive.
SCREW_SCOPE_PRIM_PATH = "/World/screws"
# Tip-to-head-face length and an M3's rough mass. The explicit inertia matters because the screw
# has no colliders for PhysX to derive one from.
SCREW_LENGTH = 0.012
SCREW_MASS = 0.002
SCREW_DIAGONAL_INERTIA = [1.0e-7, 1.0e-7, 1.0e-7]

# The bit tip in the DOCKED tool root's own frame, metres. CAD-derived from SCREWDRIVER_USD's
# mesh, not a guess -- see docs/tool-changer.md.
SCREWDRIVER_TIP_LOCAL_POSITION = [0.0, 0.0, 0.2748543]
# One screw-length past the tip, so the head's outer face lands exactly ON the bit. Identity
# orientation -- the screw's own +Z already agrees with the tool's.
SCREW_CARRY_LOCAL_POSITION = [0.0, 0.0, SCREWDRIVER_TIP_LOCAL_POSITION[2] + SCREW_LENGTH]
SCREW_CARRY_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

# Same baked-or-fallback duality as TOOL_CHANGE_TARGETS: if this prim already exists in
# mefron.usd its live pose wins and the fallback below is ignored. It does exist on this scene.
SCREW_PRESENTER_PRIM_PATH = "/World/screw_presenter"
# The prim's ORIGIN pose, not the screw's -- SCREW_PRESENTER_SEAT_LOCAL_* composes on top.
# Unused on this scene: the real presenter is baked into mefron.usd.
SCREW_PRESENTER_FALLBACK_POSITION = [2.89334, -4.964, 0.977]
SCREW_PRESENTER_FALLBACK_ORIENTATION_WXYZ = [0.0, 1.0, 0.0, 0.0]
# Where the presenter holds the screw, in ITS own scale-free frame -- the prim origin is the
# base plate ~72mm below, so this offset is what makes it a screw pose.
SCREW_PRESENTER_SEAT_LOCAL_POSITION = [0.00666, -0.086, 0.072]
# 180 deg about X (GUI-verified): points the tip down into the seat and the head UP, where a bit
# descending from above can reach it. Identity presented it head-down.
SCREW_PRESENTER_SEAT_LOCAL_ORIENTATION_WXYZ = [0.0, 1.0, 0.0, 0.0]

# The back cover, NOT main_holder: a fastener goes through the cover into the holder. Its hole
# mouths sit 15mm above main_holder's own in world.
SCREW_HOLE_MOUNT_PRIM_PATH = "/World/main_holder_back_cover"
# The nine real clearance holes, read off the cover's own mesh -- derivation in
# docs/tool-changer.md. A list, not a dict: order IS the fill sequence, a perimeter walk.
SCREW_HOLES = [
    # Identity throughout: the cover's world rotation is already 180 deg about X, so the screw's
    # local +Z (tip) comes out pointing down into the hole with no extra twist.
    {"local_position": [0.082276, -0.110276, 0.0], "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
    {"local_position": [0.085675, -0.056047, 0.0], "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
    {"local_position": [0.085675, 0.057953, 0.0], "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
    {"local_position": [0.081834, 0.109834, 0.0], "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
    {"local_position": [-0.081834, 0.109834, 0.0], "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
    {"local_position": [-0.085675, 0.057953, 0.0], "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
    {"local_position": [-0.085675, -0.056047, 0.0], "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
    {"local_position": [-0.082276, -0.110276, 0.0], "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
    {"local_position": [0.0, -0.112875, 0.0], "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
]
# How far down the hole's +Z the tip ends up. 0.00 = at the mouth, body standing proud -- a
# dropped-in screw, matching the no-driving-rotation scope. The knob for seating deeper.
SCREW_HOLE_INSERTION_DEPTH = 0.00
# Relative to each pick/place pose, never a world-Z constant. Smaller than the rack's 0.15 for a
# computed reason: the 275mm tool would put far holes past the Panda's reach. docs/tool-changer.md.
SCREW_APPROACH_CLEARANCE = 0.02
# Number-row 5/6, not numpad -- Y/U/I belong to the tool changer on this branch.
SCREW_PICK_KEY = "KEY_5"
SCREW_PLACE_KEY = "KEY_6"

# Real isaacsim.robot.schema/surface_gripper physics, distinct from the visual SUCTION_GRIPPER_*
# above -- the joint rides panda_hand permanently, whichever tool is docked.
SURFACE_GRIPPER_JOINT_PRIM_NAME = "SurfaceGripperJoint"
SURFACE_GRIPPER_PRIM_NAME = "SurfaceGripper"
# The joint's frame on the wrist's side. PRE-ATC value, and now also pre-FR5 -- needs re-deriving
# by hand-jog. See docs/tool-changer.md and docs/fr5-migration.md.
SURFACE_GRIPPER_LOCAL_POSITION = [0.0, 0.0, 0.1]
SURFACE_GRIPPER_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]
# isaac:maxGripDistance -- how far the attachment point searches. Schema default 0.01m, widened
# for first-pass teleop-approach tolerance.
SURFACE_GRIPPER_MAX_GRIP_DISTANCE = 0.03
# Hover clearance for the approach pose, distinct from the search radius above -- baked into
# suction_gripper_approach_on_screen's local z, so change both together.
SURFACE_GRIPPER_APPROACH_CLEARANCE = 0.01

# Only meaningful while the suction tool is docked. S/H/R were tried first and confirmed live to
# double as Kit viewport hotkeys -- see docs/mefron-history.md.
SUCTION_ATTACH_KEY = "V"  # Vacuum on
SUCTION_DETACH_KEY = "L"  # reLease

# Same shape as GRASP_TARGETS but for the suction cup: "key" snaps target to
# approach_relationship, P looks up assembly_relationship for whatever was last approached.
SUCTION_TARGETS = {
    "screen": {
        "key": "N",
        "approach_relationship": "suction_gripper_approach_on_screen",
        "assembly_relationship": "screen_on_main_holder",
    },
    "pcb_assembly": {
        "key": "M",
        "approach_relationship": "suction_gripper_approach_on_pcb_assembly",
        "assembly_relationship": "pcb_assembly_on_backpanel_support",
    },
}

SCREEN_PRIM_PATH = "/World/screen"

# --- Automatic tool changer (ATC): one Franka, three swappable tools ---------------------------

# Male half: a plain cylinder matching the Franka's ISO 9409-1-50 wrist flange, permanently on
# panda_hand. Why a scripted FixedJoint over Robot Assembler/SurfaceGripper: docs/tool-changer.md.
TOOL_CHANGER_MALE_PRIM_NAME = "tool_changer_male"
TOOL_CHANGER_CYLINDER_RADIUS = 0.0315  # Ø63mm
TOOL_CHANGER_CYLINDER_HEIGHT = 0.02
# Sits ON the flange, not at wrist3_link's origin: that origin is 99mm inboard, in empty space, so
# the coupler used to float clear of the wrist's own metal. Its inner face IS the mate plane.
TOOL_CHANGER_MALE_LOCAL_POSITION = [0.0, 0.0, FR5_TOOL_FLANGE_OFFSET + TOOL_CHANGER_CYLINDER_HEIGHT / 2]
TOOL_CHANGER_MALE_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

# ee_link's pose relative to a mated female coupler -- the same for every tool by design. Still an
# identity PLACEHOLDER pending hand-jog; suction/screwdriver only. See docs/tool-changer.md.
TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_POSITION = [0.0, 0.0, 0.0]
TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

# STALE: this was franka_panda.urdf's own panda_hand_joint offset, inverted -- meaningless once the
# arm is an FR5 and the tool a PGC-140. Re-derive in step 3; see docs/fr5-migration.md.
TOOL_CHANGER_GRIPPER_HAND_JOINT_LOCAL_ORIENTATION_WXYZ = [0.9238795325112867, 0.0, 0.0, 0.3826834323650898]

# Hover clearance above a rack's dock pose while aligning before the descent. Relative to each
# dock pose, not a fixed world-Z constant like ASSEMBLY_LIFT_HEIGHT.
TOOL_RACK_APPROACH_CLEARANCE = 0.15

# The real, GUI-baked rack. Each tool's rack_prim_path anchor re-syncs to wherever its baked tool
# actually sits, so moving a tool in the GUI needs no code change.
TOOL_RACK_PRIM_PATH = "/World/tool_rack"

# One entry per dockable tool. All 3 are baked prims (only read, never repositioned) and flat
# single rigid bodies. female_coupler_local_* are still placeholders. See docs/tool-changer.md.
TOOL_CHANGE_TARGETS = {
    "gripper": {
        "key": "Y",
        # Switched to baked last, after a live-referenced gripper asset kept landing a gapped dock.
        "baked_tool_prim_path": "/World/cr5_pgc140_gripper",
        # Multi-link articulation, unlike the flat suction/screwdriver tools: female_coupler must
        # hang off a real RigidBodyAPI link, not the tool root. docs/tool-changer.md gotchas 3-4.
        "female_coupler_parent_link_name": "pgc140_base_link",
        "rack_prim_path": "/World/tool_rack_gripper",
        "female_coupler_local_position": [0.0, 0.0, 0.0],
        "female_coupler_local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    },
    "suction": {
        "key": "U",
        # "rack_prim_path" below is a separate, lightweight non-physics anchor Xform synced to this
        # prim's live pose every run, which park_tool_at_rack()'s joint uses as its reference.
        "baked_tool_prim_path": "/World/suction_gripper_with_tool_female",
        "rack_prim_path": "/World/tool_rack_suction",
        "female_coupler_local_position": [0.0, 0.0, 0.0],
        "female_coupler_local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    },
    "screwdriver": {
        "key": "I",
        # Same baked-into-mefron.usd pattern as suction above.
        "baked_tool_prim_path": "/World/electric_screwdriver_with_tool_female",
        "rack_prim_path": "/World/tool_rack_screwdriver",
        "female_coupler_local_position": [0.0, 0.0, 0.0],
        "female_coupler_local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    },
}

# isaacsim.exp.full.kit's extras over isaacsim.exp.base.python.kit, enabled only AFTER the Franka
# is mounted -- loading them earlier crashes the URDF importer. Plus conveyor(.ui), deliberately.
FULL_EXPERIENCE_EXTRA_EXTENSIONS = [
    "isaacsim.app.setup",
    "isaacsim.asset.gen.conveyor",
    "isaacsim.asset.gen.conveyor.ui",
    "isaacsim.asset.gen.omap",
    "isaacsim.asset.gen.omap.ui",
    "isaacsim.asset.importer.heightmap",
    "isaacsim.asset.validation",
    "isaacsim.examples.browser",
    "isaacsim.examples.extension",
    "isaacsim.examples.interactive",
    "isaacsim.exp.base",
    "isaacsim.gui.components",
    "isaacsim.replicator.behavior.ui",
    "isaacsim.replicator.grasping.ui",
    "isaacsim.replicator.scene_blox",
    "isaacsim.replicator.synthetic_recorder",
    "isaacsim.robot.manipulators.examples",
    "isaacsim.robot.manipulators.ui",
    "isaacsim.robot.surface_gripper.ui",
    "isaacsim.robot.wheeled_robots.ui",
    "isaacsim.robot_setup.assembler",
    "isaacsim.robot_setup.gain_tuner",
    "isaacsim.robot_setup.grasp_editor",
    "isaacsim.robot_setup.xrdf_editor",
    "isaacsim.sensors.camera.ui",
    "isaacsim.sensors.physics.examples",
    "isaacsim.sensors.physics.ui",
    # Base ext, listed explicitly (not just via its .examples/.ui below): feeder.py's light-beam
    # photo-eyes need it, and it must be enabled in headless runs too.
    "isaacsim.sensors.physx",
    "isaacsim.sensors.physx.examples",
    "isaacsim.sensors.physx.ui",
    "isaacsim.sensors.rtx.ui",
    "isaacsim.util.camera_inspector",
    "isaacsim.util.merge_mesh",
    "isaacsim.util.physics",
    "omni.anim.curve.bundle",
    "omni.anim.shared.core",
    "omni.asset_validator.ui",
    "omni.graph.bundle.action",
    "omni.graph.visualization.nodes",
    "omni.graph.window.action",
    "omni.graph.window.generic",
    "omni.importer.onshape",
    "omni.isaac.block_world",
    "omni.isaac.extension_templates",
    "omni.isaac.gain_tuner",
    "omni.isaac.grasp_editor",
    "omni.isaac.occupancy_map",
    "omni.isaac.occupancy_map.ui",
    "omni.isaac.physics_inspector",
    "omni.isaac.range_sensor.examples",
    "omni.isaac.range_sensor.ui",
    "omni.isaac.robot_assembler",
    "omni.isaac.robot_description_editor",
    "omni.isaac.scene_blox",
    "omni.isaac.synthetic_recorder",
    "omni.isaac.throttling",
    "omni.kit.actions.window",
    "omni.kit.asset_converter",
    "omni.kit.browser.asset",
    "omni.kit.browser.material",
    "omni.kit.collaboration.channel_manager",
    "omni.kit.context_menu",
    "omni.kit.converter.cad",
    "omni.kit.graph.delegate.default",
    "omni.kit.hotkeys.window",
    "omni.kit.manipulator.transform",
    "omni.kit.mesh.raycast",
    "omni.kit.preferences.animation",
    "omni.kit.profiler.window",
    "omni.kit.property.collection",
    "omni.kit.property.layer",
    "omni.kit.quicklayout",
    "omni.kit.renderer.capture",
    "omni.kit.renderer.core",
    "omni.kit.scripting",
    "omni.kit.search.files",
    "omni.kit.selection",
    "omni.kit.stage.copypaste",
    "omni.kit.stage.mdl_converter",
    "omni.kit.stage_column.payload",
    "omni.kit.stage_column.variant",
    "omni.kit.stage_templates",
    "omni.kit.stagerecorder.bundle",
    "omni.kit.tool.asset_exporter",
    "omni.kit.tool.remove_unused.controller",
    "omni.kit.tool.remove_unused.core",
    "omni.kit.uiapp",
    "omni.kit.usda_edit",
    "omni.kit.variant.editor",
    "omni.kit.variant.presenter",
    "omni.kit.viewport.actions",
    "omni.kit.viewport.bundle",
    "omni.kit.viewport.rtx",
    "omni.kit.viewport_widgets_manager",
    "omni.kit.widget.cache_indicator",
    "omni.kit.widget.collection",
    "omni.kit.widget.extended_searchfield",
    "omni.kit.widget.filebrowser",
    "omni.kit.widget.layers",
    "omni.kit.widget.live",
    "omni.kit.widget.schema_api",
    "omni.kit.widget.timeline",
    "omni.kit.widget.versioning",
    "omni.kit.widgets.custom",
    "omni.kit.window.collection",
    "omni.kit.window.commands",
    "omni.kit.window.cursor",
    "omni.kit.window.extensions",
    "omni.kit.window.file",
    "omni.kit.window.filepicker",
    "omni.kit.window.material",
    "omni.kit.window.material_graph",
    "omni.kit.window.preferences",
    "omni.kit.window.quicksearch",
    "omni.kit.window.script_editor",
    "omni.kit.window.stats",
    "omni.kit.window.title",
    "omni.kit.window.usd_paths",
    "omni.physx.asset_validator",
    "omni.physx.bundle",
    "omni.resourcemonitor",
    "omni.simready.explorer",
    "omni.stats",
    "omni.usd.metrics.assembler.physics",
    "omni.usd.schema.scene.visualization",
]
