"""Constants shared across the mefron family of scripts: paths, mount pose, gripper/friction/drive
tuning, and the derived grasp/assembly relative poses. Pure data -- no omni/curobo imports, safe to
import at any point.
"""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MEFRON_USD = REPO_ROOT / "assets" / "mefron" / "factory floor" / "mefron.usd"
# Disk-persisted "Robot Description" dir the URDF importer writes on every import; see
# kit_bootstrap.clear_stale_robot_configuration().
MEFRON_CONFIGURATION_DIR = MEFRON_USD.parent / "configuration"

ROBOT_PRIM_PATH = "/World/Franka"
TARGET_PRIM_PATH = "/World/target"
# UR10 mount pedestal the Franka mounts on; position is this prim's own xformOp:translate
# (unconfirmed whether that's the top mounting flange).
# No /Factory prefix -- mefron.py opens mefron.usd directly, one level shallower than
# build_scene_mefron.py's reference.
MOUNT_PLATE_PRIM_PATH = "/World/ur10_mount"
MOUNT_POSITION = [2.625260866887235, -4.7019853821770115, 0.8093334035921127]
MOUNT_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

FRANKA_URDF_RELATIVE_PATH = "robot/franka_description/franka_panda.urdf"
FRANKA_DRIVE_STRENGTH = 1047.19751
# Bumped ~4x from the original 52.35988 (tuned for a bare wrist) -- confirmed live the ATC's rigid
# tool-changer FixedJoint underdamps once a real tool is bolted on: joint velocity pins at the
# Panda wrist joints' own hardware limit (2.61 rad/s) instead of decaying after dock_tool_to_wrist(),
# once _set_tool_collision_enabled() actually disables the tool's collision (previously silently
# no-op'ing -- see robot.py -- and accidentally providing contact-friction damping that partly
# masked this). Empirical first attempt, not yet confirmed sufficient live.
FRANKA_DRIVE_DAMPING = 210.0
FRANKA_MOTION_GEN_ROBOT_CFG = "franka.yml"

# Reach-envelope obstacles for both arms, not the whole /World/Factory backdrop. Excludes the
# conveyor/container prims -- hangs cuRobo's mesh-collision-world construction; see CLAUDE.md's
# open issues.
OBSTACLE_PRIM_PATHS = [
"/World/ConveyorBelt_A06_01",]

# Loop-timing constants for teleop.run_teleop_loop(), ported from build_scene.py.
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
_TELEOP_TIME_DILATION_FACTOR = 0.3

# Caps velocity/acceleration limits used during trajectory optimization. cuRobo treats scale <=
# 0.25 as a special case (swaps in finetune_trajopt_slow.yml); 0.2 stays under that threshold.
_TELEOP_VELOCITY_SCALE = 0.6
_TELEOP_ACCELERATION_SCALE = 0.1

# Grasp-physics constants, ported from build_scene_mefron.py's apply_gripper_friction()/stiffen_gripper_drive().
GRIPPER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]
# Narrowed to bracket finger_print_scanner's 12mm grip width -- the full stroke let one finger
# drag the part sideways. See docs/mefron-history.md.
GRIPPER_OPEN_POSITION = 0.010
GRIPPER_CLOSED_POSITION = 0.000
# Rate (m/s) the commanded gripper position ramps toward open/closed, instead of stepping
# instantly -- avoids a snap shut under the high drive stiffness.
GRIPPER_CLOSE_SPEED = 0.02
GRIPPER_FRICTION_MATERIAL_PATH = "/World/GripperFrictionMaterial"
GRIPPER_STATIC_FRICTION = 1.5
GRIPPER_DYNAMIC_FRICTION = 1.5
GRIPPER_FINGER_LINK_NAMES = ["panda_leftfinger", "panda_rightfinger"]
GRIPPER_DRIVE_STIFFNESS = 10000.0
GRIPPER_DRIVE_DAMPING = 200.0
# "force" (stiffness in N/m) over the assets' baked "acceleration", where the gains are
# mass-normalized and a few-gram finger turns STIFFNESS into well under a newton of real grip --
# parts slipped out. maxForce (20N, baked) stays the clamp. See docs/mefron-history.md.
GRIPPER_DRIVE_TYPE = "force"
HIGH_FRICTION_PRIM_PATHS = ["/World/finger_print_scanner"]

# Grasp Editor-exported poses + per-object finger widths, keyed by object name and wired to a key
# in teleop.build_gripper_keyboard_control(). See grasp.compute_grasp_approach_pose_from_file().
GRASP_TARGETS = {
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
    # pcb_assembly's K retired 2026-07-22 for arm 2's suction cup instead -- redundant-branch
    # twisting on approach, root cause + ruled-out fix in docs/mefron-history.md; the freed key
    # now drives main_holder_back_cover above.
}

# T_H_S: each part's pose expressed in main_holder's own local frame at the correctly assembled
# position, derived via grasp.compute_relative_pose() -- see docs/grasp-and-assembly-offsets.md.
ASSEMBLY_RELATIONSHIPS = {
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
    # Probed live with assets/back_cover_grasp.txt while the cover sat assembled on main_holder.
    # Zeros/identity are the probe's own output (it printed ~1e-16 float noise); z is negative
    # because main_holder's frame is flipped 180 deg about X, so this is 15mm *up* in world.
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
    # Suction gripper's own approach target expressed in screen's live frame, not a carried part's
    # mount pose (mount_prim_path=part_prim_path=screen on purpose). Derivation + measured values:
    # docs/mefron-history.md. Orientation re-derived 2026-08-06 by reading /World/target's transform
    # relative to the screen: the old value carried a 135 deg yaw about Z, which put the approach
    # visibly diagonal; identity is straight on. local_position kept from the original derivation --
    # the screen was ATTACHED to the cup during that probe, so its measured z was contact (-0.11024),
    # 5.3mm short of the standoff an approach pose needs.
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
    # Suction approach target for pcb_assembly, same derivation as suction_gripper_approach_on_screen
    # above -- see docs/mefron-history.md.
    "suction_gripper_approach_on_pcb_assembly": {
        "part_prim_path": "/World/PCB_Assembly_color_fixed",
        "mount_prim_path": "/World/PCB_Assembly_color_fixed",
        "local_position": [-0.0028148316864434492, 4.480405335696105e-06, 0.11491755932216695],
        "local_orientation_wxyz": [0.011293781372283615, 0.38247333692520136, 0.9238856699972284, 0.004676089968013461],
    },
}

# Where the O/L release weld SEATS each part, overriding ASSEMBLY_RELATIONSHIPS' offset for the weld
# only -- P still DRIVES to that dict's (motion-validated) pose. Measured together 2026-08-06 from
# ONE hand-placed assembly (every part on the jig at once, then read back), so these are mutually
# consistent rather than each measured in its own session; float noise below 1e-16 cleaned to exact
# zeros/identity. A relationship with no entry here welds at its ASSEMBLY_RELATIONSHIPS pose
# (main_holder_back_cover -- it wasn't on the jig for that measurement). The two poses differing IS
# the point, but keep them inside ASSEMBLY_WELD_MAX_DISTANCE: the gap is how far the part visibly
# jumps on release (worst here: backpanel_support, 12mm).
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
    # Mounts on backpanel_support, itself a welded part, so this was measured against wherever THAT
    # was placed -- the two stay a set. Rotation came back an exact 90 deg about Z (was 90.12).
    "pcb_assembly_on_backpanel_support": {
        "local_position": [-0.00158, -0.02139, 0.005],
        "local_orientation_wxyz": [0.7071067811865476, 0.0, 0.0, 0.7071067811865476],
    },
}

# --- Assembly weld (O/L release) ----------------------------------------------------------------
# Script-owned scope holding every welded part's anchor body + joints, wiped every run
# (robot.clear_assembly_welds()) -- same reasoning as SCREW_SCOPE_PRIM_PATH.
ASSEMBLY_WELD_SCOPE_PRIM_PATH = "/World/assembly_welds"
# How close (metres) a part must already be to its nominal ASSEMBLY_RELATIONSHIPS pose for a release
# to snap+weld it there. Past this, O/L is an ordinary release -- so aborting a grasp mid-air doesn't
# teleport the part onto the jig.
ASSEMBLY_WELD_MAX_DISTANCE = 0.05
# Parts whose colliders SURVIVE their weld, against the default of turning them off (see
# robot.weld_part_at_assembly_pose()). Opting one back in re-exposes the interpenetration-shake risk
# that default guards against -- worth it only where something still has to collide with the part.
ASSEMBLY_WELD_KEEP_COLLISION_PART_PRIM_PATHS = ["/World/main_holder_back_cover"]
# The per-mount anchor body's mass/inertia. Mostly formality -- the anchor is KINEMATIC (infinite
# mass to the solver, see robot._ensure_assembly_anchor()) -- but it carries no colliders either, so
# PhysX has nothing to derive them from. Same values as a screw's (SCREW_MASS, later in this file).
ASSEMBLY_WELD_ANCHOR_MASS = 0.002
ASSEMBLY_WELD_ANCHOR_DIAGONAL_INERTIA = [1.0e-7, 1.0e-7, 1.0e-7]

# Drives ConveyorBelt_A24 via Isaac Sim's isaacsim.asset.gen.conveyor OmniGraph node
# (CreateConveyorBelt), not a direct PhysX write -- see docs/mefron-history.md for why (two
# confirmed failure modes on the naive route).
CONVEYOR_BELT_PRIM_PATH = "/World/ConveyorBelt_A24/Belt"
# ActionGraph path CreateConveyorBelt creates at -- kept deterministic so
# setup_conveyor_belt_graph() can find/delete a stray survivor before rebuilding.
CONVEYOR_ACTION_GRAPH_PRIM_NAME = "ConveyorBeltGraph"
CONVEYOR_ACTION_GRAPH_PATH = "/World/ConveyorBelt_A24/ConveyorBeltGraph"
# ConveyorControl must set this graph variable, not the node's inputs:velocity directly (that gets
# overwritten every tick by the ReadVariable node).
CONVEYOR_VELOCITY_VARIABLE_NAME = "Velocity"
# Belt-local direction (not world) -- this axis has flipped between local X/Y across graph
# rebuilds before; treat as a starting point, not a settled fact.
CONVEYOR_LOCAL_VELOCITY_DIRECTION = [1.0, 0.0, 0.0]
CONVEYOR_SPEED = 1.0
MAIN_HOLDER_JIG_PRIM_PATH = "/World/main_holder_jig"
# Relative travel distance, not fixed world-Y endpoints -- same "fixed constant with no
# relationship to where things are" issue as ASSEMBLY_LIFT_HEIGHT (see CLAUDE.md).
CONVEYOR_TRAVEL_DISTANCE = 1.1
# Number-row "1", not numpad -- carb.input.KeyboardInput.KEY_1.
CONVEYOR_TOGGLE_KEY = "KEY_1"


# scripts/vendor_gripper_tool.py's pre-baked export of robot.mount_franka_hand_only()'s hand-only
# URDF -- referenced, not live-imported: confirmed live that importing a second robot via the URDF
# importer into mefron.usd's own stage corrupts the main arm's own link structure through a shared
# "Robot Description" cache, even at a fully distinct prim path -- see docs/tool-changer.md's
# gotcha 6. Superseded by GRIPPER_TOOL_VISUAL_ONLY_USD below as the ATC gripper tool's actual
# asset -- kept only as the (dormant) source this real multi-link mini-articulation once was.
GRIPPER_TOOL_HAND_ONLY_USD = REPO_ROOT / "robots" / "franka_panda" / "Props" / "gripper_tool_hand_only.usd"
# scripts/vendor_gripper_tool_visual_only.py's stripped export of the same hand-only URDF -- one of
# the 3 dockable ATC tools, now the one actually referenced by TOOL_CHANGE_TARGETS. Confirmed live:
# GRIPPER_TOOL_HAND_ONLY_USD's real joints/mass/inertia, rigidly bolted onto the wrist via
# dock_tool_to_wrist()'s FixedJoint, produced a genuine dynamic resonance (joint velocity pinned at
# the Panda wrist joints' own hardware limit, never decaying) independent of pose accuracy,
# collision state, or joint-damping increases -- suction/screwdriver never hit this because they're
# simple visual props with one flat RigidBodyAPI, not a separately-jointed mini-articulation. This
# asset strips all joints/RigidBodyAPI/CollisionAPI to match that same flat shape -- see
# docs/tool-changer.md.
GRIPPER_TOOL_VISUAL_ONLY_USD = REPO_ROOT / "robots" / "franka_panda" / "Props" / "gripper_tool_visual_only.usd"

# Custom Franka-flange suction gripper -- one of the 3 dockable ATC tools (see the "Automatic tool
# changer" section below). Supersedes the suction-only "suction gripper.usd" -- this one has the
# female coupler tool modeled directly onto the CAD body, same reason SCREWDRIVER_USD was swapped
# to electric_screwdriver_with_tool_female.usd. See docs/mefron-history.md for the old asset's
# alignment derivation.
SUCTION_GRIPPER_USD = REPO_ROOT / "robots" / "accessories" / "suction_gripper_with_tool_female.usd"
SUCTION_GRIPPER_PRIM_NAME = "suction_gripper_with_tool_female"
# Stale: described the old suction-only asset's own root alignment, via attach_suction_gripper()
# (unreachable from mefron.py's actual ATC flow -- spawn_dockable_tool() always docks tools at
# local identity instead). Not re-derived for this asset. See docs/mefron-history.md.
SUCTION_GRIPPER_LOCAL_POSITION = [0.0, 0.0, 0.0]
SUCTION_GRIPPER_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]
# Whether this needs a manual scale isn't predictable from the file alone -- see
# docs/tool-changer.md gotcha 7. Confirmed live via spawn_dockable_tool()'s actual code path:
# [1,1,1] gives a ~9x13x11cm world bbox (sane), same outcome as the screwdriver's asset.
SUCTION_GRIPPER_LOCAL_SCALE = [1.0, 1.0, 1.0]

# Electric-screwdriver end-effector, another of the 3 dockable ATC tools. Same panda_hand-child
# mounting pattern as SUCTION_GRIPPER_* above (see robot.attach_screwdriver_gripper()).
# Supersedes the screwdriver-only electric_screwdriver.usd -- this one has the female coupler
# tool modeled directly onto the CAD body, not a bare abstract female_coupler Xform.
SCREWDRIVER_USD = REPO_ROOT / "robots" / "accessories" / "electric_screwdriver_with_tool_female.usd"
SCREWDRIVER_PRIM_NAME = "electric_screwdriver_with_tool_female"
# Identity, not 0.001 -- this asset's raw mesh data IS mm-scale (unlike electric_screwdriver.usd's
# pre-scaled mesh), but add_reference_to_stage()'s Metrics Assembler check auto-corrects it on
# reference-add here (confirmed live, reproducibly); an explicit 0.001 on top double-scales it to
# 1e-6. See docs/tool-changer.md gotcha 7.
SCREWDRIVER_LOCAL_SCALE = [1.0, 1.0, 1.0]
SCREWDRIVER_LOCAL_POSITION = [0.0, 0.0, 0.0]
# Stale: derived for the old screwdriver-only asset's own CAD origin, via
# attach_screwdriver_gripper() (unreachable from mefron.py's actual ATC flow -- spawn_dockable_tool()
# always docks tools at local identity instead). Not re-derived for this asset.
SCREWDRIVER_LOCAL_ORIENTATION_WXYZ = [0.2705980501, 0.6532814824, -0.2705980501, 0.6532814824]

# --- Screw pick-and-place (screwdriver tool only) -----------------------------------------------
# Hand-authored placeholder fastener, NOT real CAD -- two plain Cylinders whose own origin is the
# screw's TIP, body running back along local -Z (see the asset's own comment). No colliders on
# purpose: every stage of a screw's life is joint-driven, so contact would only fight the joint.
SCREW_USD = REPO_ROOT / "assets" / "mefron" / "screw_m3.usd"
# Script-owned scope, cleared and rebuilt every run (robot.clear_screws()) -- holds every spawned
# screw plus its own joints/anchors. Deliberately separate from SCREW_PRESENTER_PRIM_PATH below,
# which may be a hand-placed prim and must survive.
SCREW_SCOPE_PRIM_PATH = "/World/screws"
# Tip-to-head-face length of the placeholder above, and an M3 screw's rough real mass. The explicit
# inertia matters because the screw has no colliders for PhysX to derive one from.
SCREW_LENGTH = 0.012
SCREW_MASS = 0.002
SCREW_DIAGONAL_INERTIA = [1.0e-7, 1.0e-7, 1.0e-7]

# The bit's tip in the DOCKED tool root's own frame, in metres, composed against that prim's
# scale-free get_world_pose(). CAD-derived, not a guess: SCREWDRIVER_USD's mesh points reach
# z=274.854mm (x/y centroid 0.00, symmetric +-6.99mm 5mm back from the tip, so on-axis), and its
# female coupler head occupies z 0->10mm, confirming +Z runs coupler->tip. See docs/tool-changer.md.
SCREWDRIVER_TIP_LOCAL_POSITION = [0.0, 0.0, 0.2748543]
# Where a carried screw sits in that same frame: one screw-length past the tip, so the head's outer
# face lands exactly ON the bit tip. Identity orientation -- the screw's own +Z (tip direction)
# already agrees with the tool's +Z, so no twist is needed.
SCREW_CARRY_LOCAL_POSITION = [0.0, 0.0, SCREWDRIVER_TIP_LOCAL_POSITION[2] + SCREW_LENGTH]
SCREW_CARRY_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

# Stand-in for the real screw presenter the user doesn't have yet. Same baked-vs-fallback duality as
# TOOL_CHANGE_TARGETS' baked_tool_prim_path/dock_position: if this prim already exists in mefron.usd
# (hand-placed in the GUI), its live world pose wins and the fallback below is ignored entirely.
SCREW_PRESENTER_PRIM_PATH = "/World/screw_presenter"
# The prim's ORIGIN pose, not the screw's -- SCREW_PRESENTER_SEAT_LOCAL_* below composes on top.
# Chosen so the composed screw still lands at [2.90, -5.05, 0.905] (free table: parts sit at
# y <= -5.20, main_holder at y >= -4.90), putting the wrist ~0.59m from MOUNT_POSITION.
SCREW_PRESENTER_FALLBACK_POSITION = [2.89334, -4.964, 0.977]
SCREW_PRESENTER_FALLBACK_ORIENTATION_WXYZ = [0.0, 1.0, 0.0, 0.0]
# Where the presenter actually holds the screw, in ITS own scale-free local frame -- metres, same
# convention as SCREW_HOLES. The baked CAD presenter's prim origin is its base-plate centre, ~72mm
# below the seat, so this offset is what makes the pose a screw pose rather than a base-plate pose.
SCREW_PRESENTER_SEAT_LOCAL_POSITION = [0.00666, -0.086, 0.072]
# 180 deg about X (GUI-verified): the screw asset's origin is its TIP with the body running back
# along local -Z, so this points the tip down into the seat and the head UP, where a bit descending
# from above can reach it. Identity presented it head-down. Same rotation the old fallback used.
SCREW_PRESENTER_SEAT_LOCAL_ORIENTATION_WXYZ = [0.0, 1.0, 0.0, 0.0]

# The back cover, NOT main_holder: a fastener goes through the cover into the holder, so the cover
# owns the holes a screw is seen entering. Its mouths sit 15mm above main_holder's own in world.
SCREW_HOLE_MOUNT_PRIM_PATH = "/World/main_holder_back_cover"
# The nine real clearance holes, read off the cover's own mesh (r=2.000mm rings at its local z=0
# face, 12mm deep) rather than a measurement sheet -- derivation in docs/tool-changer.md. Metres in
# the mount's scale-free frame, same convention as ASSEMBLY_RELATIONSHIPS. A list, not a name-keyed
# dict: order is the fill sequence, a perimeter walk up the +x edge, down the -x edge, then centre.
SCREW_HOLES = [
    # Identity orientation throughout: the cover's own world rotation is already 180 deg about X, so
    # the screw's local +Z (tip) comes out pointing down into the hole with no extra twist.
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
# How far down the hole's own +Z the screw's tip ends up. 0.00 = tip at the mouth, body standing
# proud -- a dropped-in screw, matching the no-driving-rotation scope. The knob for seating deeper.
SCREW_HOLE_INSERTION_DEPTH = 0.00
# Relative to each pick/place pose, never a world-Z constant (see ASSEMBLY_LIFT_HEIGHT in CLAUDE.md
# for that exact mistake). Smaller than TOOL_RACK_APPROACH_CLEARANCE for a computed reason: the tool
# hangs 275mm below the wrist, so 0.15m of hover puts the far holes ~0.874m from the mount, past the
# Panda's ~0.855m reach. See docs/tool-changer.md's screw section.
SCREW_APPROACH_CLEARANCE = 0.02
# Number-row 5/6, not numpad -- carb.input.KeyboardInput.KEY_5/KEY_6. Same convention as
# CONVEYOR_TOGGLE_KEY (KEY_1); numpad 1/2/3 belong to the tool changer.
SCREW_PICK_KEY = "KEY_5"
SCREW_PLACE_KEY = "KEY_6"

# Real isaacsim.robot.schema/surface_gripper physics (distinct from the pure-visual
# SUCTION_GRIPPER_* above) -- bare structural minimum only, no compliance tuning.
SURFACE_GRIPPER_JOINT_PRIM_NAME = "SurfaceGripperJoint"
SURFACE_GRIPPER_PRIM_NAME = "SurfaceGripper"
# Joint's frame on panda_hand's side: cup's physical tip, 0.1m out along +Z (base->tip, like
# SUCTION_GRIPPER_LOCAL_* above). Pre-ATC value -- now that the suction gripper docks beyond
# TOOL_CHANGER_CYLINDER_HEIGHT's standoff instead of sitting directly on panda_hand, this needs
# re-deriving by hand-jog once the coupler/female geometry exists live. See docs/tool-changer.md.
SURFACE_GRIPPER_LOCAL_POSITION = [0.0, 0.0, 0.1]
SURFACE_GRIPPER_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]
# isaac:maxGripDistance -- how far the attachment point searches for something to grab. Schema
# default is 0.01m, widened slightly for first-pass teleop-approach tolerance.
SURFACE_GRIPPER_MAX_GRIP_DISTANCE = 0.03
# Hover clearance for the approach pose (distinct from the joint's search radius above) -- baked
# into suction_gripper_approach_on_screen's local z, so change both together.
SURFACE_GRIPPER_APPROACH_CLEARANCE = 0.01

# Suction attach/detach, only meaningful once TOOL_CHANGE_TARGETS["suction"] is the currently-docked
# tool -- see teleop.py's tool-gating. Not colliding with J/B/P/C/O. S/H/R were tried first and
# confirmed live to double as Kit viewport hotkeys -- see docs/mefron-history.md.
SUCTION_ATTACH_KEY = "V"  # Vacuum on
SUCTION_DETACH_KEY = "L"  # reLease

# Per-object suction approach targets, same shape/purpose as GRASP_TARGETS but for the suction cup:
# "key" snaps target to approach_relationship, P looks up assembly_relationship for whichever
# object was last approached. Only meaningful while the suction tool is docked.
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
# Male half: a plain cylinder matching the Franka's own ISO 9409-1-50 wrist flange (see
# SUCTION_GRIPPER_USD's comment above for that Ø63mm reference), riding permanently on panda_hand.
# Female half: one per tool below, mating with the male coupler the same way every time. See
# docs/tool-changer.md for why a scripted FixedJoint was chosen over Isaac Sim's Robot Assembler
# extension or the SurfaceGripper schema.
TOOL_CHANGER_MALE_PRIM_NAME = "tool_changer_male"
TOOL_CHANGER_CYLINDER_RADIUS = 0.0315  # Ø63mm
TOOL_CHANGER_CYLINDER_HEIGHT = 0.02
# +Z half-height, not 0 -- UsdGeom.Cylinder is centered on its own local origin, so at z=0 half its
# height sits behind panda_hand's origin, overlapping panda_link8. This shifts it flush instead.
TOOL_CHANGER_MALE_LOCAL_POSITION = [0.0, 0.0, TOOL_CHANGER_CYLINDER_HEIGHT / 2]
TOOL_CHANGER_MALE_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

# ee_link's fixed pose relative to a tool's female-coupler frame once properly mated -- the whole
# point of a standardized coupler is this is the SAME for every tool, not measured per-tool.
# Reset to identity (2026-08-03): the previous hand-jogged value was measured against the
# pre-female-tool-head/pre-panda_link8-shifted-male-coupler setup, and just added a stale offset on
# top of both of today's changes. Visual-only placeholder for now -- re-derive by hand-jog (see
# docs/tool-changer.md) once the coupler/female-head geometry is final and docking accuracy matters.
# Only used for suction/screwdriver now -- the gripper docks panda_link8 directly to its own
# panda_hand instead, see TOOL_CHANGER_GRIPPER_HAND_JOINT_LOCAL_ORIENTATION_WXYZ below.
TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_POSITION = [0.0, 0.0, 0.0]
TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

# The gripper tool's panda_hand is the exact same stock mesh as the arm's own (both exported from
# franka_panda.urdf), so docking it wants that URDF's real panda_hand_joint offset (panda_link8 ->
# panda_hand: xyz="0 0 0", rpy="0 0 -0.785398163397") -- not a hand-jogged coupler approximation.
# This is that offset's inverse (translation stays zero), since it's expressed as body1's own local
# frame -- see dock_tool_to_wrist()'s gripper-specific branch.
TOOL_CHANGER_GRIPPER_HAND_JOINT_LOCAL_ORIENTATION_WXYZ = [0.9238795325112867, 0.0, 0.0, 0.3826834323650898]

# Relative hover clearance above a rack's dock pose the arm holds while aligning X/Y/orientation
# before descending to dock/undock -- relative to each dock_position, not a fixed world-Z constant
# like ASSEMBLY_LIFT_HEIGHT (see CLAUDE.md's open issue about that exact mistake).
TOOL_RACK_APPROACH_CLEARANCE = 0.15

# The real, GUI-baked rack (see feedback_static_scenery_baked_into_scene memory). Suction/
# screwdriver are baked as real children of this prim too (see "baked_tool_prim_path" below); a
# tool's rack_prim_path anchor stays in sync with wherever the baked tool actually sits, so moving
# the tool (or the rack) in the GUI doesn't need any code/constant changes.
TOOL_RACK_PRIM_PATH = "/World/tool_rack"

# One entry per dockable tool. All 3 now use "baked_tool_prim_path" (a prim already hand-placed in
# mefron.usd) rather than "asset" (a USD path robot.spawn_dockable_tool() would reference fresh) --
# the gripper switched over after a referenced GRIPPER_TOOL_VISUAL_ONLY_USD kept landing an
# imprecise/gapped dock; baking a real, hand-placed prim sidesteps whatever reference-composition
# quirk caused that. "female_coupler_parent_link_name" is left unset for all 3 tools: the gripper's
# asset used to be a real multi-link mini-articulation needing it (female_coupler had to attach
# under its specific base_link, not the tool root), but GRIPPER_TOOL_VISUAL_ONLY_USD strips that
# down to the same flat, single-rigid-body shape suction/screwdriver already use -- see
# docs/tool-changer.md for why (a real jointed mini-articulation rigidly bolted onto the wrist
# produced a genuine dynamic resonance no pose/damping fix could settle).
# female_coupler_local_* for all 3 are placeholders pending hand-jog confirmation against each
# asset's own root frame, same provenance as SUCTION_GRIPPER_LOCAL_*.
TOOL_CHANGE_TARGETS = {
    "gripper": {
        "key": "NUMPAD_1",
        # Baked directly into mefron.usd via the GUI (see feedback_static_scenery_baked_into_scene
        # memory), same pattern as suction/screwdriver below -- switched from the "asset" (live-
        # referenced) pattern after confirmed live that a referenced GRIPPER_TOOL_VISUAL_ONLY_USD
        # kept landing an imprecise/gapped dock; baking a real, hand-placed prim sidesteps whatever
        # reference-composition quirk caused that (same Metrics-Assembler-adjacent class of issue
        # noted for SUCTION_GRIPPER_LOCAL_SCALE). robot.spawn_dockable_tool() never
        # references/repositions this prim, only reads its live pose.
        "baked_tool_prim_path": "/World/gripper_tool_visual_only",
        "rack_prim_path": "/World/tool_rack_gripper",
        "female_coupler_local_position": [0.0, 0.0, 0.0],
        "female_coupler_local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    },
    "suction": {
        "key": "NUMPAD_2",
        # Baked directly into mefron.usd via the GUI (see feedback_static_scenery_baked_into_scene
        # memory) -- this is the real, hand-placed prim; robot.spawn_dockable_tool() never
        # references/repositions it, only reads its live pose. "rack_prim_path" below is a separate,
        # lightweight non-physics anchor Xform, synced to this prim's current world pose every run,
        # that park_tool_at_rack()'s joint uses as its static reference point.
        "baked_tool_prim_path": "/World/suction_gripper_with_tool_female",
        "rack_prim_path": "/World/tool_rack_suction",
        "female_coupler_local_position": [0.0, 0.0, 0.0],
        "female_coupler_local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    },
    "screwdriver": {
        "key": "NUMPAD_3",
        # Same baked-into-mefron.usd pattern as suction above.
        "baked_tool_prim_path": "/World/electric_screwdriver_with_tool_female",
        "rack_prim_path": "/World/tool_rack_screwdriver",
        "female_coupler_local_position": [0.0, 0.0, 0.0],
        "female_coupler_local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    },
}

# isaacsim.exp.full.kit's extra extensions over isaacsim.exp.base.python.kit, enabled only after
# the Franka is mounted (crashes the URDF importer if loaded earlier -- see
# docs/mefron-history.md). isaacsim.asset.gen.conveyor(.ui) is the one deliberate addition beyond
# that diff, needed by conveyor.setup_conveyor_belt_graph().
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
