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
FRANKA_DRIVE_DAMPING = 52.35988
FRANKA_MOTION_GEN_ROBOT_CFG = "franka.yml"

# Reach-envelope obstacles for both arms, not the whole /World/Factory backdrop. Excludes the
# conveyor/container prims -- hangs cuRobo's mesh-collision-world construction; see CLAUDE.md's
# open issues.
OBSTACLE_PRIM_PATHS = [
    "/World/main_holder_jig",
]

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
    # pcb_assembly (K) retired 2026-07-22 for arm 2's suction cup instead -- redundant-branch
    # twisting on approach, root cause + ruled-out fix in docs/mefron-history.md.
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
    "pcb_assembly_on_backpanel_support": {
        "part_prim_path": "/World/PCB_Assembly_color_fixed",
        "mount_prim_path": "/World/backpanel_support",
        "local_position": [-0.0015799999237060547, -0.02138996124267578, 0.008999995231628418],
        "local_orientation_wxyz": [0.7063401483274144, 0.0, 0.0, 0.7078725837753616],
    },
    # Suction gripper's own approach target expressed in screen's live frame, not a carried part's
    # mount pose (mount_prim_path=part_prim_path=screen on purpose). Derivation + measured values:
    # docs/mefron-history.md.
    "suction_gripper_approach_on_screen": {
        "part_prim_path": "/World/screen",
        "mount_prim_path": "/World/screen",
        "local_position": [0.00028, -0.00024, -0.11558],
        "local_orientation_wxyz": [0.382330, -0.000471, -0.000099, 0.924026],
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
# URDF -- one of the 3 dockable ATC tools. Referenced, not live-imported: confirmed live that
# importing a second robot via the URDF importer into mefron.usd's own stage corrupts the main
# arm's own link structure through a shared "Robot Description" cache, even at a fully distinct
# prim path -- see docs/tool-changer.md's gotcha 6.
GRIPPER_TOOL_HAND_ONLY_USD = REPO_ROOT / "robots" / "franka_panda" / "Props" / "gripper_tool_hand_only.usd"

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
TOOL_CHANGER_MALE_LOCAL_POSITION = [0.0, 0.0, 0.0]
TOOL_CHANGER_MALE_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

# ee_link's fixed pose relative to a tool's female-coupler frame once properly mated -- the whole
# point of a standardized coupler is this is the SAME for every tool, not measured per-tool.
# Placeholder (coupler-height standoff only) pending hand-jog confirmation once the coupler/female
# geometry exists live -- see docs/tool-changer.md's open issues.
TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_POSITION = [0.0, 0.0, -TOOL_CHANGER_CYLINDER_HEIGHT]
TOOL_CHANGER_DOCKED_EE_LINK_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

# Relative hover clearance above a rack's dock pose the arm holds while aligning X/Y/orientation
# before descending to dock/undock -- relative to each dock_position, not a fixed world-Z constant
# like ASSEMBLY_LIFT_HEIGHT (see CLAUDE.md's open issue about that exact mistake).
TOOL_RACK_APPROACH_CLEARANCE = 0.15

# The real, GUI-baked rack (see feedback_static_scenery_baked_into_scene memory). Suction/
# screwdriver are baked as real children of this prim too (see "baked_tool_prim_path" below); a
# tool's rack_prim_path anchor stays in sync with wherever the baked tool actually sits, so moving
# the tool (or the rack) in the GUI doesn't need any code/constant changes.
TOOL_RACK_PRIM_PATH = "/World/tool_rack"

# One entry per dockable tool. Two placement styles: "asset" (a USD path robot.spawn_dockable_tool()
# references fresh as a child of the rack anchor prim -- currently just the gripper) vs
# "baked_tool_prim_path" (a prim already hand-placed in mefron.usd -- currently suction/screwdriver;
# see docs/tool-changer.md's open issues for why). "female_coupler_parent_link_name" is only set for
# the gripper: its asset is a multi-link mini-articulation (actuated fingers, not a static prop like
# the other two), so its rack-anchor child is just an organizing Xform over its real rigid-body
# links -- female_coupler must attach under that specific link instead (see
# robot._female_coupler_parent_prim_path()), not left unset/None like the other two tools.
# dock_position/orientation for the gripper are placeholders pending user GUI placement, same
# provenance as MOUNT_POSITION; female_coupler_local_* for all 3 are placeholders pending hand-jog
# confirmation against each asset's own root frame, same provenance as SUCTION_GRIPPER_LOCAL_*.
TOOL_CHANGE_TARGETS = {
    "gripper": {
        "key": "NUMPAD_1",
        "asset": GRIPPER_TOOL_HAND_ONLY_USD,
        "female_coupler_parent_link_name": "base_link",
        "rack_prim_path": "/World/tool_rack_gripper",
        "dock_position": [2.9, -4.4, 0.85],
        "dock_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
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
