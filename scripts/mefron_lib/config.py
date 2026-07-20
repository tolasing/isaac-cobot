"""Constants shared across the mefron family of scripts: paths, mount pose, gripper/friction/drive
tuning, and the derived grasp/assembly relative poses. Pure data -- no omni/curobo imports, safe to
import at any point.
"""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MEFRON_USD = REPO_ROOT / "assets" / "mefron" / "factory floor" / "mefron.usd"
# Disk-persisted "Robot Description" dir the URDF importer writes on every import into this
# file-backed stage; see kit_bootstrap.clear_stale_robot_configuration().
MEFRON_CONFIGURATION_DIR = MEFRON_USD.parent / "configuration"

ROBOT_PRIM_PATH = "/World/Franka"
TARGET_PRIM_PATH = "/World/target"
# UR10 mount pedestal (Isaac asset: Props/Mounts/ur10_mount.usd) the Franka mounts on -- replaced
# the original SEKTION cabinet plate (itself a replacement for Pedestal_plates/Cube_05) once the
# packing table was swapped for a conveyor line. MOUNT_POSITION below is taken directly from this
# prim's own authored xformOp:translate (first-pass -- its payload can't be inspected offline to
# confirm whether its local origin sits at the top mounting flange, same as every other mount
# position in this file, pending the user's own GUI check).
# No /Factory prefix: mefron.py opens mefron.usd directly, one level shallower than build_scene_mefron.py's reference.
MOUNT_PLATE_PRIM_PATH = "/World/ur10_mount"
MOUNT_POSITION = [2.625260866887235, -4.7019853821770115, 0.8093334035921127]
MOUNT_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

FRANKA_URDF_RELATIVE_PATH = "robot/franka_description/franka_panda.urdf"
FRANKA_DRIVE_STRENGTH = 1047.19751
FRANKA_DRIVE_DAMPING = 52.35988
FRANKA_MOTION_GEN_ROBOT_CFG = "franka.yml"

# Nearby scene objects within the Franka's reach envelope, not the whole /World/Factory backdrop
# (which would add scan time for no benefit). Includes both robots' own prim paths so each arm's
# cuRobo world treats the OTHER arm as a real collision obstacle -- teleop.get_obstacles() excludes
# an arm from its own obstacle world via ignore_substring, not by leaving it out of this list.
# packing_table/packing_table_01 (the previous entries here) no longer exist on the stage -- the
# packing table was replaced by a conveyor line (assets/mefron/factory floor/Conveyors/), and both
# mount pedestals are included below since they now sit close enough together (~0.65m apart) that
# each arm needs to see the OTHER arm's own pedestal as an obstacle, not just the other arm itself.
# Deliberately NOT including the new ConveyorBelt_*/container_h20 prims here: confirmed live that
# adding them (even just the 5 conveyor + 4 container top-level Xforms) made
# get_obstacles_from_stage()'s mesh-collision-world construction (past "Creating new Mesh cache: 95"
# -- cuRobo recurses each Xform into every child mesh, so 9 top-level prims expanded into ~95
# individual meshes) hang for over an hour with zero forward progress and steady CPU/GPU load --
# real industrial conveyor-line CAD assemblies (rollers, frame, guards, motor housing, etc., see
# the 13-113MB per-file sizes under Conveyors/) are far more geometrically complex than the single
# packing_table prop they replaced, well past what cuRobo's mesh-based collision checker can
# preprocess in reasonable time. Getting conveyor/container collision-awareness working needs
# either primitive/cuboid obstacle approximations instead of the raw CAD meshes, or narrowing to
# specific lightweight sub-prims -- not attempted here; see CLAUDE.md's open-issues list.
OBSTACLE_PRIM_PATHS = [
    "/World/main_holder_jig",  # must match ROBOT_2_PRIM_PATH below (defined later in this file)
]

# Loop-timing constants for teleop.run_teleop_loop(), ported from build_scene.py.
_TELEOP_INIT_FRAMES = 10
_TELEOP_SETTLE_FRAMES = 20
_TELEOP_OBSTACLE_RESCAN_INTERVAL = 1000
_POSE_DELTA_THRESHOLD = 1.0e-3
_STATIC_JOINT_VELOCITY_THRESHOLD = 0.5

# World-frame Z height P holds while it aligns X/Y/orientation to the assembly-placement pose, before
# dropping straight down in Z to the actual placement pose -- a direct point-to-point plan_single to
# the final pose was clipping/dragging the carried object through the table and nearby props.
ASSEMBLY_LIFT_HEIGHT = 1.21

# Frames to wait after is_playing() first turns True before constructing SingleArticulation --
# PhysX needs a few real steps before its simulation view is actually ready.
_ROBOT_INIT_SETTLE_FRAMES = 5

# Uniformly re-times the already-planned trajectory to play out slower; does not change the
# optimizer's relative speed profile or planning success. See _TELEOP_VELOCITY_SCALE for capping actual limits.
_TELEOP_TIME_DILATION_FACTOR = 0.3

# Caps velocity/acceleration limits used during trajectory optimization. cuRobo treats scale <= 0.25 as a
# special case: it swaps in finetune_trajopt_slow.yml and raises maximum_trajectory_dt to compensate; 0.2 stays under that threshold.
_TELEOP_VELOCITY_SCALE = 0.4
_TELEOP_ACCELERATION_SCALE = 0.4

# Grasp-physics constants, ported from build_scene_mefron.py's apply_gripper_friction()/stiffen_gripper_drive().
GRIPPER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]
# Narrowed from the full 0-0.04m stroke to bracket finger_print_scanner's actual 12mm grip width
# (measured via UsdGeom.BBoxCache local bound) -- the full stroke let one finger contact and drag the
# part sideways well before the other closed the remaining distance. CLOSED is the symmetric half-width
# (6mm/side); OPEN adds a 4mm/side clearance margin for approach.
GRIPPER_OPEN_POSITION = 0.010
GRIPPER_CLOSED_POSITION = 0.000
# Rate (m/s) the commanded gripper position is ramped toward open/closed, instead of stepping instantly --
# avoids a snap shut under the high drive stiffness. 0.02 m/s takes ~0.2s for the now-narrowed 0.004m travel.
GRIPPER_CLOSE_SPEED = 0.02
GRIPPER_FRICTION_MATERIAL_PATH = "/World/GripperFrictionMaterial"
GRIPPER_STATIC_FRICTION = 1.5
GRIPPER_DYNAMIC_FRICTION = 1.5
GRIPPER_FINGER_LINK_NAMES = ["panda_leftfinger", "panda_rightfinger"]
GRIPPER_DRIVE_STIFFNESS = 10000.0
GRIPPER_DRIVE_DAMPING = 200.0
HIGH_FRICTION_PRIM_PATHS = ["/World/finger_print_scanner"]

# Grasp Editor-exported grasp-approach poses + per-object finger widths, keyed by object name and
# wired to a keyboard key in teleop.build_gripper_keyboard_control(). "key" is a carb.input.KeyboardInput
# attribute name (resolved via getattr in teleop.py, since this module stays free of omni/curobo imports).
# See grasp.compute_grasp_approach_pose_from_file()/compute_grasp_finger_widths_from_file().
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
    "pcb_assembly": {
        "key": "K",
        "yaml_path": REPO_ROOT / "assets" / "PCB_assembly.yaml",
        "grasp_name": "grasp_0",
        "part_prim_path": "/World/PCB_Assembly_color_fixed",
    },
}

# T_H_S: finger_print_scanner's / backpanel_support's pose expressed in main_holder's own local frame
# at the correctly assembled position, derived via grasp.compute_relative_pose() from each part's live
# world pose (no reparenting needed -- see docs/grasp-and-assembly-offsets.md).
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
        "local_position": [0.023463946069672652, -0.013916167562435, 0.006499950486007643],
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
    # Not a part-on-mount relationship like the other 3 -- reuses the same generic "dependent pose
    # relative to a reference frame" math (grasp.compute_part_target_pose() never reads
    # part_prim_path on this call path). mount_prim_path=part_prim_path=screen on purpose: this is
    # the suction gripper's own *approach target*, expressed in screen's live frame, not a carried
    # part's mount pose. Re-derived live 2026-07-17 by hand-jogging target2 until the cup tip sat
    # dead-center on screen's top face (tip at [0.0007, -0.0008, -0.0049] in screen's frame) and
    # reading back target2-wrt-screen via grasp.compute_relative_pose() -- same methodology as
    # docs/grasp-and-assembly-offsets.md. This supersedes the first-pass
    # scripts/mefron_screen_approach_probe.py bbox-top derivation, which was wrong twice over: it
    # ignored the 100mm cup length (put the *ee* ~12mm from screen's origin), and it had local z
    # positive-side-up -- screen's own frame is flipped ~180deg about X (local +Z points DOWN in
    # world), so "above the screen" is NEGATIVE local z. Measured contact pose was
    # [0.00028, -0.00024, -0.10558]; the z below has SURFACE_GRIPPER_APPROACH_CLEARANCE (0.01) of
    # hover baked in (more negative = higher in world), so S snaps to a 10mm hover from which V's
    # SurfaceGripper (maxGripDistance 0.03) can still reach and attach.
    "suction_gripper_approach_on_screen": {
        "part_prim_path": "/World/screen",
        "mount_prim_path": "/World/screen",
        "local_position": [0.00028, -0.00024, -0.11558],
        "local_orientation_wxyz": [0.382330, -0.000471, -0.000099, 0.924026],
    },
}

# --- Second arm (see docs/mefron-history.md / the "second arm" plan) -------------------------
# `/World/ur10_mount_01` is the second UR10 mount pedestal the user placed by hand in the GUI for
# the second Franka to stand on -- replaced an earlier second SEKTION cabinet
# (`sektion_cabinet_instanceable_01`, still present on the stage but now ~2m away, out of reach)
# once the packing table became a conveyor line. MOUNT_2_POSITION is taken directly from this
# prim's own authored xformOp:translate, same first-pass caveat as MOUNT_POSITION above.
# MOUNT_2_ORIENTATION_WXYZ keeps the previously-confirmed 180deg-about-Z orientation
# (wxyz [0, 0, 0, 1]) as a starting point -- unrelated to the mount swap, still pending the user's
# own GUI check like every other pose constant here.
ROBOT_2_PRIM_PATH = "/World/Franka2"
MOUNT_2_POSITION = [3.796979317755996, -4.7095468668675435, 0.78]
MOUNT_2_ORIENTATION_WXYZ = [0.0, 0.0, 0.0, 1.0]
TARGET_2_PRIM_PATH = "/World/target2"

ROBOT_3_PRIM_PATH = "/World/Franka3"
MOUNT_3_POSITION = [3.85262, -3.55485, 0.78]
MOUNT_3_ORIENTATION_WXYZ = [0.0, 0.0, 0.0, 1.0]
TARGET_3_PRIM_PATH = "/World/target3"

# ConveyorBelt_A24. Driven directly by teleop.ConveyorControl via PhysX's own
# PhysxSurfaceVelocityAPI/surfaceVelocity attribute on CONVEYOR_BELT_PRIM_PATH -- NOT through an
# OmniGraph/IsaacConveyor node. That was tried first (Create > Isaac Sim > Conveyor); abandoned
# 2026-07-20 after repeated live-confirmed failures: the node's own "Enabled" input ended up
# unchecked (silently means the node never computes at all), and separately, deleting the graph
# entirely left a nonzero surfaceVelocity value permanently orphaned on Belt's own USD spec, since
# that attribute is authored directly on the rigid body and isn't cleared by removing whatever
# graph/node used to drive it -- only removing the PhysxSurfaceVelocityAPI schema itself stopped
# PhysX from continuing to apply it. Driving the attribute directly from this one Python class avoids
# all of that: nothing is hand-authored in mefron.usd for it to depend on.
CONVEYOR_BELT_PRIM_PATH = "/World/ConveyorBelt_A24/Belt"
# Local-frame direction (Belt's own axes, not world) confirmed live 2026-07-20 to move the belt
# forward at this magnitude -- re-verify once ConveyorControl is actually driving it, though: this
# axis flipped between local X and Y more than once across earlier graph deletions/recreations, so
# treat this as a starting point, not a settled fact.
CONVEYOR_LOCAL_VELOCITY_DIRECTION = [1.0, 0.0, 0.0]
CONVEYOR_SPEED = 1.0
MAIN_HOLDER_JIG_PRIM_PATH = "/World/main_holder_jig"
CONVEYOR_JIG_FORWARD_Y = -3.6
CONVEYOR_JIG_BACKWARD_Y = -4.7
# Number-row "1", not numpad -- carb.input.KeyboardInput.KEY_1.
CONVEYOR_TOGGLE_KEY = "KEY_1"


# Suction gripper end-effector, added onto arm 2 only (arm 1 keeps the Franka's stock parallel-jaw
# hand). Custom-designed in SolidWorks for this Franka flange directly (Ø63mm mount face = Franka's
# own ISO 9409-1-50 flange OD, Ø50mm suction tip -- see robots/franka_panda/Props/ for the exported
# asset), superseding the earlier borrowed robots/ur10_suction/short_gripper.usd (still kept, see its
# own SOURCE.md, but no longer referenced here). Referenced under panda_hand, cuRobo's own franka.yml
# ee_link (see grasp.py's docstring), so it rides along rigidly with the gripper frame.
SUCTION_GRIPPER_USD = REPO_ROOT / "robots" / "franka_panda" / "Props" / "suction gripper.usd"
SUCTION_GRIPPER_PRIM_NAME = "suction_gripper"
# Unlike the borrowed UR10 asset (which needed a solved offset+rotation because its internal "wrist"
# reference frame didn't line up with panda_hand's own axes), this asset's own root IS its mount face
# already: verified live that both wrapper Xforms above the Mesh are identity, the Ø63mm base ring
# sits exactly at local (0, 0, 0), and +Z runs base->tip (0 to 0.1m) -- matching panda_hand's own
# convention of origin = flange point, +Z toward the fingers (confirmed via panda_finger_joint1/2's
# localPos0 = (0, 0, 0.0584) on the panda_hand body). So the asset's root is already coincident with
# panda_hand's frame with no correction needed.
SUCTION_GRIPPER_LOCAL_POSITION = [0.0, 0.0, 0.0]
SUCTION_GRIPPER_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]

# Electric-screwdriver end-effector, added onto arm 3 only. Same panda_hand-child mounting pattern as
# SUCTION_GRIPPER_* above (see robot.attach_screwdriver_gripper()).
SCREWDRIVER_USD = REPO_ROOT / "robots" / "grippers" / "electric_screwdriver.usd"
SCREWDRIVER_PRIM_NAME = "electric_screwdriver"
# The asset's own root Xform is correctly scaled (0.001) inside electric_screwdriver.usd itself now --
# this forces attach_screwdriver_gripper()'s own reference-holding wrapper prim to identity scale so
# nothing in robot.py compounds an extra scale factor on top of that.
SCREWDRIVER_LOCAL_SCALE = [1.0, 1.0, 1.0]
SCREWDRIVER_LOCAL_POSITION = [0.0, 0.0, 0.0]
# Derived from the user's live-jogged GUI pose (Orient X/Y/Z = 90/45/90 degrees, USD's rotateXYZ
# convention: composed as Rx * Ry * Rz, i.e. Z applied first, then Y, then X), converted to wxyz.
SCREWDRIVER_LOCAL_ORIENTATION_WXYZ = [0.2705980501, 0.6532814824, -0.2705980501, 0.6532814824]

# Real isaacsim.robot.schema/isaacsim.robot.surface_gripper physics, distinct from the SUCTION_GRIPPER_*
# constants above (which are pure visual geometry with zero physics of its own). Kept deliberately
# minimal -- no hand-authored PhysicsLimitAPI/PhysicsDriveAPI compliance tuning, no touching any other
# prim's existing physics setup -- just the bare structural minimum the extension itself requires: one
# joint tagged as an attachment point, and the IsaacSurfaceGripper bookkeeping prim pointing at it.
SURFACE_GRIPPER_JOINT_PRIM_NAME = "SurfaceGripperJoint"
SURFACE_GRIPPER_PRIM_NAME = "SurfaceGripper"
# Joint's frame on panda_hand's side: coincident with the cup's physical tip, 0.1m out along
# panda_hand's own +Z -- same base->tip convention as SUCTION_GRIPPER_LOCAL_POSITION/ORIENTATION_WXYZ
# above, just offset to the tip instead of the base.
SURFACE_GRIPPER_LOCAL_POSITION = [0.0, 0.0, 0.1]
SURFACE_GRIPPER_LOCAL_ORIENTATION_WXYZ = [1.0, 0.0, 0.0, 0.0]
# isaac:maxGripDistance -- how far the attachment point searches for something to grab. Schema
# default is 0.01m; widened slightly for first-pass teleop-approach tolerance.
SURFACE_GRIPPER_MAX_GRIP_DISTANCE = 0.03
# Hover clearance for the *teleop approach pose* (distinct from the joint's own search radius
# above) -- how far above the measured contact pose the S key's approach target sits. Baked into
# ASSEMBLY_RELATIONSHIPS["suction_gripper_approach_on_screen"]'s local z (it can't be referenced
# there directly -- that dict literal is defined earlier in this file), so changing this value
# alone does nothing: re-bake the relationship's z too.
SURFACE_GRIPPER_APPROACH_CLEARANCE = 0.01

# Arm 2 keys -- none collide with arm 1's J/B/K/P/C/O or dormant mefron2.py's G. S, H, and R were
# all tried first (approach/screen and release, respectively) and confirmed live 2026-07-17 to
# double as Kit's own viewport hotkeys, firing that Kit UI/action alongside our handler -- N and L
# instead, neither a Kit viewport manipulator hotkey.
SUCTION_APPROACH_KEY = "N"  # sNap arm 2's target to the screen-approach pose
SUCTION_ATTACH_KEY = "V"  # Vacuum on
SUCTION_DETACH_KEY = "L"  # reLease

SCREEN_PRIM_PATH = "/World/screen"

# Mostly the extension set isaacsim.exp.full.kit adds on top of isaacsim.exp.base.python.kit (diffed
# directly from both .kit files' [dependencies] tables). Mounting a second Franka (a second native
# URDF import in one process) crashes Kit's isaacsim.asset.importer.urdf plugin if these are already
# loaded at import time -- confirmed live -- but enabling them AFTER both Frankas are mounted
# reproduces the identical final feature set with zero crash (confirmed live: all 122 enable cleanly,
# zero failures, matching what mefron.py needs for its Physics debug-viz menu). See
# robot.mount_franka()'s own docstring and kit_experience.enable_full_experience_extensions().
#
# isaacsim.asset.gen.conveyor/.ui are the one deliberate addition beyond that diffed set -- neither is
# a dependency of either experience. Not required by teleop.ConveyorControl itself anymore (that
# class now drives ConveyorBelt_A24's PhysxSurfaceVelocityAPI directly, see
# CONVEYOR_BELT_PRIM_PATH's own comment for why the OmniGraph/IsaacConveyor node approach was
# abandoned), but kept here since .ui's "Create > Isaac Sim > Conveyor" menu command is still useful
# for building any of the other ConveyorBelt_A01-A49 assets the same way.
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
