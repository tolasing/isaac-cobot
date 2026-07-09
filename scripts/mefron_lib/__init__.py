"""Shared library backing the mefron family of scripts (mefron.py, mefron_gripper_probe.py,
mefron_grasp_editor_scene.py, franka_grasp_editor_scene.py, test_mefron_*_headless.py).

Deliberately empty: entry-point scripts must create SimulationApp and run
kit_bootstrap.preload_real_packaging() *before* importing config/grasp/robot/teleop (those import
omni/curobo at module level). If this file imported those submodules eagerly, `from mefron_lib import
kit_bootstrap` alone would pull them in too early. Import specific submodules directly instead.
"""
