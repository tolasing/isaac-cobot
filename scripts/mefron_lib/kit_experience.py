"""Deferred-enabling of the full experience's extra extensions, after both Frankas are mounted --
see robot.mount_franka()'s own docstring for the full diagnosis of why this has to happen in this
order (mounting a second Franka while these are already loaded crashes Kit's URDF importer plugin;
enabling them afterward doesn't). Needs omni.kit.app, so unlike kit_bootstrap.py this can only be
imported once a SimulationApp already exists.

A prior version of this function scaled back to a small hand-picked subset (just omni.physx.bundle,
for the Physics debug-viz menu) after observing the full config.FULL_EXPERIENCE_EXTRA_EXTENSIONS list
(~122 names) blank the viewport shortly after enabling. That subset traded away real functionality
piecemeal and unpredictably (Script Editor, then isaacsim.robot_setup.grasp_editor's
import_grasps_from_file() needed manual Window->Extensions enabling or crashed outright with
ModuleNotFoundError) -- so this goes back to enabling the full list.

Root cause of the "blanking" (confirmed by reading isaacsim.app.setup's own source, not guessed):
it isn't a rendering/menu issue at all -- isaacsim.app.setup.CreateSetupExtension.on_startup() reads
the carb setting /isaac/startup/create_new_stage, and if true (its own extension.toml default),
schedules an async task a few frames out that unconditionally opens a brand-new stage, discarding
whatever's loaded. Since this function runs after mefron.usd is already open and both Frankas are
mounted, that fires late and wipes the running scene -- not "too many extensions at once". Forcing
the setting off before enabling fixes it without dropping anything from the full list.
"""

from __future__ import annotations

import carb.settings

from . import config


def enable_full_experience_extensions() -> None:
    """Enables config.FULL_EXPERIENCE_EXTRA_EXTENSIONS (~122 names), called once right after both
    Frankas are mounted. See this module's own docstring for the stage-swap regression this
    previously hit and the setting that fixes it."""
    import omni.kit.app

    # Must happen before isaacsim.app.setup enables below -- see this module's docstring. Set
    # unconditionally rather than relying on list order, since dict/list enable order isn't guaranteed.
    carb.settings.get_settings().set_bool("/isaac/startup/create_new_stage", False)

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    failures = []
    for name in config.FULL_EXPERIENCE_EXTRA_EXTENSIONS:
        if not ext_manager.set_extension_enabled_immediate(name, True):
            failures.append(name)
    if failures:
        print(f"[mefron_lib] WARNING: failed to enable extensions: {failures}", flush=True)
