"""Enables the full experience's extra extensions AFTER the Franka is mounted -- see
robot.mount_franka(). Needs a live SimulationApp, unlike kit_bootstrap.py."""

from __future__ import annotations

import carb.settings

from . import config


def enable_full_experience_extensions() -> None:
    """Enables config.FULL_EXPERIENCE_EXTRA_EXTENSIONS (~122 names), called once right after both
    Frankas are mounted. See this module's own docstring for the regression this works around."""
    import omni.kit.app

    # Must happen before isaacsim.app.setup enables below -- set unconditionally since dict/list
    # enable order isn't guaranteed.
    carb.settings.get_settings().set_bool("/isaac/startup/create_new_stage", False)

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    failures = []
    for name in config.FULL_EXPERIENCE_EXTRA_EXTENSIONS:
        if not ext_manager.set_extension_enabled_immediate(name, True):
            failures.append(name)
    if failures:
        print(f"[mefron_lib] WARNING: failed to enable extensions: {failures}", flush=True)
