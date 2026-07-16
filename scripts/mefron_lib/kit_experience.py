"""Deferred-enabling of the full experience's extra extensions, after both Frankas are mounted --
see robot.mount_franka()'s own docstring for the full diagnosis of why this has to happen in this
order (mounting a second Franka while these are already loaded crashes Kit's URDF importer plugin;
enabling them afterward doesn't). Needs omni.kit.app, so unlike kit_bootstrap.py this can only be
imported once a SimulationApp already exists.

NOT YET FULLY WORKING: enabling the full config.FULL_EXPERIENCE_EXTRA_EXTENSIONS list (~122 names)
at runtime avoids the crash, but was observed live to blank the viewport shortly after (right when
Kit logs "bad menu item" warnings for the newly-registered extensions' menus) -- excluding the
obvious renderer/viewport-bundle suspects didn't fix it, so the real cause is still unconfirmed
(possibly a window-layout reset triggered by enabling many window-owning extensions at once, not a
renderer conflict -- unverified, no display access to check visually). Trying a much smaller,
targeted set below instead of the full list, to narrow down what's actually needed vs. what's
triggering the blank-out.
"""

from __future__ import annotations

# Only what CLAUDE.md actually says the full experience is *for* -- the Physics
# debug-visualization menu -- rather than the full ~122-extension list, to shrink the blast radius
# while the viewport-blanking cause is still unconfirmed.
_MINIMAL_EXTENSIONS = [
    "omni.physx.bundle",
]


def enable_full_experience_extensions() -> None:
    """Enables a minimal extension set for the Physics debug-viz menu, called once right after both
    Frankas are mounted. Scaled back from the full config.FULL_EXPERIENCE_EXTRA_EXTENSIONS list --
    see this module's own docstring for why."""
    import omni.kit.app

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    failures = []
    for name in _MINIMAL_EXTENSIONS:
        if not ext_manager.set_extension_enabled_immediate(name, True):
            failures.append(name)
    if failures:
        print(f"[mefron_lib] WARNING: failed to enable extensions: {failures}", flush=True)
