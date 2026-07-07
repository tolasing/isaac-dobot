"""Builds the same mefron scene + mounted Franka as mefron.py, but skips
cuRobo's motion_gen warmup and run_teleop_loop() entirely -- leaves the robot
idle, with no SingleArticulation actively constructed/driving it, so external
tools (the Grasp Editor, specifically) can freely construct their own
exclusive SingleArticulation on it without contention.

Why this exists: mefron.py's own run_teleop_loop() holds a live
SingleArticulation on /World/Franka for as long as it runs, and that loop's
own `while simulation_app.is_running(): simulation_app.update()` is what
keeps the whole Kit application -- including any other panel, like the Grasp
Editor -- responsive at all. There's no way to "pause" just the teleop loop's
own articulation ownership without stopping the whole process. Confirmed
live: the moment the Grasp Editor's own internal code (isaacsim.robot_setup.
grasp_editor's _on_finished_selection_frame) calls timeline.play() to
construct its own SingleArticulation on the same prim, mefron.py's
still-running loop reacts to that same event and races to reconstruct its
own -- one of the two ends up with a broken (empty subtree scan) Articulation.
This script sidesteps the whole problem by simply never constructing an
articulation of its own in the first place, and never touching cuRobo at all
(the Grasp Editor has no cuRobo dependency, so there's nothing to warm up).

Reuses mefron.py's own functions as a library rather than duplicating scene-
build logic (same established pattern as test_mefron_teleop_headless.py).

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/mefron_grasp_editor_scene.py
"""

from __future__ import annotations

import os
import sys

from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # Full experience for interactive runs, matching mefron.py's own choice --
    # the Grasp Editor panel is a full-experience-only UI extension.
    experience = "" if _headless else f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    simulation_app = SimulationApp({"headless": _headless}, experience=experience)

import carb.settings  # noqa: E402
import omni.usd  # noqa: E402

import mefron  # noqa: E402 -- importing (not running as __main__) also runs
# mefron.py's own top-level packaging-preload workaround unconditionally, so
# it doesn't need to be duplicated here.


def main() -> None:
    mefron.simulation_app = simulation_app

    # Same fix mefron.main() applies -- see mefron.py's own comment on this
    # setting for why it matters (SingleArticulation.initialize() depends on
    # it indirectly through SimulationManager's own warm-start chain).
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    mefron.clear_stale_robot_configuration()
    omni.usd.get_context().open_stage(str(mefron.MEFRON_USD))
    for _ in range(120):
        simulation_app.update()

    mefron.mount_franka()
    mefron.apply_gripper_friction()
    mefron.stiffen_gripper_drive()

    stage = omni.usd.get_context().get_stage()
    for status_path in [mefron.ROBOT_PRIM_PATH, *mefron.OBSTACLE_PRIM_PATHS]:
        prim = stage.GetPrimAtPath(status_path)
        print(f"[mefron_grasp_editor_scene] {status_path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

    print(
        "[mefron_grasp_editor_scene] Scene ready -- no cuRobo, no teleop loop, robot idle and uncontested.",
        flush=True,
    )
    print(
        "[mefron_grasp_editor_scene] Open the Grasp Editor now -- it can freely claim /World/Franka.",
        flush=True,
    )

    if _headless:
        simulation_app.close()
        return

    while simulation_app.is_running():
        simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
