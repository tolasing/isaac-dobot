"""Headless regression test for mefron_lib.teleop's run_teleop_loop(): fakes a mouse
drag by monkeypatching target.get_world_pose(), then asserts the robot moved.

Run: ${ISAACSIM_ROOT_PATH}/python.sh scripts/test_mefron_teleop_headless.py --headless
"""

from __future__ import annotations

import sys

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": _headless})

# Must run before any omni/curobo import -- see mefron_lib/kit_bootstrap.py's docstring.
from mefron_lib.kit_bootstrap import clear_stale_robot_configuration, preload_real_packaging  # noqa: E402

preload_real_packaging()

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from pxr import UsdPhysics  # noqa: E402
from mefron_lib import config, robot, teleop  # noqa: E402

# Both signs of the same offset, from the target's reachable starting pose -- a direction-dependent mount
# orientation bug (arm reaches smoothly on one side, jerks/reverses on the other) can pass a single-
# direction drag test while still being broken, which is exactly what happened here; shrunk from the
# Franka-era [0, 0.05, 0.05] the same way scripts/test_teleop_headless.py needed for the CR5's different
# reachable envelope, adjust further if either direction still exceeds it.
_DRAG_CASES = [
    ("positive", np.array([0.0, 0.02, 0.02])),
    ("negative", np.array([0.0, -0.02, -0.02])),
]
_MAX_ITERATIONS = 200
# Must clear run_teleop_loop's own init/settle frame counts first.
_SETTLE_CALLS = 40


def main() -> None:
    # Replicates mefron.main()'s fix; main() itself returns early in headless mode.
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    clear_stale_robot_configuration(config.MEFRON_CONFIGURATION_DIR)
    omni.usd.get_context().open_stage(str(config.MEFRON_USD))
    for _ in range(120):
        simulation_app.update()

    robot.mount_cr5()
    robot.apply_gripper_friction()

    print("[test_mefron_teleop_headless] warming up cuRobo motion_gen...", flush=True)
    motion_gen, robot_cfg = teleop.setup_motion_gen()

    target = teleop.build_teleop_target(robot_cfg)

    # Must exist before timeline.play(); run_teleop_loop() only defines it later.
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid() and not stage.GetPrimAtPath("/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    # A few frames so is_playing()/the physics view are actually live before
    # run_teleop_loop's own init/settle phase starts counting.
    for _ in range(5):
        simulation_app.update()

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    start_positions = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])
    start_position, start_orientation = target.get_world_pose()

    # run_teleop_loop() re-snaps the arm to default_config in its own init phase on every fresh call
    # (its past_pose/target_pose/idx_list are all local, rebuilt from scratch each time) -- so each case
    # below is an independent drag from retract, not a continuation of the previous case, and comparing
    # each case's end pose against the shared start_positions is a valid, apples-to-apples check.
    all_passed = True
    for label, offset in _DRAG_CASES:
        dragged_position = start_position + offset
        call_count = {"n": 0}

        def fake_get_world_pose(dragged_position=dragged_position, call_count=call_count):
            call_count["n"] += 1
            if call_count["n"] < _SETTLE_CALLS:
                return start_position, start_orientation
            return dragged_position, start_orientation

        target.get_world_pose = fake_get_world_pose

        teleop.run_teleop_loop(simulation_app, motion_gen, robot_cfg, target, max_iterations=_MAX_ITERATIONS)

        # Constructed only after run_teleop_loop's own SingleArticulation goes out of scope.
        verify_robot = SingleArticulation(prim_path=config.ROBOT_PRIM_PATH, name=f"verify_robot_{label}")
        verify_robot.initialize()
        idx_list = [verify_robot.get_dof_index(x) for x in j_names]
        end_positions = verify_robot.get_joint_positions(idx_list)
        max_delta = float(np.max(np.abs(end_positions - start_positions)))
        print(f"[test_mefron_teleop_headless] [{label}] max joint-position delta: {max_delta:.4f} rad", flush=True)
        if max_delta < 0.05:
            print(
                f"[test_mefron_teleop_headless] FAIL ({label}): robot did not move meaningfully in response to the simulated drag.",
                flush=True,
            )
            all_passed = False
        else:
            print(f"[test_mefron_teleop_headless] PASS ({label}): robot followed the simulated drag.", flush=True)
        del verify_robot  # must go out of scope before the next case's run_teleop_loop() builds its own

    if all_passed:
        print("[test_mefron_teleop_headless] OVERALL PASS: robot followed the simulated drag in both directions.", flush=True)
    else:
        print("[test_mefron_teleop_headless] OVERALL FAIL: see per-direction results above.", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
