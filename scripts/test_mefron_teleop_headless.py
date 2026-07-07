"""Headless verification that mefron.py's run_teleop_loop() actually drives
the mounted Franka via cuRobo's MotionGen, without hitting the
robot.initialize() crash this session repeatedly chased and fixed:
AttributeError: 'NoneType' object has no attribute 'create_articulation_view'.

Root cause (confirmed by reading isaacsim.core.simulation_manager's actual
source, not guessed): SingleArticulation.initialize() depends on
SimulationManager.get_physics_sim_view(), which only ever gets set via a
specific chain -- timeline PLAY event -> _warm_start() -> gated behind the
carb setting /app/player/playSimulations -> if true, initialize_physics() ->
dispatches PHYSICS_WARMUP -> _create_simulation_view() actually sets the
view. If that setting is off (a real toggle in the Play button's own toolbar
dropdown), timeline.is_playing() still correctly returns True, but the
simulation view never gets created -- no amount of Play-timing or settle
frames fixes it. mefron.py's own main() now forces this setting on; this
script does the same explicitly, since it doesn't call mefron.main() itself
(see below for why).

mefron.py's own main() only calls run_teleop_loop() in the interactive
(non-headless) path -- with --headless it builds the scene, prints status,
and closes before ever reaching the teleop loop (no Play button, no mouse to
drag headlessly). This script exercises run_teleop_loop() directly instead,
reusing mefron.py's own functions as a library rather than calling main()
wholesale (so this script controls play-triggering itself, the same
established pattern test_teleop_headless.py already uses for build_scene.py):
faking a mouse-drag by monkeypatching target.get_world_pose() to hold the
target's real starting pose for enough frames to clear run_teleop_loop's own
init/settle phase, then jump to a second, nearby pose -- indistinguishable
from a real drag.

Same two gotchas test_teleop_headless.py's own docstring documents for
build_scene.py, both confirmed to apply identically here:

  - run_teleop_loop() references a module-level `simulation_app` global
    inside mefron's own namespace, only ever set there under
    `if __name__ == "__main__"` -- this script must assign
    `mefron.simulation_app` explicitly before calling run_teleop_loop().
  - run_teleop_loop() only defines /physicsScene (or reuses /PhysicsScene)
    at its own top, right before its while loop -- too late if this script
    calls timeline.play() first. Defined here explicitly before playing,
    matching run_teleop_loop's own required ordering.

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/test_mefron_teleop_headless.py --headless
"""

from __future__ import annotations

import sys

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": _headless})

import carb.settings  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

import mefron  # noqa: E402

# Small enough to stay within reach of the target's own starting pose (the
# robot's own retract-config end-effector pose, guaranteed reachable).
_DRAG_OFFSET = np.array([0.0, 0.05, 0.05])
_MAX_ITERATIONS = 200
# Must clear mefron._ROBOT_INIT_SETTLE_FRAMES (5) + _TELEOP_INIT_FRAMES (10)
# + _TELEOP_SETTLE_FRAMES (20) before the simulated drag lands, so
# run_teleop_loop's debounce logic sees a genuinely static target first.
_SETTLE_CALLS = 40


def main() -> None:
    if __name__ == "__main__":
        mefron.simulation_app = simulation_app

    # Same fix mefron.main() itself now applies -- replicated here since this
    # script calls mefron's individual functions directly rather than main()
    # itself (main() would also `return` early in the headless path before
    # ever reaching run_teleop_loop(), which is the whole thing being tested).
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    mefron.clear_stale_robot_configuration()
    omni.usd.get_context().open_stage(str(mefron.MEFRON_USD))
    for _ in range(120):
        simulation_app.update()

    mefron.mount_franka()
    mefron.apply_gripper_friction()
    mefron.stiffen_gripper_drive()

    print("[test_mefron_teleop_headless] warming up cuRobo motion_gen...", flush=True)
    motion_gen, robot_cfg = mefron.setup_motion_gen()

    target = mefron.build_teleop_target(robot_cfg)

    # Must exist before the timeline plays -- run_teleop_loop() defines this
    # itself too, but only at its own top, too late here since we call
    # timeline.play() before run_teleop_loop() even starts.
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
    dragged_position = start_position + _DRAG_OFFSET

    call_count = {"n": 0}

    def fake_get_world_pose():
        call_count["n"] += 1
        if call_count["n"] < _SETTLE_CALLS:
            return start_position, start_orientation
        return dragged_position, start_orientation

    target.get_world_pose = fake_get_world_pose

    mefron.run_teleop_loop(motion_gen, robot_cfg, target, max_iterations=_MAX_ITERATIONS)

    # Only constructed now, after run_teleop_loop's own internal
    # SingleArticulation has gone out of scope (see test_teleop_headless.py's
    # own docstring for why holding two at once breaks PhysX's tensor view).
    robot = SingleArticulation(prim_path=mefron.ROBOT_PRIM_PATH, name="verify_robot")
    robot.initialize()
    idx_list = [robot.get_dof_index(x) for x in j_names]
    end_positions = robot.get_joint_positions(idx_list)
    max_delta = float(np.max(np.abs(end_positions - start_positions)))
    print(f"[test_mefron_teleop_headless] max joint-position delta: {max_delta:.4f} rad", flush=True)
    if max_delta < 0.05:
        print(
            "[test_mefron_teleop_headless] FAIL: robot did not move meaningfully in response to the simulated drag.",
            flush=True,
        )
    else:
        print("[test_mefron_teleop_headless] PASS: robot followed the simulated drag.", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
