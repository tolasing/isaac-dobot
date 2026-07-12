"""Headless verification that dragging the teleop target actually drives the
mounted robot via cuRobo's MotionGen.

scripts/build_scene.py's own main() only calls run_teleop_loop() in the
interactive (non---headless) path -- with --headless it builds the scene,
prints status, and closes before ever reaching the teleop loop (there is no
Play button and no mouse to drag headlessly). This script exercises
run_teleop_loop() directly instead, faking a mouse-drag by monkeypatching
target.get_world_pose(): it returns the target's real starting pose for
enough frames to clear run_teleop_loop's own init/settle phase (so the
debounce logic sees a genuinely static target, exactly like
CLAUDE.md's prior ad-hoc verification of this same chain), then jumps to a
second, nearby pose -- what run_teleop_loop sees is indistinguishable from a
real drag.

Reuses build_scene.py's own functions as a library rather than duplicating
scene-build logic. Two gotchas specific to that reuse:

  - run_teleop_loop() (like build_scene.main()) references a module-level
    `simulation_app` global inside build_scene's own namespace, only ever
    set there under `if __name__ == "__main__"` -- since this script is the
    one creating the real SimulationApp when run standalone, it must assign
    `build_scene.simulation_app` explicitly before calling
    run_teleop_loop(), or that name lookup fails with NameError.
  - run_teleop_loop() only defines the `/physicsScene` prim (via
    `UsdPhysics.Scene.Define()`) at its own top, right before its while
    loop -- build_scene.py's normal interactive flow never plays the
    timeline before calling run_teleop_loop(), so that ordering is never a
    problem there. Confirmed live that calling `timeline.play()` (plus a
    few `simulation_app.update()`s) *before* run_teleop_loop() steps
    physics with no PhysicsScene prim on the stage yet, which corrupts
    PhysX's tensor simulationView -- run_teleop_loop's own later
    `SingleArticulation(...)` construction then crashes with
    `AttributeError: 'NoneType' object has no attribute 'link_names'` deep
    in isaacsim.core.prims, even though that construction is the *first*
    SingleArticulation on this prim (ruled out via a separate repro: the
    same crash happens whether or not this script also builds its own
    SingleArticulation beforehand). Fixed by defining `/physicsScene`
    ourselves before calling `timeline.play()`, matching the order
    run_teleop_loop's own docstring/comments already establish as required
    (see build_scene.py) -- run_teleop_loop's own `IsValid()` guard then
    just finds it already there and moves on.
  - Also avoid holding two separate SingleArticulation instances on the
    same prim at once regardless: this script reads the guaranteed
    starting pose from `robot_cfg["kinematics"]["cspace"]["retract_config"]`
    (what run_teleop_loop's own init phase drives the robot to) instead of
    constructing its own SingleArticulation before calling
    run_teleop_loop(), and only builds one afterward, once
    run_teleop_loop's internal instance has gone out of scope, to read the
    ending pose.

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/test_teleop_headless.py --headless
"""

from __future__ import annotations

import sys

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": _headless})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

import build_scene  # noqa: E402

# Small enough to stay within reach of the target's own starting pose (see
# table_layout.yaml's teleop_target comment -- that starting pose is itself
# already a confirmed-reachable retract-config pose).
#
# [0.0, 0.05, 0.05] (copied from examples/curobo_reference/motion_gen_reacher.py's
# own convention) was confirmed live to IK_FAIL specifically for the CR5's
# own bent-elbow retract_config (cr5.yml) -- not a bug, just an arbitrary
# magic number that happens to exceed this particular pose's +Y reachable
# envelope (individually-tested offsets in every other direction, and
# smaller +Y+Z magnitudes, all succeed). The test only needs *some* valid
# reachable perturbation to exercise the teleop pipeline, so this was
# shrunk rather than re-tuning retract_config again to fit an otherwise-
# meaningless constant.
_DRAG_OFFSET = np.array([0.0, 0.02, 0.02])
_MAX_ITERATIONS = 200
# Must clear build_scene._TELEOP_INIT_FRAMES (10) + _TELEOP_SETTLE_FRAMES
# (20) before the simulated drag lands, so run_teleop_loop's debounce logic
# sees a genuinely static target first, same as a real pre-drag pause would.
_SETTLE_CALLS = 40


def main() -> None:
    if __name__ == "__main__":
        build_scene.simulation_app = simulation_app

    cfg = build_scene.load_config()
    build_scene.build_factory(cfg)
    for _ in range(120):
        simulation_app.update()
    build_scene.prune_factory_dressing(cfg)
    build_scene.build_ergo_tables(cfg)
    build_scene.build_assembly_parts(cfg)

    robot_prim_path = cfg["cr5_mount"]["prim_path"]
    build_scene.mount_cr5(cfg)
    build_scene.mount_cr5_pedestal(cfg)

    print("[test_teleop_headless] warming up cuRobo motion_gen...", flush=True)
    motion_gen, robot_cfg = build_scene.setup_curobo_motion_gen(cfg)
    if motion_gen is None:
        print("[test_teleop_headless] cuRobo not installed -- nothing to test.", flush=True)
        simulation_app.close()
        return

    target = build_scene.build_teleop_target(cfg, robot_prim_path=robot_prim_path, robot_cfg=robot_cfg)

    # Must exist before the timeline plays -- run_teleop_loop() (called
    # below) defines this itself too, but only at its own top, too late
    # here since we call timeline.play() before run_teleop_loop() even
    # starts.
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    # A few frames so is_playing()/the physics view are actually live before
    # run_teleop_loop's own init/settle phase starts counting -- mirrors
    # main()'s own factory-load frame pump, just for physics instead.
    for _ in range(5):
        simulation_app.update()

    # The guaranteed starting pose -- run_teleop_loop's own init phase drives
    # the robot here for its first _TELEOP_INIT_FRAMES frames, so there's no
    # need for a separate SingleArticulation just to observe it (see this
    # module's own docstring for why that would break run_teleop_loop's own
    # later one).
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

    build_scene.run_teleop_loop(
        cfg, motion_gen, robot_cfg, target, robot_prim_path=robot_prim_path, max_iterations=_MAX_ITERATIONS
    )

    # Only constructed now, after run_teleop_loop's own internal
    # SingleArticulation has gone out of scope -- see this module's own
    # docstring for why holding two at once breaks PhysX's tensor view.
    robot = SingleArticulation(prim_path=robot_prim_path, name="verify_robot")
    robot.initialize()
    idx_list = [robot.get_dof_index(x) for x in j_names]
    end_positions = robot.get_joint_positions(idx_list)
    max_delta = float(np.max(np.abs(end_positions - start_positions)))
    print(f"[test_teleop_headless] max joint-position delta: {max_delta:.4f} rad", flush=True)
    if max_delta < 0.05:
        print("[test_teleop_headless] FAIL: robot did not move meaningfully in response to the simulated drag.", flush=True)
    else:
        print("[test_teleop_headless] PASS: robot followed the simulated drag.", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
