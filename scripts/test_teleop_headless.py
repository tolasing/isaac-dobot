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

import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import UsdGeom, UsdPhysics  # noqa: E402

import build_scene  # noqa: E402
import import_cr5  # noqa: E402

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
# table_layout.yaml's teleop_time_dilation_factor/teleop_velocity_scale/
# teleop_acceleration_scale (added to fix a start/stop tracking oscillation --
# see that file's own comment) make this same drag's trajectory ~10x longer
# in applied-waypoint count (confirmed live: 350 waypoints post-fix vs. 33
# before). 200 predates that change and was silently truncating the run
# before the trajectory ever reached its deceleration phase -- run_teleop_loop
# has no "stop once cmd_plan finishes early" exit, so a too-small budget here
# doesn't fail loudly, it just quietly stops observing partway through the
# move. 3000 gives ~2x headroom over the observed 350-waypoint completion, to
# absorb a slower/faster machine changing how many step_index ticks fit in
# the same real-world interpolation_dt window (the gate is wall-clock-timed,
# not step-count-timed -- see run_teleop_loop's own comment).
#
# GRIPPER ADDITION -- CONFIRMED LIVE 3000 is no longer enough once the arm
# plan actually succeeds (it was failing outright before an unrelated
# self-collision fix, which meant every iteration was cheap -- no real
# physics/dynamics work to do -- so the gripper's own wall-clock-gated ramp
# had the whole budget to itself and finished easily). Once the arm is
# actually tracking a real trajectory, each simulation_app.update() does
# more physics work and real time elapses more slowly per iteration;
# empirically, only ~0.68s of the gripper's needed 1.25s ramp
# (0.025m / 0.02 m/s) elapsed by iteration 3000 with the arm plan
# succeeding. Raised to give real headroom -- re-confirm this is still
# enough if the arm-side per-iteration cost changes again (e.g. a heavier
# scene, different hardware).
_MAX_ITERATIONS = 8000
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
    # ARM-only subset: cspace.joint_names/retract_config now also include
    # the gripper's own driven joint (configs/curobo/cr5.yml's
    # pgc140_finger1_joint), but this check's purpose is specifically "did
    # the arm move in response to the simulated drag." Excluding the
    # gripper joint here keeps that meaning intact regardless of what the
    # gripper is simultaneously commanded to do below -- a naive shared
    # max_delta across both would let the gripper's own, unrelated closing
    # motion mask a genuine arm-teleop regression (a broken arm could still
    # show a false PASS purely from the gripper closing).
    all_j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    all_retract_config = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])
    arm_mask = [name not in import_cr5.GRIPPER_JOINT_NAMES for name in all_j_names]
    j_names = [name for name, is_arm in zip(all_j_names, arm_mask) if is_arm]
    start_positions = all_retract_config[arm_mask]

    start_position, start_orientation = target.get_world_pose()
    dragged_position = start_position + _DRAG_OFFSET

    call_count = {"n": 0}

    def fake_get_world_pose():
        call_count["n"] += 1
        if call_count["n"] < _SETTLE_CALLS:
            return start_position, start_orientation
        return dragged_position, start_orientation

    target.get_world_pose = fake_get_world_pose

    # GRIPPER ADDITION: constructed directly rather than via
    # build_scene.build_gripper_keyboard_control() (which subscribes to
    # real keyboard input, unavailable/unnecessary headless) and commanded
    # programmatically -- mirrors how this script already fakes the arm's
    # drag via monkeypatching target.get_world_pose() rather than
    # simulating real mouse input. Commanded closed from the very start of
    # the run (not mid-run) so the whole _MAX_ITERATIONS budget is
    # available for the ramp to reach its target -- NOT yet confirmed live
    # that this is actually enough real wall-clock time (the ramp is gated
    # on time.time(), not iteration count); if this assertion below turns
    # out to fail solely because the ramp hadn't finished yet, that's a
    # test-budget problem, not a functional one -- increase _MAX_ITERATIONS
    # or add a dedicated post-loop settle period rather than loosening the
    # tolerance.
    gripper_control = build_scene.GripperKeyboardControl()
    gripper_control.set_closed(True)

    build_scene.run_teleop_loop(
        cfg,
        motion_gen,
        robot_cfg,
        target,
        robot_prim_path=robot_prim_path,
        max_iterations=_MAX_ITERATIONS,
        gripper_control=gripper_control,
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

    # GRIPPER ADDITION: doesn't exist even for the Franka pipeline
    # currently (docs/mefron-history.md notes this as a known gap there
    # too) -- reads back real, simulated joint positions the same way as
    # the arm check above, rather than trusting the commanded setpoint.
    gripper_cfg = cfg["cr5_mount"]["gripper"]
    closed_target = gripper_cfg["closed_position"]
    gripper_tolerance = 0.002  # 2mm -- small relative to the 25mm full stroke
    driven_joint_names = [name for name in import_cr5.GRIPPER_JOINT_NAMES if name in robot.dof_names]
    if not driven_joint_names:
        print(
            "[test_teleop_headless] SKIP: no gripper joint present on this "
            "articulation (no gripper mounted, or names don't match robots/pgc140/ -- check import_cr5.GRIPPER_JOINT_NAMES).",
            flush=True,
        )
    else:
        gripper_idx_list = [robot.get_dof_index(name) for name in driven_joint_names]
        gripper_end_positions = robot.get_joint_positions(gripper_idx_list)
        print(
            f"[test_teleop_headless] gripper joints {driven_joint_names} end positions: {gripper_end_positions} "
            f"(target: {closed_target})",
            flush=True,
        )
        if np.max(np.abs(gripper_end_positions - closed_target)) > gripper_tolerance:
            print(
                "[test_teleop_headless] FAIL: gripper did not reach the commanded closed position -- "
                "see this script's own comment above if this is purely a ramp-timing budget issue.",
                flush=True,
            )
        else:
            print("[test_teleop_headless] PASS: gripper closed to the commanded position.", flush=True)
        if len(driven_joint_names) == 1:
            print(
                "[test_teleop_headless] NOTE: only pgc140_finger1_joint is an independent DOF on this "
                "articulation -- pgc140_finger2_joint is presumably PhysX-mimic-constrained and was not "
                "independently checked here; confirm visually in the GUI that it actually tracked finger1.",
                flush=True,
            )

        # GRIPPER ADDITION: open-direction check, previously untested --
        # the user reported (live, in the GUI) that motion is sequential
        # and *reverses direction* between opening and closing, so a
        # close-only check can't tell the two directions apart. Drives the
        # already-constructed `robot` directly (bypassing run_teleop_loop's
        # ramp/cmd_plan machinery entirely -- this is a raw, direct
        # ArticulationAction loop) rather than a second full
        # run_teleop_loop() call, since a second call would re-trigger its
        # own Stop/Play-style rebuild and snap instantly rather than
        # ramping, defeating the point of watching real settled behavior.
        open_target = gripper_cfg["open_position"]
        for _ in range(_MAX_ITERATIONS):
            simulation_app.update()
            action = ArticulationAction(
                np.array([open_target] * len(gripper_idx_list)), joint_indices=gripper_idx_list
            )
            robot.get_articulation_controller().apply_action(action)
        gripper_open_positions = robot.get_joint_positions(gripper_idx_list)
        print(
            f"[test_teleop_headless] gripper joints {driven_joint_names} OPEN end positions: "
            f"{gripper_open_positions} (target: {open_target})",
            flush=True,
        )
        if np.max(np.abs(gripper_open_positions - open_target)) > gripper_tolerance:
            print("[test_teleop_headless] FAIL: gripper did not reach the commanded open position.", flush=True)
        else:
            print("[test_teleop_headless] PASS: gripper opened to the commanded position.", flush=True)

        # GRIPPER ADDITION: direct PhysX overlap query at the settled-open
        # state, to see exactly what (if anything) each finger is
        # contacting -- answers "what is it touching" directly instead of
        # continuing to infer from joint-position symptoms alone.
        bbox_cache = UsdGeom.BBoxCache(0, ["default"], useExtentsHint=False)
        sq = omni.physx.get_physx_scene_query_interface()
        for link in ["pgc140_finger1_link", "pgc140_finger2_link"]:
            prim = stage.GetPrimAtPath(f"{robot_prim_path}/{link}")
            bbox = bbox_cache.ComputeWorldBound(prim)
            r = bbox.ComputeAlignedRange()
            center = [(r.GetMin()[i] + r.GetMax()[i]) / 2 for i in range(3)]
            half_extent = [(r.GetMax()[i] - r.GetMin()[i]) / 2 + 0.005 for i in range(3)]
            hits = []

            def _report_hit(hit, hits=hits):
                hits.append((hit.rigid_body, hit.collision))
                return True

            sq.overlap_box(half_extent, center, [0, 0, 0, 1], _report_hit, False)
            other = [rb for rb, _ in hits if link not in rb]
            print(f"[test_teleop_headless] {link} OPEN-state overlaps (excluding self): {other}", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
