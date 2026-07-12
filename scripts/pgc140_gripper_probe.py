"""Standalone PGC-140 gripper probe: imports just the vendored gripper
(robots/pgc140/urdf/pgc140_robot.urdf, no CR5 arm) fixed-base into an empty
stage, with a C/O keyboard control that opens/closes it -- same
GripperKeyboardControl/build_gripper_keyboard_control pattern
scripts/build_scene.py already uses for the full CR5+gripper teleop loop
(itself modeled on scripts/mefron_lib/teleop.py's C/O control for the
Franka), reused here rather than re-implemented.

Purpose: isolate the gripper's own open/close behavior from the CR5 arm
entirely. This is what originally confirmed CLAUDE.md's bug #10 (full
CR5+gripper articulation: closing/opening reliably left one finger stuck
at a small, repeatable offset from its commanded target while the other
reached cleanly, direction-dependent -- which finger stalled flipped
between open and close) as RESOLVED rather than merely CR5-mount-specific:
running the same import_cr5.tune_gripper_drive()/
filter_self_collision_from_curobo_config()/disable_gripper_finger_gravity()
calls build_scene.py's mount_cr5() uses -- reused directly here rather than
duplicated -- on the gripper ALONE also closes/opens both fingers cleanly
within tolerance, confirming disable_gripper_finger_gravity()'s fix is
correct on its own terms and not an artifact of anything CR5-mount-specific
(the identity Link6-to-gripper transform, the arm's own idx_list apply
loop, etc.). See CLAUDE.md's gripper section (bug #10) for the exact
numbers from both this probe and the full test_teleop_headless.py run.

Run standalone (interactive -- drag nothing needed, just press C/O in the
viewport to open/close):
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/pgc140_gripper_probe.py

Run headless (drives the same C/O ramp programmatically, prints PASS/FAIL
for both directions plus an overlap-box query at the open state -- mirrors
test_teleop_headless.py's own gripper-check section exactly):
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/pgc140_gripper_probe.py --headless
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    experience = "" if _headless else f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    simulation_app = SimulationApp({"headless": _headless}, experience=experience)

import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import UsdGeom, UsdPhysics  # noqa: E402

from build_scene import GripperKeyboardControl, build_gripper_keyboard_control  # noqa: E402
from import_cr5 import (  # noqa: E402
    GRIPPER_JOINT_NAMES,
    disable_gripper_finger_gravity,
    filter_self_collision_from_curobo_config,
    import_cr5,
    tune_gripper_drive,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
PGC140_URDF_PATH = REPO_ROOT / "robots" / "pgc140" / "urdf" / "pgc140_robot.urdf"
PROBE_PRIM_PATH = "/World/GripperProbe"

# Same numbers table_layout.yaml's cr5_mount.gripper block uses for the
# mounted gripper -- kept identical here so this probe is actually testing
# the same tuning the full integration runs, not a differently-tuned copy.
GRIPPER_STIFFNESS = 625.0
GRIPPER_DAMPING = 50.0
GRIPPER_MAX_FORCE = 140.0
OPEN_POSITION = 0.0
CLOSED_POSITION = 0.025
CLOSE_SPEED = 0.02  # m/s, matches table_layout.yaml's cr5_mount.gripper.close_speed

_TELEOP_INIT_FRAMES = 10
_TELEOP_SETTLE_FRAMES = 20
# Full stroke (0.025m) at CLOSE_SPEED (0.02 m/s) takes 1.25s -- give ~3x
# headroom the same way test_teleop_headless.py does for the mounted case.
_MAX_ITERATIONS = 4000
_GRIPPER_TOLERANCE = 0.002  # 2mm, same as test_teleop_headless.py


def spawn_gripper_probe(simulation_app, prim_path: str = PROBE_PRIM_PATH) -> str:
    """Imports the bare PGC-140 URDF fixed-base and applies the same
    tuning/self-collision/gravity fixes build_scene.py's mount_cr5()
    applies to the mounted gripper -- reused directly (all three functions
    are generic over prim_path, not CR5-specific) rather than duplicated.

    CONFIRMED LIVE (root cause, via direct URDFParseAndImportFile tracing):
    this bare headless stage has no /World prim at all -- unlike
    build_scene.py's real pipeline, where build_factory()'s own
    add_reference_to_stage(usd_path=..., prim_path="/World/Factory") call
    creates /World as a side effect (that utility defines any missing
    ancestor prims of its target path) before mount_cr5() ever runs. This
    probe never calls that, so without defining /World explicitly first,
    import_cr5()'s own `MovePrim(path_from=imported_prim_path,
    path_to=prim_path)` step (imported_prim_path came back as
    "/pgc140_robot", a stage-root sibling matching the URDF's own <robot
    name=...>) silently fails -- import_cr5() never checks MovePrim's own
    return status -- leaving prim_path (e.g. /World/GripperProbe) a valid
    but childless prim while the robot's real link/joint hierarchy stays
    behind at the unmoved /pgc140_robot. The resulting SingleArticulation()
    call then fails with `AttributeError: 'NoneType' object has no
    attribute 'is_homogeneous'` (PhysX has nothing to bind at prim_path),
    which reads exactly like a physics-view-timing race but isn't one --
    confirmed by tracing the raw command's own imported_prim_path return
    value directly, not by adding more settle frames (tried first; had no
    effect, since nothing was ever going to move into place no matter how
    long the wait).
    """
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/World").IsValid():
        UsdGeom.Xform.Define(stage, "/World")
    import_cr5(urdf_path=PGC140_URDF_PATH, prim_path=prim_path)
    # URDFParseAndImportFile's own asset population still runs a beat behind
    # the command's return (isaacsim.asset.importer.urdf's "Creating Asset
    # in an in-memory stage" log line was observed printing after this
    # function's own next steps in an earlier version of this script) --
    # this pump was added at the same time as the /World fix above; kept as
    # a real, still-worthwhile safety margin before tune_gripper_drive() etc.
    # below need the imported joints to actually exist, same convention
    # build_scene.py's build_factory() uses for its own asynchronous
    # backdrop reference.
    for _ in range(120):
        simulation_app.update()
    tune_gripper_drive(
        prim_path=prim_path,
        stiffness=GRIPPER_STIFFNESS,
        damping=GRIPPER_DAMPING,
        max_force=GRIPPER_MAX_FORCE,
    )
    # No CR5 arm links exist under this prim_path, so cr5.yml's own
    # self_collision_ignore entries naming Link5/Link6/etc. simply won't
    # resolve to a valid prim and are skipped (see that function's own
    # IsValid() guard) -- only the pgc140_base_link/finger entries apply
    # here, which is exactly what's relevant for a standalone probe.
    filter_self_collision_from_curobo_config(prim_path=prim_path)
    disable_gripper_finger_gravity(prim_path=prim_path)
    return prim_path


def run_probe_loop(
    simulation_app,
    prim_path: str,
    max_iterations: int | None = None,
    gripper_control: GripperKeyboardControl | None = None,
) -> SingleArticulation:
    """Same Stop/Play-rebuild-aware, wall-clock-ramped gripper-apply loop as
    build_scene.py's run_teleop_loop() -- just without the arm/motion_gen
    machinery around it, since this probe has neither."""
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
        # A few frames so the physics view is actually live before this
        # function's own init/settle phase starts counting -- confirmed live
        # (see test_teleop_headless.py's own identical pump) that
        # SimulationManager.get_physics_sim_view() stays None for a few
        # frames right after play(), even with /physicsScene already
        # defined; building a SingleArticulation before that returns
        # AttributeError: 'NoneType' object has no attribute 'is_homogeneous'
        # deep in isaacsim.core.prims (confirmed live on this exact probe:
        # omitting this pump reproduced that crash on the very first
        # SingleArticulation() construction below).
        for _ in range(5):
            simulation_app.update()

    robot = None
    idx_list = None
    articulation_controller = None
    step_index = 0
    was_playing = False
    not_playing_frames = 0
    gripper_setpoint = None
    last_gripper_time = None

    while simulation_app.is_running():
        simulation_app.update()

        if not timeline.is_playing():
            was_playing = False
            not_playing_frames += 1
            if not_playing_frames % 100 == 0:
                print("[pgc140_gripper_probe] Click Play to start.", flush=True)
            continue

        if not was_playing:
            idx_list = None
            articulation_controller = None
            step_index = 0
            gripper_setpoint = None
            last_gripper_time = None
            was_playing = True

        step_index += 1
        if max_iterations is not None and step_index > max_iterations:
            return robot

        if idx_list is None:
            if step_index < _TELEOP_INIT_FRAMES:
                continue
            robot = SingleArticulation(prim_path=prim_path, name="gripper_probe_robot")
            robot.initialize()
            idx_list = [robot.get_dof_index(x) for x in GRIPPER_JOINT_NAMES]
            articulation_controller = robot.get_articulation_controller()

        if step_index < _TELEOP_SETTLE_FRAMES:
            continue

        if gripper_control is not None:
            gripper_target = CLOSED_POSITION if gripper_control.closed else OPEN_POSITION
            if gripper_setpoint is None:
                gripper_setpoint = gripper_target
            now = time.time()
            if last_gripper_time is not None:
                max_step = CLOSE_SPEED * (now - last_gripper_time)
                if gripper_setpoint < gripper_target:
                    gripper_setpoint = min(gripper_setpoint + max_step, gripper_target)
                elif gripper_setpoint > gripper_target:
                    gripper_setpoint = max(gripper_setpoint - max_step, gripper_target)
            last_gripper_time = now
            action = ArticulationAction(
                np.array([gripper_setpoint, gripper_setpoint]),
                joint_indices=idx_list,
            )
            articulation_controller.apply_action(action)

    return robot


def _report_overlaps(stage, prim_path: str) -> None:
    bbox_cache = UsdGeom.BBoxCache(0, ["default"], useExtentsHint=False)
    sq = omni.physx.get_physx_scene_query_interface()
    for link in ["pgc140_finger1_link", "pgc140_finger2_link"]:
        prim = stage.GetPrimAtPath(f"{prim_path}/{link}")
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
        print(f"[pgc140_gripper_probe] {link} overlaps (excluding self): {other}", flush=True)


def _check_direction(robot: SingleArticulation, target: float, label: str) -> bool:
    idx_list = [robot.get_dof_index(name) for name in GRIPPER_JOINT_NAMES]
    positions = robot.get_joint_positions(idx_list)
    print(f"[pgc140_gripper_probe] {GRIPPER_JOINT_NAMES} {label} end positions: {positions} (target: {target})", flush=True)
    ok = bool(np.max(np.abs(positions - target)) <= _GRIPPER_TOLERANCE)
    print(f"[pgc140_gripper_probe] {'PASS' if ok else 'FAIL'}: gripper {'reached' if ok else 'did NOT reach'} the commanded {label} position.", flush=True)
    return ok


def run_headless_check() -> None:
    spawn_gripper_probe(simulation_app)
    control = GripperKeyboardControl()
    control.set_closed(True)
    robot = run_probe_loop(simulation_app, PROBE_PRIM_PATH, max_iterations=_MAX_ITERATIONS, gripper_control=control)
    _check_direction(robot, CLOSED_POSITION, "CLOSED")

    # Deliberately NOT a second run_probe_loop() call: its own gripper_setpoint
    # resets to None on every "fresh Play"-style entry (see its own was_playing
    # handling), which would snap straight to the open target on the very
    # first post-settle frame instead of ramping down from the current closed
    # position -- exactly the bug test_teleop_headless.py's own open-direction
    # check already worked around the same way, for the same reason (see that
    # script's own comment). Drive the already-built `robot` directly instead,
    # a raw per-frame ArticulationAction at the open target, no ramp.
    idx_list = [robot.get_dof_index(name) for name in GRIPPER_JOINT_NAMES]
    for _ in range(_MAX_ITERATIONS):
        simulation_app.update()
        action = ArticulationAction(np.array([OPEN_POSITION, OPEN_POSITION]), joint_indices=idx_list)
        robot.get_articulation_controller().apply_action(action)
    _check_direction(robot, OPEN_POSITION, "OPEN")

    _report_overlaps(omni.usd.get_context().get_stage(), PROBE_PRIM_PATH)
    simulation_app.close()


def main() -> None:
    if _headless:
        run_headless_check()
        return

    spawn_gripper_probe(simulation_app)
    control = build_gripper_keyboard_control()
    print(
        "[pgc140_gripper_probe] Imported at "
        f"{PROBE_PRIM_PATH} (no arm, no motion_gen). Click Play, then press C/O in the "
        "viewport to close/open the gripper.",
        flush=True,
    )
    run_probe_loop(simulation_app, PROBE_PRIM_PATH, gripper_control=control)


if __name__ == "__main__":
    main()
