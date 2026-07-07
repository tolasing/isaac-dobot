"""Builds the mefron scanner-assembly scene (mefron.usd's own factory
backdrop, packing tables, and scanner-assembly parts) and mounts cuRobo's
bundled Franka Panda on its pre-built mount plate, then runs an
interactive teleop loop -- drag the ghost end-effector target in the GUI
and the robot follows via MotionGen.plan_single().

Variant of build_scene.py for configs/scene/mefron_layout.yaml: mefron.usd
already has its own backdrop/tables/CAD parts authored, so there's no
pruning, ergo-table copy, or assembly-part step here. Uses
add_reference_to_stage() (not open_stage()) to keep the stage's root layer
anonymous/in-memory -- opening mefron.usd directly makes the URDF importer
persist a multi-layer robot structure to disk that breaks CopyPrim (see
scripts/mefron.py's own docstring); referencing it in avoids that, at the
cost of nesting mefron.usd's own content one level under /World/Factory
(its own /World/Factory becomes /World/Factory/Factory here; its /World
siblings like packing_table become /World/Factory/packing_table).

Only creates its own SimulationApp when run standalone.

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/build_scene_mefron.py
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # SimulationApp defaults to the minimal isaacsim.exp.base.python.kit
    # experience, which is missing most UI extensions (Physics debug
    # visualization, full menu bar, etc.) -- load the same
    # isaacsim.exp.full.kit experience isaac-sim.sh itself uses instead,
    # for interactive runs only (headless verification doesn't need it).
    experience = "" if _headless else f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    simulation_app = SimulationApp({"headless": _headless}, experience=experience)

# The full experience's extra extensions make `import packaging` (and
# specifically `packaging.version`) resolve to an incomplete internal
# pip-bootstrap bundle instead of the real site-packages install --
# confirmed live this is NOT a sys.path ordering issue (that bundle's
# path never appears in sys.path at all) and NOT fixed by only
# pre-registering `sys.modules["packaging"]` (still resolved the broken
# `packaging.version` afterward regardless -- whatever finder is doing
# this intercepts the submodule import by name, ignoring the parent
# module's own __path__). cuRobo's own torch_utils does `from packaging
# import version`; fixed by pre-loading both `packaging` and
# `packaging.version` explicitly and setting the latter as a plain
# attribute on the former, so the `from X import Y` statement resolves
# via attribute lookup alone, without any further import-machinery
# involvement for either name.
import importlib.util  # noqa: E402

_REAL_PACKAGING_DIR = "/isaac-sim/kit/python/lib/python3.11/site-packages/packaging"


def _preload_real_submodule(pkg_module, name):
    spec = importlib.util.spec_from_file_location(f"packaging.{name}", f"{_REAL_PACKAGING_DIR}/{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[f"packaging.{name}"] = module
    spec.loader.exec_module(module)
    setattr(pkg_module, name, module)


if "packaging" not in sys.modules and os.path.isdir(_REAL_PACKAGING_DIR):
    _spec = importlib.util.spec_from_file_location(
        "packaging", f"{_REAL_PACKAGING_DIR}/__init__.py", submodule_search_locations=[_REAL_PACKAGING_DIR]
    )
    _packaging_module = importlib.util.module_from_spec(_spec)
    sys.modules["packaging"] = _packaging_module
    _spec.loader.exec_module(_packaging_module)
    _preload_real_submodule(_packaging_module, "version")

import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

from import_cr5 import import_cr5  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = REPO_ROOT / "configs" / "scene" / "mefron_layout.yaml"


def load_config() -> dict:
    return yaml.safe_load(CONFIG_PATH.read_text())


def build_factory(cfg: dict) -> None:
    factory_cfg = cfg["factory"]
    backdrop_usd = REPO_ROOT / factory_cfg["backdrop_usd"]
    if not backdrop_usd.is_file():
        raise FileNotFoundError(f"{backdrop_usd} not found.")
    add_reference_to_stage(usd_path=str(backdrop_usd), prim_path=factory_cfg["prim_path"])


def mount_cr5(cfg: dict) -> None:
    mount_cfg = cfg["cr5_mount"]
    override = mount_cfg.get("robot_override")
    if override and override.get("enabled"):
        # Lazy import: keeps this script working in the `base` profile (no cuRobo).
        from curobo.util_file import get_assets_path, join_path

        urdf_path = Path(join_path(get_assets_path(), override["urdf_relative_path"]))
        import_cr5(
            urdf_path=urdf_path,
            prim_path=mount_cfg["prim_path"],
            default_drive_strength=override["default_drive_strength"],
            default_position_drive_damping=override["default_position_drive_damping"],
        )
    else:
        import_cr5(prim_path=mount_cfg["prim_path"])
    xform = SingleXFormPrim(prim_path=mount_cfg["prim_path"])
    xform.set_world_pose(
        position=np.array(mount_cfg["position"]),
        orientation=np.array(mount_cfg["orientation_wxyz"]),
    )
    xform.set_local_scale(np.array(mount_cfg["scale"]))


GRIPPER_FRICTION_MATERIAL_PATH = "/World/GripperFrictionMaterial"
GRIPPER_STATIC_FRICTION = 0.9
GRIPPER_DYNAMIC_FRICTION = 0.8
GRIPPER_FINGER_LINK_NAMES = ["panda_leftfinger", "panda_rightfinger"]


def apply_gripper_friction(cfg: dict, robot_prim_path: str) -> None:
    """Authors one high-friction physics material and binds it to both the
    Franka's fingertip links and cfg's high_friction_prim_paths (the
    object(s) meant to be grasped).

    Runtime-only, not persisted: mefron.usd is referenced into this script's
    own anonymous in-memory stage (see build_factory()'s own module comment
    on why), and this script never calls stage.Save(), so this material is
    re-authored fresh every run rather than written back into mefron.usd
    itself.

    Confirmed via a read-only inspection of mefron.usd that NO physics
    material was authored anywhere in the file (on the scanner parts) or via
    import_cr5()/the Franka URDF (on the fingertips) -- both sides of every
    grasp contact were relying entirely on PhysX's un-overridden engine
    default friction, which is why grasps were slipping regardless of
    object mass.
    """
    from omni.physx.scripts import utils as physx_utils
    from omni.physx.scripts.physicsUtils import add_physics_material_to_prim

    stage = omni.usd.get_context().get_stage()
    physx_utils.addRigidBodyMaterial(
        stage,
        GRIPPER_FRICTION_MATERIAL_PATH,
        staticFriction=GRIPPER_STATIC_FRICTION,
        dynamicFriction=GRIPPER_DYNAMIC_FRICTION,
        restitution=0.0,
    )

    target_paths = [f"{robot_prim_path}/{name}" for name in GRIPPER_FINGER_LINK_NAMES]
    target_paths += cfg.get("high_friction_prim_paths", [])
    for prim_path in target_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            print(f"[build_scene_mefron] WARNING: {prim_path} not found -- skipping friction bind.", flush=True)
            continue
        add_physics_material_to_prim(stage, prim, GRIPPER_FRICTION_MATERIAL_PATH)


GRIPPER_DRIVE_STIFFNESS = 10000.0
GRIPPER_DRIVE_DAMPING = 200.0


def stiffen_gripper_drive(robot_prim_path: str) -> None:
    """Raises the finger joints' own position-drive stiffness/damping well
    above the whole-robot import-time default (import_cr5()'s
    default_drive_strength=1047.2/default_position_drive_damping=52.36,
    applied uniformly to every joint at import -- tuned for holding the
    arm's much heavier links against gravity).

    Confirmed via a headless inspection of the actual imported joint prims
    (/World/CR5/joints/panda_finger_joint1|2, UsdPhysics.DriveAPI,
    drive type "linear") that the finger joints land at stiffness=625.0,
    damping=10.0 -- not the configured 1047.2/52.36 (the importer evidently
    derives a different effective value for prismatic joints, and the
    URDF's own <dynamics damping="10.0"/> on these two joints takes
    precedence over the importer's default damping). At that stiffness, a
    typical 1-2cm grasp position error only reaches ~6-12N of the joints'
    own hard maxForce=20 ceiling (also confirmed, matches the URDF's
    effort="20") -- real headroom being left unused, which likely
    contributes to the grasped object dangling/rotating rather than being
    held firmly.

    Runtime-only, not persisted -- see apply_gripper_friction()'s own
    module comment; the same reasoning applies here.
    """
    stage = omni.usd.get_context().get_stage()
    for joint_name in GRIPPER_JOINT_NAMES:
        joint_prim = stage.GetPrimAtPath(f"{robot_prim_path}/joints/{joint_name}")
        if not joint_prim.IsValid():
            print(f"[build_scene_mefron] WARNING: {joint_prim.GetPath()} not found -- skipping stiffen.", flush=True)
            continue
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "linear")
        drive.CreateStiffnessAttr().Set(GRIPPER_DRIVE_STIFFNESS)
        drive.CreateDampingAttr().Set(GRIPPER_DRIVE_DAMPING)


def build_teleop_target(cfg: dict, robot_prim_path: str, robot_cfg: dict) -> SingleXFormPrim:
    """Ghost copy of the robot's own end-effector visual mesh, placed at teleop_target's configured pose."""
    target_cfg = cfg["teleop_target"]
    ee_link = robot_cfg["kinematics"]["ee_link"]
    source_path = f"{robot_prim_path}/{ee_link}/visuals"
    omni.kit.commands.execute("CopyPrim", path_from=source_path, path_to=target_cfg["prim_path"])
    xform = SingleXFormPrim(prim_path=target_cfg["prim_path"])
    xform.set_world_pose(
        position=np.array(target_cfg["position"]),
        orientation=np.array(target_cfg["orientation_wxyz"]),
    )
    return xform


def get_teleop_obstacles(cfg: dict, robot_prim_path: str):
    """Scoped cuRobo collision scan: obstacle_prim_paths plus the mount surface, not the whole backdrop."""
    from curobo.util.usd_helper import UsdHelper

    target_cfg = cfg["teleop_target"]
    only_paths = [*cfg["obstacle_prim_paths"], cfg["cr5_mount"]["mount_surface"]["prim_path"]]

    usd_help = UsdHelper()
    usd_help.load_stage(omni.usd.get_context().get_stage())
    return usd_help.get_obstacles_from_stage(
        only_paths=only_paths,
        reference_prim_path=robot_prim_path,
        ignore_substring=[robot_prim_path, target_cfg["prim_path"], "/curobo"],
    ).get_collision_check_world()


def setup_curobo_motion_gen(cfg: dict):
    """Builds and warms up a cuRobo MotionGen; returns (None, None) if cuRobo isn't installed."""
    try:
        from curobo.types.base import TensorDeviceType
        from curobo.util_file import get_robot_configs_path, join_path, load_yaml
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
    except ImportError:
        print("[build_scene_mefron] cuRobo not installed -- skipping MotionGen setup.", flush=True)
        return None, None

    mount_cfg = cfg["cr5_mount"]
    override = mount_cfg.get("robot_override")
    if override and override.get("enabled"):
        robot_cfg = load_yaml(join_path(get_robot_configs_path(), override["motion_gen_robot_cfg"]))["robot_cfg"]
    else:
        cr5_yml = REPO_ROOT / "configs" / "curobo" / "cr5.yml"
        robot_cfg = load_yaml(str(cr5_yml))["robot_cfg"]
        k = robot_cfg["kinematics"]
        k["urdf_path"] = str(REPO_ROOT / k["urdf_path"])
        k["asset_root_path"] = str(REPO_ROOT / k["asset_root_path"])
        k["collision_spheres"] = str(cr5_yml.parent / k["collision_spheres"])

    # A real, populated world must be passed at construction -- an empty one
    # leaves world_coll_checker as None and fails warmup().
    world_cfg = get_teleop_obstacles(cfg, robot_prim_path=mount_cfg["prim_path"])
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        {"robot_cfg": robot_cfg}, world_cfg, tensor_args=TensorDeviceType()
    )
    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup()
    return motion_gen, robot_cfg


_TELEOP_INIT_FRAMES = 10
_TELEOP_SETTLE_FRAMES = 20
_TELEOP_OBSTACLE_RESCAN_INTERVAL = 1000

# Values confirmed by reading franka_panda.urdf directly (both
# panda_finger_joint1/2 are prismatic with limit lower=0.0 upper=0.04), not
# assumed -- 0.04 also matches franka.yml's own lock_joints/retract_config
# value, so "open" here is the same pose cuRobo already holds the fingers at
# whenever it plans an arm trajectory (see run_teleop_loop's own gripper
# actuation block below for why that matters).
GRIPPER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]
GRIPPER_OPEN_POSITION = 0.04
GRIPPER_CLOSED_POSITION = 0.0


class GripperKeyboardControl:
    """Open/closed request for the Franka's gripper, read once per teleop
    frame. Real instances are driven by carb.input keyboard events (see
    build_gripper_keyboard_control()); a test can construct one directly and
    call set_closed() to simulate a keypress without a live keyboard device,
    the same trick test_teleop_headless.py already uses for simulating a
    mouse-drag via target.get_world_pose()."""

    def __init__(self) -> None:
        self.closed = False

    def set_closed(self, closed: bool) -> None:
        self.closed = closed


def build_gripper_keyboard_control() -> GripperKeyboardControl:
    """Subscribes to real keyboard events: C closes the gripper, O opens it.

    carb.input/omni.appwindow are Kit-level (not Isaac-Sim-specific) APIs,
    confirmed against this install's own stubs
    (isaac-sim/kit/kernel/py/carb/input.pyi,
    isaac-sim/extscache/omni.appwindow-*/omni/appwindow/_appwindow.pyi)
    rather than assumed from memory.
    """
    import carb.input
    import omni.appwindow

    control = GripperKeyboardControl()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    input_iface = carb.input.acquire_input_interface()

    def _on_keyboard_event(event) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == carb.input.KeyboardInput.C:
                control.set_closed(True)
            elif event.input == carb.input.KeyboardInput.O:
                control.set_closed(False)
        return True

    # Kept alive on the control object itself -- nothing else holds a
    # reference to keyboard/input_iface/the subscription id otherwise, and an
    # unreferenced subscription is liable to be garbage-collected away.
    control._keyboard = keyboard
    control._input_iface = input_iface
    control._subscription_id = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    return control


def run_teleop_loop(
    cfg: dict,
    motion_gen,
    robot_cfg: dict,
    target: SingleXFormPrim,
    robot_prim_path: str,
    max_iterations: int | None = None,
    gripper_control: GripperKeyboardControl | None = None,
) -> None:
    """Drag `target` in the GUI viewport; the robot follows via cuRobo's MotionGen.

    Rebuilds the robot's SingleArticulation on every fresh Play, not just
    the first: clicking Stop tears down PhysX's simulation view, and
    reusing a stale SingleArticulation afterward leaves the robot
    permanently unresponsive (confirmed live).

    `gripper_control`: if given, the Franka's two finger joints are driven
    directly to GRIPPER_OPEN_POSITION/GRIPPER_CLOSED_POSITION every playing
    frame based on `gripper_control.closed`, independent of cmd_plan/cuRobo
    -- franka.yml already locks these two joints out of cuRobo's own
    IK/trajopt (see GRIPPER_JOINT_NAMES' comment above), so this is the
    correct point of separation, not a workaround. None disables gripper
    actuation entirely (e.g. no live keyboard device available).
    """
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.state import JointState
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/physicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/physicsScene")

    tensor_args = TensorDeviceType()
    plan_config = MotionGenPlanConfig()
    timeline = omni.timeline.get_timeline_interface()

    mount_cfg = cfg["cr5_mount"]
    robot_base_pose = Pose(
        position=tensor_args.to_device(np.array(mount_cfg["position"])),
        quaternion=tensor_args.to_device(np.array(mount_cfg["orientation_wxyz"])),
    )

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = np.array(robot_cfg["kinematics"]["cspace"]["retract_config"])
    target_cfg = cfg["teleop_target"]
    pose_delta_threshold = target_cfg["pose_delta_threshold"]
    static_joint_velocity_threshold = target_cfg["static_joint_velocity_threshold"]

    robot = None
    idx_list = None
    gripper_idx_list = None
    articulation_controller = None

    past_pose = None
    past_orientation = None
    target_pose = None
    target_orientation = None
    cmd_plan = None
    cmd_idx = 0
    # Real elapsed time (not frame count) since the last waypoint was applied,
    # and the plan's own intended per-waypoint duration -- gates playback speed
    # to what the plan was actually generated for, independent of render FPS.
    # See run_teleop_loop's own module comment for why this matters.
    last_cmd_time = None
    interpolation_dt = 0.02
    obstacles = None
    step_index = 0
    not_playing_frames = 0
    was_playing = False

    while simulation_app.is_running():
        simulation_app.update()

        if not timeline.is_playing():
            was_playing = False
            not_playing_frames += 1
            if not_playing_frames % 100 == 0:
                print("[build_scene_mefron] Click Play to start cuRobo teleop.", flush=True)
            continue

        if not was_playing:
            # Fresh Play (first ever, or after a Stop) -- rebuild everything
            # bound to the previous physics view/step count.
            idx_list = None
            gripper_idx_list = None
            articulation_controller = None
            past_pose = None
            past_orientation = None
            target_pose = None
            target_orientation = None
            cmd_plan = None
            cmd_idx = 0
            last_cmd_time = None
            obstacles = None
            step_index = 0
            was_playing = True

        step_index += 1
        if max_iterations is not None and step_index > max_iterations:
            return

        if idx_list is None:
            robot = SingleArticulation(prim_path=robot_prim_path, name="teleop_robot")
            robot.initialize()
            idx_list = [robot.get_dof_index(x) for x in j_names]
            gripper_idx_list = [robot.get_dof_index(x) for x in GRIPPER_JOINT_NAMES]
            articulation_controller = robot.get_articulation_controller()

        if step_index < _TELEOP_INIT_FRAMES:
            robot.set_joint_positions(default_config, idx_list)
            continue
        if step_index < _TELEOP_SETTLE_FRAMES:
            continue

        if obstacles is None or step_index % _TELEOP_OBSTACLE_RESCAN_INTERVAL == 0:
            obstacles = get_teleop_obstacles(cfg, robot_prim_path)
            motion_gen.update_world(obstacles)

        cube_position, cube_orientation = target.get_world_pose()
        if past_pose is None:
            past_pose = cube_position
        if target_pose is None:
            target_pose = cube_position
        if target_orientation is None:
            target_orientation = cube_orientation
        if past_orientation is None:
            past_orientation = cube_orientation

        sim_js = robot.get_joints_state()
        if sim_js is None:
            continue
        sim_js_names = robot.dof_names
        cu_js = JointState(
            position=tensor_args.to_device(sim_js.positions),
            velocity=tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=sim_js_names,
        )
        cu_js = cu_js.get_ordered_joint_state(motion_gen.kinematics.joint_names)

        robot_static = bool(np.max(np.abs(sim_js.velocities)) < static_joint_velocity_threshold)

        if (
            (
                np.linalg.norm(cube_position - target_pose) > pose_delta_threshold
                or np.linalg.norm(cube_orientation - target_orientation) > pose_delta_threshold
            )
            and np.linalg.norm(past_pose - cube_position) == 0.0
            and np.linalg.norm(past_orientation - cube_orientation) == 0.0
            and robot_static
            and cmd_plan is None
        ):
            world_target_pose = Pose(
                position=tensor_args.to_device(cube_position),
                quaternion=tensor_args.to_device(cube_orientation),
            )
            ik_goal = robot_base_pose.compute_local_pose(world_target_pose)
            result = motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, plan_config)
            print(f"[build_scene_mefron] teleop plan_single success={result.success.item()}", flush=True)
            if result.success.item():
                cmd_plan = motion_gen.get_full_js(result.get_interpolated_plan())
                cmd_plan = cmd_plan.get_ordered_joint_state(sim_js_names)
                cmd_idx = 0
                # result.interpolation_dt (not a motion_gen-level attribute --
                # MotionGen itself doesn't expose one; MotionGenResult does)
                # is this specific plan's own intended per-waypoint duration.
                interpolation_dt = result.interpolation_dt
                last_cmd_time = None
            target_pose = cube_position
            target_orientation = cube_orientation

        past_pose = cube_position
        past_orientation = cube_orientation

        if cmd_plan is not None:
            # Gate on real elapsed time, not frame count -- get_interpolated_plan()
            # spaces waypoints interpolation_dt seconds apart (0.02s here), but this
            # loop's own render rate (confirmed live at ~119 FPS, far above the 50Hz
            # the plan assumes) has nothing to do with that. Applying one waypoint
            # per frame instead of one per interpolation_dt played the whole
            # trajectory back at ~2.4x its intended speed and cut its final
            # deceleration-to-zero-velocity ramp short, leaving real residual
            # velocity for the position-hold drive to absorb once cmd_plan ran out
            # -- the actual cause of both the too-fast motion and the arrival
            # oscillation, not a stiffness/damping problem.
            now = time.time()
            if last_cmd_time is None or (now - last_cmd_time) >= interpolation_dt:
                cmd_state = cmd_plan[cmd_idx]
                art_action = ArticulationAction(
                    cmd_state.position.cpu().numpy(),
                    cmd_state.velocity.cpu().numpy(),
                    joint_indices=idx_list,
                )
                articulation_controller.apply_action(art_action)
                cmd_idx += 1
                last_cmd_time = now
                if cmd_idx >= len(cmd_plan.position):
                    cmd_idx = 0
                    cmd_plan = None

        # Independent of cmd_plan/cuRobo -- applied every frame (not just
        # while a plan is interpolating), so it always wins that frame's
        # drive-target write for the finger indices even when the block
        # above just also wrote to them (get_full_js() re-applies the
        # lock_joints value on every planned frame; see GRIPPER_JOINT_NAMES'
        # comment above for why fingers are locked out of cuRobo at all).
        if gripper_control is not None:
            gripper_target = GRIPPER_CLOSED_POSITION if gripper_control.closed else GRIPPER_OPEN_POSITION
            gripper_action = ArticulationAction(
                np.array([gripper_target, gripper_target]),
                joint_indices=gripper_idx_list,
            )
            articulation_controller.apply_action(gripper_action)


def main() -> None:
    cfg = load_config()
    build_factory(cfg)

    # mefron.usd's own content resolves asynchronously.
    for _ in range(120):
        simulation_app.update()

    robot_prim_path = cfg["cr5_mount"]["prim_path"]
    mount_cr5(cfg)
    apply_gripper_friction(cfg, robot_prim_path)
    stiffen_gripper_drive(robot_prim_path)

    print(
        "[build_scene_mefron] warming up cuRobo motion_gen (viewport will look frozen/black until this finishes)...",
        flush=True,
    )
    motion_gen, robot_cfg = setup_curobo_motion_gen(cfg)
    print(f"[build_scene_mefron] curobo motion_gen: {'READY' if motion_gen else 'SKIPPED'}", flush=True)

    target = None
    if motion_gen is not None:
        target = build_teleop_target(cfg, robot_prim_path=robot_prim_path, robot_cfg=robot_cfg)

    stage = omni.usd.get_context().get_stage()
    status_paths = [
        cfg["factory"]["prim_path"],
        robot_prim_path,
        cfg["cr5_mount"]["mount_surface"]["prim_path"],
        *cfg["obstacle_prim_paths"],
    ]
    if target is not None:
        status_paths.append(cfg["teleop_target"]["prim_path"])
    for prim_path in status_paths:
        prim = stage.GetPrimAtPath(prim_path)
        print(f"[build_scene_mefron] {prim_path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

    if _headless:
        simulation_app.close()
        return

    if motion_gen is not None:
        gripper_control = build_gripper_keyboard_control()
        print("[build_scene_mefron] Gripper: press C to close, O to open.", flush=True)
        run_teleop_loop(
            cfg, motion_gen, robot_cfg, target, robot_prim_path=robot_prim_path, gripper_control=gripper_control
        )
    else:
        print("[build_scene_mefron] cuRobo not installed -- falling back to a bare update loop.", flush=True)
        while simulation_app.is_running():
            simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
