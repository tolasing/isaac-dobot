# CLAUDE.md

Project-specific context for **isaac-cobot**. Kept to the essentials for
working in the sim day-to-day — full history, forensic detail, and "why"
narratives live in `docs/` (linked below) rather than here.

## What this repo is

An NVIDIA Isaac Sim project using cuRobo for collision-aware motion
planning. **There is no physical robot hardware** — everything targets
Isaac Sim only; treat all sim behavior (contact dynamics, motion timing,
gripper interaction) as illustrative, not validated against real hardware.

The active work is a scanner-assembly pick-and-place task built on
`assets/mefron/` — a hand-authored scene (`mefron.usd`: factory floor,
packing tables, and a scanner-assembly CAD mockup — `finger_print_scanner`,
`main_holder`, `screen`, `backpanel_support`). A Franka Panda (cuRobo's own
bundled config) is mounted into this scene and driven by an interactive
cuRobo drag-teleop loop (`motion_gen.plan_single()`); grasp and
assembly-placement poses are derived by manually jogging the robot to a
good pose in the GUI and reading back the relative transform, not measured
on real hardware.

**`dobot` branch**: separate track testing the actual target hardware (a
Dobot CR5) instead of continuing further with the Franka stand-in — see
"CR5 validation (`dobot` branch)" below. Doesn't touch the mefron/Franka
pipeline above at all; scoped to the generic `scripts/build_scene.py` +
`configs/scene/table_layout.yaml` testbed.

## Where things live

- **This file** — current state, and gotchas that will immediately break
  something if you don't know them.
- `docs/mefron-history.md` — full chronological bug/fix log for every
  mefron script, plus the cuRobo/PhysX/URDF-importer Conventions this file
  only summarizes.
- `docs/grasp-and-assembly-offsets.md` — how the grasp/assembly relative-
  pose constants were derived (`compute_relative_pose()` methodology, the
  abandoned Grasp Editor investigation, the open grasp-centering problem).
- `docs/docker-and-devcontainer.md` — Docker/devcontainer environment setup
  (generic Isaac Sim/cuRobo infra, not scene-specific).
- `scripts/build_scene.py` + `configs/scene/table_layout.yaml` — the generic
  ergo-table testbed the CR5 work above lives in (not the mefron scene).
  `robots/cr5/` (vendored URDF+meshes, `SOURCE.md`) and
  `configs/curobo/cr5.yml` + `cr5_collision_spheres.yml` (cuRobo config) are
  the CR5-specific pieces it mounts. `scripts/test_teleop_headless.py` is
  the headless PASS/FAIL regression test for this whole path.
- `examples/curobo_reference/` — pristine, unmodified copy of cuRobo's own
  interactive teleop demo. **Do not modify these two files**; write a
  separate script instead (`scripts/mefron.py` is exactly that). See
  `docs/mefron-history.md` for the environment fixes needed to run it and
  a license caveat on its header comments.
- `scripts/mefron_lib/` — shared package backing every mefron entry-point
  script (`mefron.py`, `mefron_gripper_probe.py`,
  `mefron_grasp_editor_scene.py`, `franka_grasp_editor_scene.py`,
  `test_mefron_*_headless.py`): `kit_bootstrap.py` (packaging preload +
  stale-config cleanup, stdlib-only so it's safe to import before
  `SimulationApp` exists), `config.py` (all constants), `grasp.py` (pose
  math), `robot.py` (mount/friction/drive), `teleop.py` (keyboard control +
  `run_teleop_loop()`). `mefron2.py` (dormant/superseded, see below) keeps
  its own diverged copies of everything except the packaging-preload block.

## Active script + current state

`scripts/mefron.py` is the live script, now a thin entry point over
`scripts/mefron_lib/`: mounts cuRobo's bundled Franka Panda onto
`assets/mefron/`'s SEKTION-cabinet mount plate (world position
`[2.74097, -4.782, 0.7924]`), runs a drag-follow teleop loop, and provides
J(grasp, via an NVIDIA Grasp Editor-exported pose)/P(lace) keys that snap
the teleop target to a live-computed pose, plus C/O keys for the gripper.
P's placement pose is computed by measuring the CURRENT live gripper-to-
part offset (not a fixed constant) and applying it to `finger_print_scanner`'s
live-computed target pose on `main_holder` — so it self-corrects to
whatever grasp J actually produced. There is no G key: an earlier
hand-derived-constant grasp-approach pose has been removed in favor of J.
Opens `mefron.usd` directly via `open_stage()`.

`scripts/mefron_gripper_probe.py` imports just the Franka hand +
`panda_leftfinger`/`panda_rightfinger` + `ee_link` (no arm, no motion_gen)
onto its own free-floating `base_link`, for dragging into place against a
part mesh in Stop mode to measure a grasp pose directly, without cuRobo
IK/teleop in the way.

`scripts/build_scene_mefron.py` (+ `configs/scene/mefron_layout.yaml`) is
architecturally preferred — same goal, but a fresh anonymous stage with
`mefron.usd` referenced in, which avoids most of `mefron.py`'s bugs by
construction — but is currently **dormant**: deriving the grasp/assembly
offsets requires temporarily reparenting prims in the Stage tree, which
only works when `mefron.usd` is opened directly, so active work happens in
`mefron.py`. See `docs/mefron-history.md` for both files' full histories.

Current constants (`scripts/mefron_lib/config.py`):
- `ASSEMBLY_RELATIONSHIPS["finger_print_scanner_on_main_holder"]`:
  `local_position=[-0.05765, 0.02069, 0.01565]`,
  `local_orientation_wxyz=[1.0, 0.0, 0.0, 0.0]`. (There is no
  `GRASP_OFFSET_POSITION`/`ORIENTATION_WXYZ` anymore — P measures the
  live grasp offset instead of using a fixed constant; see above.)
- `_TELEOP_VELOCITY_SCALE = _TELEOP_ACCELERATION_SCALE = 0.5`,
  `GRIPPER_CLOSE_SPEED = 0.02` m/s, `GRIPPER_DRIVE_STIFFNESS = 10000.0`.

Currently open issues (see the linked docs for full diagnosis):
- **Grasp-centering**: `finger_print_scanner` isn't equidistant from both
  fingertips at grasp time, so one finger contacts first and shifts the
  part sideways. Not a joint/drive asymmetry (explicitly ruled out) — see
  `docs/grasp-and-assembly-offsets.md`.
- Assembly placement still lands off on the Y axis — likely the same
  grasp-centering issue, not confirmed.
- `attach_objects_to_robot()`/`detach_object_from_robot()` (cuRobo's
  carried-object collision awareness) is not yet wired to the C/O keys —
  `franka.yml` already has a spare `attached_object` link ready for it.
- `main_holder`'s convex-decomposition collision tuning (fixes sinking +
  lost mounting studs) is researched but not yet applied/saved to
  `mefron.usd` — see `docs/mefron-history.md`.

## CR5 validation (`dobot` branch)

**Status: bare-arm CR5 mount + cuRobo teleop confirmed working.**
`configs/scene/table_layout.yaml`'s `cr5_mount.robot_override.enabled` is
now `false` — `scripts/build_scene.py` mounts the real, already-vendored
CR5 (`robots/cr5/`, from the official `Dobot-Arm/TCP-IP-ROS-6AXis` repo —
no USD exists anywhere for the CR5, official or community) instead of the
Franka stand-in it used before. `scripts/test_teleop_headless.py --headless`
passes: `plan_single success=True`, robot moves in response to a simulated
drag. No gripper yet (see below) and the mefron scanner-assembly scene
(`build_scene_mefron.py`/`mefron_layout.yaml`) hasn't been touched — this
was validated in the generic `table_layout.yaml` testbed only.

Getting a passing run required fixing four real, previously-undiscovered
bugs in the CR5's first-draft config (not just flipping the override flag):

1. **URDF joint velocity limits were `velocity="0"` on every joint**
   (`robots/cr5/urdf/cr5_robot.urdf`) — a SolidWorks-exporter artifact,
   documented as harmless in `robots/cr5/SOURCE.md` before this branch, but
   actually a hard failure: cuRobo reads joint velocity limits straight
   from the URDF with **no config-level override** (confirmed against
   `curobo/cuda_robot_model/cuda_robot_generator.py`), and
   `curobo/rollout/cost/bound_cost.py`'s `set_bounds()` raises
   `ValueError: Joint velocity limits is zero` the first time
   `MotionGenConfig.load_from_robot_config()` actually builds a full
   `MotionGen` (not caught by earlier "loads without error" testing, which
   never got that far). Fixed by setting all six joints to `velocity="3.14"`
   (180°/s, Dobot's published CR5 spec) — see `robots/cr5/SOURCE.md`.
2. **A placeholder collision sphere overlapped the CR5's own mount
   pedestal** (`configs/curobo/cr5_collision_spheres.yml`'s `base_link`
   sphere dipped below the mount plane into `RobotPedestal`), causing
   `MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION` at the very first
   pose tested. Fixed by repositioning that sphere to clear both the mount
   plane and `collision_sphere_buffer`'s added padding — see that file's
   own comment for the exact math (this is easy to get wrong: buffer adds
   to every declared radius, so "tangent to the mount plane" isn't
   actually clear of it).
3. **Placeholder self-collision-ignore list only covered adjacent link
   pairs**, but the CR5's own joint origins fold some non-adjacent links
   (`base_link`/`Link2`, `Link3`/`Link5`, `Link4`/`Link6`) into genuine
   overlap at the (then all-zero) retract config — masked until bug #2 was
   fixed, since world-collision failed first. Extended
   `configs/curobo/cr5.yml`'s `self_collision_ignore` to match (same
   pattern cuRobo's own bundled `ur5e.yml` uses — ignoring several
   non-adjacent pairs is normal, not unique to a rough sphere set).
4. **The placeholder `retract_config` (`[0,0,0,0,0,0]`) is a severe
   kinematic singularity** — confirmed via a numerical Jacobian (finite
   differences on FK): singular values collapse to
   `[48989.8, 0.95, 2.65e-08, 7.79e-17, 7.16e-25, 3.86e-41]`, effectively
   rank 2 instead of 6, so IK fails for almost any nearby target. Not a
   sim-only concern — a real 6-axis arm is never parked fully straight for
   exactly this reason. Replaced with a bent-elbow pose
   (`[0, -0.5, 0.8, 0, 0.5, 0]`, condition number ~47) confirmed both
   collision-free and well-conditioned before adopting it. Any future
   retract-pose change should be sanity-checked the same way (compute FK +
   numerical Jacobian, don't assume a "reasonable-looking" pose is
   non-singular).

Both fixes to `retract_config` and the URDF-import mount pose meant
`table_layout.yaml`'s `teleop_target.position`/`orientation_wxyz` (originally
derived from the *Franka's* retract-pose FK, back when the override was
enabled) had to be recomputed from the CR5's own FK at its new
`retract_config` — see that file's own comment for the exact
`CudaRobotModel.get_state().ee_pose` + `Pose.multiply()` snippet used.
`scripts/test_teleop_headless.py`'s hardcoded `_DRAG_OFFSET` also needed
shrinking (`[0, 0.05, 0.05]` → `[0, 0.02, 0.02]`) — not a bug, just an
arbitrary magic number (copied from a Franka-era reference example) that
happened to exceed the CR5's new retract pose's reachable envelope in +Y;
every other direction, and smaller magnitudes, worked fine.

**Environment note**: this was all validated on a cloud GPU box whose
actual card is an **NVIDIA L4 (Ada Lovelace, sm_89)**, confirmed via
`nvidia-smi` — not the RTX PRO 4000 Blackwell (sm_120) the "Pinned
versions" section below describes. `docker/Dockerfile.curobo`'s
`TORCH_CUDA_ARCH_LIST` was changed to `8.9+PTX` to match (see that file's
own comment) — Blackwell-compiled kernels do not run on Ada hardware (PTX
compatibility is forward-only). Driver 570.211.01 was used deliberately,
not whatever `cuda-drivers` picks as "latest" — a first attempt on driver
610.43.02 (the newest available) segfaulted deep in Isaac Sim's own RTX
renderer plugin during Kit startup, before any project code ran; NVIDIA's
own docs list 535.161.07 as Isaac Sim 5.1.0's recommended driver, but 570.x
was chosen instead as the closest match to what CUDA 12.8 (cuRobo's own
requirement) actually ships against.

**Known gap, not fixed here**: `table_layout.yaml`'s `assembly_parts` (a
PCB-assembly pick-place part, unrelated to the CR5 arm itself) references
`mantra scanner/STEP_Mantra_Scanner/PCB Assembly_color_fixed.usd` — real
user-uploaded CAD, gitignored, no public download URL (unlike the factory
backdrop). Not present on this particular VM. The raw (non-color-fixed)
CAD *is* recoverable from git history on a different branch
(`origin/newton`, commit `c7b8399`) if ever needed — see that commit's
tree for `mantra scanner/STEP_Mantra_Scanner/PCB Assembly.usd`, then run
`scripts/fix_cad_import_colors.py` on it to reproduce the `_color_fixed`
variant. All CR5 validation in this branch skipped `build_assembly_parts()`
entirely rather than chasing this down, since it's orthogonal to the arm.

**Gripper: not yet added.** DH-Robotics (the manufacturer behind "DH
electric servo gripper") has official URDF+meshes for **AG-95, AG-145,
PGC-140, and DH3** (`github.com/DH-Robotics/dh_gripper_ros`) but **no USD
for any model** — same URDF-only situation as the CR5 arm itself. Dobot
officially lists AG-95/AG-145, PGC-50/140/300, PGE-5/8/15/50, and RGI-14/30
as CR-series-compatible accessories (Modbus RTU/RS485, optional TCP/IP;
CR5's controller supports Modbus master natively via the official
`TCP-IP-CR-Python-V4` SDK). PGE-50-40 specifically is a real, current
product and is actually the *default* `GripperModel` in DH's own Modbus
driver — but has no URDF anywhere, unlike AG-95/PGC-140. If/when a gripper
is added: vendor its URDF the same way `robots/cr5/SOURCE.md` documents
for the arm, mount it on `Link6`, and expect to re-derive a new
`retract_config`/collision-sphere set again (adding a gripper changes the
kinematic chain and mass distribution) — don't assume this branch's
current `cr5.yml` values carry over unchanged.

## Must-know gotchas

- **cuRobo joint velocity/accel/jerk limits reject zero silently until
  `MotionGen` is fully built.** `MotionGenConfig.load_from_robot_config()`
  can "succeed" at loading a robot yml while still hard-failing later
  (`ValueError: Joint velocity limits is zero`) the first time
  `bound_cost.set_bounds()` actually runs — don't trust an early
  "loads without error" smoke test as proof a new robot config works.
  Velocity has **no config-level override** (read straight from the URDF);
  jerk/acceleration do (`cspace.max_jerk`/`max_acceleration`). See "CR5
  validation" above for the full story.
- **cuRobo's `collision_sphere_buffer` adds to every sphere's radius**,
  not just a display margin — a sphere that looks "tangent" to some plane
  using its raw declared radius will still poke through it once the buffer
  is added. Account for it explicitly when hand-placing collision spheres.
- **A `MotionGenStatus.INVALID_START_STATE_*`/`IK_FAIL` at a robot's own
  retract/home pose can mean the pose itself is a kinematic singularity,
  not a target-reachability bug.** Cheap to rule out: compute a numerical
  Jacobian (finite-difference FK, no cuRobo API needed) and check its
  singular values/condition number before assuming a nearby config or
  collision-sphere problem is at fault. See "CR5 validation" above.
- **`PhysicsScene` required.** `SingleArticulation.initialize()` (and
  anything else needing a PhysX simulation view) silently breaks without
  a real `PhysicsScene` prim on the stage — nothing in this repo's
  robot-import path creates one automatically. Define one explicitly:
  `UsdPhysics.Scene.Define(stage, "/physicsScene")`.
- **`timeline.play()` timing.** Calling it before `/physicsScene` exists,
  or before a long blocking call like `motion_gen.warmup()` (~30s, no
  `simulation_app.update()` of its own) finishes, corrupts PhysX's tensor
  simulationView — symptom: `AttributeError: 'NoneType' object has no
  attribute 'link_names'` on the next `SingleArticulation(...)`. Only
  drive `timeline.play()` yourself after both are done; normally a human
  clicks Play in the GUI instead.
- **Stop/Play rebuild.** A `SingleArticulation` is only valid for the
  PhysX simulation view that existed when it was built — clicking Stop
  tears that view down, and reusing a pre-Stop `SingleArticulation` after
  a later Play leaves the robot permanently unresponsive. Any interactive
  loop must track not-playing→playing transitions and rebuild
  `robot`/`idx_list`/`articulation_controller` on *every* fresh Play (see
  `run_teleop_loop()` in `scripts/mefron_lib/teleop.py`).
- **cuRobo plans in the robot's base-link frame, never world space.** Any
  USD world pose (e.g. a dragged teleop target) must go through
  `robot_base_pose.compute_local_pose(world_pose)` first, where
  `robot_base_pose` comes from wherever the robot was actually mounted.
- **URDF importer + file-backed stages.** Importing a robot into a stage
  opened directly from a real `.usd` file (`open_stage()`, what
  `mefron.py` does) makes the importer write a disk-persisted, multi-layer
  "Robot Description" (`configuration/` folder) as a side effect — this
  breaks `CopyPrim`-based duplication of anything under the robot (use
  `prim.GetReferences().AddInternalReference()` instead). A fresh
  anonymous stage with content brought in via `add_reference_to_stage()`
  (what `build_scene_mefron.py` does) avoids this entirely. Full detail:
  `docs/mefron-history.md`.
- **`SimulationApp` full experience breaks cuRobo's `packaging` import.**
  Passing `experience=.../isaacsim.exp.full.kit` (needed for the Physics
  debug-visualization menu) makes `from packaging import version` resolve
  to a broken internal copy. Fix: pre-load `packaging`/`packaging.version`
  from real `site-packages` before anything else imports cuRobo — see
  `scripts/mefron_lib/kit_bootstrap.py`'s `preload_real_packaging()`,
  called from the top of every mefron entry-point script. Full root-cause:
  `docs/mefron-history.md`.
- **`ninja`/`pip` are broken in this environment.** cuRobo's CUDA kernels
  JIT-compile (needs `ninja`) on a torch ABI mismatch; `pip install`
  itself doesn't work here. Fixed via `apt-get install ninja-build` — see
  `docs/docker-and-devcontainer.md`.

## Pinned versions

Isaac Sim `5.1.0`, cuRobo commit `ebb71702f3f70e767f40fd8e050674af0288abe8`,
torch `2.11.0+cu128` (CUDA 12.8). Dev GPU: RTX PRO 4000 Blackwell (sm_120)
— `TORCH_CUDA_ARCH_LIST` must be `12.0+PTX` for this GPU.

`TORCH_CUDA_ARCH_LIST` is GPU-specific, not a fixed constant — the `dobot`
branch's CR5 validation ran on a cloud box with an **NVIDIA L4 (Ada
Lovelace, sm_89)** instead, which needs `8.9+PTX` (already set in
`docker/Dockerfile.curobo`, with the GPU-detection reasoning in that file's
own comment). Check `nvidia-smi` before assuming the Blackwell value above
applies. NVIDIA driver: use something in the 570.x branch (matches CUDA
12.8's own reference release) — confirmed live that driver 610.43.02 (the
newest available via `cuda-drivers` at the time) segfaults in Isaac Sim
5.1.0's own RTX renderer during Kit startup, unrelated to CUDA/torch
(those worked fine even on 610.x) — NVIDIA's docs list `535.161.07` as
Isaac Sim 5.1.0's official recommended driver, which predates CUDA 12.8
support; 570.x was the practical middle ground.
