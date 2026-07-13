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
  the headless PASS/FAIL regression test for this whole path. The mounted
  gripper is a second vendored piece, `robots/pgc140/` (URDF+meshes,
  `SOURCE.md`) — `scripts/generate_cr5_pgc140_urdf.py` splices it onto the
  CR5 arm into a generated `robots/cr5_pgc140/urdf/combined.urdf`, which is
  what `configs/curobo/cr5.yml` and `import_cr5.py` actually import/consume
  (see "Gripper" below — now fully confirmed live, both mounted and
  standalone). `scripts/pgc140_gripper_probe.py` imports just the PGC-140
  (no CR5 arm) fixed-base into an empty stage with the same C/O
  keyboard control as `build_scene.py`'s teleop loop, for isolating the
  gripper's own open/close behavior from the arm entirely.
- `examples/curobo_reference/` — pristine, unmodified copy of cuRobo's own
  interactive teleop demo. **Do not modify these two files**; write a
  separate script instead (`scripts/mefron.py` is exactly that). See
  `docs/mefron-history.md` for the environment fixes needed to run it and
  a license caveat on its header comments.
- `scripts/mefron_lib/` — shared package backing every mefron entry-point
  script (`mefron.py`, `mefron_gripper_probe.py`, `test_mefron_*_headless.py`):
  `kit_bootstrap.py` (packaging preload + stale-config cleanup, stdlib-only
  so it's safe to import before `SimulationApp` exists), `config.py` (all
  constants), `grasp.py` (pose math), `robot.py` (mount/friction/drive),
  `teleop.py` (keyboard control + `run_teleop_loop()`).
  `mefron2.py` (superseded diverged copy), `build_scene_mefron.py`
  (see its own entry below), and the standalone Grasp Editor diagnostic
  scripts this repo's `docs/grasp-and-assembly-offsets.md` refers to
  (`mefron_grasp_editor_scene.py`, `franka_grasp_editor_scene.py`,
  `panda_hand_grasp_editor_scene.py`) were all removed as unused bloat —
  their history is preserved in git and in that doc; nothing currently
  depends on them.

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

`scripts/build_scene_mefron.py` (+ `configs/scene/mefron_layout.yaml`) was
the architecturally-preferred alternative — same goal, but a fresh
anonymous stage with `mefron.usd` referenced in, which avoided most of
`mefron.py`'s bugs by construction — but sat **dormant** (deriving the
grasp/assembly offsets requires temporarily reparenting prims in the Stage
tree, which only works when `mefron.usd` is opened directly, so active
work always happened in `mefron.py` instead) and was **removed** as unused
bloat. See `docs/mefron-history.md` for its full history if ever revived.

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

**Status: bare-arm CR5 mount + cuRobo teleop confirmed working, and
confirmed smooth** (see bug #5 below — an earlier pass only checked that
the arm reached the goal, not how cleanly). `configs/scene/table_layout.yaml`'s
`cr5_mount.robot_override.enabled` is now `false` — `scripts/build_scene.py`
mounts the real, already-vendored CR5 (`robots/cr5/`, from the official
`Dobot-Arm/TCP-IP-ROS-6AXis` repo — no USD exists anywhere for the CR5,
official or community) instead of the Franka stand-in it used before.
`scripts/test_teleop_headless.py --headless` passes: `plan_single
success=True`, robot moves in response to a simulated drag. No gripper yet
(see below) and the mefron scanner-assembly scene
(`build_scene_mefron.py`/`mefron_layout.yaml`) hasn't been touched — this
was validated in the generic `table_layout.yaml` testbed only.

Getting a passing run required fixing four real, previously-undiscovered
bugs in the CR5's first-draft config (not just flipping the override flag),
plus a fifth found on a later pass chasing a motion-*quality* bug (the arm
reached the goal, just not cleanly):

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
5. **The CR5's actual joint drives were running with `damping=0`** — a
   fully undamped spring — regardless of `import_cr5.py`'s stated
   `default_drive_strength=1e5`/`default_position_drive_damping=1e4`.
   Reported as "the arm swings back and forth (totem pole) right as a move
   starts and right as it stops, but is smooth mid-traversal" — exactly
   where a time-optimal trajectory's jerk peaks, and exactly what an
   undamped drive rings hardest against. Confirmed two compounding causes:
   - `run_teleop_loop()`/`setup_curobo_motion_gen()` ran cuRobo's plan at
     its full, natural (un-derated, non-dilated) speed — `MotionGenPlanConfig()`
     and `MotionGenConfig.load_from_robot_config()` were called with no
     `time_dilation_factor`/`velocity_scale`/`acceleration_scale` at all,
     unlike the one other teleop loop in this repo
     (`scripts/mefron_lib/config.py`'s `_TELEOP_VELOCITY_SCALE`/
     `_TELEOP_ACCELERATION_SCALE` = 0.5, `_TELEOP_TIME_DILATION_FACTOR` =
     0.3, driving a Franka there). Fixed by adding the same three knobs,
     config-driven via `table_layout.yaml`'s `teleop_target.teleop_*`
     keys — confirmed via headless per-joint velocity logging that the
     *planned* trajectory was already a clean trapezoid; this alone wasn't
     enough (see next point), but is a real, necessary derating on top of
     it. Required also bumping `test_teleop_headless.py`'s
     `_MAX_ITERATIONS` 200 → 3000, since a properly-derated/dilated
     trajectory takes ~10x more applied waypoints (350 vs. 33) to finish,
     and `run_teleop_loop()` has no "stop once the plan completes early"
     exit — a too-small budget doesn't fail loudly, it silently stops
     observing partway through the move.
   - Even after that, per-joint (not just aggregate) planned-vs-measured
     velocity logging showed `joint4`/`joint6` specifically diverging
     from the plan — *growing*, not decaying, worst during deceleration
     (`joint4` climbing to +0.58 rad/s, `joint6` to -0.48..-0.61 rad/s,
     while the plan called for ~0). Ruled out physical self-collision
     between those links first (`import_cr5.py`'s `self_collision=False`
     disables all self-collision contact for this articulation, confirmed
     against the importer's own UI tooltip). Root cause: introspecting the
     live USD `DriveAPI` directly after import showed every joint as
     `type=acceleration, stiffness=625, damping=0`, even though the
     `ImportConfig` object itself held `default_drive_strength=1e5`/
     `default_position_drive_damping=1e4` correctly right before
     `URDFParseAndImportFile` ran — those two fields simply don't reach
     the authored joints on this Isaac Sim version (also tried
     `ImportConfig.override_joint_dynamics = True`: changes damping to
     small per-joint values instead of 0, but stiffness stays pinned at
     625 and neither field's requested value ever lands). Fixed by
     explicitly re-authoring `UsdPhysics.DriveAPI` stiffness/damping
     directly on each joint right after import — a new opt-in
     `joint_drive_stiffness`/`joint_drive_damping` param on `import_cr5()`,
     wired up only for the real CR5 (not the Franka-override branch, whose
     own tuning hasn't been checked against this same behavior), values
     from `table_layout.yaml`'s `cr5_mount.joint_drive` (`stiffness=625`,
     kept from the importer's own incidental value; `damping=50`, exactly
     critically-damped for that stiffness in acceleration-mode terms:
     `2*zeta*omega_n` with `omega_n=sqrt(625)=25`, `zeta=1`). Confirmed via
     the same per-joint logging that this — not the derating above — is
     what actually removes both the initial kick and the end-of-motion
     divergence.

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

**Gripper: PGC-140 mounted, arm+drag regression passes live, both fingers
now confirmed reaching their commanded open/closed targets.** DH-Robotics
has official URDF+meshes for
**AG-95, AG-145, PGC-140, and DH3** (`github.com/DH-Robotics/dh_gripper_ros`)
but **no USD for any model** — same URDF-only situation as the CR5 arm
itself. The originally-targeted **PGE-50-40 has no URDF, USD, or public
CAD anywhere** (confirmed against DH-Robotics' own repo, a community ROS2
port, and general web/CAD-site search) despite being a real, current
product and the *default* `GripperModel` in DH's own Modbus driver — so
**PGC-140** was substituted: same parallel-jaw (non-adaptive-linkage)
mechanism as PGE, closest stroke of the four vendor-URDF'd options (50mm
vs. PGE's 40mm, vs. AG-95/AG-145's 95/145mm), and Dobot's own documented
CR-series accessory. Tradeoff: PGC-140 is ~2.5x heavier than PGE-50-40
would have been (~1kg vs ~0.4kg).

Vendored at `robots/pgc140/` (URDF+STL meshes from `dh_gripper_ros`'s
`dh_pgc140_urdf/`, commit `f59f9c2f4bc8eb116448b1d798791424bf64e337`,
license **BSD** per that package's `package.xml` — no repo-root `LICENSE`
file exists upstream to copy verbatim, unlike the CR5's MIT one). Every
link/joint name was renamed with a `pgc140_` prefix (`SOURCE.md`'s own
"Modifications made" section) to resolve a confirmed hard collision: both
the CR5 URDF and the PGC-140 URDF name their root link `base_link`. The
vendored URDF's own `<mimic joint="pgc140_finger1_joint" .../>` tag on
`pgc140_finger2_joint` was **removed** (a real, live-confirmed bug in this
Isaac Sim version — see bug #8 below), not kept — both finger joints are
now ordinary, independent prismatic joints.

cuRobo needs one `urdf_path` for the whole arm+gripper chain (matches how
its own bundled `franka.yml` includes hand+fingers in the arm's own URDF),
so `scripts/generate_cr5_pgc140_urdf.py` splices the two vendored URDFs
into a generated `robots/cr5_pgc140/urdf/combined.urdf` (mechanically
generated — regenerate via that script after changing either source,
don't hand-edit it) via one new fixed joint, `Link6` → `pgc140_base_link`
(currently an identity transform — a first-draft guess, not yet visually
confirmed flush against the real mounting face). `configs/curobo/cr5.yml`
was updated to match: `urdf_path` now points at the combined file,
`ee_link` moved from `Link6` to `pgc140_base_link` (a real-geometry link,
not a synthetic TCP frame — `build_teleop_target()` needs `ee_link` to
have actual `/visuals` content), both `pgc140_finger1_joint` and
`pgc140_finger2_joint` were added to `cspace` + `lock_joints` (locked
open — see bug #7 below for why "open" is `0.0`, not the URDF's own
`upper` value), mirroring `franka.yml`'s convention of tracking both its
independent finger joints. `import_cr5.py` gained `tune_gripper_drive()`
(linear, not angular, DriveAPI — the same distinction `mefron_lib/
robot.py`'s `stiffen_gripper_drive()` draws for the Franka) and
`filter_gripper_self_collision()` (see bug #9). `build_scene.py`'s
`run_teleop_loop()` gained C/O keyboard gripper control, ported from
`mefron_lib/teleop.py`'s pattern — same ramped-setpoint-applied-every-
frame mechanism, with the arm's own `idx_list`/`cmd_plan` explicitly kept
gripper-joint-free (a real, confirmed-necessary fix — see bug #6) so the
gripper-override block is the sole writer for that DOF.

**Confirmed live** (NVIDIA RTX A6000, driver 570.211.01, via
`${ISAACSIM_ROOT_PATH}/python.sh scripts/test_teleop_headless.py
--headless`; config tests run the same way via `tests/test_configs.py`'s
functions called directly, since `pytest` isn't installed and `pip` is
broken here — see "Must-know gotchas"): the combined URDF imports
cleanly, `plan_single` succeeds, the arm-drag regression passes
(`max joint-position delta: 1.1846 rad`), and **both** finger joints now
reliably reach their commanded target in both directions —
`pgc140_finger1_joint`/`pgc140_finger2_joint` closed end positions
`[0.02499904, 0.02492332]` (target `0.025`) and open end positions
`[0.0, 1.68e-07]` (target `0.0`), both well within the 2mm tolerance. This
supersedes bug #10 below (now RESOLVED, see its own entry). Four real,
previously-unknown bugs were found and fixed getting this far (numbered
continuing from the arm's own list above, all confirmed via the same
"don't trust it, check it live" discipline that list already
establishes):

6. **The arm's own `cmd_plan`-apply block was also writing to the
   gripper's DOF.** `cspace.joint_names` growing to include the gripper
   joints meant `idx_list`/`j_names` (built directly from that list)
   included them too — every `interpolation_dt` tick, the arm's periodic
   waypoint-apply re-asserted cuRobo's `lock_joints` constant onto the
   gripper's own joint, fighting the gripper-override block's every-frame
   write. Fixed by filtering the arm's own `idx_list`/`cmd_plan`-reindex
   down to just the 6 arm joints (`scripts/build_scene.py`'s
   `run_teleop_loop()`, see its own comment). Confirmed this fix alone
   does *not* explain the still-open bug #10 below — tried in isolation,
   zero measurable effect on it.
7. **The gripper's open/closed convention was backwards.** `q=0` (the
   URDF's own `lower` limit) is actually the *open* position (fingers
   spread, ~30mm off the gripper centerline) and `q=0.025` (`upper`) is
   *closed* (fingers converged to ~13mm) — confirmed by directly computing
   each finger's world position at both extremes via each joint's own
   `<origin rpy=.../>`. An earlier draft had `retract_config`/
   `lock_joints`/`table_layout.yaml`'s `open_position`/`closed_position`
   all backwards, silently locking the gripper *closed* during planning
   (the opposite of the intended "most permissive default collision
   footprint"), and self-collided the two fingers' own spheres against
   each other at that (actually-closed) locked value —
   `MotionGenStatus.INVALID_START_STATE_SELF_COLLISION` on every
   `plan_single` call. Fixed in `configs/curobo/cr5.yml`,
   `configs/scene/table_layout.yaml`, and `tests/test_configs.py`
   together; also needed two new `self_collision_ignore` entries found via
   the same all-pairwise sphere-distance check bug #3 used (`Link5`↔
   `pgc140_base_link`, `Link6`↔ both finger links — confirmed live, zero
   overlapping pairs remain).
8. **The URDF importer mis-imports a prismatic `<mimic>` tag.**
   `pgc140_finger2_joint`'s vendored `<mimic>` tag got imported as
   `PhysxMimicJointAPI:rotX` — a *rotational* mimic API applied to what is
   actually a *linear* joint — with mangled limits (`lower=-0.005,
   upper=0.030` instead of the URDF's own `0`/`0.025`) and no
   `UsdPhysics.DriveAPI` attached at all, confirmed by direct USD
   introspection right after import. Fixed by removing the `<mimic>` tag
   from `robots/pgc140/urdf/pgc140_robot.urdf` entirely (see that
   directory's `SOURCE.md`) and treating both finger joints as ordinary,
   independently-driven joints — the same pattern already proven for the
   Franka's own two (never mimic-linked) finger joints.
9. **`import_cr5()`'s `self_collision=False` does not actually author any
   USD-level self-collision exclusion.** Confirmed by introspecting the
   imported stage directly: neither a `PhysxArticulationAPI.
   enabledSelfCollisions` attribute nor any `UsdPhysics.FilteredPairsAPI`
   relationship exists anywhere under the articulation root, despite this
   import setting. Same "importer setting doesn't reliably land" pattern
   as bug #5's drive strength/damping. Fixed by explicitly authoring a
   `FilteredPairsAPI` exclusion between the two finger links
   (`import_cr5.py`'s `filter_gripper_self_collision()`) — a real,
   worthwhile fix on its own terms, but **confirmed live it does NOT
   explain bug #10 below** (applied in isolation, zero change to that
   symptom).

**RESOLVED (bug #10):** previously, commanding both finger joints closed
reliably left `pgc140_finger1_joint` stuck at a small, exactly-repeatable
intermediate position (~0.0089) while `pgc140_finger2_joint` reached its
own target cleanly — direction-dependent (which finger stalled flipped
between open and close). `import_cr5.py`'s `disable_gripper_finger_gravity()`
(direction-dependent steady-state offset under a P+D-only acceleration-mode
drive matches a constant-disturbance signature; the two fingers' motion
axes are deliberately mirrored, so gravity's component along each finger's
own axis differs between them — see that function's own docstring for the
full reasoning) was written to fix this but, as of the last update to this
file, had not yet been confirmed live. **Now confirmed live, on two
independent paths**: (1) `scripts/test_teleop_headless.py --headless` on
the full CR5+gripper articulation — both directions now PASS within the
2mm tolerance (see the "Confirmed live" paragraph above); (2)
`scripts/pgc140_gripper_probe.py --headless` — the gripper *alone* (no CR5
arm, same `tune_gripper_drive()`/`filter_self_collision_from_curobo_config()`/
`disable_gripper_finger_gravity()` calls reused directly from
`import_cr5.py`) also closes/opens both fingers cleanly (`[0.02499996,
0.02499996]` closed, `[0.0, 0.0]` open) — confirming the fix isn't an
artifact of anything CR5-mount-specific, and that the gripper's own
tuning is correct in isolation too.

Building that standalone probe surfaced two new, previously-unknown bugs
in the *general* URDF-import pipeline (not gripper-specific, but only ever
exercised via `build_scene.py`'s full pipeline before, which happens to
route around both by construction — see each entry):

10. **`MovePrim` inside `import_cr5()` never checks its own return status,
    and silently no-ops if the destination's parent prim doesn't exist
    yet.** `build_scene.py`'s real pipeline never hits this: its
    `build_factory()` calls `add_reference_to_stage(usd_path=...,
    prim_path="/World/Factory")` before `mount_cr5()` ever runs, and that
    utility defines any missing ancestor prims of its target path as a
    side effect, so `/World` always already exists by the time
    `import_cr5()`'s own `MovePrim(path_from=imported_prim_path,
    path_to=prim_path)` runs. `scripts/pgc140_gripper_probe.py`'s bare
    stage has no such prior step, so `/World` never existed — confirmed
    live via direct command tracing that `URDFParseAndImportFile` returned
    `imported_prim_path='/pgc140_robot'` (a stage-root sibling, matching
    the URDF's own `<robot name=...>`) and `MovePrim` to `/World/GripperProbe`
    silently failed, leaving `prim_path` a valid but permanently childless
    prim while the real link/joint hierarchy stayed behind, unmoved, at
    `/pgc140_robot`. The resulting `SingleArticulation()` call then failed
    with `AttributeError: 'NoneType' object has no attribute
    'is_homogeneous'` — reads exactly like the physics-view-timing race
    the "Must-know gotchas" section already documents, but isn't one (more
    settle frames had zero effect, since nothing was ever going to move
    into place no matter how long the wait — only tracing the raw
    command's own return value surfaced the real cause). Fixed in
    `scripts/pgc140_gripper_probe.py` by explicitly defining `/World` via
    `UsdGeom.Xform.Define()` before importing — see "Must-know gotchas"
    below for the general-purpose version of this gotcha.
11. **`URDFParseAndImportFile`'s asset population runs a beat behind the
    command's own return** (its `isaacsim.asset.importer.urdf` "Creating
    Asset in an in-memory stage" log line was observed printing after
    code that already assumed the import had finished) — `build_scene.py`'s
    real pipeline never notices because `setup_curobo_motion_gen()`'s
    `motion_gen.warmup()` (~30s) always runs between `import_cr5()` and the
    first `SingleArticulation` build, incidentally giving the async import
    plenty of time; `pgc140_gripper_probe.py` has no such gap. Fixed with
    an explicit `120`-frame `simulation_app.update()` pump right after
    import, the same convention `build_scene.py`'s `build_factory()`
    already uses for the (also asynchronous) factory backdrop reference.

**Still open, not yet checked live:**
- `cr5_collision_spheres.yml`'s three gripper-link sphere sets are rough,
  joint-origin-derived placeholders, not fit to real mesh geometry.
- `table_layout.yaml`'s `cr5_mount.gripper.joint_drive.stiffness`/`damping`
  reuse the arm's own `625.0`/`50.0` as a starting placeholder, not
  re-derived — plausibly fine as-is (both are acceleration-mode drives,
  which divide out effective inertia, so mass alone doesn't necessitate
  new numbers), and now indirectly confirmed reasonable by bug #10's own
  resolution (both fingers converge cleanly to their targets with these
  values), but still not independently re-derived via per-joint
  planned-vs-measured velocity logging the way bug #5's arm values were.
- `maxForce` (`table_layout.yaml`'s `cr5_mount.gripper.max_force: 140.0`,
  matching the URDF's own `effort="140"`) was confirmed to land correctly
  via direct DriveAPI read-back in isolation (a bare gripper-only import,
  no arm) — not yet re-confirmed on the full combined articulation.
- The `Link6`↔`pgc140_base_link` mount transform (identity) is still an
  unverified first-draft guess — dial in visually in the GUI the same way
  `cr5_mount.position` itself was, then recompute `teleop_target.position`/
  `orientation_wxyz` (currently numerically correct *only* because the
  mount transform happens to be identity right now — see that config's own
  comment).

**Grasp Editor asset**: `assets/Grasp_Editor/` (renamed from
`assets/grasp_editor_tutorial/Grasp_Editor_Tutorial_Stage/`, flattened up
one level — `grasp_editor.usd` is the renamed top-level scene, was
`grasp_editor_tutorial.usd`; no other tracked file in this repo hardcodes
the old path, so nothing else needed updating for the rename —
`scripts/mefron_lib/config.py`'s `GRASP_EDITOR_YAML_PATH` is unrelated,
pointing at `assets/finger_print_scanner.yaml`) is NVIDIA's own official
Grasp Editor Tutorial sample content (the scene
+ `Isaac/Robots/Franka/` assets) — used successfully elsewhere in this
repo already (consumed by `mefron.py`'s J key), unlike the *different*,
abandoned in-repo attempt against `mefron.usd`'s own broken Franka import
(see `docs/grasp-and-assembly-offsets.md`). Confirmed live by direct USD
introspection: the scene never actually references
`Isaac/Robots/Franka/franka.usd` (the full arm, unused); it builds
`/World/panda_hand` directly instead — the Franka hand+fingers only,
physics joints authored right there in that stage.

A first attempt built a *scripted*, gripper-only equivalent for the CR5
(`scripts/generate_cr5_gripper_grasp_editor_usd.py`, importing
`robots/pgc140/urdf/pgc140_robot.urdf` via `import_cr5()` into an anonymous
stage and extracting its subtree with `Sdf.CopySpec()`). It produced a
structurally-valid 2-DOF articulation (confirmed live) but **crashed the
real Grasp Editor extension** on selection
(`AttributeError: 'NoneType' object has no attribute 'link_names'`, preceded
by `prim '.../root_joint' was deleted while being used by a tensor view
class`) — root-caused by reading the extension's own source
(`isaacsim.robot_setup.grasp_editor/ui_builder.py`) and comparing against
the tutorial's working Franka: `import_cr5()`'s default `fix_base=True`
authors a real `PhysicsFixedJoint` anchoring the robot to the world with
`ArticulationRootAPI` on *that joint*, whereas the working Franka has
`ArticulationRootAPI` on a plain Xform ancestor with nothing anchoring it to
the world. `import_cr5()` gained a `fix_base: bool = True` parameter (still
defaults to preserve every other caller's behavior) to address this, but the
generator script itself was **removed** (per direct user feedback: one-off
generator scripts for single output assets were adding more repo bloat than
value) in favor of doing the equivalent import by hand in the GUI.

**Current state (manual GUI workflow, replacing the deleted script)**:
`robots/cr5_pgc140_gripper/urdf/cr5_pgc140_gripper.urdf` is a hand-edited
copy of the combined CR5+gripper URDF (**the full arm, not gripper-only**
like the Franka's own `/World/panda_hand` — a deliberate deviation from that
established convention, not yet reconciled) with a `dummy_link`/`dummy_joint`
pair prepended as the new root, imported via the Isaac Sim GUI's own URDF
Importer with **Links → Moveable Base** (the GUI's current name for
`fix_base=False`) into
`assets/Grasp_Editor/Isaac/Robots/CR5/cr5_pgc140_gripper/`
(as a "Referenced Model" — produces the same layered `configuration/` folder
described in "Must-know gotchas" below, confirmed live from this exact
import). **Confirmed live**: Moveable Base fixes the crash — the Grasp
Editor now opens this gripper without crashing, `/World/cr5_pgc140_gripper`
selectable as the articulation, `/World/mug` as the rigid body.

**RESOLVED** (see above): `ArticulationRootAPI` moved from `dummy_link`
to `/cr5_pgc140_robot` itself, mirroring `panda_hand`'s own "root
container, no RigidBodyAPI" pattern — fixes Select-Frames-of-Reference.
Also needed here, none of it covered by `import_cr5()` for this manually
GUI-imported asset: a `rootJoint` (plain `UsdPhysics.Joint`, `body1`
empty) so it doesn't fall under gravity; `disable_gripper_finger_gravity()`/
`tune_gripper_drive()` (damping was `0.25`, ended up matching Franka's own
`stiffness=10000`/`damping=1000` — `maxForce` stays `140` from the PGC-140's
real spec, not Franka's unrelated `7.2N`)/self-collision filtering
reapplied by hand; plus a `pgc140_finger1_link`↔`finger2_link` filter
beyond `cr5.yml`'s own list, since here the fingers can close fully
together with nothing between them. Articulation/rigid-body
`solverPositionIterationCount`/`solverVelocityIterationCount` (velocity
was `1` everywhere, a real bottleneck for a symmetric two-finger grasp's
simultaneous contacts) also needed raising, on both the gripper *and*
whatever it's grasping.

Any `CollisionAPI` must sit on the actual mesh, not a parent Xform, or
it's silently inert (hit this repeatedly — the gripper, a test
`/World/mug`, and `finger_print_scanner`). A `PhysX error: ...
foundLostAggregatePairsCapacity` in the console means an undersized GPU
dynamics buffer is silently dropping collisions. **A newer, sneakier
variant of the same duplicate-collider mistake**: editing a *referenced*
asset's collider live in the GUI (Colliders Preset, changing
Approximation) authors the change as an override in whichever file is
currently open — not in the referenced source asset — so a part's own
`.usd` file can look perfectly clean when opened standalone while the
*composed* scene still carries an extra, conflicting collider (e.g. a
stray `convexDecomposition` layered on top of a correct `sdf` collider).
Always check the fully composed stage, not just the source file, and use
`Usd.PrimRange.AllPrims`/`GetAllChildren()` rather than the default
traversal when auditing — USD silently skips instancing `Prototypes`
scopes (and their real geometry) under the default predicate.

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
- **URDF importer's `ImportConfig.default_drive_strength`/
  `default_position_drive_damping` don't reliably reach the actual
  authored joints** (confirmed on the pinned Isaac Sim version here) —
  the `ImportConfig` object holds whatever you set correctly, but the
  resulting `UsdPhysics.DriveAPI` on each joint can come out completely
  different (`type=acceleration, stiffness=625, damping=0` regardless of a
  requested `1e5`/`1e4`, for `robots/cr5/urdf/cr5_robot.urdf`) — don't
  trust these fields actually took effect just because the import
  succeeded; read back `UsdPhysics.DriveAPI.Get(joint_prim, "angular")`'s
  own attributes after import to check. If they're wrong, re-author them
  directly post-import instead (see `import_cr5()`'s
  `joint_drive_stiffness`/`joint_drive_damping` params and "CR5
  validation"'s bug #5 above) — don't assume adjusting the `ImportConfig`
  values will change anything.
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
  (what `build_scene.py`'s `build_factory()` does) avoids this entirely.
  Also confirmed live via the Isaac Sim GUI's own URDF importer: choosing
  "Referenced Model" output mode produces this exact same layered
  `configuration/` folder (see `assets/Grasp_Editor/.../CR5/
  cr5_pgc140_gripper/configuration/` for a live example) — it's not
  specific to the Python `URDFParseAndImportFile` command used elsewhere in
  this repo, it's how the importer behaves whenever the target stage is
  file-backed rather than anonymous, GUI or scripted alike. Full detail:
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
- **`MovePrim` silently no-ops if the destination's parent doesn't exist,
  and `import_cr5()`/similar importer wrappers don't check its return
  status.** A bare/anonymous stage (e.g. a headless script that never
  calls `add_reference_to_stage()` first) may have no `/World` prim at
  all — importing a robot at `prim_path="/World/Something"` then leaves
  the real content behind, unmoved, at wherever
  `URDFParseAndImportFile` actually put it (a stage-root sibling matching
  the URDF's own `<robot name=...>`), while the intended path is a valid
  but permanently empty prim. The resulting `SingleArticulation()` call
  fails with `AttributeError: 'NoneType' object has no attribute
  'is_homogeneous'`/`'link_names'` — reads exactly like the physics-view-
  timing race below, but isn't; more settle frames won't fix it. Confirm
  by tracing the raw `omni.kit.commands.execute("URDFParseAndImportFile",
  ...)` return value directly if a "did not match any rigid bodies"
  PhysX error shows up. Fix: `UsdGeom.Xform.Define(stage, "/World")` (or
  whatever the target's parent is) before importing — see
  `scripts/pgc140_gripper_probe.py`'s `spawn_gripper_probe()` and CR5
  validation's bug #10 above.
- **`URDFParseAndImportFile`'s asset population is asynchronous** — the
  command returns a prim path immediately, but the stage isn't actually
  populated yet (confirmed live: its own "Creating Asset in an in-memory
  stage" log line printed *after* code that already assumed the import
  had finished). `build_scene.py`'s real pipeline never notices because
  `motion_gen.warmup()` (~30s) always runs first, incidentally giving it
  time; a script that imports and immediately builds a
  `SingleArticulation` with nothing in between needs an explicit
  `simulation_app.update()` pump after import (see `build_factory()`'s
  120-frame pump for the same reasoning applied to the factory backdrop
  reference, and CR5 validation's bug #11 above).

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
