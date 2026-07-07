# Session brief: mefron-scene gripper control + grasp-physics tuning

**Purpose of this file**: a handoff so a fresh Claude Code session (or a
teammate) can pick up exactly where this session left off, without having
to re-derive anything already verified here. Everything below was directly
confirmed against this repo's actual files or this Isaac Sim install's
actual schema/source — not assumed — unless explicitly marked as
"not yet tested" or "pending."

This is a session artifact, not project documentation — see `CLAUDE.md` for
the authoritative, permanent project record. If you want the findings below
folded into `CLAUDE.md`'s own style/conventions, that's a reasonable next
ask but wasn't done automatically.

## Repo state as of this session (uncommitted)

```
 M docker/Dockerfile.curobo        # pre-existing, not touched this session
 M CLAUDE.md                       # pre-existing, not touched this session
?? scripts/build_scene_mefron.py   # NEW this session's target file (untracked before too)
?? configs/scene/mefron_layout.yaml
?? scripts/mefron.py               # pre-existing, superseded, not touched
?? scripts/test_teleop_headless.py # pre-existing, not touched (targets build_scene.py, not build_scene_mefron.py)
?? assets/mefron/, .claude/, "mantra scanner/", scenes/   # pre-existing untracked dirs
```

All code changes this session are in **`scripts/build_scene_mefron.py`**
and **`configs/scene/mefron_layout.yaml`** — nothing has been committed.

## 1. Keyboard gripper open/close (done, verified headlessly)

Context: the project had cuRobo-driven arm teleop (drag a target, robot
follows) but zero gripper control anywhere. Compared Isaac Sim's **Grasp
Editor** (GUI tool, authors reusable grasp poses for automated/scripted
picking) vs simple **keyboard open/close** (direct joint control in the
live teleop loop). Chose keyboard: cuRobo's own `franka.yml` already
`lock_joints`-excludes the two finger joints from IK/trajopt entirely, so
finger actuation was always going to be orthogonal to cuRobo either way —
Grasp Editor only pays off once something *autonomously* computes a grasp
target pose, which nothing here does yet (every `plan_single()` call is
driven by a human-dragged target).

**Scope note**: originally planned against `scripts/build_scene.py` (the
main `table_layout.yaml` pipeline), then explicitly redirected mid-session
to `scripts/build_scene_mefron.py` instead, since that's the scene the user
was actively working in. **`build_scene.py`'s `run_teleop_loop()` does NOT
have gripper control** — this was never ported there. Same for
`scripts/mefron.py` (superseded by `build_scene_mefron.py`, per
`CLAUDE.md`'s own notes).

Implementation in `build_scene_mefron.py`:
- `GripperKeyboardControl` class + `build_gripper_keyboard_control()` —
  subscribes to real `carb.input` keyboard events (`C` closes, `O` opens).
  API confirmed against this exact install's own stubs
  (`carb/input.pyi`, `omni/appwindow/_appwindow.pyi`), not memory.
- `GRIPPER_OPEN_POSITION = 0.04` / `GRIPPER_CLOSED_POSITION = 0.0` —
  confirmed by reading `franka_panda.urdf`'s actual joint limits
  (`panda_finger_joint1/2`, prismatic, `lower="0.0" upper="0.04"`).
- Wired into `run_teleop_loop()` via a new `gripper_control` param — applied
  every playing frame via its own `ArticulationAction`, placed *after* the
  existing arm `cmd_plan` block so it always wins that frame's write to the
  finger joints (necessary because `get_full_js()` re-applies cuRobo's own
  locked-open value on every planned frame otherwise).
- **Verified**: `build_scene_mefron.py --headless` runs clean end-to-end
  (`curobo motion_gen: READY`, all status prims `OK`).
- **Not yet verified**: real interactive keypress in the GUI (only the
  underlying joint-drive mechanism was verified headlessly — no live
  keyboard device in a headless run). Also no permanent headless regression
  test was added for this (offered, user didn't request it).

## 2. Mefron scene remount: pedestal → SEKTION table

User removed `mefron.usd`'s old `Pedestal_plates/Cube_05` mount plate in
the GUI and added a SEKTION table (`/World/sektion_cabinet_instanceable`,
a `/World` sibling of `Factory` — same level as `packing_table` etc., NOT
nested inside `Factory` like the old pedestal was). Manually placed a
Franka copy on top of it to derive the new mount pose.

Updated in `configs/scene/mefron_layout.yaml`:
- `cr5_mount.position` → `[2.74097, -4.782, 0.7924]` (read from the user's
  manually-placed Franka's Translate in the GUI — **not** independently
  verified via a `get_world_pose()`/`BBoxCache` script this time, unlike
  most other poses in this project; if the robot ends up floating/clipping
  through the table, re-check this value).
- `cr5_mount.pedestal` renamed to **`cr5_mount.mount_surface`**,
  `prim_path` → `/World/Factory/sektion_cabinet_instanceable`. Two call
  sites in `build_scene_mefron.py` (`get_teleop_obstacles()`, the status
  print list) updated to match the rename.
- `teleop_target.position` recomputed **algebraically** (old target minus
  old mount, applied to the new mount position — valid only because
  `cr5_mount.orientation_wxyz` is still identity in both cases) rather than
  re-derived from scratch → `[2.851551, -4.782, 1.383112]`.

**Verified**: `build_scene_mefron.py --headless` runs clean with these
values (`curobo motion_gen: READY`, all status paths `OK`, including the
new table path).

**Known loose end**: there's a stray, unpositioned `/panda` (or
`/World/panda`) prim in the user's live GUI session from an earlier manual
URDF import — confirmed via `import_cr5()`'s actual `MovePrim` behavior
that this is **not** produced by `mount_franka()`/`mount_cr5()` (which
always renames to `/World/Franka` or `/World/CR5`). User said they'd delete
it themselves; not confirmed done.

## 3. Grasp physics: three real, stacked bugs found and fixed

Symptom chain: object (`finger_print_scanner`) slipped out of the gripper
entirely → reducing its mass to 0.01kg made it hang on but "dangle"/swing
→ still slipped at the physically-real 0.05kg.

### 3a. No physics material anywhere (fixed)

Read-only headless inspection of `mefron.usd` found **zero**
`PhysxMaterialAPI` authored anywhere in the file — not on
`finger_print_scanner`/`main_holder`/`screen`, not a usable one on
`backpanel_support` (its bound material, `Black_Paint_01`, is a pure
render/`OmniPBR` material with no `PhysxMaterialAPI`), and no
`PhysicsScene`-level default either. Cross-checked the Franka side too:
`franka_panda.urdf` has no friction tags, `import_cr5.py` sets no friction
— so **both sides of every grasp contact were on PhysX's un-overridden
engine default** the whole time, explaining the slipping independent of
mass.

Fix: `apply_gripper_friction(cfg, robot_prim_path)` in
`build_scene_mefron.py` — creates one shared material at
`/World/GripperFrictionMaterial` (static friction 0.9, dynamic 0.8,
restitution 0.0) using the real, existing
`omni.physx.scripts.utils.addRigidBodyMaterial()` /
`physicsUtils.add_physics_material_to_prim()` helpers (not hand-rolled),
bound to `panda_leftfinger`, `panda_rightfinger`, and everything listed in
a new **`high_friction_prim_paths`** config key (currently just
`/World/Factory/finger_print_scanner`). **Runtime-only, not persisted** to
`mefron.usd` — deliberate, matches how every other robot-tied property in
this scene works, and the user explicitly chose "code now, GUI+save later
if I want it portable" for this category of fix.

**Verified**: headless read-back confirmed `static=0.9`/`dynamic=0.8` on
the material itself, and that all three target prims' physics-material
binding resolves to it via `UsdShade.MaterialBindingAPI.ComputeBoundMaterial(materialPurpose="physics")`.

### 3b. Collider approximation: Triangle Mesh on a dynamic body (fixed by user, via GUI)

`finger_print_scanner` is a dynamic rigid body but its collider was
`Triangle Mesh` (exact/concave) — a known-unstable combination in PhysX,
whose dynamic-body contact solver is built around convex shapes. User
switched it to **Convex Decomposition** via the GUI, on my recommendation
(preserves concave/pinch geometry that a plain Convex Hull would bulge
over and lose).

### 3c. Grip felt "dangling" even after 3a/3b (fixed, not yet live-tested)

Diagnosis: a two-finger pinch grip has almost no resistance to *rotation*
around the line between the two contact points — an object with off-center
mass (user described a "plateau/mountain" bulge on one side) will settle
at a tilted rest angle under gravity, same as a real gripper would. This
part is expected physics, not fully "fixable" by parameter tuning alone —
the residual **swinging/looseness** on top of that tilt is what's
addressable.

User hypothesized insufficient grip force. Verified: URDF hard-caps each
finger at `effort="20"` N (confirmed by reading `franka_panda.urdf`
directly) — ~40x the object's whole weight-equivalent force, so the
*ceiling* isn't the bottleneck. But a headless inspection of the actual
imported joint prims (`/World/CR5/joints/panda_finger_joint1|2`,
`UsdPhysics.DriveAPI` type `"linear"`) found `stiffness=625.0`,
`damping=10.0` — **not** the configured
`default_drive_strength=1047.2`/`default_position_drive_damping=52.36`
(the URDF importer evidently derives a different effective value for
prismatic joints than what's configured, and the URDF's own
`<dynamics damping="10.0"/>` on these two joints takes precedence over the
importer's default damping). At that stiffness, a typical 1-2cm grasp
position error only reaches ~6-12N of the 20N budget — real headroom left
unused, likely contributing to a soft/loose-feeling grip.

Fix: `stiffen_gripper_drive(robot_prim_path)` in `build_scene_mefron.py` —
raises `stiffness` to `10000.0` and `damping` to `200.0` via
`UsdPhysics.DriveAPI` on both finger joints, called right after
`apply_gripper_friction()` in `main()`.

**Verified**: headless read-back confirmed `stiffness=10000.0`,
`damping=200.0`, `maxForce=20.0` (ceiling correctly unchanged) on both
joints. **Not yet tested live** — this was the last code change made this
session; the user was about to relaunch and test when the conversation
moved to `main_holder`'s unrelated collision issue (see below).

## 4. Isaac Sim GPU/renderer crash — pre-existing, not caused by this session

Mid-session, the user's live GUI session segfaulted (crash deep in Kit's
Vulkan/RTX renderer plugin, not in Python, not in any code from this
session). Checked `~/.nvidia-omniverse/logs/carb.crashreporter.log`: **this
exact Isaac Sim install has crashed repeatedly across the whole project's
history** — 2026-07-01 (×2), 2026-07-03 (×3), 2026-07-04, and this one on
2026-07-06. Conclusion: pre-existing GPU/renderer flakiness in this
environment, not a regression. No fix attempted (nothing actionable found —
no readable pre-crash diagnostic log, only a binary minidump). Just relaunch
and continue; flag it again if it starts happening at a specific,
reproducible trigger point rather than "eventually, during interactive use."

## 5. `main_holder`'s convex decomposition — researched, GUI fix given, not yet applied

Same underlying issue as 3b hit a second part: switching `main_holder`'s
collider from Convex Hull to Convex Decomposition (via GUI) made it sink
slightly into the table and lose its small mounting studs. Ran a full
research **Workflow** (3 parallel research agents + 3 adversarial verify
agents, every claim grounded against this Isaac Sim install's real schema
files, not memory) to get the exact tuning knobs right. Confirmed facts:

- Schema: `PhysxSchema.PhysxConvexDecompositionCollisionAPI` (single-apply),
  applied alongside `UsdPhysics.MeshCollisionAPI` with
  `approximation="convexDecomposition"`.
- Real schema defaults: `hullVertexLimit=64`, `maxConvexHulls=32`,
  `minThickness=0.001`, `voxelResolution=500000`, `errorPercentage=10`,
  `shrinkWrap=False`.
- GUI layout (confirmed via `displayGroup` schema metadata): "Hull Vertex
  Limit"/"Max Convex Hulls" render *outside* the "Advanced" fold;
  "Error Percentage"/"Min Thickness"/"Shrink Wrap"/"Voxel Resolution"
  render *under* "Advanced".
- Mechanism (VHACD-family: voxelize → cluster → convex-hull-per-cluster →
  optional shrink-wrap re-projection): **sinking** happens because
  `shrinkWrap` defaults to `False`, so nothing re-projects the
  voxel-quantized hull back onto the true surface. **Small-feature loss**
  happens because `voxelResolution` is a budget spread over the *whole
  part's bounding box*, not per-feature — mm-scale studs on a much larger
  flat part can fail to rasterize at all, or get merged away during the
  volume-error-driven clustering step.
- `main_holder`'s actual collider prim (confirmed via headless inspection,
  not assumed by analogy): **`/World/Factory/main_holder/tn__mainholder_kA`**
  — currently `approximation=convexHull` on-disk in `mefron.usd` (the
  user's live GUI edit to `convexDecomposition` is unsaved).

Recommended values (given to the user as a GUI walkthrough, **not**
implemented in code — user explicitly pushed back on hardcoding
per-part collision tuning as "not scalable," and this is an
asset-intrinsic property of `mefron.usd` anyway, unlike the robot-tied
friction/stiffness fixes above):
- **Shrink Wrap → ON** (fixes sinking)
- **Voxel Resolution → ~3,000,000–5,000,000** (fixes stud loss; ceiling is
  5,000,000)
- **Max Convex Hulls → ~128** (secondary, gives budget for small features)
- **Error Percentage → ~1–2** (secondary, less tolerant of ignoring stud
  volume)
- Leave Hull Vertex Limit and Min Thickness at defaults.
- **Then save `mefron.usd`** (`Ctrl+S`) — this is the one fix in this
  session meant to be persisted into the asset file directly, not
  reproduced by code.

**Not yet applied/tested by the user** as of the end of this session.

## Open items / suggested next steps, in rough priority order

1. Relaunch `build_scene_mefron.py`, Play, and test the actual grasp —
   confirm 3a+3c (friction + stiffness) together produce a firm, non-slipping,
   less-dangling hold on `finger_print_scanner`.
2. Apply the `main_holder` convex-decomposition GUI tuning from §5 and save
   `mefron.usd`.
3. Delete the stray `/panda` prim from the earlier manual import (§2), if
   not already done.
4. Decide whether a permanent headless regression test is wanted for the
   keyboard gripper control (§1) — offered, not built.
5. Longer-term/lower-priority, from `CLAUDE.md`'s own existing "Needs
   verification" list (unrelated to this session): the CR5-vs-Franka
   `robot_override` swap in the *main* `table_layout.yaml` scene is still
   pending reversion, and `build_scene.py`'s own `run_teleop_loop()` still
   lacks both the gripper control and the Stop→Play `SingleArticulation`
   rebuild fix that `build_scene_mefron.py` already has.
