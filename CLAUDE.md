# CLAUDE.md

Project-specific context for **isaac-dobot**.

## What this repo is

An NVIDIA Isaac Sim project that builds a simulated factory cell: a generic
factory backdrop (SimReady warehouse assets), an L-shaped assembly table, and
a Dobot CR5 6-DOF cobot mounted at the table's inner corner.

**There is no physical CR5 hardware.** Everything here targets Isaac Sim
only. Since real drag-teach hardware isn't available, waypoint teaching is
done in-sim instead: the CR5 is imported via URDF, cuRobo provides
collision-aware IK / motion generation, and joint-space waypoints are
recorded and played back through `motion_gen.plan_single_js()`. Treat all
sim behavior (contact dynamics, motion timing, gripper interaction) as
illustrative, not validated against real hardware.

## Repo layout

- `robots/cr5/` — vendored CR5 URDF + meshes (MIT license, from
  `Dobot-Arm/TCP-IP-ROS-6AXis`; provenance in `robots/cr5/SOURCE.md`). Mesh
  URIs were rewritten from `package://dobot_description/...` to relative
  `../meshes/...` paths so the URDF resolves standalone.
- `docker/`, `.devcontainer/` — Isaac Lab-style container tooling, adapted
  from `tolasing/groot`. Two profiles:
  - `base` — Isaac Sim + Isaac Lab only.
  - `curobo` — adds the CUDA 12.8 toolchain + a pinned cuRobo build.
- `configs/scene/` — L-table geometry and CR5 placement.
- `configs/curobo/` — cuRobo robot config (`cr5.yml`) and collision spheres.
- `configs/rmpflow/` — optional Lula/RMPflow config for live reactive
  end-effector dragging (secondary to cuRobo waypoint planning).
- `scripts/` — scene building, URDF import, cuRobo setup, waypoint
  teach/playback.
- `data/waypoints/` — recorded waypoint JSON (joint-space, not Cartesian).

## Conventions

- USD hierarchy: `/World/CR5` is a **sibling** of `/World/Factory`, not a
  child — this keeps the robot's transform independent of any scale applied
  to factory dressing.
- CR5 URDF quirk: every joint has `effort="0" velocity="0"` (an artifact of
  the SolidWorks exporter). Override drive strength at import time via
  `URDFImporterConfig(default_drive_strength=1e5)`, otherwise the
  articulation won't hold a pose.
- Waypoints are joint-space (`Waypoint.joint_positions`, radians, 6 values
  for joint1..joint6), not Cartesian poses.
- Pinned versions: Isaac Sim `5.1.0`, cuRobo commit
  `ebb71702f3f70e767f40fd8e050674af0288abe8`.

## Provenance / licensing

- CR5 URDF + meshes: MIT, vendored verbatim except for the mesh URI rewrite
  noted above. See `robots/cr5/LICENSE-cr5-upstream` and
  `robots/cr5/SOURCE.md`.
- `docker/utils/`, `docker/container.py`, and the devcontainer scaffolding
  are adapted from `tolasing/groot`, which itself follows Isaac Lab's
  BSD-3-Clause container tooling pattern.
