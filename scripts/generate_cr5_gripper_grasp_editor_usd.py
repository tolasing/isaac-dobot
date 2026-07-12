"""Generates a standalone, self-contained USD file for the CR5's PGC-140
gripper (no arm), for use with the official Grasp Editor Tutorial content
at assets/grasp_editor_tutorial/ -- the CR5-gripper equivalent of that
tutorial's own Isaac/Robots/Franka/franka.usd, but scoped to just the
gripper (matching what the tutorial's own grasp_editor_tutorial.usd stage
actually uses: /World/panda_hand -- the Franka hand+fingers only, not the
full arm from franka.usd, which sits in that folder unused by the tutorial
stage itself).

Confirmed live (direct USD introspection) that grasp_editor_tutorial.usd
never references franka.usd at all -- it builds /World/panda_hand directly,
with PhysicsPrismaticJoint schemas authored right there in that stage and
per-link geometry pulled in from Isaac/Robots/Franka/Props/*.usd. This
script produces the CR5-gripper analogue of that /World/panda_hand subtree
as its own standalone file (geometry + physics both baked in, rather than
hand-split into separate Props files -- an organizational nicety the
Franka assets happen to use, not a requirement) so it can be referenced
into a grasp_editor_tutorial.usd-style scene in place of /World/panda_hand.

Reuses import_cr5()/tune_gripper_drive()/filter_self_collision_from_curobo_config()/
disable_gripper_finger_gravity() directly from import_cr5.py -- the exact
same, now-confirmed-working tuning scripts/pgc140_gripper_probe.py already
validated in isolation (see CLAUDE.md's gripper section, bug #10) -- rather
than re-deriving new physics for a supposedly-simpler "just for grasp
editing" gripper.

Exports only the imported gripper's own prim subtree (via Sdf.CopySpec into
a fresh Sdf.Layer), not the whole bare stage -- a bare SimulationApp stage
also contains Kit's own default /Render, /OmniverseKit_Persp, etc. cruft
prims that have no business in a clean, minimal deliverable asset (confirmed
via earlier diagnostics on this same import path).

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/generate_cr5_gripper_grasp_editor_usd.py
"""

from __future__ import annotations

from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd  # noqa: E402
from pxr import Sdf, UsdGeom  # noqa: E402

from import_cr5 import (  # noqa: E402
    disable_gripper_finger_gravity,
    filter_self_collision_from_curobo_config,
    import_cr5,
    tune_gripper_drive,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
PGC140_URDF_PATH = REPO_ROOT / "robots" / "pgc140" / "urdf" / "pgc140_robot.urdf"
IMPORT_PRIM_PATH = "/World/cr5_gripper"
DEST_USD_PATH = (
    REPO_ROOT
    / "assets"
    / "grasp_editor_tutorial"
    / "Grasp_Editor_Tutorial_Stage"
    / "Isaac"
    / "Robots"
    / "CR5"
    / "cr5_gripper.usd"
)

# Same values as table_layout.yaml's cr5_mount.gripper block and
# scripts/pgc140_gripper_probe.py -- this asset should behave identically to
# the already-confirmed-working mounted/standalone gripper, not a
# differently-tuned copy.
GRIPPER_STIFFNESS = 625.0
GRIPPER_DAMPING = 50.0
GRIPPER_MAX_FORCE = 140.0


def main() -> None:
    stage = omni.usd.get_context().get_stage()
    # See CLAUDE.md's "MovePrim silently no-ops" gotcha -- a bare headless
    # stage has no /World prim, which would otherwise break import_cr5()'s
    # own internal MovePrim step silently.
    if not stage.GetPrimAtPath("/World").IsValid():
        UsdGeom.Xform.Define(stage, "/World")

    import_cr5(urdf_path=PGC140_URDF_PATH, prim_path=IMPORT_PRIM_PATH)
    # URDFParseAndImportFile's asset population is asynchronous -- see
    # CLAUDE.md's own gotcha and scripts/pgc140_gripper_probe.py's
    # spawn_gripper_probe(), which this pump is copied from.
    for _ in range(120):
        simulation_app.update()

    tune_gripper_drive(
        prim_path=IMPORT_PRIM_PATH,
        stiffness=GRIPPER_STIFFNESS,
        damping=GRIPPER_DAMPING,
        max_force=GRIPPER_MAX_FORCE,
    )
    filter_self_collision_from_curobo_config(prim_path=IMPORT_PRIM_PATH)
    disable_gripper_finger_gravity(prim_path=IMPORT_PRIM_PATH)

    # Each link's own visuals/collisions Xform is not real geometry itself --
    # it's an internal reference arc (SdfReference with no asset path, just
    # an SdfPath) to a sibling root-level /visuals/<link>, /colliders/<link>
    # scope, a dedup mechanism the URDF importer uses for shared geometry
    # across links (confirmed live via direct reference-metadata
    # introspection). CONFIRMED LIVE Stage.Flatten() does NOT resolve this:
    # it flattens sublayers, not reference/payload composition arcs -- the
    # flattened layer's own /visuals scope still exists as a separate root
    # prim, and the reference arc pointing to it survives unresolved. So
    # copying only the /World/cr5_gripper subtree leaves those references
    # dangling in the destination file (nothing at "/visuals/..." there) --
    # confirmed live: an earlier version of this script produced a file
    # whose visuals/collisions Xforms had zero children, no Mesh anywhere.
    # Fixed by also copying the root-level /visuals, /colliders, /meshes
    # scopes themselves (as siblings of the robot's own root prim, at the
    # same absolute paths the reference arcs already point to) into the
    # destination file, so those same references resolve correctly there.
    DEST_USD_PATH.parent.mkdir(parents=True, exist_ok=True)
    dest_layer = Sdf.Layer.CreateNew(str(DEST_USD_PATH))
    root_name = Sdf.Path(IMPORT_PRIM_PATH).name
    dest_root_path = Sdf.Path(f"/{root_name}")
    src_layer = stage.GetRootLayer()
    Sdf.CopySpec(src_layer, Sdf.Path(IMPORT_PRIM_PATH), dest_layer, dest_root_path)
    for shared_scope in ("/visuals", "/colliders", "/meshes"):
        if src_layer.GetPrimAtPath(shared_scope) is not None:
            Sdf.CopySpec(src_layer, Sdf.Path(shared_scope), dest_layer, Sdf.Path(shared_scope))
    dest_layer.defaultPrim = root_name
    dest_layer.Save()
    print(f"[generate_cr5_gripper_grasp_editor_usd] wrote {DEST_USD_PATH}", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
