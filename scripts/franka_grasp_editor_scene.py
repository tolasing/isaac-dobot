"""Builds a minimal, standalone scene for authoring the T_S_G grasp via the
Grasp Editor: just the Franka + finger_print_scanner's own standalone asset
file, on a fresh ANONYMOUS SimulationApp stage -- deliberately not
mefron.usd itself.

Why this exists: confirmed live that importing the Franka into mefron.usd's
own file-backed stage (mefron.py's approach, needed for the earlier T_H_S
nesting-derivation, which genuinely requires that file to be the edit
target) triggers Isaac Sim's URDF importer's "layered Robot Description"
mechanism (writes configuration/mefron_*.usd sublayer files with
cross-references between them) -- and that mechanism produces broken
internal references for this Franka in this file every time (confirmed via
"Could not open asset"/"Unresolved reference prim path" warnings even on a
freshly-cleared configuration/ folder, not just after a crash). That
brokenness doesn't affect PhysX's own separately-cached DOF/joint data
(SingleArticulation.dof_names resolves fine, confirmed live) -- but it does
mean a pure USD composition query finds nothing real under /World/Franka,
which is exactly what breaks the Grasp Editor: its "Select Frames of
Reference" panel populates its dropdowns via Usd.PrimRange() and comes back
empty, and its later "Joint Settings" panel crashes outright (AttributeError:
'NoneType' object has no attribute 'is_active').

An anonymous, in-memory stage (this script's own, matching build_scene.py's
already-established convention) never triggers that layered-import
mechanism at all -- so importing the Franka fresh here, and bringing in
finger_print_scanner via its own standalone asset file
(assets/mefron/scanner assembly/finger print scanner.usd) rather than
through mefron.usd's factory backdrop, sidesteps the whole bug class
instead of working around it. T_S_G is a purely relative transform between
the gripper and the object, so neither one needs to sit at any particular
world pose, or share a stage with the rest of the factory scene, for the
Grasp Editor to compute it correctly -- and skipping the whole factory
backdrop (hundreds of shaders/materials that have made every mefron.usd
session slow to load this whole project) makes this much faster to iterate
on too.

Reuses mefron.py's own mount_franka()/apply_gripper_friction()/
stiffen_gripper_drive() as a library rather than duplicating them (same
established pattern as test_mefron_teleop_headless.py) -- importing mefron
also runs its own top-level packaging-preload workaround unconditionally,
so that doesn't need to be duplicated here either.

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/franka_grasp_editor_scene.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from isaacsim import SimulationApp

_headless = "--headless" in sys.argv
if __name__ == "__main__":
    # Full experience for interactive runs, matching mefron.py's own choice --
    # the Grasp Editor panel is a full-experience-only UI extension.
    experience = "" if _headless else f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit'
    simulation_app = SimulationApp({"headless": _headless}, experience=experience)

import carb.settings  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402

import mefron  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
SCANNER_USD = REPO_ROOT / "assets" / "mefron" / "scanner assembly" / "finger print scanner.usd"
SCANNER_PRIM_PATH = "/World/finger_print_scanner"


def main() -> None:
    mefron.simulation_app = simulation_app

    # Same fix mefron.main() applies -- see mefron.py's own comment on this
    # setting for why it matters.
    carb.settings.get_settings().set_bool("/app/player/playSimulations", True)

    # Deliberately no open_stage() call anywhere in this script --
    # SimulationApp's own default stage is already a fresh, anonymous,
    # in-memory one. That is the entire fix: importing the Franka into THAT
    # stage, instead of mefron.usd's own file-backed one, means the URDF
    # importer never triggers its layered-import mechanism at all -- see
    # this module's own docstring for why that mechanism is what broke the
    # Grasp Editor in the first place.
    mefron.mount_franka()
    for _ in range(60):
        simulation_app.update()

    if not SCANNER_USD.is_file():
        raise FileNotFoundError(f"{SCANNER_USD} not found.")
    add_reference_to_stage(usd_path=str(SCANNER_USD), prim_path=SCANNER_PRIM_PATH)
    for _ in range(60):
        simulation_app.update()

    # Needs finger_print_scanner to already be resolved on the stage (its
    # own HIGH_FRICTION_PRIM_PATHS entry) -- called after the reference pump
    # above, not before.
    mefron.apply_gripper_friction()
    mefron.stiffen_gripper_drive()

    stage = omni.usd.get_context().get_stage()
    for status_path in [mefron.ROBOT_PRIM_PATH, SCANNER_PRIM_PATH]:
        prim = stage.GetPrimAtPath(status_path)
        print(f"[franka_grasp_editor_scene] {status_path}: {'OK' if prim.IsValid() else 'MISSING'}", flush=True)

    print("[franka_grasp_editor_scene] Scene ready -- open the Grasp Editor now.", flush=True)

    if _headless:
        simulation_app.close()
        return

    while simulation_app.is_running():
        simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
