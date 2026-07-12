"""Imports the vendored CR5 URDF into /World/CR5 as a sibling of /World/Factory.

Verified against a live Isaac Sim 5.1.0 install (real GPU), both standalone
and imported as a library by build_scene.py.

Only creates its own SimulationApp when run standalone (`__main__`); when
imported (e.g. by build_scene.py, which already has one running),
import_cr5() reuses the caller's Kit process instead of starting a second
one -- the isaacsim/omni imports below just need *some* Kit app to already
be up, not specifically the one this module would create.

Run standalone:
    ${ISAACSIM_ROOT_PATH}/python.sh scripts/import_cr5.py
"""

from __future__ import annotations

from pathlib import Path

from isaacsim import SimulationApp

if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": False})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

URDF_PATH = Path(__file__).resolve().parent.parent / "robots" / "cr5" / "urdf" / "cr5_robot.urdf"
CR5_PRIM_PATH = "/World/CR5"


def import_cr5(
    urdf_path: Path = URDF_PATH,
    prim_path: str = CR5_PRIM_PATH,
    default_drive_strength: float = 1e5,
    default_position_drive_damping: float = 1e4,
    joint_drive_stiffness: float | None = None,
    joint_drive_damping: float | None = None,
) -> str:
    """Imports a URDF (the CR5 by default) via URDFParseAndImportFile.

    `default_drive_strength`/`default_position_drive_damping` default to
    the CR5's own tuning -- a workaround for its URDF's degenerate
    effort="0" velocity="0" joints (see robots/cr5/SOURCE.md), not a
    generally-correct value for any robot. Callers importing a different,
    properly-specified URDF (e.g. build_scene.py's temporary Franka swap,
    which passes cuRobo's own tuned 1047.19751 / 52.35988) should override
    both.

    CONFIRMED LIVE these two ImportConfig fields do NOT reliably reach the
    actual authored joints on this pinned Isaac Sim version, regardless of
    what they're set to: introspecting the resulting USD DriveAPI directly
    after import showed every CR5 joint authored as
    `type=acceleration, stiffness=625, damping=0` even though the
    `ImportConfig` object itself held `default_drive_strength=1e5`/
    `default_position_drive_damping=1e4` correctly right before the import
    command ran (also tried `ImportConfig.override_joint_dynamics = True`:
    it changes damping to small, per-joint-varying values instead of 0, but
    stiffness stays pinned at 625 and neither field's requested value ever
    lands -- the mismatch is inside the importer's own closed-source
    authoring step, not this repo's config-building code). `damping=0` is a
    fully undamped spring, and drove the CR5's whole "swings back and forth
    at the start/stop of a move, not mid-traversal" bug (a time-optimal
    trajectory's peak jerk sits at its two ends, exactly where an undamped
    drive rings hardest) -- confirmed by headless per-joint planned-vs-
    measured velocity logging both before and after the fix below.

    `joint_drive_stiffness`/`joint_drive_damping`: when given (not None),
    explicitly (re-)authors every imported joint's angular
    `UsdPhysics.DriveAPI` stiffness/damping directly via USD attributes
    right after import, bypassing whatever `default_drive_strength`/
    `default_position_drive_damping` actually did above -- the only
    mechanism confirmed to reliably land. None (the default) skips this
    and leaves whatever the importer itself authored, since this fix is
    CR5-specific and hasn't been verified against the Franka-override
    branch's own (different) tuning above.
    """
    # isaacsim.asset.importer.urdf doesn't export a directly-constructible
    # config class -- the URDFCreateImportConfig command is the only way to
    # get a properly-initialized isaacsim.asset.importer.urdf._urdf.ImportConfig.
    import_config = omni.kit.commands.execute("URDFCreateImportConfig")[1]
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.import_inertia_tensor = True
    import_config.self_collision = False
    import_config.distance_scale = 1.0
    import_config.default_drive_strength = default_drive_strength
    import_config.default_position_drive_damping = default_position_drive_damping

    status, imported_prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
    )
    if not status:
        raise RuntimeError(f"URDF import failed for {urdf_path}")

    if imported_prim_path != prim_path:
        omni.kit.commands.execute("MovePrim", path_from=imported_prim_path, path_to=prim_path)

    if joint_drive_stiffness is not None or joint_drive_damping is not None:
        stage = omni.usd.get_context().get_stage()
        joints_root = stage.GetPrimAtPath(f"{prim_path}/joints")
        for joint_prim in joints_root.GetChildren():
            drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            if not drive:
                continue
            if joint_drive_stiffness is not None:
                drive.CreateStiffnessAttr().Set(joint_drive_stiffness)
            if joint_drive_damping is not None:
                drive.CreateDampingAttr().Set(joint_drive_damping)

    return prim_path


def main() -> None:
    import_cr5()
    while simulation_app.is_running():
        simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
