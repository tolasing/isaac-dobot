"""Imports the vendored CR5+PGC-140 combined URDF into /World/CR5 as a
sibling of /World/Factory.

Verified against a live Isaac Sim 5.1.0 install (real GPU, NVIDIA RTX
A6000), both the bare-arm CR5 import path and the PGC-140 gripper addition
below (URDF_PATH pointing at the generated combined URDF,
tune_gripper_drive(), filter_self_collision_from_curobo_config()) -- see
robots/pgc140/SOURCE.md and this module's own function docstrings for the
real, live-confirmed bugs found and fixed along the way (a broken
prismatic <mimic> import, and self_collision=False not actually filtering
real contact between links this repo already knows are close -- see
filter_self_collision_from_curobo_config()'s own docstring for the fuller,
two-part story).

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

import yaml
from isaacsim import SimulationApp

if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": False})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from pxr import PhysxSchema, UsdPhysics  # noqa: E402

URDF_PATH = Path(__file__).resolve().parent.parent / "robots" / "cr5_pgc140" / "urdf" / "combined.urdf"
CR5_PRIM_PATH = "/World/CR5"
CUROBO_CONFIG_PATH = Path(__file__).resolve().parent.parent / "configs" / "curobo" / "cr5.yml"

# PGC-140 gripper joints, see robots/pgc140/SOURCE.md for the rename
# history. Both are ordinary, independent prismatic joints -- the
# vendored URDF's own <mimic> tag on pgc140_finger2_joint was removed
# (see SOURCE.md) after confirming live that this Isaac Sim version
# imports a prismatic <mimic> as a broken, rotational PhysxMimicJointAPI
# with mangled limits and no attached drive. Both names get identical
# tuning here and identical commanded values in build_scene.py's gripper
# teleop control, the same pattern already proven for the Franka's own
# two (never mimic-linked) finger joints.
GRIPPER_JOINT_NAMES = ["pgc140_finger1_joint", "pgc140_finger2_joint"]


def import_cr5(
    urdf_path: Path = URDF_PATH,
    prim_path: str = CR5_PRIM_PATH,
    default_drive_strength: float = 1e5,
    default_position_drive_damping: float = 1e4,
    joint_drive_stiffness: float | None = None,
    joint_drive_damping: float | None = None,
    fix_base: bool = True,
) -> str:
    """Imports a URDF (the CR5 by default) via URDFParseAndImportFile.

    `fix_base`: True (the default, correct for every existing caller -- the
    real CR5, bolted to a pedestal) anchors the imported root to the world
    via a real `PhysicsFixedJoint`, with `ArticulationRootAPI` landing on
    that joint prim itself. CONFIRMED LIVE this is the wrong shape for a
    free-standing asset meant to be repositioned by other tooling (e.g.
    `scripts/generate_cr5_gripper_grasp_editor_usd.py`'s gripper-only
    export for NVIDIA's Grasp Editor extension): with `fix_base=True`, the
    Grasp Editor's own `initialize_objects()` crashed with `AttributeError:
    'NoneType' object has no attribute 'link_names'` right after a
    `[omni.physx.tensors.plugin] prim '.../root_joint' was deleted while
    being used by a tensor view class` warning -- traced to the tutorial's
    own working Franka `/World/panda_hand` asset having a fundamentally
    different shape (`ArticulationRootAPI` on a plain Xform *body*
    container, a non-constraining generic `PhysicsJoint` with nothing
    anchoring it to the world, i.e. a genuinely free rigid body) that this
    fixed-base import doesn't reproduce. `fix_base=False` matches that
    shape instead: no root joint at all, `ArticulationRootAPI` lands
    directly on the base link's own Xform, free/dynamic body.

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


def tune_gripper_drive(
    prim_path: str = CR5_PRIM_PATH,
    joint_names: list[str] | None = None,
    stiffness: float | None = None,
    damping: float | None = None,
    max_force: float | None = None,
) -> None:
    """(Re-)authors the PGC-140 gripper's own prismatic finger joint(s)'
    `UsdPhysics.DriveAPI` directly, separately from import_cr5()'s own
    reauthoring loop above.

    Deliberately a separate function/pass, not folded into import_cr5()'s
    loop, for two reasons:
      - import_cr5()'s loop only ever requests the "angular" DriveAPI
        (correct for the arm's own revolute joints) and silently skips
        anything else (`if not drive: continue`) -- the gripper's
        joints are prismatic and need the "linear" variant instead, the
        same distinction scripts/mefron_lib/robot.py's
        stiffen_gripper_drive() already draws for the Franka's own
        prismatic finger joints.
      - The gripper needs its OWN stiffness/damping values, not the arm's
        (configs/scene/table_layout.yaml's cr5_mount.joint_drive
        stiffness=625.0/damping=50.0 was derived specifically for the
        bare arm's own importer-assigned stiffness -- there is no reason
        to expect the same numbers apply to a completely different,
        much lighter joint).

    `max_force`: a genuinely new concern this repo's arm-only import never
    had -- neither this function nor import_cr5()'s own reauthoring loop
    touched maxForce before, because the arm only ever needed smooth
    *position* tracking. A gripper's entire purpose is bounded *contact
    force*, so this is set explicitly here rather than left at whatever
    (unconfirmed) default the importer assigns -- read back
    `drive.GetMaxForceAttr().Get()` after calling this to confirm it
    actually landed, the same "don't trust it just because nothing raised"
    discipline import_cr5()'s own docstring already established for
    stiffness/damping.

    All three of stiffness/damping/max_force default to None (no-op) --
    the caller (configs/scene/table_layout.yaml-driven, see
    build_scene.py's mount_cr5()) must supply real, explicitly-derived
    values, not implicit defaults baked in here.
    """
    if joint_names is None:
        joint_names = GRIPPER_JOINT_NAMES
    stage = omni.usd.get_context().get_stage()
    joints_root = stage.GetPrimAtPath(f"{prim_path}/joints")
    for joint_name in joint_names:
        joint_prim = joints_root.GetChild(joint_name)
        if not joint_prim.IsValid():
            # Defensive, not expected in practice: both PGC-140 finger
            # joints are ordinary prismatic joints (robots/pgc140/urdf/
            # pgc140_robot.urdf has no <mimic> tag -- see that directory's
            # SOURCE.md for why one was removed), so both should always
            # import as real, independently-drivable joint prims here.
            continue
        drive = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
        if not drive:
            continue
        if stiffness is not None:
            drive.CreateStiffnessAttr().Set(stiffness)
        if damping is not None:
            drive.CreateDampingAttr().Set(damping)
        if max_force is not None:
            drive.CreateMaxForceAttr().Set(max_force)


def filter_self_collision_from_curobo_config(
    prim_path: str = CR5_PRIM_PATH,
    self_collision_ignore: dict[str, list[str]] | None = None,
) -> None:
    """Explicitly authors a `UsdPhysics.FilteredPairsAPI` exclusion for
    every link pair listed in cuRobo's own `self_collision_ignore`
    (`configs/curobo/cr5.yml` by default), directly on the imported USD
    prims.

    Two-part story, both confirmed live:
      1. Introspected the imported stage directly and confirmed neither a
         `PhysxArticulationAPI.enabledSelfCollisions` attribute nor any
         `UsdPhysics.FilteredPairsAPI` relationship exists anywhere under
         the articulation root despite import_cr5()'s own
         `import_config.self_collision = False` (same "importer setting
         doesn't reliably land" pattern this project has hit before for
         drive strength/damping) -- so PhysX was never actually filtering
         *any* self-collision, contrary to what that setting implies.
      2. An earlier version of this function only filtered ONE pair
         (`pgc140_finger1_link` <-> `pgc140_finger2_link`), based on a
         hypothesis that finger-vs-finger contact explained
         `pgc140_finger1_joint` reproducibly stalling partway through a
         close motion. Applying that narrow fix in isolation produced NO
         measurable change to the symptom -- ruling out finger-vs-finger
         contact specifically. What it missed: `cr5.yml`'s own
         `self_collision_ignore` already documented real overlaps between
         `Link6` and *both* finger links (~3.6mm each, found via the
         all-pairwise sphere check when that config was first written) --
         a pair this function never filtered at the PhysX level, only at
         cuRobo's planning level. Generalizing to cover the *entire*
         `self_collision_ignore` dict (not one hand-picked pair) closes
         that gap by construction, and keeps simulation-time filtering in
         sync with planning-time filtering automatically if that dict
         grows again later.

    Confirm this actually resolves the stalling/asymmetric-motion symptom
    via a live overlap query (e.g. `omni.physx.get_physx_scene_query_interface().overlap_box()`)
    at both the open and closed extremes, not just by reading joint
    positions -- see CLAUDE.md's gripper section for the current
    diagnostic state.
    """
    if self_collision_ignore is None:
        robot_cfg = yaml.safe_load(CUROBO_CONFIG_PATH.read_text())["robot_cfg"]
        self_collision_ignore = robot_cfg["kinematics"]["self_collision_ignore"]

    stage = omni.usd.get_context().get_stage()
    for link_a, ignored_links in self_collision_ignore.items():
        prim_a = stage.GetPrimAtPath(f"{prim_path}/{link_a}")
        if not prim_a.IsValid():
            continue
        filtered_pairs_rel = UsdPhysics.FilteredPairsAPI.Apply(prim_a).CreateFilteredPairsRel()
        for link_b in ignored_links:
            prim_b = stage.GetPrimAtPath(f"{prim_path}/{link_b}")
            if prim_b.IsValid():
                filtered_pairs_rel.AddTarget(prim_b.GetPath())


def disable_gripper_finger_gravity(
    prim_path: str = CR5_PRIM_PATH,
    link_names: list[str] | None = None,
) -> None:
    """Sets `PhysxRigidBodyAPI.disableGravity = True` on the gripper's
    finger links.

    Motivated by a specific, reproducible signature found investigating
    why `pgc140_finger1_joint`/`pgc140_finger2_joint` each settle short of
    their commanded target instead of reaching it: closing gets
    `pgc140_finger1_joint` stuck at ~0.0089 of its 0.025 range while
    `pgc140_finger2_joint` reaches 0.025 cleanly; opening (the reverse
    target) gets `pgc140_finger2_joint` stuck at ~0.0161 while
    `pgc140_finger1_joint` reaches 0.0 cleanly -- i.e. *which* joint stalls
    flips with direction. A live `overlap_box` scene query at the stalled
    state did show the two finger links and `pgc140_base_link`
    geometrically overlapping, but that query reports raw geometric
    proximity regardless of collision filtering (it does not distinguish
    "close" from "generating contact force"), and this repo's own
    `filter_self_collision_from_curobo_config()` already suppresses real
    contact response between exactly these pairs -- so overlap alone isn't
    good evidence of the actual cause.

    Direction-dependent steady-state offset that's stable and repeatable
    (not noise) is the textbook signature of a proportional+derivative-only
    drive (no integral term -- which is what `type=acceleration` PhysX
    drives are, see `tune_gripper_drive()`) settling under a *constant
    disturbance force*: `stiffness * position_error = disturbance /
    effective_mass` at equilibrium. The two finger joints' motion axes are
    deliberately mirrored (opposite local-frame signs, so identical
    commanded values produce symmetric physical convergence -- see
    robots/pgc140/SOURCE.md), which means gravity's component along each
    finger's *own* axis differs between them -- exactly the kind of
    asymmetric, direction-flipping disturbance that would produce this
    exact symptom. Disabling gravity on these two (14g each, real-world
    negligible for a device whose whole purpose is grip force, not
    weight-bearing) removes that disturbance entirely rather than trying
    to out-tune it with even higher stiffness.

    CONFIRMED LIVE this resolves the stall, on two independent paths: the
    full CR5+gripper articulation (scripts/test_teleop_headless.py) and the
    gripper alone, no CR5 arm (scripts/pgc140_gripper_probe.py) both now
    reach their commanded open/closed targets on both finger joints within
    a 2mm tolerance -- see CLAUDE.md's gripper section (bug #10) for the
    exact numbers.
    """
    if link_names is None:
        link_names = ["pgc140_finger1_link", "pgc140_finger2_link"]
    stage = omni.usd.get_context().get_stage()
    for link_name in link_names:
        prim = stage.GetPrimAtPath(f"{prim_path}/{link_name}")
        if not prim.IsValid():
            continue
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr().Set(True)


def main() -> None:
    import_cr5()
    while simulation_app.is_running():
        simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
