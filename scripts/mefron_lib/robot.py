"""Mounting the CR5+PGC-140 onto mefron.usd's SEKTION cabinet plate, and the gripper physics tuning
(friction material, drive stiffness) needed for a stable grasp.
"""

from __future__ import annotations

import numpy as np
import omni.usd
from isaacsim.core.prims import SingleXFormPrim

from import_cr5 import (
    disable_gripper_finger_gravity,
    filter_self_collision_from_curobo_config,
    import_cr5,
    tune_gripper_drive,
)

from . import config


def mount_cr5() -> None:
    """Mirrors build_scene.py's mount_cr5() non-override branch: import_cr5()'s own
    default_drive_strength/default_position_drive_damping kwargs don't reliably land for the CR5 URDF,
    so arm-joint drive tuning goes through joint_drive_stiffness/damping instead (re-authored directly
    post-import)."""
    import_cr5(
        prim_path=config.ROBOT_PRIM_PATH,
        joint_drive_stiffness=config.CR5_JOINT_DRIVE_STIFFNESS,
        joint_drive_damping=config.CR5_JOINT_DRIVE_DAMPING,
    )
    tune_gripper_drive(
        prim_path=config.ROBOT_PRIM_PATH,
        stiffness=config.GRIPPER_DRIVE_STIFFNESS,
        damping=config.GRIPPER_DRIVE_DAMPING,
        max_force=config.GRIPPER_MAX_FORCE,
    )
    filter_self_collision_from_curobo_config(prim_path=config.ROBOT_PRIM_PATH)
    disable_gripper_finger_gravity(prim_path=config.ROBOT_PRIM_PATH)
    xform = SingleXFormPrim(prim_path=config.ROBOT_PRIM_PATH)
    xform.set_world_pose(
        position=np.array(config.MOUNT_POSITION),
        orientation=np.array(config.MOUNT_ORIENTATION_WXYZ),
    )


def apply_gripper_friction() -> None:
    """Authors one high-friction physics material and binds it to the gripper's fingertip links and
    HIGH_FRICTION_PRIM_PATHS. Runtime-only (never persisted via stage.Save()); re-authored fresh every run."""
    from omni.physx.scripts import utils as physx_utils
    from omni.physx.scripts.physicsUtils import add_physics_material_to_prim

    stage = omni.usd.get_context().get_stage()
    physx_utils.addRigidBodyMaterial(
        stage,
        config.GRIPPER_FRICTION_MATERIAL_PATH,
        staticFriction=config.GRIPPER_STATIC_FRICTION,
        dynamicFriction=config.GRIPPER_DYNAMIC_FRICTION,
        restitution=0.0,
    )

    target_paths = [f"{config.ROBOT_PRIM_PATH}/{name}" for name in config.GRIPPER_FINGER_LINK_NAMES]
    target_paths += config.HIGH_FRICTION_PRIM_PATHS
    for prim_path in target_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            print(f"[mefron_lib] WARNING: {prim_path} not found -- skipping friction bind.", flush=True)
            continue
        add_physics_material_to_prim(stage, prim, config.GRIPPER_FRICTION_MATERIAL_PATH)
