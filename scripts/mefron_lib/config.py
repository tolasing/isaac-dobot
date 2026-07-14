"""Constants shared across the mefron family of scripts: paths, mount pose, gripper/friction/drive
tuning, and the derived grasp/assembly relative poses. Pure data -- no omni/curobo imports, safe to
import at any point.
"""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MEFRON_USD = REPO_ROOT / "assets" / "mefron" / "factory floor" / "mefron.usd"
# Disk-persisted "Robot Description" dir the URDF importer writes on every import into this
# file-backed stage; see kit_bootstrap.clear_stale_robot_configuration().
MEFRON_CONFIGURATION_DIR = MEFRON_USD.parent / "configuration"

ROBOT_PRIM_PATH = "/World/CR5"
TARGET_PRIM_PATH = "/World/target"
# SEKTION cabinet table the arm mounts on (replaced the original Pedestal_plates/Cube_05 plate).
# No /Factory prefix: mefron.py opens mefron.usd directly, one level shallower than build_scene_mefron.py's reference.
MOUNT_PLATE_PRIM_PATH = "/World/sektion_cabinet_instanceable"
MOUNT_POSITION = [2.74097, -4.782, 0.7924]
# 180-deg yaw about world Z, not identity. The identity value inherited from the Franka-era config was
# only ever validated for the FRANKA's own base_link convention (manually placed in the GUI, see
# docs/mefron-history.md) -- confirmed live (screen recording) that reusing it for the CR5 made the arm
# reach smoothly to one side of the workspace and jerk/reverse on the other, consistent with the CR5's
# own joint2 shoulder-frame convention (see robots/cr5/urdf/cr5_robot.urdf) not matching Franka's.
MOUNT_ORIENTATION_WXYZ = [0.0, 0.0, 0.0, 1.0]

# Repo-local (not cuRobo-bundled, unlike franka.yml) -- teleop.setup_motion_gen() must patch its
# urdf_path/asset_root_path/collision_spheres to absolute paths after loading, same as build_scene.py does.
CR5_CUROBO_CONFIG_PATH = REPO_ROOT / "configs" / "curobo" / "cr5.yml"
# Re-authored post-import via import_cr5()'s joint_drive_stiffness/damping params -- its
# default_drive_strength/default_position_drive_damping kwargs don't reliably land for the CR5 URDF
# (confirmed bug, see import_cr5.py's own docstring). Values from configs/scene/table_layout.yaml's
# cr5_mount.joint_drive, already proven live for this exact arm.
CR5_JOINT_DRIVE_STIFFNESS = 625.0
CR5_JOINT_DRIVE_DAMPING = 50.0

# Nearby scene objects within the Franka's reach envelope, not the whole /World/Factory backdrop
# (which would add scan time for no benefit).
OBSTACLE_PRIM_PATHS = [
    "/World/packing_table",
    "/World/packing_table_01",
    "/World/finger_print_scanner",
    "/World/main_holder",
    "/World/screen",
    "/World/backpanel_support",
    MOUNT_PLATE_PRIM_PATH,
]

# Loop-timing constants for teleop.run_teleop_loop(), ported from build_scene.py.
_TELEOP_INIT_FRAMES = 10
_TELEOP_SETTLE_FRAMES = 20
_TELEOP_OBSTACLE_RESCAN_INTERVAL = 1000
_POSE_DELTA_THRESHOLD = 1.0e-3
_STATIC_JOINT_VELOCITY_THRESHOLD = 0.5

# Frames to wait after is_playing() first turns True before constructing SingleArticulation --
# PhysX needs a few real steps before its simulation view is actually ready.
_ROBOT_INIT_SETTLE_FRAMES = 5

# Uniformly re-times the already-planned trajectory to play out slower; does not change the
# optimizer's relative speed profile or planning success. See _TELEOP_VELOCITY_SCALE for capping actual limits.
_TELEOP_TIME_DILATION_FACTOR = 0.3

# Caps velocity/acceleration limits used during trajectory optimization. cuRobo treats scale <= 0.25 as a
# special case: it swaps in finetune_trajopt_slow.yml and raises maximum_trajectory_dt to compensate; 0.2 stays under that threshold.
_TELEOP_VELOCITY_SCALE = 0.5
_TELEOP_ACCELERATION_SCALE = 0.5

# Grasp-physics constants for the PGC-140. Duplicated here rather than imported from
# scripts/import_cr5.py: that module does unconditional omni.kit.commands/omni.usd/pxr imports at
# module scope, which would break this file's own "no omni/curobo imports" contract.
GRIPPER_JOINT_NAMES = ["pgc140_finger1_joint", "pgc140_finger2_joint"]
# PGC-140's full stroke -- NOTE inverted convention vs. the old Franka values (there, larger position
# was more open). Not yet narrowed to finger_print_scanner's actual grip width the way the Franka's
# were; that's a follow-up tuning pass, not a blocker.
GRIPPER_OPEN_POSITION = 0.0
GRIPPER_CLOSED_POSITION = 0.025
# Rate (m/s) the commanded gripper position is ramped toward open/closed, instead of stepping instantly --
# avoids a snap shut under the high drive stiffness.
GRIPPER_CLOSE_SPEED = 0.02
GRIPPER_FRICTION_MATERIAL_PATH = "/World/GripperFrictionMaterial"
GRIPPER_STATIC_FRICTION = 1.5
GRIPPER_DYNAMIC_FRICTION = 1.5
GRIPPER_FINGER_LINK_NAMES = ["pgc140_finger1_link", "pgc140_finger2_link"]
# Reused placeholder from the arm's own joint_drive values (configs/scene/table_layout.yaml's
# cr5_mount.gripper.joint_drive comment) -- not independently re-derived yet.
GRIPPER_DRIVE_STIFFNESS = 625.0
GRIPPER_DRIVE_DAMPING = 50.0
GRIPPER_MAX_FORCE = 140.0
HIGH_FRICTION_PRIM_PATHS = ["/World/finger_print_scanner"]

# Grasp Editor-exported grasp-approach pose, wired to the J key -- the sole grasp-approach method now
# that the hand-derived-constant G key has been removed. See grasp.compute_grasp_approach_pose_from_file().
GRASP_EDITOR_YAML_PATH = REPO_ROOT / "assets" / "Grasp_Editor" / "pgc_finger_print_scanner.yaml"
GRASP_EDITOR_GRASP_NAME = "grasp_0"

# T_H_S: finger_print_scanner's pose expressed in main_holder's own local frame at the correctly
# assembled position, derived via grasp.compute_relative_pose() after temporarily reparenting in mefron.usd.
ASSEMBLY_RELATIONSHIPS = {
    "finger_print_scanner_on_main_holder": {
        "part_prim_path": "/World/finger_print_scanner",
        "mount_prim_path": "/World/main_holder",
        "local_position": [-0.05765, 0.02069, 0.01565],
        "local_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
}
