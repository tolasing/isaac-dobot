from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
SCENE_CONFIG = REPO_ROOT / "configs" / "scene" / "table_layout.yaml"
CUROBO_CONFIG = REPO_ROOT / "configs" / "curobo" / "cr5.yml"
COLLISION_SPHERES = REPO_ROOT / "configs" / "curobo" / "cr5_collision_spheres.yml"


def test_scene_config_has_required_keys():
    cfg = yaml.safe_load(SCENE_CONFIG.read_text())
    assert "factory" in cfg
    assert "cr5_mount" in cfg
    assert set(cfg["ergo_tables"]) >= {"source_prim_path", "instances"}
    for instance in cfg["ergo_tables"]["instances"]:
        assert set(instance) >= {"prim_path", "position_xy", "orientation_wxyz"}
    assert set(cfg["teleop_target"]) >= {"prim_path", "position", "orientation_wxyz"}
    for instance in cfg["assembly_parts"]["instances"]:
        assert set(instance) >= {"name", "usd_path", "prim_path", "scale", "position", "orientation_wxyz"}
    # GRIPPER ADDITION
    gripper_cfg = cfg["cr5_mount"]["gripper"]
    assert set(gripper_cfg) >= {"open_position", "closed_position", "close_speed", "joint_drive", "max_force"}
    assert set(gripper_cfg["joint_drive"]) >= {"stiffness", "damping"}
    # Confirmed live via direct FK: pgc140_finger1_joint's own axis points
    # inward as its value increases, so closed (fingers converged) is the
    # LARGER value and open (fingers spread) is the smaller one -- opposite
    # of the URDF's raw lower/upper naming.
    assert gripper_cfg["closed_position"] > gripper_cfg["open_position"]


def test_curobo_config_has_required_keys():
    cfg = yaml.safe_load(CUROBO_CONFIG.read_text())
    kinematics = cfg["robot_cfg"]["kinematics"]
    assert kinematics["urdf_path"] == "robots/cr5_pgc140/urdf/combined.urdf"
    assert kinematics["base_link"] == "base_link"
    # GRIPPER ADDITION: moved off Link6 to the gripper's own real-geometry
    # link -- see cr5.yml's own ee_link comment for why (build_teleop_target()
    # needs ee_link to have real /visuals content).
    assert kinematics["ee_link"] == "pgc140_base_link"
    # Both finger joints are tracked -- the vendored URDF's <mimic> tag on
    # pgc140_finger2_joint was removed (see robots/pgc140/SOURCE.md: this
    # Isaac Sim version imports a prismatic <mimic> as a broken rotational
    # PhysX mimic API with no attached drive), so both are ordinary,
    # independent joints, matching cuRobo's own bundled franka.yml
    # convention of tracking both its (non-mimic-linked) finger joints.
    assert kinematics["cspace"]["joint_names"] == [f"joint{i}" for i in range(1, 7)] + [
        "pgc140_finger1_joint",
        "pgc140_finger2_joint",
    ]
    # 0.0, not the joints' upper limit -- confirmed live that q=0.0 is
    # actually the OPEN position for these joints (see cr5.yml's own
    # retract_config comment for the full derivation).
    assert kinematics["lock_joints"] == {"pgc140_finger1_joint": 0.0, "pgc140_finger2_joint": 0.0}


def test_curobo_cspace_arrays_are_consistent_length():
    # A silent-mismatch risk flagged directly while adding the gripper's
    # two finger joints: cuRobo has no schema-level check that joint_names,
    # retract_config, null_space_weight, and cspace_distance_weight all
    # stay the same length as each other -- catch a drift here rather than
    # discovering it as an obscure runtime error deep inside MotionGen.
    cfg = yaml.safe_load(CUROBO_CONFIG.read_text())
    cspace = cfg["robot_cfg"]["kinematics"]["cspace"]
    n = len(cspace["joint_names"])
    assert len(cspace["retract_config"]) == n
    assert len(cspace["null_space_weight"]) == n
    assert len(cspace["cspace_distance_weight"]) == n


def test_collision_spheres_cover_all_collision_links():
    cfg = yaml.safe_load(CUROBO_CONFIG.read_text())
    collision_link_names = cfg["robot_cfg"]["kinematics"]["collision_link_names"]
    sphere_links = yaml.safe_load(COLLISION_SPHERES.read_text())["collision_spheres"]

    for link_name in collision_link_names:
        assert link_name in sphere_links
        assert len(sphere_links[link_name]) > 0
