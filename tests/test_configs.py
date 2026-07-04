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


def test_curobo_config_has_required_keys():
    cfg = yaml.safe_load(CUROBO_CONFIG.read_text())
    kinematics = cfg["robot_cfg"]["kinematics"]
    assert kinematics["urdf_path"] == "robots/cr5/urdf/cr5_robot.urdf"
    assert kinematics["base_link"] == "base_link"
    assert kinematics["ee_link"] == "Link6"
    assert kinematics["cspace"]["joint_names"] == [f"joint{i}" for i in range(1, 7)]


def test_collision_spheres_cover_all_collision_links():
    cfg = yaml.safe_load(CUROBO_CONFIG.read_text())
    collision_link_names = cfg["robot_cfg"]["kinematics"]["collision_link_names"]
    sphere_links = yaml.safe_load(COLLISION_SPHERES.read_text())["collision_spheres"]

    for link_name in collision_link_names:
        assert link_name in sphere_links
        assert len(sphere_links[link_name]) > 0
