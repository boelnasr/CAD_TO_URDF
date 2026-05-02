"""Unit tests for src/cad2urdf/core/config/loader.py schema validation."""

from __future__ import annotations

from pathlib import Path

import pytest

from cad2urdf.core.config.loader import load_joints_config


def test_load_valid_yaml(tmp_path: Path) -> None:
    p = tmp_path / "ok.yaml"
    p.write_text("""
robot_name: r
base_link: a
joints:
  - name: j1
    type: fixed
    parent: a
    child: b
""")
    cfg = load_joints_config(p)
    assert cfg.robot_name == "r"
    assert cfg.base_link == "a"
    assert len(cfg.joints) == 1
    assert cfg.joints[0].name == "j1"


def test_load_missing_file_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="joints config not found"):
        load_joints_config(tmp_path / "nope.yaml")


def test_load_empty_yaml_raises(tmp_path: Path) -> None:
    p = tmp_path / "empty.yaml"
    p.write_text("")
    with pytest.raises(ValueError, match="must be a YAML mapping"):
        load_joints_config(p)


def test_load_missing_robot_name_raises(tmp_path: Path) -> None:
    p = tmp_path / "x.yaml"
    p.write_text("base_link: a\njoints: []\n")
    with pytest.raises(ValueError, match="missing required field 'robot_name'"):
        load_joints_config(p)


def test_load_wrong_type_for_robot_name_raises(tmp_path: Path) -> None:
    p = tmp_path / "x.yaml"
    p.write_text("robot_name: 42\nbase_link: a\njoints: []\n")
    with pytest.raises(ValueError, match="must be str"):
        load_joints_config(p)


def test_load_joint_missing_name_raises(tmp_path: Path) -> None:
    p = tmp_path / "x.yaml"
    p.write_text("""
robot_name: r
base_link: a
joints:
  - type: fixed
    parent: a
    child: b
""")
    with pytest.raises(ValueError, match="missing required field 'name'"):
        load_joints_config(p)


def test_load_joint_axis_wrong_length_raises(tmp_path: Path) -> None:
    p = tmp_path / "x.yaml"
    p.write_text("""
robot_name: r
base_link: a
joints:
  - name: j1
    type: fixed
    parent: a
    child: b
    axis: [1, 0]
""")
    with pytest.raises(ValueError, match="must be a 3-element list"):
        load_joints_config(p)


def test_load_materials_wrong_type_raises(tmp_path: Path) -> None:
    p = tmp_path / "x.yaml"
    p.write_text("""
robot_name: r
base_link: a
materials: notamap
""")
    with pytest.raises(ValueError, match="'materials' must be a mapping"):
        load_joints_config(p)


def test_load_joints_not_a_list_raises(tmp_path: Path) -> None:
    p = tmp_path / "x.yaml"
    p.write_text("robot_name: r\nbase_link: a\njoints: not_a_list\n")
    with pytest.raises(ValueError, match="'joints' must be a list"):
        load_joints_config(p)


def test_load_joint_origin_xyz_wrong_length_raises(tmp_path: Path) -> None:
    p = tmp_path / "x.yaml"
    p.write_text("""
robot_name: r
base_link: a
joints:
  - name: j1
    type: revolute
    parent: a
    child: b
    origin:
      xyz: [0, 0]
""")
    with pytest.raises(ValueError, match=r"origin\.xyz.*must be a 3-element list"):
        load_joints_config(p)


def test_load_valid_materials(tmp_path: Path) -> None:
    p = tmp_path / "mats.yaml"
    p.write_text("""
robot_name: r
base_link: a
joints: []
materials:
  link_a: steel
  link_b: aluminum_6061
""")
    cfg = load_joints_config(p)
    assert cfg.materials == {"link_a": "steel", "link_b": "aluminum_6061"}


def test_load_empty_joints_list_is_valid(tmp_path: Path) -> None:
    p = tmp_path / "empty_joints.yaml"
    p.write_text("robot_name: my_robot\nbase_link: base\n")
    cfg = load_joints_config(p)
    assert cfg.joints == []
    assert cfg.robot_name == "my_robot"


def test_load_axis_with_nan_raises(tmp_path: Path) -> None:
    p = tmp_path / "x.yaml"
    p.write_text("""
robot_name: r
base_link: a
joints:
  - name: j1
    type: fixed
    parent: a
    child: b
    axis: [.nan, 0, 0]
""")
    with pytest.raises(ValueError, match="must be finite"):
        load_joints_config(p)


def test_load_axis_with_inf_raises(tmp_path: Path) -> None:
    p = tmp_path / "x.yaml"
    p.write_text("""
robot_name: r
base_link: a
joints:
  - name: j1
    type: fixed
    parent: a
    child: b
    axis: [.inf, 0, 0]
""")
    with pytest.raises(ValueError, match="must be finite"):
        load_joints_config(p)


def test_load_origin_xyz_with_nan_raises(tmp_path: Path) -> None:
    p = tmp_path / "x.yaml"
    p.write_text("""
robot_name: r
base_link: a
joints:
  - name: j1
    type: fixed
    parent: a
    child: b
    origin:
      xyz: [.nan, 0, 0]
""")
    with pytest.raises(ValueError, match="must be finite"):
        load_joints_config(p)


def test_load_limit_lower_with_inf_raises(tmp_path: Path) -> None:
    p = tmp_path / "x.yaml"
    p.write_text("""
robot_name: r
base_link: a
joints:
  - name: j1
    type: revolute
    parent: a
    child: b
    limits:
      lower: .inf
""")
    with pytest.raises(ValueError, match="must be finite"):
        load_joints_config(p)
