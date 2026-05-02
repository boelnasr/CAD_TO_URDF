from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from cad2urdf.core.kinematic.model import (
    InertialOverride,
    Joint,
    Link,
    Robot,
)
from cad2urdf.core.urdf.emit import emit_urdf


def _two_link_robot() -> Robot:
    base = Link(
        name="base",
        visual_mesh_path=Path("meshes/visual/base.stl"),
        collision_mesh_path=Path("meshes/collision/base.stl"),
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )
    tip = Link(
        name="tip",
        visual_mesh_path=Path("meshes/visual/tip.stl"),
        collision_mesh_path=Path("meshes/collision/tip.stl"),
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )
    j = Joint(
        name="j1",
        type="revolute",
        parent="base",
        child="tip",
        axis=np.array([0.0, 0.0, 1.0]),
        origin=np.eye(4),
        limit_lower=-3.14,
        limit_upper=3.14,
        effort=10.0,
        velocity=2.0,
    )
    return Robot(
        name="my_arm", base_link="base", links={"base": base, "tip": tip}, joints={"j1": j}
    )


def test_emit_urdf_writes_root_element_with_robot_name(tmp_path: Path) -> None:
    out = tmp_path / "robot.urdf"
    emit_urdf(_two_link_robot(), out, package_name="my_pkg")
    tree = ET.parse(out)
    root = tree.getroot()
    assert root.tag == "robot"
    assert root.attrib["name"] == "my_arm"


def test_emit_urdf_emits_two_links_one_joint(tmp_path: Path) -> None:
    out = tmp_path / "robot.urdf"
    emit_urdf(_two_link_robot(), out, package_name="my_pkg")
    tree = ET.parse(out)
    root = tree.getroot()
    assert len(root.findall("link")) == 2
    assert len(root.findall("joint")) == 1
    j = root.find("joint")
    assert j is not None
    assert j.attrib["type"] == "revolute"
    limit = j.find("limit")
    assert limit is not None
    assert limit.attrib["lower"] == "-3.14"


def test_emit_urdf_uses_package_uri_for_meshes(tmp_path: Path) -> None:
    out = tmp_path / "robot.urdf"
    emit_urdf(_two_link_robot(), out, package_name="my_pkg")
    text = out.read_text()
    assert "package://my_pkg/meshes/visual/base.stl" in text
    assert "package://my_pkg/meshes/collision/tip.stl" in text


def test_emit_urdf_is_deterministic(tmp_path: Path) -> None:
    out_a = tmp_path / "a.urdf"
    out_b = tmp_path / "b.urdf"
    emit_urdf(_two_link_robot(), out_a, package_name="p")
    emit_urdf(_two_link_robot(), out_b, package_name="p")
    assert out_a.read_bytes() == out_b.read_bytes()


def test_emit_urdf_skips_axis_for_fixed_joints(tmp_path: Path) -> None:
    base = Link(
        name="base",
        visual_mesh_path=Path("meshes/visual/base.stl"),
        collision_mesh_path=Path("meshes/collision/base.stl"),
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )
    tip = Link(
        name="tip",
        visual_mesh_path=Path("meshes/visual/tip.stl"),
        collision_mesh_path=Path("meshes/collision/tip.stl"),
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )
    j = Joint(
        name="weld",
        type="fixed",
        parent="base",
        child="tip",
        axis=np.array([1.0, 0.0, 0.0]),
        origin=np.eye(4),
    )
    robot = Robot(
        name="welded", base_link="base", links={"base": base, "tip": tip}, joints={"weld": j}
    )
    out = tmp_path / "r.urdf"
    emit_urdf(robot, out, package_name="p")
    tree = ET.parse(out)
    joint_el = tree.getroot().find("joint")
    assert joint_el is not None
    # fixed joints don't need axis or limits
    assert joint_el.find("axis") is None
    assert joint_el.find("limit") is None
