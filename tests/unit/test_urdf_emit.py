from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest

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


# ---------------------------------------------------------------------------
# New tests for Codex code-review findings
# ---------------------------------------------------------------------------


def _simple_two_link_robot(joint: Joint) -> Robot:
    """Helper: build a minimal Robot around the given joint."""
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
    return Robot(
        name="test_arm",
        base_link="base",
        links={"base": base, "tip": tip},
        joints={joint.name: joint},
    )


def test_emit_urdf_rejects_revolute_without_limits(tmp_path: Path) -> None:
    """P1: revolute joint with any None limit fields must raise ValueError."""
    j = Joint(
        name="j_no_limits",
        type="revolute",
        parent="base",
        child="tip",
        axis=np.array([0.0, 0.0, 1.0]),
        origin=np.eye(4),
        # All limit fields default to None
    )
    out = tmp_path / "robot.urdf"
    with pytest.raises(ValueError, match="revolute/prismatic joints require"):
        emit_urdf(_simple_two_link_robot(j), out, package_name="p")


def test_emit_urdf_rejects_absolute_mesh_path(tmp_path: Path) -> None:
    """P2: absolute visual_mesh_path must raise ValueError before creating a broken URI."""
    base = Link(
        name="base",
        visual_mesh_path=Path("/tmp/abs.stl"),  # absolute — forbidden
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
    robot = Robot(
        name="bad_mesh_arm",
        base_link="base",
        links={"base": base, "tip": tip},
        joints={"j1": j},
    )
    out = tmp_path / "robot.urdf"
    with pytest.raises(ValueError, match="mesh path must be relative"):
        emit_urdf(robot, out, package_name="p")


def test_emit_urdf_skips_axis_for_floating_joints(tmp_path: Path) -> None:
    """P2: floating joints must NOT emit an <axis> element."""
    j = Joint(
        name="j_float",
        type="floating",
        parent="base",
        child="tip",
        axis=np.array([1.0, 0.0, 0.0]),  # field required by dataclass; must not appear in XML
        origin=np.eye(4),
    )
    out = tmp_path / "robot.urdf"
    emit_urdf(_simple_two_link_robot(j), out, package_name="p")
    tree = ET.parse(out)
    joint_el = tree.getroot().find("joint")
    assert joint_el is not None
    assert joint_el.attrib["type"] == "floating"
    assert joint_el.find("axis") is None


def test_emit_urdf_includes_color_for_known_material(tmp_path: Path) -> None:
    """P3: robot-level <material> must contain a <color rgba=...> for known materials."""
    out = tmp_path / "robot.urdf"
    emit_urdf(_two_link_robot(), out, package_name="p")
    tree = ET.parse(out)
    root = tree.getroot()
    # There should be exactly one top-level material declaration (both links share aluminum_6061)
    mat_els = root.findall("material")
    assert len(mat_els) == 1
    color_el = mat_els[0].find("color")
    assert color_el is not None, "<color> element missing inside <material>"
    rgba = color_el.attrib["rgba"]
    # aluminum_6061: [0.85, 0.85, 0.88, 1.0] → "0.85 0.85 0.88 1"
    parts = rgba.split()
    assert len(parts) == 4
    assert abs(float(parts[0]) - 0.85) < 1e-6
    assert abs(float(parts[2]) - 0.88) < 1e-6


def test_emit_urdf_clamps_pitch_for_numerical_drift(tmp_path: Path) -> None:
    """P3: arcsin input slightly beyond [-1, 1] must not produce NaN — clamp prevents it."""
    # Construct a joint whose rotation has R[2,0] = -1.0000001 (out-of-range for arcsin)
    origin = np.eye(4)
    origin[2, 0] = 1.0000001  # so -R[2,0] = -1.0000001
    j = Joint(
        name="j_drift",
        type="revolute",
        parent="base",
        child="tip",
        axis=np.array([0.0, 0.0, 1.0]),
        origin=origin,
        limit_lower=-3.14,
        limit_upper=3.14,
        effort=10.0,
        velocity=2.0,
    )
    out = tmp_path / "robot.urdf"
    emit_urdf(_simple_two_link_robot(j), out, package_name="p")
    # Parse and verify the rpy attribute is finite (no "nan")
    tree = ET.parse(out)
    joint_el = tree.getroot().find("joint")
    assert joint_el is not None
    origin_el = joint_el.find("origin")
    assert origin_el is not None
    rpy_str = origin_el.attrib["rpy"]
    rpy_vals = [float(v) for v in rpy_str.split()]
    assert all(np.isfinite(rpy_vals)), f"Non-finite RPY values after clamp: {rpy_vals}"


def test_emit_urdf_rejects_mesh_path_with_dotdot(tmp_path: Path) -> None:
    """P2: mesh path with '..' segments must raise ValueError to prevent package traversal."""
    base = Link(
        name="base",
        visual_mesh_path=Path("../escape/evil.stl"),  # traversal path — forbidden
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
    robot = Robot(
        name="traversal_arm",
        base_link="base",
        links={"base": base, "tip": tip},
        joints={"j1": j},
    )
    out = tmp_path / "robot.urdf"
    with pytest.raises(ValueError, match=r"must not contain '\.\.' segments"):
        emit_urdf(robot, out, package_name="p")
