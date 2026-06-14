from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Joint, Link, Robot
from cad2urdf.core.project.save import load_project, save_project
from cad2urdf.core.urdf.emit import emit_urdf


def _frame(name: str) -> Link:
    return Link(name, None, None, None, None, InertialOverride())


def test_frame_link_allows_no_mesh_or_material() -> None:
    link = Link(
        name="frame",
        visual_mesh_path=None,
        collision_mesh_path=None,
        material_density=None,
        material_name=None,
        inertial_override=InertialOverride(),
    )
    assert link.visual_mesh_path is None
    assert link.material_density is None


def test_mesh_link_still_rejects_nonpositive_density() -> None:
    with pytest.raises(ValueError, match="material_density"):
        Link(
            name="part",
            visual_mesh_path=Path("m/part.stl"),
            collision_mesh_path=Path("m/part.stl"),
            material_density=0.0,
            material_name="aluminum_6061",
            inertial_override=InertialOverride(),
        )


def test_emit_frame_link_has_no_visual(tmp_path: Path) -> None:
    base = _frame("root")
    arm = Link(
        "arm",
        Path("meshes/visual/arm.stl"),
        Path("meshes/collision/arm.stl"),
        2700.0,
        "aluminum_6061",
        InertialOverride(mass=1.0, com=np.zeros(3), inertia=np.eye(3)),
    )
    robot = Robot(
        name="r",
        base_link="root",
        links={"root": base, "arm": arm},
        joints={
            "arm_to_root": Joint(
                name="arm_to_root",
                type="fixed",
                parent="root",
                child="arm",
                axis=np.array([1.0, 0.0, 0.0]),
            )
        },
    )
    out = tmp_path / "r.urdf"
    emit_urdf(robot, out, package_name="r")
    tree = ET.parse(out)
    links = {el.get("name"): el for el in tree.getroot().findall("link")}
    assert "root" in links  # the frame link still emits a bare <link> element
    assert links["root"].find("visual") is None
    assert links["root"].find("collision") is None
    assert links["root"].find("inertial") is None
    assert links["arm"].find("visual") is not None


def test_frame_link_project_roundtrip(tmp_path: Path) -> None:
    robot = Robot(
        name="r",
        base_link="root",
        links={"root": _frame("root")},
        joints={},
    )
    p = tmp_path / "r.cad2urdf"
    save_project(robot, p)
    loaded = load_project(p)
    assert loaded.links["root"].visual_mesh_path is None
    assert loaded.links["root"].material_density is None
