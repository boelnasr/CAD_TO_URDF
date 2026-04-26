from pathlib import Path

import numpy as np

from cad2urdf.core.kinematic.model import (
    InertialOverride,
    Joint,
    Link,
    Robot,
)
from cad2urdf.core.kinematic.validate import ValidationIssue, validate_robot


def _mk_link(name: str) -> Link:
    return Link(
        name=name,
        visual_mesh_path=Path(f"m/{name}.stl"),
        collision_mesh_path=Path(f"m/{name}.stl"),
        material_density=1.0,
        material_name="m",
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )


def _mk_joint(name: str, parent: str, child: str) -> Joint:
    return Joint(
        name=name,
        type="fixed",
        parent=parent,
        child=child,
        axis=np.array([1.0, 0.0, 0.0]),
        origin=np.eye(4),
    )


def test_valid_two_link_chain_has_no_issues() -> None:
    r = Robot(
        name="r",
        base_link="a",
        links={"a": _mk_link("a"), "b": _mk_link("b")},
        joints={"ab": _mk_joint("ab", "a", "b")},
    )
    issues = validate_robot(r)
    assert issues == []


def test_dangling_link_is_flagged() -> None:
    r = Robot(
        name="r",
        base_link="a",
        links={"a": _mk_link("a"), "b": _mk_link("b"), "ghost": _mk_link("ghost")},
        joints={"ab": _mk_joint("ab", "a", "b")},
    )
    issues = validate_robot(r)
    assert any(
        isinstance(i, ValidationIssue) and i.kind == "dangling_link" and i.target == "ghost"
        for i in issues
    )


def test_two_parents_for_one_child_is_flagged() -> None:
    r = Robot(
        name="r",
        base_link="a",
        links={"a": _mk_link("a"), "b": _mk_link("b"), "c": _mk_link("c")},
        joints={
            "ab": _mk_joint("ab", "a", "b"),
            "cb": _mk_joint("cb", "c", "b"),  # b has two parents
        },
    )
    issues = validate_robot(r)
    assert any(i.kind == "multi_parent" and i.target == "b" for i in issues)
