from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import (
    InertialOverride,
    Joint,
    Link,
    Robot,
)
from cad2urdf.core.kinematic.tree import (
    add_link,
    children_of,
    parent_of,
    remove_link,
    reparent_joint,
)


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


def _three_link_robot() -> Robot:
    return Robot(
        name="r",
        base_link="a",
        links={"a": _mk_link("a"), "b": _mk_link("b"), "c": _mk_link("c")},
        joints={
            "ab": _mk_joint("ab", "a", "b"),
            "bc": _mk_joint("bc", "b", "c"),
        },
    )


def test_children_of_returns_direct_children() -> None:
    r = _three_link_robot()
    assert children_of(r, "a") == ["b"]
    assert children_of(r, "b") == ["c"]
    assert children_of(r, "c") == []


def test_parent_of_returns_parent_link() -> None:
    r = _three_link_robot()
    assert parent_of(r, "b") == "a"
    assert parent_of(r, "c") == "b"
    assert parent_of(r, "a") is None


def test_add_link_appends_with_joint() -> None:
    r = _three_link_robot()
    new = _mk_link("d")
    j = _mk_joint("cd", "c", "d")
    r2 = add_link(r, new, j)
    assert "d" in r2.links
    assert "cd" in r2.joints
    # original unchanged (immutable style)
    assert "d" not in r.links


def test_add_link_rejects_duplicate_link_name() -> None:
    r = _three_link_robot()
    j = _mk_joint("xx", "a", "b")
    with pytest.raises(ValueError, match=r"link 'b' already exists"):
        add_link(r, _mk_link("b"), j)


def test_add_link_rejects_unknown_parent() -> None:
    r = _three_link_robot()
    new = _mk_link("d")
    j = _mk_joint("cd", "ghost", "d")
    with pytest.raises(ValueError, match=r"parent link 'ghost' not in robot"):
        add_link(r, new, j)


def test_add_link_rejects_mismatched_joint_child() -> None:
    r = _three_link_robot()
    new = _mk_link("d")
    j = _mk_joint("cd", "c", "e")  # joint child != link name
    with pytest.raises(ValueError, match=r"joint.child 'e' != link.name 'd'"):
        add_link(r, new, j)


def test_remove_link_removes_subtree() -> None:
    r = _three_link_robot()
    r2 = remove_link(r, "b")
    assert "b" not in r2.links
    assert "c" not in r2.links  # subtree gone
    assert "ab" not in r2.joints
    assert "bc" not in r2.joints
    assert "a" in r2.links


def test_remove_link_rejects_base() -> None:
    r = _three_link_robot()
    with pytest.raises(ValueError, match=r"cannot remove base_link 'a'"):
        remove_link(r, "a")


def test_remove_link_rejects_unknown() -> None:
    r = _three_link_robot()
    with pytest.raises(ValueError, match=r"link 'ghost' not in robot"):
        remove_link(r, "ghost")


def test_reparent_joint_changes_parent() -> None:
    r = _three_link_robot()
    r2 = reparent_joint(r, "bc", new_parent="a")
    assert r2.joints["bc"].parent == "a"
    # original unchanged
    assert r.joints["bc"].parent == "b"


def test_reparent_joint_rejects_unknown_joint() -> None:
    r = _three_link_robot()
    with pytest.raises(ValueError, match=r"joint 'ghost' not in robot"):
        reparent_joint(r, "ghost", new_parent="a")


def test_reparent_joint_rejects_unknown_new_parent() -> None:
    r = _three_link_robot()
    with pytest.raises(ValueError, match=r"new parent 'ghost' not in robot"):
        reparent_joint(r, "bc", new_parent="ghost")
