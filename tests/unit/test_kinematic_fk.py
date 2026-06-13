"""Forward kinematics: world-frame transform of each link via the joint chain."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from cad2urdf.core.kinematic.model import InertialOverride, Joint, Link, Robot
from cad2urdf.core.kinematic.tree import link_world_transforms


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


def _translate(x: float, y: float, z: float) -> np.ndarray:
    t = np.eye(4)
    t[:3, 3] = (x, y, z)
    return t


def _mk_joint(name: str, parent: str, child: str, origin: np.ndarray) -> Joint:
    return Joint(
        name=name,
        type="fixed",
        parent=parent,
        child=child,
        axis=np.array([1.0, 0.0, 0.0]),
        origin=origin,
    )


def test_base_link_world_transform_is_identity() -> None:
    r = Robot(name="r", base_link="a", links={"a": _mk_link("a")}, joints={})
    w = link_world_transforms(r)
    np.testing.assert_allclose(w["a"], np.eye(4))


def test_translation_chain_composes_down_the_tree() -> None:
    r = Robot(
        name="r",
        base_link="a",
        links={"a": _mk_link("a"), "b": _mk_link("b"), "c": _mk_link("c")},
        joints={
            "ab": _mk_joint("ab", "a", "b", _translate(0, 0, 1.0)),
            "bc": _mk_joint("bc", "b", "c", _translate(0, 0, 2.0)),
        },
    )
    w = link_world_transforms(r)
    np.testing.assert_allclose(w["a"][:3, 3], [0, 0, 0])
    np.testing.assert_allclose(w["b"][:3, 3], [0, 0, 1.0])
    np.testing.assert_allclose(w["c"][:3, 3], [0, 0, 3.0])  # 1 + 2


def test_rotation_then_translation_uses_parent_orientation() -> None:
    # b is rotated 90deg about x, then c is offset +z in b's frame -> world +y.
    rot = np.eye(4)
    rot[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)  # Rx(90)
    r = Robot(
        name="r",
        base_link="a",
        links={"a": _mk_link("a"), "b": _mk_link("b"), "c": _mk_link("c")},
        joints={
            "ab": _mk_joint("ab", "a", "b", rot),
            "bc": _mk_joint("bc", "b", "c", _translate(0, 0, 1.0)),
        },
    )
    w = link_world_transforms(r)
    np.testing.assert_allclose(w["c"][:3, 3], [0, -1.0, 0], atol=1e-9)


def test_orphan_link_defaults_to_identity() -> None:
    # 'b' has no joint connecting it to the base -> unreachable -> identity.
    r = Robot(
        name="r",
        base_link="a",
        links={"a": _mk_link("a"), "b": _mk_link("b")},
        joints={},
    )
    w = link_world_transforms(r)
    assert set(w) == {"a", "b"}
    np.testing.assert_allclose(w["b"], np.eye(4))
