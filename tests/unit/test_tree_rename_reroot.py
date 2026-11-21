from pathlib import Path
from typing import Any

import numpy as np
import pytest
from numpy.typing import NDArray

from cad2urdf.core.config.loader import origin_from_xyz_rpy
from cad2urdf.core.kinematic.model import InertialOverride, Joint, JointType, Link, Robot
from cad2urdf.core.kinematic.tree import rename_link, set_base_link


def _link(name: str) -> Link:
    return Link(
        name=name,
        visual_mesh_path=Path(f"/tmp/{name}.stl"),
        collision_mesh_path=Path(f"/tmp/{name}.stl"),
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
    )


def _chain() -> Robot:
    # base -> a -> b, all fixed joints at identity origins.
    j1 = Joint(name="j1", type="fixed", parent="base", child="a", axis=np.array([1.0, 0, 0]))
    j2 = Joint(name="j2", type="fixed", parent="a", child="b", axis=np.array([1.0, 0, 0]))
    return Robot(
        name="r",
        base_link="base",
        links={"base": _link("base"), "a": _link("a"), "b": _link("b")},
        joints={"j1": j1, "j2": j2},
    )


def test_rename_link_updates_links_and_joints():
    out = rename_link(_chain(), "a", "shoulder")
    assert "shoulder" in out.links
    assert "a" not in out.links
    assert out.joints["j1"].child == "shoulder"
    assert out.joints["j2"].parent == "shoulder"


def test_rename_base_link_updates_base():
    out = rename_link(_chain(), "base", "root")
    assert out.base_link == "root"
    assert "root" in out.links
    assert "base" not in out.links
    assert out.joints["j1"].parent == "root"


def test_rename_link_rejects_unknown_old():
    with pytest.raises(ValueError, match="not in robot"):
        rename_link(_chain(), "nosuchlink", "x")


def test_rename_link_rejects_existing_name():
    with pytest.raises(ValueError, match="already exists"):
        rename_link(_chain(), "a", "b")


def test_set_base_link_reroots_chain():
    # base -> a -> b ; make b the new base.
    out = set_base_link(_chain(), "b")
    assert out.base_link == "b"
    # j2 used to be a->b; now b->a.
    assert out.joints["j2"].parent == "b"
    assert out.joints["j2"].child == "a"
    # j1 used to be base->a; now a->base.
    assert out.joints["j1"].parent == "a"
    assert out.joints["j1"].child == "base"


def test_set_base_link_noop_when_same():
    out = set_base_link(_chain(), "base")
    assert out.base_link == "base"
    assert out.joints["j1"].parent == "base"


def test_set_base_link_rejects_unknown():
    with pytest.raises(ValueError, match="not in robot"):
        set_base_link(_chain(), "ghost")


def test_set_base_link_inverts_origin():
    r = _chain()
    moved = r.joints["j2"].origin.copy()
    moved[:3, 3] = [1.0, 2.0, 3.0]
    r.joints["j2"].origin = moved
    out = set_base_link(r, "b")
    # reversed edge origin is the matrix inverse: translation negates for pure translation.
    assert np.allclose(out.joints["j2"].origin, np.linalg.inv(moved), atol=1e-12)


def _rot(axis: NDArray[Any], q: float) -> NDArray[Any]:
    """4x4 rotation by q [rad] about unit ``axis`` (Rodrigues)."""
    a = np.asarray(axis, dtype=float)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    out = np.eye(4)
    out[:3, :3] = np.eye(3) + np.sin(q) * k + (1.0 - np.cos(q)) * k @ k
    return out


def _trans(axis: NDArray[Any], q: float) -> NDArray[Any]:
    """4x4 translation by q [m] along unit ``axis``."""
    out = np.eye(4)
    out[:3, 3] = q * np.asarray(axis, dtype=float)
    return out


def _single_joint(jtype: JointType, origin: NDArray[Any]) -> Robot:
    joint = Joint(
        name="j1",
        type=jtype,
        parent="base",
        child="l1",
        axis=np.array([0.0, 0.0, 1.0]),
        origin=origin,
        limit_lower=-1.0,
        limit_upper=1.0,
        effort=1.0,
        velocity=1.0,
    )
    return Robot(
        name="r",
        base_link="base",
        links={"base": _link("base"), "l1": _link("l1")},
        joints={"j1": joint},
    )


@pytest.mark.parametrize("q", [-0.9, -0.3, 0.4, 0.7])
def test_set_base_link_revolute_axis_reproduces_inverse_motion_issue8(q: float) -> None:
    """Reversed revolute joint must undo the forward motion at the same q.

    The URDF axis lives in the joint (child) frame, so after reversal it must be
    re-expressed in the old parent frame. The pivot sits on the axis line, as it
    always does for CAD-inferred revolute joints.
    """
    origin = origin_from_xyz_rpy([0.0, 0.0, 0.0], [np.pi / 2, 0.0, 0.0])
    origin[:3, 3] = origin[:3, :3] @ np.array([0.0, 0.0, 0.2])
    r = _single_joint("revolute", origin)
    fwd = r.joints["j1"]

    rev = set_base_link(r, "l1").joints["j1"]

    chain = fwd.origin @ _rot(fwd.axis, q) @ rev.origin @ _rot(rev.axis, q)
    assert np.allclose(chain, np.eye(4), atol=1e-12)
    assert (rev.limit_lower, rev.limit_upper) == (-1.0, 1.0)


@pytest.mark.parametrize("q", [-0.05, 0.02, 0.1])
def test_set_base_link_prismatic_axis_reproduces_inverse_motion_issue8(q: float) -> None:
    origin = origin_from_xyz_rpy([0.1, -0.2, 0.3], [np.pi / 2, 0.3, -0.4])
    r = _single_joint("prismatic", origin)
    fwd = r.joints["j1"]

    rev = set_base_link(r, "l1").joints["j1"]

    chain = fwd.origin @ _trans(fwd.axis, q) @ rev.origin @ _trans(rev.axis, q)
    assert np.allclose(chain, np.eye(4), atol=1e-12)
