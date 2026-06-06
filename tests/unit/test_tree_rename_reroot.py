from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Joint, Link, Robot
from cad2urdf.core.kinematic.tree import rename_link


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
