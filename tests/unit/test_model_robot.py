from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import (
    InertialOverride,
    Joint,
    Link,
    Robot,
)


def _link(name: str) -> Link:
    return Link(
        name=name,
        visual_mesh_path=Path(f"meshes/{name}.stl"),
        collision_mesh_path=Path(f"meshes/{name}.stl"),
        material_density=2700.0,
        material_name="aluminum",
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )


def _fixed_joint(name: str, parent: str, child: str) -> Joint:
    return Joint(
        name=name,
        type="fixed",
        parent=parent,
        child=child,
        axis=np.array([1.0, 0.0, 0.0]),
        origin=np.eye(4),
    )


def test_robot_constructs_with_two_links_one_joint() -> None:
    r = Robot(
        name="my_arm",
        base_link="base",
        links={"base": _link("base"), "tip": _link("tip")},
        joints={"j1": _fixed_joint("j1", "base", "tip")},
    )
    assert r.name == "my_arm"
    assert r.base_link == "base"
    assert len(r.links) == 2


def test_robot_rejects_unknown_base_link() -> None:
    with pytest.raises(ValueError, match=r"base_link 'ghost' not in links"):
        Robot(
            name="bad",
            base_link="ghost",
            links={"base": _link("base")},
            joints={},
        )


def test_robot_rejects_joint_referencing_unknown_link() -> None:
    with pytest.raises(ValueError, match=r"joint 'j1' references unknown"):
        Robot(
            name="bad",
            base_link="base",
            links={"base": _link("base")},
            joints={"j1": _fixed_joint("j1", "base", "ghost")},
        )


def test_robot_rejects_empty_name() -> None:
    with pytest.raises(ValueError, match="name must not be empty"):
        Robot(
            name="",
            base_link="base",
            links={"base": _link("base")},
            joints={},
        )
