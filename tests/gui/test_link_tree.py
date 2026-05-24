"""LinkTreeDock: shows the Robot's link/joint tree rooted at base_link."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Joint, Link, Robot

pytestmark = pytest.mark.gui


def _mk_link(name: str) -> Link:
    return Link(
        name=name,
        visual_mesh_path=Path(f"m/{name}.stl"),
        collision_mesh_path=Path(f"m/{name}.stl"),
        material_density=1.0,
        material_name="aluminum_6061",
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


def test_tree_renders_hierarchy_after_replace(qtbot) -> None:
    from cad2urdf.gui.panels.link_tree import LinkTreeDock
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    dock = LinkTreeDock(c)
    qtbot.addWidget(dock)
    c.replace(_three_link_robot())

    model = dock.tree_view.model()
    assert model.rowCount() == 1
    root = model.index(0, 0)
    assert model.data(root) == "a"
    assert model.rowCount(root) == 1
    b = model.index(0, 0, root)
    assert model.data(b) == "b"
    assert model.rowCount(b) == 1


def test_tree_emits_linkSelected_on_click(qtbot) -> None:
    from cad2urdf.gui.panels.link_tree import LinkTreeDock
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    dock = LinkTreeDock(c)
    qtbot.addWidget(dock)
    c.replace(_three_link_robot())

    received: list[str] = []
    dock.linkSelected.connect(lambda name: received.append(name))

    model = dock.tree_view.model()
    root = model.index(0, 0)
    b_index = model.index(0, 0, root)
    dock.tree_view.setCurrentIndex(b_index)

    assert received[-1] == "b"


def test_tree_refreshes_on_robotChanged(qtbot) -> None:
    from cad2urdf.core.kinematic.tree import remove_link
    from cad2urdf.gui.panels.link_tree import LinkTreeDock
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    dock = LinkTreeDock(c)
    qtbot.addWidget(dock)
    c.replace(_three_link_robot())
    c.apply(lambda r: remove_link(r, "c"), label="rm c")

    model = dock.tree_view.model()
    root = model.index(0, 0)
    b = model.index(0, 0, root)
    assert model.rowCount(b) == 0
