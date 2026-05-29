"""Drag-reparent: dropping link B onto link C calls reparent_joint(j_ab, C)."""

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


def test_reparent_link_helper_pushes_through_controller(qtbot) -> None:
    _ = qtbot  # ensures QApplication is running for QWidget construction
    from cad2urdf.gui.panels.link_tree import LinkTreeDock
    from cad2urdf.gui.state.controller import RobotController

    r = Robot(
        name="r",
        base_link="a",
        links={"a": _mk_link("a"), "b": _mk_link("b"), "c": _mk_link("c")},
        joints={
            "ab": _mk_joint("ab", "a", "b"),
            "ac": _mk_joint("ac", "a", "c"),
        },
    )
    c = RobotController()
    dock = LinkTreeDock(c)
    c.replace(r)

    dock.reparent_link("b", new_parent="c")
    j = c.current().joints["ab"]
    assert j.parent == "c"
    assert c.can_undo()


def test_reparent_rejects_dropping_link_on_itself(qtbot) -> None:
    _ = qtbot  # ensures QApplication is running for QWidget construction
    from cad2urdf.gui.panels.link_tree import LinkTreeDock
    from cad2urdf.gui.state.controller import RobotController

    r = Robot(
        name="r",
        base_link="a",
        links={"a": _mk_link("a"), "b": _mk_link("b")},
        joints={"ab": _mk_joint("ab", "a", "b")},
    )
    c = RobotController()
    dock = LinkTreeDock(c)
    c.replace(r)

    with pytest.raises(ValueError, match="onto itself"):
        dock.reparent_link("b", new_parent="b")


def test_reparent_rejects_dropping_onto_descendant(qtbot) -> None:
    """Reparenting 'b' under 'c' (its own descendant) would create a cycle."""
    _ = qtbot  # ensures QApplication is running for QWidget construction
    from cad2urdf.gui.panels.link_tree import LinkTreeDock
    from cad2urdf.gui.state.controller import RobotController

    r = Robot(
        name="r",
        base_link="a",
        links={"a": _mk_link("a"), "b": _mk_link("b"), "c": _mk_link("c")},
        joints={
            "ab": _mk_joint("ab", "a", "b"),
            "bc": _mk_joint("bc", "b", "c"),
        },
    )
    c = RobotController()
    dock = LinkTreeDock(c)
    c.replace(r)

    with pytest.raises(ValueError, match="would create a cycle"):
        dock.reparent_link("b", new_parent="c")
