"""RobotController: holds the AST, mediates mutations, supports undo/redo."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Joint, Link, Robot

pytestmark = pytest.mark.gui


def _mk_link(name: str) -> Link:
    return Link(
        name=name,
        visual_mesh_path=Path(f"meshes/visual/{name}.stl"),
        collision_mesh_path=Path(f"meshes/collision/{name}.stl"),
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )


def _mk_fixed_joint(name: str, parent: str, child: str) -> Joint:
    return Joint(
        name=name,
        type="fixed",
        parent=parent,
        child=child,
        axis=np.array([1.0, 0.0, 0.0]),
        origin=np.eye(4),
    )


def _two_link_robot() -> Robot:
    return Robot(
        name="r",
        base_link="base",
        links={"base": _mk_link("base"), "arm": _mk_link("arm")},
        joints={"j": _mk_fixed_joint("j", "base", "arm")},
    )


def test_controller_starts_with_an_empty_robot(qtbot) -> None:
    _ = qtbot
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    r = c.current()
    assert isinstance(r, Robot)
    assert len(r.links) == 1
    assert r.base_link in r.links


def test_replace_emits_robotChanged(qtbot) -> None:
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    with qtbot.waitSignal(c.robotChanged, timeout=1000):
        c.replace(_two_link_robot())
    assert len(c.current().links) == 2


def test_apply_undo_redo_round_trip(qtbot) -> None:
    _ = qtbot
    from cad2urdf.core.kinematic.tree import remove_link
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    c.replace(_two_link_robot())
    assert "arm" in c.current().links

    c.apply(lambda r: remove_link(r, "arm"), label="remove arm")
    assert "arm" not in c.current().links
    assert c.can_undo()
    assert not c.can_redo()

    c.undo()
    assert "arm" in c.current().links
    assert c.can_redo()

    c.redo()
    assert "arm" not in c.current().links


def test_apply_emits_change_signal_with_label(qtbot) -> None:
    from cad2urdf.core.kinematic.tree import remove_link
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    c.replace(_two_link_robot())

    received: list[str] = []
    c.historyChanged.connect(lambda label: received.append(label))

    c.apply(lambda r: remove_link(r, "arm"), label="remove arm")
    assert received == ["remove arm"]


def test_apply_rejects_invalid_transform_without_corrupting_state(qtbot) -> None:
    _ = qtbot
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    c.replace(_two_link_robot())
    before = c.current()

    with pytest.raises(ValueError, match="boom"):
        c.apply(lambda _r: (_ for _ in ()).throw(ValueError("boom")), label="bad")

    # The bad transform must not have mutated state nor pushed an undo entry.
    assert c.current() is before
    assert not c.can_undo()
