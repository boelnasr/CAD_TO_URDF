"""JointEditorDock: edits the parent joint of the currently-selected link."""

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


def _robot_with_revolute() -> Robot:
    return Robot(
        name="r",
        base_link="a",
        links={"a": _mk_link("a"), "b": _mk_link("b")},
        joints={
            "j1": Joint(
                name="j1",
                type="revolute",
                parent="a",
                child="b",
                axis=np.array([0.0, 0.0, 1.0]),
                origin=np.eye(4),
                limit_lower=-1.0,
                limit_upper=1.0,
                effort=10.0,
                velocity=2.0,
            )
        },
    )


def test_editor_populates_when_link_selected(qtbot) -> None:
    from cad2urdf.gui.panels.joint_editor import JointEditorDock
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    c.replace(_robot_with_revolute())
    dock = JointEditorDock(c)
    qtbot.addWidget(dock)

    dock.show_link("b")
    assert dock.type_combo.currentText() == "revolute"
    assert pytest.approx(dock.axis_x.value()) == 0.0
    assert pytest.approx(dock.axis_z.value()) == 1.0
    assert pytest.approx(dock.lower_limit.value()) == -1.0


def test_editor_is_blank_for_base_link(qtbot) -> None:
    from cad2urdf.gui.panels.joint_editor import JointEditorDock
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    c.replace(_robot_with_revolute())
    dock = JointEditorDock(c)
    qtbot.addWidget(dock)

    dock.show_link("a")
    assert not dock.form_widget.isEnabled()


def test_apply_button_pushes_change_into_controller(qtbot) -> None:
    from cad2urdf.gui.panels.joint_editor import JointEditorDock
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    c.replace(_robot_with_revolute())
    dock = JointEditorDock(c)
    qtbot.addWidget(dock)

    dock.show_link("b")
    dock.lower_limit.setValue(-2.5)
    dock.upper_limit.setValue(2.5)
    dock.apply_button.click()

    j = c.current().joints["j1"]
    assert j.limit_lower == pytest.approx(-2.5)
    assert j.limit_upper == pytest.approx(2.5)
    assert c.can_undo()
