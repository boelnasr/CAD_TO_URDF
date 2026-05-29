"""InertiaEditorDock: material/density plus auto-computed inertia values."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Link, Robot

pytestmark = pytest.mark.gui


def _mk_link_with_real_mesh(name: str, stl: Path) -> Link:
    return Link(
        name=name,
        visual_mesh_path=stl,
        collision_mesh_path=stl,
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )


def _robot_with_real_mesh(stl: Path) -> Robot:
    return Robot(
        name="r",
        base_link="base",
        links={"base": _mk_link_with_real_mesh("base", stl)},
        joints={},
    )


def test_editor_lists_materials(qtbot) -> None:
    from cad2urdf.gui.panels.inertia_editor import InertiaEditorDock
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    dock = InertiaEditorDock(c)
    qtbot.addWidget(dock)
    items = [dock.material_combo.itemText(i) for i in range(dock.material_combo.count())]
    assert "aluminum_6061" in items


def test_editor_populates_density_from_link(qtbot, base_stl) -> None:
    from cad2urdf.gui.panels.inertia_editor import InertiaEditorDock
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    c.replace(_robot_with_real_mesh(base_stl))
    dock = InertiaEditorDock(c)
    qtbot.addWidget(dock)
    dock.show_link("base")
    assert dock.material_combo.currentText() == "aluminum_6061"
    assert dock.density_spin.value() == pytest.approx(2700.0)


def test_recompute_updates_auto_mass_label(qtbot, base_stl) -> None:
    from cad2urdf.gui.panels.inertia_editor import InertiaEditorDock
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    c.replace(_robot_with_real_mesh(base_stl))
    dock = InertiaEditorDock(c)
    qtbot.addWidget(dock)
    dock.show_link("base")

    with qtbot.waitSignal(dock.recomputed, timeout=5000):
        dock.recompute_button.click()

    assert "2700" in dock.auto_mass_label.text()


def test_changing_material_pushes_link_update(qtbot, base_stl) -> None:
    from cad2urdf.gui.panels.inertia_editor import InertiaEditorDock
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    c.replace(_robot_with_real_mesh(base_stl))
    dock = InertiaEditorDock(c)
    qtbot.addWidget(dock)
    dock.show_link("base")

    dock.material_combo.setCurrentText("steel_1018")
    dock.apply_material_button.click()
    link = c.current().links["base"]
    assert link.material_name == "steel_1018"
    assert link.material_density == pytest.approx(7850.0)
