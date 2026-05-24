"""MainWindow shell: menus, toolbar actions, dock layout, status strip."""

from __future__ import annotations

import pytest

pytestmark = pytest.mark.gui


def test_main_window_has_required_menus(qtbot) -> None:
    from cad2urdf.gui.windows.main_window import MainWindow

    win = MainWindow()
    qtbot.addWidget(win)
    menu_titles = [a.text() for a in win.menuBar().actions()]
    for required in ("&File", "&Edit", "&View", "&Export", "&Help"):
        assert required in menu_titles


def test_main_window_has_three_docks(qtbot) -> None:
    from PyQt6.QtWidgets import QDockWidget

    from cad2urdf.gui.windows.main_window import MainWindow

    win = MainWindow()
    qtbot.addWidget(win)
    docks = win.findChildren(QDockWidget)
    titles = sorted(d.windowTitle() for d in docks)
    assert titles == ["Inertia Editor", "Joint Editor", "Link Tree"]


def test_main_window_status_bar_shows_default_message(qtbot) -> None:
    from cad2urdf.gui.windows.main_window import MainWindow

    win = MainWindow()
    qtbot.addWidget(win)
    msg = win.statusBar().currentMessage()
    assert "Ready" in msg or "ready" in msg


def test_undo_action_disabled_initially(qtbot) -> None:
    from cad2urdf.gui.windows.main_window import MainWindow

    win = MainWindow()
    qtbot.addWidget(win)
    assert not win.action_undo.isEnabled()
    assert not win.action_redo.isEnabled()


def test_controller_replace_enables_undo(qtbot) -> None:
    from pathlib import Path

    import numpy as np

    from cad2urdf.core.kinematic.model import InertialOverride, Joint, Link, Robot
    from cad2urdf.core.kinematic.tree import remove_link
    from cad2urdf.gui.windows.main_window import MainWindow

    win = MainWindow()
    qtbot.addWidget(win)
    r = Robot(
        name="r",
        base_link="b",
        links={
            "b": Link(
                name="b",
                visual_mesh_path=Path("m.stl"),
                collision_mesh_path=Path("m.stl"),
                material_density=1.0,
                material_name="aluminum_6061",
                inertial_override=InertialOverride(),
                origin=np.eye(4),
            ),
            "c": Link(
                name="c",
                visual_mesh_path=Path("m.stl"),
                collision_mesh_path=Path("m.stl"),
                material_density=1.0,
                material_name="aluminum_6061",
                inertial_override=InertialOverride(),
                origin=np.eye(4),
            ),
        },
        joints={
            "j": Joint(
                name="j",
                type="fixed",
                parent="b",
                child="c",
                axis=np.array([1.0, 0.0, 0.0]),
                origin=np.eye(4),
            )
        },
    )
    win.controller.replace(r)
    win.controller.apply(lambda rr: remove_link(rr, "c"), label="rm c")
    assert win.action_undo.isEnabled()
