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


def test_cad_import_action_and_dialog_cover_step_and_iges(
    qtbot, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Regressing the action or picker to STEP-only hides IGES from GUI users."""
    from PyQt6.QtWidgets import QFileDialog, QMainWindow

    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.windows.main_window import MainWindow

    captured: list[tuple[str, str]] = []

    def fake_get_open_file_name(*args):
        captured.append((args[1], args[3]))
        return "", ""

    monkeypatch.setattr(QFileDialog, "getOpenFileName", fake_get_open_file_name)
    # Exercise the real action and slot without constructing the VTK viewport.
    win = QMainWindow()
    win.controller = RobotController()
    qtbot.addWidget(win)
    MainWindow._build_actions(win)

    assert "CAD" in win.action_import_cad.text()
    MainWindow._on_import_cad_clicked(win)
    assert captured == [
        ("Import CAD assembly", "STEP/IGES (*.step *.stp *.iges *.igs)")
    ]


def test_closing_window_removes_current_generated_cad_assets(qtbot) -> None:
    """Closing the GUI without another import must still release session assets."""
    from cad2urdf.gui.windows.main_window import MainWindow

    win = MainWindow()
    qtbot.addWidget(win)
    assets = win.controller.create_cad_import_dir()
    (assets / "body.stl").write_text("mesh")
    win.controller.replace_with_cad_assets(win.controller.current(), asset_dir=assets)

    win.close()

    assert not assets.exists()


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
