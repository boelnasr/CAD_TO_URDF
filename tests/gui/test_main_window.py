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


def test_closing_window_stops_control_queue_before_controller(qtbot, monkeypatch) -> None:
    """No queued command may outlive the controller's mutation boundary."""
    from cad2urdf.gui.windows.main_window import MainWindow

    calls: list[str] = []

    class FakeServer:
        def stop(self) -> None:
            calls.append("server")

    win = MainWindow()
    qtbot.addWidget(win)
    win.set_control_server(FakeServer())  # type: ignore[arg-type]
    monkeypatch.setattr(
        win.controller,
        "shutdown",
        lambda: calls.append("controller"),
    )

    win.close()

    assert calls == ["server", "controller"]


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


def test_validate_action_ignores_clicks_while_running_issue13(qtbot, monkeypatch) -> None:
    """A second click used to drop the only reference to a running QThread (SIGABRT)."""
    import threading

    import cad2urdf.gui.workers.validate as validate_mod
    from cad2urdf.gui.windows.main_window import MainWindow

    release = threading.Event()
    started: list[int] = []

    def fake_build_validate_job(**_kwargs):
        def job(_report):
            started.append(1)
            release.wait(5)
            return validate_mod.ValidateReport(urdf_written=True)

        return job

    monkeypatch.setattr(validate_mod, "build_validate_job", fake_build_validate_job)
    win = MainWindow()
    qtbot.addWidget(win)

    win.action_validate.trigger()
    try:
        assert not win.action_validate.isEnabled()
        win.action_validate.trigger()
    finally:
        release.set()
        qtbot.waitUntil(
            lambda: win.statusBar().currentMessage().startswith("validation OK"), timeout=5000
        )

    assert started == [1]
    assert win.action_validate.isEnabled()


def test_validate_action_re_enabled_after_failure_issue13(qtbot, monkeypatch) -> None:
    import cad2urdf.gui.workers.validate as validate_mod
    from cad2urdf.gui.windows.main_window import MainWindow

    def failing_build_validate_job(**_kwargs):
        def job(_report):
            raise RuntimeError("boom")

        return job

    monkeypatch.setattr(validate_mod, "build_validate_job", failing_build_validate_job)
    win = MainWindow()
    qtbot.addWidget(win)

    win.action_validate.trigger()
    qtbot.waitUntil(
        lambda: win.statusBar().currentMessage().startswith("validate failed"), timeout=5000
    )

    assert win.action_validate.isEnabled()


def test_validate_succeeds_for_robot_name_invalid_as_ros_package_issue22(
    qtbot, monkeypatch, base_stl
) -> None:
    """The scratch package is internal; 'UR5' is a fine robot name but not a ROS package name."""
    import cad2urdf.gui.workers.validate as validate_mod
    from cad2urdf.gui.dialogs.import_meshes import run_import_into_controller
    from cad2urdf.gui.windows.main_window import MainWindow

    real_build_validate_job = validate_mod.build_validate_job

    def without_manipulapy(**kwargs):
        # The ManipulaPy import is slow and irrelevant to the package-name check.
        return real_build_validate_job(**{**kwargs, "run_manipulapy": False})

    monkeypatch.setattr(validate_mod, "build_validate_job", without_manipulapy)
    win = MainWindow()
    qtbot.addWidget(win)
    run_import_into_controller(controller=win.controller, mesh_paths=[base_stl], robot_name="UR5")

    win.action_validate.trigger()
    qtbot.waitUntil(
        lambda: not win.statusBar().currentMessage().startswith("validating"), timeout=10000
    )

    assert win.statusBar().currentMessage().startswith("validation OK")


@pytest.mark.parametrize("outcome", ["finished", "failed"])
def test_validate_removes_scratch_dir_issue28(qtbot, monkeypatch, tmp_path, outcome) -> None:
    """Each Validate click used to leak one cad2urdf_validate_* temp directory."""
    import tempfile

    import cad2urdf.gui.workers.validate as validate_mod
    from cad2urdf.gui.windows.main_window import MainWindow

    monkeypatch.setattr(tempfile, "tempdir", str(tmp_path))
    out_dirs: list = []

    def fake_build_validate_job(*, out_dir, **_kwargs):
        out_dirs.append(out_dir)

        def job(_report):
            (out_dir / "robot.urdf").write_text("<robot/>")
            if outcome == "failed":
                raise RuntimeError("boom")
            return validate_mod.ValidateReport(urdf_written=True)

        return job

    monkeypatch.setattr(validate_mod, "build_validate_job", fake_build_validate_job)
    win = MainWindow()
    qtbot.addWidget(win)

    win.action_validate.trigger()
    qtbot.waitUntil(win.action_validate.isEnabled, timeout=5000)

    assert out_dirs and out_dirs[0].parent == tmp_path  # scratch was under tmp_path
    assert list(tmp_path.glob("cad2urdf_validate_*")) == []
