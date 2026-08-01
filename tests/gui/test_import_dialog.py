"""ImportMeshesDialog: given a list of mesh paths + a robot name, seeds the controller."""

from __future__ import annotations

import threading
from pathlib import Path

import pytest
from PyQt6.QtCore import QTimer

pytestmark = pytest.mark.gui


def test_run_import_replaces_controller_robot(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.gui.dialogs.import_meshes import run_import_into_controller
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    with qtbot.waitSignal(c.robotChanged, timeout=8000):
        run_import_into_controller(
            controller=c, mesh_paths=[base_stl, arm_stl], robot_name="my_arm"
        )
    r = c.current()
    assert r.name == "my_arm"
    assert set(r.links) == {"base", "arm"}


def test_failed_cad_import_removes_generated_assets_but_not_source(
    monkeypatch: pytest.MonkeyPatch, qapp, tmp_path: Path
) -> None:
    """A failed GUI worker must retire its generated tree and preserve user input."""
    from cad2urdf.gui.dialogs import import_meshes
    from cad2urdf.gui.state.controller import RobotController

    try:
        from cad2urdf.gui.state.cad_assets import CadAssetOwner
    except ModuleNotFoundError:
        pytest.fail("generated CAD assets do not have an explicit owner")

    source = tmp_path / "source.iges"
    source.write_text("user input")
    owned_root = tmp_path / "owned"
    controller = RobotController(cad_assets=CadAssetOwner(root=owned_root))

    def failing_builder(*, path: Path, mesh_dir: Path, robot_name: str):
        def job(_report):
            (mesh_dir / "partial.stl").write_text("partial")
            raise ValueError("broken CAD")

        return job

    monkeypatch.setattr(import_meshes, "build_cad_import_job", failing_builder)

    with pytest.raises(RuntimeError, match="broken CAD"):
        import_meshes.run_cad_import_into_controller(
            controller=controller,
            cad_path=source,
            robot_name="robot",
        )

    assert source.read_text() == "user input"
    assert list(owned_root.iterdir()) == []


def test_window_close_defers_pending_cleanup_until_gui_worker_stops(
    monkeypatch: pytest.MonkeyPatch, qtbot, tmp_path: Path
) -> None:
    """Closing the window must not delete the directory a CAD worker is writing."""
    from cad2urdf.gui.dialogs import import_meshes
    from cad2urdf.gui.windows.main_window import MainWindow

    win = MainWindow()
    qtbot.addWidget(win)
    started = threading.Event()
    release = threading.Event()
    mesh_dirs: list[Path] = []
    exists_after_close: list[bool] = []
    worker_wrote_after_close: list[bool] = []

    def blocking_builder(*, path: Path, mesh_dir: Path, robot_name: str):
        mesh_dirs.append(mesh_dir)

        def job(_report):
            started.set()
            assert release.wait(timeout=2.0)
            worker_wrote_after_close.append(mesh_dir.exists())
            (mesh_dir / "finished.stl").write_text("finished")
            return win.controller.current()

        return job

    monkeypatch.setattr(import_meshes, "build_cad_import_job", blocking_builder)

    def close_when_started() -> None:
        if not started.is_set():
            QTimer.singleShot(1, close_when_started)
            return
        win.close()
        exists_after_close.append(mesh_dirs[0].exists())
        release.set()

    QTimer.singleShot(0, close_when_started)
    with pytest.raises(RuntimeError, match="shutting down"):
        import_meshes.run_cad_import_into_controller(
            controller=win.controller,
            cad_path=tmp_path / "robot.iges",
            robot_name="robot",
        )

    assert exists_after_close == [True]
    assert worker_wrote_after_close == [True]
    assert not mesh_dirs[0].exists()
