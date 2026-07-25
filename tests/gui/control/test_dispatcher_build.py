import threading
from dataclasses import replace
from pathlib import Path

import pytest
import trimesh
from PyQt6.QtCore import QTimer

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.state.controller import RobotController


@pytest.fixture()
def two_meshes(tmp_path):
    a = tmp_path / "base.stl"
    b = tmp_path / "arm.stl"
    trimesh.creation.box(extents=(1, 1, 1)).export(a)
    trimesh.creation.box(extents=(1, 1, 1)).export(b)
    return [str(a), str(b)]


@pytest.fixture()
def dispatcher(qapp):
    return CommandDispatcher(RobotController())


def _imported(dispatcher, two_meshes):
    return dispatcher.dispatch(
        {"command": "import_meshes", "args": {"paths": two_meshes, "robot_name": "rig"}}
    )


def test_import_meshes_builds_links(dispatcher, two_meshes):
    resp = _imported(dispatcher, two_meshes)
    assert resp["ok"] is True
    names = {link["name"] for link in resp["result"]["links"]}
    assert names == {"base", "arm"}


def test_rename_link(dispatcher, two_meshes):
    _imported(dispatcher, two_meshes)
    resp = dispatcher.dispatch(
        {"command": "rename_link", "args": {"old": "arm", "new": "shoulder"}}
    )
    names = {link["name"] for link in resp["result"]["links"]}
    assert "shoulder" in names and "arm" not in names


def test_set_link_material_changes_density(dispatcher, two_meshes):
    _imported(dispatcher, two_meshes)
    resp = dispatcher.dispatch(
        {"command": "set_link_material", "args": {"link": "arm", "material": "steel_1018"}}
    )
    arm = next(link for link in resp["result"]["links"] if link["name"] == "arm")
    assert arm["material_name"] == "steel_1018"
    assert arm["material_density"] == pytest.approx(7850.0)


def test_remove_link(dispatcher, two_meshes):
    _imported(dispatcher, two_meshes)
    resp = dispatcher.dispatch({"command": "remove_link", "args": {"link": "arm"}})
    names = {link["name"] for link in resp["result"]["links"]}
    assert names == {"base"}


def test_set_base_link(dispatcher, two_meshes):
    _imported(dispatcher, two_meshes)
    resp = dispatcher.dispatch({"command": "set_base_link", "args": {"link": "arm"}})
    assert resp["result"]["base_link"] == "arm"


def test_import_rejects_missing_args(dispatcher):
    resp = dispatcher.dispatch({"command": "import_meshes", "args": {}})
    assert resp["ok"] is False


def test_import_cad_replaces_document_and_clears_history(
    monkeypatch: pytest.MonkeyPatch, qapp, tmp_path: Path
) -> None:
    """Using controller.apply would make a fresh CAD document incorrectly undoable."""
    controller = RobotController()
    controller.apply(lambda robot: replace(robot, name="edited"), label="edit")
    assert controller.can_undo()

    def fake_builder(*, path: Path, mesh_dir: Path, robot_name: str):
        assert path == tmp_path / "robot.iges"
        assert mesh_dir.is_dir()
        return lambda _report: replace(controller.current(), name=robot_name)

    monkeypatch.setattr(
        "cad2urdf.gui.control.dispatcher.build_cad_import_job", fake_builder
    )
    dispatcher = CommandDispatcher(controller)

    response = dispatcher.dispatch(
        {
            "command": "import_cad",
            "args": {"path": str(tmp_path / "robot.iges"), "robot_name": "fresh"},
        }
    )

    assert response["ok"] is True
    assert response["result"]["name"] == "fresh"
    assert controller.current().name == "fresh"
    assert not controller.can_undo()
    assert not controller.can_redo()


def test_import_cad_runs_heavy_job_off_dispatch_thread(
    monkeypatch: pytest.MonkeyPatch, qapp, tmp_path: Path
) -> None:
    """Calling the CAD job directly in the dispatcher would freeze Qt and fail this."""
    controller = RobotController()
    dispatch_thread = threading.get_ident()
    heavy_threads: list[int] = []
    replace_threads: list[int] = []
    controller.robotChanged.connect(lambda _robot: replace_threads.append(threading.get_ident()))

    def fake_builder(*, path: Path, mesh_dir: Path, robot_name: str):
        def job(_report):
            heavy_threads.append(threading.get_ident())
            return replace(controller.current(), name=robot_name)

        return job

    monkeypatch.setattr(
        "cad2urdf.gui.control.dispatcher.build_cad_import_job", fake_builder
    )

    response = CommandDispatcher(controller).dispatch(
        {
            "command": "import_cad",
            "args": {"path": str(tmp_path / "robot.iges"), "robot_name": "fresh"},
        }
    )

    assert response["ok"] is True
    assert response["result"]["name"] == "fresh"
    assert heavy_threads and heavy_threads[0] != dispatch_thread
    assert replace_threads == [dispatch_thread]


def test_dispatcher_shutdown_defers_pending_cleanup_until_worker_stops(
    monkeypatch: pytest.MonkeyPatch, qapp, tmp_path: Path
) -> None:
    """Shutdown must not remove a mesh directory underneath the MCP CAD worker."""
    controller = RobotController()
    started = threading.Event()
    release = threading.Event()
    mesh_dirs: list[Path] = []
    exists_after_shutdown: list[bool] = []
    worker_wrote_after_shutdown: list[bool] = []

    def blocking_builder(*, path: Path, mesh_dir: Path, robot_name: str):
        mesh_dirs.append(mesh_dir)

        def job(_report):
            started.set()
            assert release.wait(timeout=2.0)
            worker_wrote_after_shutdown.append(mesh_dir.exists())
            (mesh_dir / "finished.stl").write_text("finished")
            return replace(controller.current(), name=robot_name)

        return job

    monkeypatch.setattr(
        "cad2urdf.gui.control.dispatcher.build_cad_import_job", blocking_builder
    )

    def shutdown_when_started() -> None:
        if not started.is_set():
            QTimer.singleShot(1, shutdown_when_started)
            return
        controller.shutdown()
        exists_after_shutdown.append(mesh_dirs[0].exists())
        release.set()

    QTimer.singleShot(0, shutdown_when_started)
    response = CommandDispatcher(controller).dispatch(
        {
            "command": "import_cad",
            "args": {"path": str(tmp_path / "robot.iges"), "robot_name": "fresh"},
        }
    )

    assert response["ok"] is False
    assert "shutting down" in response["detail"]
    assert exists_after_shutdown == [True]
    assert worker_wrote_after_shutdown == [True]
    assert not mesh_dirs[0].exists()
