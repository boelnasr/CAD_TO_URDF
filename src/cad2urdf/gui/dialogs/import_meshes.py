"""Import wizard helpers. The QFileDialog is opened from MainWindow; this module
exposes `run_import_into_controller(...)` so the worker plumbing is testable
without a real dialog.
"""

from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import QEventLoop

from cad2urdf.core.kinematic.model import Robot
from cad2urdf.gui.state.controller import RobotController
from cad2urdf.gui.workers.base import Worker
from cad2urdf.gui.workers.import_meshes import build_import_job


def run_import_into_controller(
    *,
    controller: RobotController,
    mesh_paths: list[Path],
    robot_name: str,
) -> None:
    """Spin up the import worker and block on a local QEventLoop until done."""
    worker = Worker(build_import_job(paths=mesh_paths, robot_name=robot_name))
    loop = QEventLoop()
    error_holder: dict[str, str] = {}

    def _on_done(result: object) -> None:
        if isinstance(result, Robot):
            controller.replace(result)
        loop.quit()

    def _on_failed(err: str, _trace: str) -> None:
        error_holder["err"] = err
        loop.quit()

    worker.finished.connect(_on_done)
    worker.failed.connect(_on_failed)
    worker.start()
    loop.exec()

    if "err" in error_holder:
        raise RuntimeError(error_holder["err"])
