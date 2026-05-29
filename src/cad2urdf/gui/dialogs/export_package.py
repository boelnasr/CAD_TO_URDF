"""Export wizard glue: synchronous helper for tests + dialog used by MainWindow."""

from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import QEventLoop

from cad2urdf.gui.state.controller import RobotController
from cad2urdf.gui.workers.base import Worker
from cad2urdf.gui.workers.export_package import ExportReport, build_export_job


def run_export_into_dir(
    *,
    controller: RobotController,
    out_dir: Path,
    package_name: str,
    maintainer: str,
    maintainer_email: str,
    run_manipulapy: bool,
) -> ExportReport:
    """Run the export worker and block on a local QEventLoop until done."""
    worker = Worker(
        build_export_job(
            robot=controller.current(),
            out_dir=out_dir,
            package_name=package_name,
            maintainer=maintainer,
            maintainer_email=maintainer_email,
            run_manipulapy=run_manipulapy,
        )
    )
    loop = QEventLoop()
    holder: dict[str, ExportReport | str] = {}

    def _on_done(result: object) -> None:
        if isinstance(result, ExportReport):
            holder["report"] = result
        loop.quit()

    def _on_failed(err: str, _trace: str) -> None:
        holder["err"] = err
        loop.quit()

    worker.finished.connect(_on_done)
    worker.failed.connect(_on_failed)
    worker.start()
    loop.exec()

    if "err" in holder:
        raise RuntimeError(holder["err"])
    return holder["report"]  # type: ignore[return-value]
