"""Generic Worker — wraps any `(report) -> T` callable in a QThread.

`report(current, total, message)` is supplied to the job; the worker forwards
those calls onto the GUI thread via the `progress` signal. The job's return
value comes back via `finished`; any exception becomes a `failed` signal
(string error + traceback) instead of crashing the thread.
"""

from __future__ import annotations

import logging
import traceback
from collections.abc import Callable
from typing import Any

from PyQt6.QtCore import QEventLoop, QObject, QThread, pyqtSignal

_log = logging.getLogger(__name__)

ProgressReporter = Callable[[int, int, str], None]
JobFunc = Callable[[ProgressReporter], Any]


class _JobRunner(QObject):
    finished = pyqtSignal(object)
    failed = pyqtSignal(object, str)
    progress = pyqtSignal(int, int, str)

    def __init__(self, job: JobFunc) -> None:
        super().__init__()
        self._job = job

    def run(self) -> None:
        try:
            result = self._job(lambda c, t, m: self.progress.emit(c, t, m))
        except BaseException as e:
            self.failed.emit(e, traceback.format_exc())
            return
        self.finished.emit(result)


class Worker(QObject):
    """Public worker handle. Owns a QThread + _JobRunner; signals forwarded."""

    finished = pyqtSignal(object)
    failed = pyqtSignal(str, str)
    failed_exception = pyqtSignal(object, str)
    progress = pyqtSignal(int, int, str)

    def __init__(self, job: JobFunc, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._thread = QThread(self)
        self._runner = _JobRunner(job)
        self._runner.moveToThread(self._thread)
        self._thread.started.connect(self._runner.run)
        self._runner.finished.connect(self._on_finished)
        self._runner.failed.connect(self._on_failed)
        self._runner.progress.connect(self.progress)

    def start(self) -> None:
        self._thread.start()

    def _shutdown_thread(self) -> None:
        self._thread.quit()
        if not self._thread.wait(2000):
            _log.warning("Worker thread did not exit within 2 s; forcing termination.")
            self._thread.terminate()

    def _on_finished(self, result: Any) -> None:
        self.finished.emit(result)
        self._shutdown_thread()

    def _on_failed(self, error: BaseException, trace: str) -> None:
        self.failed_exception.emit(error, trace)
        self.failed.emit(str(error), trace)
        self._shutdown_thread()


def run_worker_job(job: JobFunc) -> Any:
    """Run ``job`` on a Worker thread while a local Qt loop awaits its result."""
    worker = Worker(job)
    loop = QEventLoop()
    result_holder: list[Any] = []
    error_holder: list[BaseException] = []

    def _on_finished(result: Any) -> None:
        result_holder.append(result)
        loop.quit()

    def _on_failed(error: BaseException, _trace: str) -> None:
        error_holder.append(error)
        loop.quit()

    worker.finished.connect(_on_finished)
    worker.failed_exception.connect(_on_failed)
    worker.start()
    loop.exec()

    if error_holder:
        raise error_holder[0]
    if not result_holder:
        raise RuntimeError("worker exited without a result")
    return result_holder[0]
