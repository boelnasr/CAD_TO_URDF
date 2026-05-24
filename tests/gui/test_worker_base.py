"""Worker base: runs a callable on a QThread, emits progress / finished / failed."""

from __future__ import annotations

import pytest

pytestmark = pytest.mark.gui


def test_worker_emits_finished_with_result(qtbot) -> None:
    from cad2urdf.gui.workers.base import Worker

    def job(report) -> int:
        report(0, 1, "starting")
        report(1, 1, "done")
        return 42

    w = Worker(job)
    with qtbot.waitSignal(w.finished, timeout=2000) as blocker:
        w.start()
    assert blocker.args == [42]


def test_worker_emits_failed_on_exception(qtbot) -> None:
    from cad2urdf.gui.workers.base import Worker

    def job(report) -> int:
        raise RuntimeError("nope")

    w = Worker(job)
    with qtbot.waitSignal(w.failed, timeout=2000) as blocker:
        w.start()
    err, _trace = blocker.args
    assert "nope" in err


def test_worker_progress_signal_fires(qtbot) -> None:
    from cad2urdf.gui.workers.base import Worker

    received: list[tuple[int, int, str]] = []

    def job(report) -> str:
        report(1, 3, "step 1")
        report(2, 3, "step 2")
        report(3, 3, "step 3")
        return "ok"

    w = Worker(job)
    w.progress.connect(lambda c, t, m: received.append((c, t, m)))

    with qtbot.waitSignal(w.finished, timeout=2000):
        w.start()

    assert received == [(1, 3, "step 1"), (2, 3, "step 2"), (3, 3, "step 3")]
