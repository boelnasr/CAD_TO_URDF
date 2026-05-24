"""ImportMeshesWorker: loads STL/OBJ files into a seed Robot with fixed joints."""

from __future__ import annotations

from pathlib import Path

import pytest

from cad2urdf.core.kinematic.model import Robot

pytestmark = pytest.mark.gui


def test_import_two_meshes_returns_robot_with_two_links(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.gui.workers.base import Worker
    from cad2urdf.gui.workers.import_meshes import build_import_job

    job = build_import_job(paths=[base_stl, arm_stl], robot_name="r")
    w = Worker(job)
    with qtbot.waitSignal(w.finished, timeout=5000) as blocker:
        w.start()
    robot = blocker.args[0]
    assert isinstance(robot, Robot)
    assert robot.name == "r"
    assert set(robot.links) == {"base", "arm"}
    # base_link must be the first imported mesh; the rest are fixed-joined to it.
    assert robot.base_link == "base"
    assert len(robot.joints) == 1
    only_joint = next(iter(robot.joints.values()))
    assert only_joint.type == "fixed"
    assert only_joint.parent == "base"
    assert only_joint.child == "arm"


def test_import_rejects_unsupported_extension(qtbot, tmp_path: Path) -> None:
    from cad2urdf.gui.workers.base import Worker
    from cad2urdf.gui.workers.import_meshes import build_import_job

    bogus = tmp_path / "thing.xyz"
    bogus.write_text("not a mesh")

    w = Worker(build_import_job(paths=[bogus], robot_name="r"))
    with qtbot.waitSignal(w.failed, timeout=5000) as blocker:
        w.start()
    err, _ = blocker.args
    assert ".xyz" in err or "unsupported" in err.lower()


def test_import_step_path_emits_helpful_error(qtbot, tmp_path: Path) -> None:
    from cad2urdf.gui.workers.base import Worker
    from cad2urdf.gui.workers.import_meshes import build_import_job

    step = tmp_path / "thing.step"
    step.write_text("not a real step")

    w = Worker(build_import_job(paths=[step], robot_name="r"))
    with qtbot.waitSignal(w.failed, timeout=5000) as blocker:
        w.start()
    err, _ = blocker.args
    assert "pythonOCC" in err or "STEP" in err
