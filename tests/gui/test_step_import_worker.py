from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("OCP")

from cad2urdf.core.kinematic.model import Robot
from cad2urdf.gui.workers.step_import import build_step_import_job
from tests.unit.test_step_parser import _write_named_assembly

pytestmark = pytest.mark.gui


def test_step_import_job_builds_robot(tmp_path: Path) -> None:
    step = tmp_path / "robot.step"
    _write_named_assembly(step)
    job = build_step_import_job(path=step, mesh_dir=tmp_path / "m", robot_name="robot")
    reports: list[tuple[int, int]] = []
    robot = job(lambda c, t, m: reports.append((c, t)))
    assert isinstance(robot, Robot)
    assert {"base", "arm"} <= set(robot.links)
    # The worker emits exactly three numbered progress reports.
    assert reports == [(1, 3), (2, 3), (3, 3)]
    # The assembly root is a meshless frame base link; leaves carry meshes on disk.
    assert robot.base_link == "robot"
    assert robot.links["robot"].visual_mesh_path is None
    assert (tmp_path / "m" / "base.stl").is_file()
