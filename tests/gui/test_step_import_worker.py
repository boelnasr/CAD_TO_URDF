from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("OCP")

from cad2urdf.core.kinematic.model import Robot
from cad2urdf.gui.workers import step_import
from tests.unit.test_step_parser import _write_named_assembly
from tests.unit.test_iges_parser import _write_flat_iges

pytestmark = pytest.mark.gui


def test_step_import_job_builds_robot(tmp_path: Path) -> None:
    step = tmp_path / "robot.step"
    _write_named_assembly(step)
    job = step_import.build_step_import_job(
        path=step, mesh_dir=tmp_path / "m", robot_name="robot"
    )
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


def test_cad_import_job_selects_iges_loader_for_flat_shape(tmp_path: Path) -> None:
    """Leaving the worker STEP-only breaks GUI and dispatcher IGES imports."""
    iges = tmp_path / "body.igs"
    _write_flat_iges(iges)
    builder = getattr(step_import, "build_cad_import_job", None)
    assert callable(builder)

    job = builder(path=iges, mesh_dir=tmp_path / "m", robot_name="robot")
    robot = job(lambda _c, _t, _m: None)

    assert isinstance(robot, Robot)
    assert robot.base_link == "body"
    assert set(robot.links) == {"body"}
    assert robot.joints == {}
    assert (tmp_path / "m" / "body.stl").is_file()


def test_step_import_job_remains_a_compatibility_alias() -> None:
    """Removing the legacy builder name needlessly breaks existing GUI callers."""
    assert step_import.build_step_import_job is step_import.build_cad_import_job
