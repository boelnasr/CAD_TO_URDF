"""Validate worker: bundles AST + URDF validation into one off-thread call."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Link, Robot

pytestmark = pytest.mark.gui


def _trivial_robot(stl: Path) -> Robot:
    return Robot(
        name="r",
        base_link="base",
        links={
            "base": Link(
                name="base",
                visual_mesh_path=stl,
                collision_mesh_path=stl,
                material_density=2700.0,
                material_name="aluminum_6061",
                inertial_override=InertialOverride(),
                origin=np.eye(4),
            )
        },
        joints={},
    )


def test_validate_returns_report_with_ast_issues_and_urdf_status(qtbot, base_stl, tmp_path) -> None:
    from cad2urdf.gui.workers.base import Worker
    from cad2urdf.gui.workers.validate import ValidateReport, build_validate_job

    job = build_validate_job(
        robot=_trivial_robot(base_stl),
        out_dir=tmp_path / "pkg",
        package_name="r_pkg",
        urdf_relname="r.urdf",
        run_manipulapy=False,
    )
    w = Worker(job)
    with qtbot.waitSignal(w.finished, timeout=10000) as blocker:
        w.start()
    report = blocker.args[0]
    assert isinstance(report, ValidateReport)
    assert report.ast_issues == []
    assert report.urdf_written is True
    assert report.manipulapy_ok is None  # skipped
