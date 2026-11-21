from __future__ import annotations

from pathlib import Path

import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Link, Robot
from cad2urdf.gui.workers.export_package import build_export_job

pytestmark = pytest.mark.gui


def _frame(name: str) -> Link:
    return Link(name, None, None, None, None, InertialOverride())


def test_export_job_handles_frame_link_without_mesh(tmp_path: Path) -> None:
    """A STEP-imported robot whose base is a meshless frame link must export
    without crashing on Link.visual_mesh_path being None."""
    robot = Robot(
        name="r",
        base_link="root",
        links={"root": _frame("root")},
        joints={},
    )
    job = build_export_job(
        robot=robot,
        out_dir=tmp_path / "r_description",
        package_name="r_description",
        maintainer="m",
        maintainer_email="m@example.com",
        run_manipulapy=False,
    )
    report = job(lambda c, t, m: None)
    assert report.urdf_path.is_file()
    assert '<link name="root"' in report.urdf_path.read_text()


@pytest.mark.parametrize("unsafe", ["../../../escaped", "ABS"])
def test_export_rejects_link_name_that_escapes_package_issue9(
    tmp_path: Path, base_stl: Path, unsafe: str
) -> None:
    """Link names become mesh filenames; one must never place a file outside out_dir."""
    if unsafe == "ABS":
        # Absolute name inside tmp_path so the escape is observable without root.
        unsafe = str(tmp_path / "abs" / "x")
    out_dir = tmp_path / "ws" / "pkg"
    link = Link(unsafe, base_stl, base_stl, 2700.0, "aluminum_6061", InertialOverride())
    robot = Robot(name="r", base_link=unsafe, links={unsafe: link}, joints={})
    job = build_export_job(
        robot=robot,
        out_dir=out_dir,
        package_name="pkg",
        maintainer="m",
        maintainer_email="m@example.com",
        run_manipulapy=False,
    )
    with pytest.raises(ValueError, match="link name"):
        job(lambda c, t, m: None)
    escaped = [p for p in tmp_path.rglob("*.stl") if not p.is_relative_to(out_dir)]
    assert escaped == []
