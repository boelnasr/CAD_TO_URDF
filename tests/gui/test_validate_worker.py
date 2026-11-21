"""Validate worker: bundles AST + URDF validation into one off-thread call."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Joint, Link, Robot

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


def _mesh_link(name: str, stl: Path) -> Link:
    return Link(
        name=name,
        visual_mesh_path=stl,
        collision_mesh_path=stl,
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
    )


def _fixed(parent: str, child: str) -> Joint:
    return Joint(
        name=f"{parent}_to_{child}",
        type="fixed",
        parent=parent,
        child=child,
        axis=np.array([1.0, 0.0, 0.0]),
    )


def test_validate_passes_frame_links_through_issue4(base_stl, tmp_path) -> None:
    """STEP/IGES imports always root the tree at a meshless frame link."""
    from cad2urdf.gui.workers.validate import build_validate_job

    robot = Robot(
        name="r",
        base_link="asm",
        links={
            "asm": Link("asm", None, None, None, None, InertialOverride()),
            "arm": _mesh_link("arm", base_stl),
        },
        joints={"asm_to_arm": _fixed("asm", "arm")},
    )
    job = build_validate_job(
        robot=robot,
        out_dir=tmp_path / "pkg",
        package_name="r_pkg",
        urdf_relname="r.urdf",
        run_manipulapy=False,
    )
    report = job(lambda c, t, m: None)
    assert report.urdf_written is True
    assert '<link name="asm"' in report.urdf_path.read_text()


def _inertial_masses(urdf: Path) -> dict[str, float]:
    root = ET.parse(urdf).getroot()
    return {
        link.get("name"): float(link.find("inertial/mass").get("value"))
        for link in root.iter("link")
        if link.find("inertial") is not None
    }


def test_validate_bakes_same_inertia_as_export_issue16(base_stl, arm_stl, tmp_path) -> None:
    """The validated URDF must be the one export writes, not a massless variant."""
    from cad2urdf.gui.workers.export_package import build_export_job
    from cad2urdf.gui.workers.validate import build_validate_job

    robot = Robot(
        name="r",
        base_link="base",
        links={"base": _mesh_link("base", base_stl), "arm": _mesh_link("arm", arm_stl)},
        joints={"base_to_arm": _fixed("base", "arm")},
    )
    validated = build_validate_job(
        robot=robot,
        out_dir=tmp_path / "val",
        package_name="r_pkg",
        urdf_relname="r.urdf",
        run_manipulapy=False,
    )(lambda c, t, m: None)
    exported = build_export_job(
        robot=robot,
        out_dir=tmp_path / "exp",
        package_name="r_pkg",
        maintainer="m",
        maintainer_email="m@example.com",
        run_manipulapy=False,
    )(lambda c, t, m: None)

    val_masses = _inertial_masses(validated.urdf_path)
    assert set(val_masses) == {"base", "arm"}
    assert val_masses == pytest.approx(_inertial_masses(exported.urdf_path))
