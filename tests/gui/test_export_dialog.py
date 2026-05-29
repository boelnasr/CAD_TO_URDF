"""Export wizard: writes a ROS 2 package from the current Robot AST."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Link, Robot

pytestmark = pytest.mark.gui


def _real_robot(stl: Path) -> Robot:
    return Robot(
        name="my_arm",
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


def test_export_writes_full_ros_package(qtbot, base_stl, tmp_path) -> None:
    from cad2urdf.gui.dialogs.export_package import run_export_into_dir
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    c.replace(_real_robot(base_stl))
    out = tmp_path / "my_arm_description"

    report = run_export_into_dir(
        controller=c,
        out_dir=out,
        package_name="my_arm_description",
        maintainer="me",
        maintainer_email="me@example.com",
        run_manipulapy=False,
    )

    assert (out / "package.xml").is_file()
    assert (out / "CMakeLists.txt").is_file()
    assert (out / "urdf" / "my_arm.urdf").is_file()
    assert (out / "meshes" / "visual" / "base.stl").is_file()
    assert report.urdf_path == out / "urdf" / "my_arm.urdf"
