"""End-to-end smoke: launch the GUI, import two meshes, export, check the package."""

from __future__ import annotations

import pytest

pytestmark = pytest.mark.gui


def test_import_then_export_writes_valid_package(qtbot, base_stl, arm_stl, tmp_path) -> None:
    from cad2urdf.gui.dialogs.export_package import run_export_into_dir
    from cad2urdf.gui.dialogs.import_meshes import run_import_into_controller
    from cad2urdf.gui.windows.main_window import MainWindow

    win = MainWindow()
    qtbot.addWidget(win)

    run_import_into_controller(
        controller=win.controller,
        mesh_paths=[base_stl, arm_stl],
        robot_name="smoke_arm",
    )
    out = tmp_path / "smoke_arm_description"
    report = run_export_into_dir(
        controller=win.controller,
        out_dir=out,
        package_name="smoke_arm_description",
        maintainer="t",
        maintainer_email="t@example.com",
        run_manipulapy=False,
    )
    assert (out / "package.xml").is_file()
    assert (out / "CMakeLists.txt").is_file()
    assert report.urdf_path.is_file()
    urdf_text = report.urdf_path.read_text()
    assert '<robot name="smoke_arm">' in urdf_text
    assert "package://smoke_arm_description/meshes/visual/base.stl" in urdf_text
    assert "package://smoke_arm_description/meshes/visual/arm.stl" in urdf_text
