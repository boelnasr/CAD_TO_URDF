"""End-to-end IGES pipeline: a flat .iges shape becomes a ROS package."""

from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("OCP")

from cad2urdf.cli.main import main
from tests.unit.test_iges_parser import _write_flat_iges


@pytest.mark.parametrize("suffix", [".iges", ".igs"])
def test_iges_cli_emits_ros_package_without_joints_yaml(
    tmp_path: Path, suffix: str
) -> None:
    """Routing either IGES suffix through the mesh/YAML path breaks this."""
    iges = tmp_path / f"body{suffix}"
    _write_flat_iges(iges)
    out_pkg = tmp_path / f"body_{suffix[1:]}_description"

    rc = main(
        [
            str(iges),
            "-o",
            str(out_pkg),
            "--robot-name",
            "body_robot",
            "--package-name",
            "body_description",
            "--no-validate",
        ]
    )

    assert rc == 0
    urdf = out_pkg / "urdf" / "body_robot.urdf"
    assert urdf.is_file()
    assert (out_pkg / "package.xml").is_file()
    assert (out_pkg / "CMakeLists.txt").is_file()
    assert (out_pkg / "meshes" / "visual" / "body.stl").is_file()
    text = urdf.read_text()
    assert '<link name="body"' in text
    assert "package://body_description/meshes/visual/body.stl" in text


def test_cli_rejects_mixing_iges_with_other_inputs(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """Allowing a CAD assembly into the many-mesh route violates the document boundary."""
    iges = tmp_path / "body.iges"
    other = tmp_path / "other.stl"
    joints = tmp_path / "joints.yaml"
    _write_flat_iges(iges)
    other.write_bytes(b"not needed: routing must reject before loading")
    joints.write_text("robot_name: r\nbase_link: other\njoints: []\n")

    rc = main(
        [
            str(iges),
            str(other),
            "--joints",
            str(joints),
            "-o",
            str(tmp_path / "out"),
            "--no-validate",
        ]
    )

    assert rc == 2
    assert "single input" in capsys.readouterr().err
