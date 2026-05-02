"""End-to-end CLI pipeline test: STL inputs → ROS URDF package."""

from __future__ import annotations

from pathlib import Path

import trimesh

from cad2urdf.cli.main import main


def _write_cube_stl(path: Path) -> None:
    cube = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    cube.export(str(path), file_type="stl")


def test_full_pipeline_two_stl_cubes(tmp_path: Path, fixtures_dir: Path) -> None:
    # Build a small fixture: two STL cubes
    stl_dir = tmp_path / "inputs"
    stl_dir.mkdir()
    cube_a = stl_dir / "cube_a.stl"
    cube_b = stl_dir / "cube_b.stl"
    _write_cube_stl(cube_a)
    _write_cube_stl(cube_b)

    out_pkg = tmp_path / "two_cubes_description"

    rc = main(
        [
            str(cube_a),
            str(cube_b),
            "--joints",
            str(fixtures_dir / "joints" / "two_cubes_stl.yaml"),
            "-o",
            str(out_pkg),
            "--package-name",
            "two_cubes_description",
            "--no-validate",
        ]
    )
    assert rc == 0
    assert (out_pkg / "urdf" / "two_cubes.urdf").is_file()
    assert (out_pkg / "package.xml").is_file()
    assert (out_pkg / "CMakeLists.txt").is_file()
    assert (out_pkg / "launch" / "display.launch.py").is_file()
    assert (out_pkg / "rviz" / "display.rviz").is_file()
    assert (out_pkg / "meshes" / "visual" / "cube_a.stl").is_file()
    assert (out_pkg / "meshes" / "visual" / "cube_b.stl").is_file()
    assert (out_pkg / "meshes" / "collision" / "cube_a.stl").is_file()
    assert (out_pkg / "meshes" / "collision" / "cube_b.stl").is_file()

    urdf_text = (out_pkg / "urdf" / "two_cubes.urdf").read_text()
    # Single-pass emit + pre-computed inertia means the final URDF has <inertial> blocks.
    assert urdf_text.count("<inertial>") == 2  # one per link
    assert "<mass value=" in urdf_text
    assert "<inertia ixx=" in urdf_text


def test_step_input_returns_clear_error(tmp_path: Path, fixtures_dir: Path) -> None:
    # Empty .step file — content doesn't matter, the CLI should reject by extension
    fake_step = tmp_path / "anything.step"
    fake_step.write_text("ISO-10303-21;")
    out_pkg = tmp_path / "out"
    rc = main(
        [
            str(fake_step),
            "--joints",
            str(fixtures_dir / "joints" / "two_cubes_stl.yaml"),
            "-o",
            str(out_pkg),
            "--no-validate",
        ]
    )
    assert rc == 2  # error exit code
