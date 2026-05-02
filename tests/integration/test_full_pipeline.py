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


def test_partial_write_rolled_back_on_invalid_joints_yaml(tmp_path: Path) -> None:
    """If yaml is invalid AFTER mesh writes start, the partial output is cleaned up."""
    stl_dir = tmp_path / "inputs"
    stl_dir.mkdir()
    cube = stl_dir / "cube.stl"
    _write_cube_stl(cube)

    bad_yaml = tmp_path / "bad.yaml"
    bad_yaml.write_text("not_a_mapping_just_a_string\n")

    out_pkg = tmp_path / "rollback_test_pkg"
    rc = main([str(cube), "--joints", str(bad_yaml), "-o", str(out_pkg), "--no-validate"])
    assert rc == 2
    # Output directory should NOT exist (rolled back since we created it).
    assert not out_pkg.exists(), f"expected rollback but {out_pkg} still exists"


def test_partial_write_rolled_back_on_post_write_failure(tmp_path: Path) -> None:
    """If the pipeline fails AFTER mesh writes have started, the partial output
    is cleaned up (when we created the out dir).

    Triggers the post-write failure by referencing a link in the joints YAML
    that doesn't exist among the input meshes — _build_robot raises during
    Phase 2 (joint building), AFTER Phase 1 wrote meshes.
    """
    stl_dir = tmp_path / "inputs"
    stl_dir.mkdir()
    cube_a = stl_dir / "cube_a.stl"
    cube_b = stl_dir / "cube_b.stl"
    trimesh.creation.box(extents=(1, 1, 1)).export(str(cube_a), file_type="stl")
    trimesh.creation.box(extents=(1, 1, 1)).export(str(cube_b), file_type="stl")

    # joints YAML references "ghost_link" which is NOT in the input meshes —
    # Robot.__post_init__ will raise during Phase 2.
    bad_joints = tmp_path / "bad_joints.yaml"
    bad_joints.write_text(
        """
robot_name: bad
base_link: cube_a
joints:
  - name: bad_joint
    type: fixed
    parent: cube_a
    child: ghost_link
    axis: [1, 0, 0]
"""
    )

    out_pkg = tmp_path / "out_pkg_post_write_rollback"
    rc = main(
        [
            str(cube_a),
            str(cube_b),
            "--joints",
            str(bad_joints),
            "-o",
            str(out_pkg),
            "--no-validate",
        ]
    )
    assert rc == 2  # error exit
    # Output dir should be cleaned up since we created it.
    assert not out_pkg.exists(), f"expected rollback to remove {out_pkg}, but it still exists"


def test_partial_write_NOT_rolled_back_when_out_dir_pre_existed(tmp_path: Path) -> None:
    """If --out already existed before we ran, we do NOT delete it on error
    (don't nuke the user's stuff)."""
    stl_dir = tmp_path / "inputs"
    stl_dir.mkdir()
    cube = stl_dir / "cube_a.stl"
    trimesh.creation.box(extents=(1, 1, 1)).export(str(cube), file_type="stl")

    bad_joints = tmp_path / "bad.yaml"
    bad_joints.write_text(
        """
robot_name: bad
base_link: cube_a
joints:
  - name: bad
    type: fixed
    parent: cube_a
    child: ghost
    axis: [1, 0, 0]
"""
    )

    # Pre-create the output dir with a sentinel file.
    out_pkg = tmp_path / "preexisting_pkg"
    out_pkg.mkdir()
    sentinel = out_pkg / "user_file.txt"
    sentinel.write_text("important user content")

    rc = main([str(cube), "--joints", str(bad_joints), "-o", str(out_pkg), "--no-validate"])
    assert rc == 2
    # Output dir survives (we didn't create it).
    assert out_pkg.exists()
    assert sentinel.exists(), "user content was destroyed by rollback"


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
