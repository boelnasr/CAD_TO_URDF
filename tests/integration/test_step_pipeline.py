"""End-to-end STEP pipeline: .step assembly -> Robot -> ROS URDF package.

These tests reuse the OCP-authored fixture writer from the unit suite. They are
guarded by ``importorskip("OCP")`` so the suite stays green without the [step] extra.
"""

from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("OCP")

from cad2urdf.cli.main import main
from cad2urdf.core.extract.build_robot import build_robot_from_tree
from cad2urdf.core.extract.mate_classify import classify_default_fixed
from cad2urdf.core.extract.spanning_tree import build_spanning_tree
from cad2urdf.core.inertia.materials import lookup
from cad2urdf.core.parsers.step import load_step_assembly
from tests.unit.test_step_parser import _write_named_assembly  # reuse fixture writer


def test_step_pipeline_builds_expected_robot(tmp_path: Path) -> None:
    """The parse -> classify -> spanning-tree -> build_robot pipeline yields the
    expected link/joint structure (sub-assembly frame root + two mesh leaves)."""
    step = tmp_path / "robot.step"
    _write_named_assembly(step)
    asm = load_step_assembly(step)
    tree = build_spanning_tree(classify_default_fixed(asm.edges), base_id=asm.root_id)
    robot = build_robot_from_tree(
        parts=asm.parts,
        tree=tree,
        base_id=asm.root_id,
        robot_name="robot",
        mesh_dir=tmp_path / "meshes",
        material=lookup("aluminum_6061"),
    )
    assert set(robot.links) == {"robot", "base", "arm"}
    assert robot.base_link == "robot"
    # The sub-assembly node is a meshless frame link; the leaves carry meshes.
    assert robot.links["robot"].visual_mesh_path is None
    assert robot.links["base"].visual_mesh_path is not None
    assert robot.links["arm"].visual_mesh_path is not None
    # Two fixed joints, one per leaf.
    assert set(robot.joints) == {"base_to_robot", "arm_to_robot"}
    assert all(j.type == "fixed" for j in robot.joints.values())
    # Joint origin poses carry through from STEP (arm sits +0.01 m above the root).
    assert robot.joints["arm_to_robot"].origin[2, 3] == pytest.approx(0.01, rel=1e-3)
    assert robot.joints["base_to_robot"].origin[2, 3] == pytest.approx(0.0, abs=1e-9)


def test_step_cli_emits_ros_package(tmp_path: Path) -> None:
    """`cad2urdf robot.step -o pkg` routes through the STEP pipeline and emits a
    full ROS package whose URDF references package://-relative mesh paths."""
    step = tmp_path / "robot.step"
    _write_named_assembly(step)
    out_pkg = tmp_path / "robot_description"

    rc = main(
        [
            str(step),
            "-o",
            str(out_pkg),
            "--robot-name",
            "robot",
            "--package-name",
            "robot_description",
            "--no-validate",
        ]
    )
    assert rc == 0

    urdf_path = out_pkg / "urdf" / "robot.urdf"
    assert urdf_path.is_file()
    assert (out_pkg / "package.xml").is_file()
    assert (out_pkg / "CMakeLists.txt").is_file()
    # Per-part STLs written into the package mesh tree.
    assert (out_pkg / "meshes" / "visual" / "base.stl").is_file()
    assert (out_pkg / "meshes" / "visual" / "arm.stl").is_file()
    assert (out_pkg / "meshes" / "collision" / "base.stl").is_file()

    urdf_text = urdf_path.read_text()
    assert "<joint" in urdf_text
    assert urdf_text.count('type="fixed"') == 2
    # package://-relative mesh URIs (no absolute /tmp paths leaked into the URDF).
    assert "package://robot_description/meshes/visual/base.stl" in urdf_text
    assert "/tmp" not in urdf_text
    # The frame root link is bare (no visual block of its own).
    assert '<link name="robot"' in urdf_text


def test_step_cli_rejects_unreadable_step(tmp_path: Path) -> None:
    """A STEP file OCP cannot read exits 2 (and the freshly-created out dir is removed)."""
    bad = tmp_path / "broken.step"
    bad.write_text("ISO-10303-21;\nnot a real step file\n")
    out_pkg = tmp_path / "broken_description"
    rc = main([str(bad), "-o", str(out_pkg), "--no-validate"])
    assert rc == 2
    assert not out_pkg.exists()
