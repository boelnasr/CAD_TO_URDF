"""Shared XCAF assembly walk behavior."""

from __future__ import annotations

from dataclasses import fields, is_dataclass
from pathlib import Path
from typing import Any

import pytest

pytest.importorskip("OCP")

from tests.unit.test_step_parser import _write_named_assembly


def _step_reader() -> Any:
    from OCP.STEPCAFControl import STEPCAFControl_Reader

    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    return reader


def _contains_topods_shape(value: object) -> bool:
    """Return whether a parser/kinematic object graph contains an OCP B-rep."""
    from OCP.TopoDS import TopoDS_Shape

    if isinstance(value, TopoDS_Shape):
        return True
    if is_dataclass(value) and not isinstance(value, type):
        return any(_contains_topods_shape(getattr(value, field.name)) for field in fields(value))
    if isinstance(value, dict):
        return any(
            _contains_topods_shape(key) or _contains_topods_shape(item)
            for key, item in value.items()
        )
    if isinstance(value, (list, tuple, set, frozenset)):
        return any(_contains_topods_shape(item) for item in value)
    return False


def test_load_caf_assembly_returns_shapes_for_leaf_parts_only(tmp_path: Path) -> None:
    """The shared walk exposes each leaf B-rep but no assembly-frame shape."""
    from cad2urdf.core.parsers.caf import load_caf_assembly

    path = tmp_path / "robot.step"
    _write_named_assembly(path)

    assembly, shapes = load_caf_assembly(path=path, reader_factory=_step_reader)

    leaf_ids = {part.id for part in assembly.parts if part.is_leaf}
    assert {part.name for part in assembly.parts if part.is_leaf} == {"base", "arm"}
    assert set(shapes) == leaf_ids
    assert all(shape is not None for shape in shapes.values())


def test_step_wrapper_exposes_leaf_shape_side_map(tmp_path: Path) -> None:
    """The STEP convenience API retains transient leaf B-reps alongside the assembly."""
    from cad2urdf.core.parsers.step import load_step_assembly_with_shapes

    path = tmp_path / "robot.step"
    _write_named_assembly(path)

    assembly, shapes = load_step_assembly_with_shapes(path)

    assert set(shapes) == {part.id for part in assembly.parts if part.is_leaf}


def test_shape_side_map_stays_outside_robot_project_round_trip(tmp_path: Path) -> None:
    """Imported B-reps remain transient when the resulting Robot is persisted."""
    from OCP.XCAFApp import XCAFApp_Application

    from cad2urdf.core.extract.build_robot import build_robot_from_tree
    from cad2urdf.core.extract.mate_classify import classify_default_fixed
    from cad2urdf.core.extract.spanning_tree import build_spanning_tree
    from cad2urdf.core.inertia.materials import lookup
    from cad2urdf.core.parsers.caf import load_caf_assembly
    from cad2urdf.core.project.save import load_project, save_project

    step_path = tmp_path / "robot.step"
    project_path = tmp_path / "robot.cad2urdf"
    _write_named_assembly(step_path)

    app = XCAFApp_Application.GetApplication_s()
    documents_before_parse = app.NbDocuments()
    assembly, shapes = load_caf_assembly(path=step_path, reader_factory=_step_reader)
    leaf_ids = {part.id for part in assembly.parts if part.is_leaf}
    assert set(shapes) == leaf_ids
    assert app.NbDocuments() == documents_before_parse
    assert all(not shape.IsNull() for shape in shapes.values())

    tree = build_spanning_tree(classify_default_fixed(assembly.edges), base_id=assembly.root_id)
    robot = build_robot_from_tree(
        parts=assembly.parts,
        tree=tree,
        base_id=assembly.root_id,
        robot_name="robot",
        mesh_dir=tmp_path / "meshes",
        material=lookup("aluminum_6061"),
    )

    save_project(robot, project_path)
    loaded = load_project(project_path)

    assert set(loaded.links) == {"robot", "base", "arm"}
    assert not _contains_topods_shape(robot)
    assert not _contains_topods_shape(loaded)
