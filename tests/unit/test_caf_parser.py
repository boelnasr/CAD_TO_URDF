"""Shared XCAF assembly walk behavior."""

from __future__ import annotations

import math
from dataclasses import fields, is_dataclass
from pathlib import Path
from typing import Any, Callable

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


def test_repeated_component_definitions_create_distinct_occurrences(tmp_path: Path) -> None:
    """Two placements of one XCAF definition must become two Robot candidates."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Trsf, gp_Vec
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    from cad2urdf.core.parsers.caf import load_caf_assembly

    class RepeatedReader:
        def ReadFile(self, _path: str):  # noqa: N802
            return IFSelect_RetDone

        def Transfer(self, doc):  # noqa: N802
            tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
            bolt = tool.AddShape(BRepPrimAPI_MakeBox(2.0, 2.0, 10.0).Shape(), False)
            TDataStd_Name.Set_s(bolt, TCollection_ExtendedString("bolt"))
            assembly = tool.NewShape()
            TDataStd_Name.Set_s(assembly, TCollection_ExtendedString("fixture"))
            tool.AddComponent(assembly, bolt, TopLoc_Location(gp_Trsf()))
            moved = gp_Trsf()
            moved.SetTranslation(gp_Vec(20.0, 0.0, 0.0))
            tool.AddComponent(assembly, bolt, TopLoc_Location(moved))
            tool.UpdateAssemblies()

    source = tmp_path / "repeated.step"
    source.touch()
    assembly, shapes = load_caf_assembly(path=source, reader_factory=RepeatedReader)

    leaves = [part for part in assembly.parts if part.is_leaf]
    assert [part.id for part in leaves] == ["bolt", "bolt_2"]
    assert set(shapes) == {"bolt", "bolt_2"}
    assert [edge.child_id for edge in assembly.edges] == ["bolt", "bolt_2"]
    assert assembly.edges[0].relative_pose[0, 3] == pytest.approx(0.0)
    assert assembly.edges[1].relative_pose[0, 3] == pytest.approx(0.02)


def _box_at(corner_mm: tuple[float, float, float]) -> Any:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt

    return BRepPrimAPI_MakeBox(gp_Pnt(*corner_mm), 10.0, 10.0, 10.0).Shape()


def _tube() -> Any:
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder

    return BRepAlgoAPI_Cut(
        BRepPrimAPI_MakeCylinder(10.0, 30.0).Shape(),
        BRepPrimAPI_MakeCylinder(5.0, 30.0).Shape(),
    ).Shape()


@pytest.mark.parametrize(
    ("make_shape", "true_volume_m3"),
    [
        pytest.param(lambda: _box_at((20.0, 20.0, 20.0)), 1e-6, id="box_at_+20mm"),
        pytest.param(lambda: _box_at((-40.0, -40.0, -40.0)), 1e-6, id="box_at_-40mm"),
        # Hollow cylinder r 10/5 mm, h 30 mm; the faceted mesh undershoots the
        # true curved volume slightly, so the tolerance is the mesh deflection.
        pytest.param(_tube, math.pi * (10.0**2 - 5.0**2) * 30.0 * 1e-9, id="tube_r10_r5_h30"),
    ],
)
def test_tessellate_winds_reversed_faces_outward_issue2(
    make_shape: Callable[[], Any], true_volume_m3: float
) -> None:
    """REVERSED faces must be re-wound so the signed volume is placement-independent."""
    from cad2urdf.core.parsers.caf import tessellate

    mesh = tessellate(make_shape())

    assert mesh.is_watertight
    assert mesh.is_winding_consistent
    assert mesh.volume == pytest.approx(true_volume_m3, rel=1e-2)


def test_untessellable_part_error_names_file_and_part_issue24(tmp_path: Path) -> None:
    """A sketch/wire-only part must be reported by file and part name."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt, gp_Trsf
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    from cad2urdf.core.parsers.step import load_step_assembly_with_shapes

    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    XCAFDoc_DocumentTool.Set_s(doc.Main())
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    body = tool.AddShape(BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(), False)
    sketch = tool.AddShape(
        BRepBuilderAPI_MakeEdge(gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(10.0, 0.0, 0.0)).Edge(), False
    )
    asm = tool.NewShape()
    for label, name in ((body, "body"), (sketch, "axis_sketch"), (asm, "robot")):
        TDataStd_Name.Set_s(label, TCollection_ExtendedString(name))
    for label in (body, sketch):
        tool.AddComponent(asm, label, TopLoc_Location(gp_Trsf()))
    tool.UpdateAssemblies()
    path = tmp_path / "with_sketch.step"
    writer = STEPCAFControl_Writer()
    writer.Transfer(doc)
    assert writer.Write(str(path)) == IFSelect_RetDone

    with pytest.raises(ValueError) as excinfo:
        load_step_assembly_with_shapes(path)

    message = str(excinfo.value)
    assert "with_sketch.step" in message
    assert "axis_sketch" in message
