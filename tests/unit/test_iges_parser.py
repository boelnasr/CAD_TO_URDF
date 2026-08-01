from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import trimesh

pytest.importorskip("OCP")


def _write_flat_iges(path: Path) -> None:
    """Write the weak-structure IGES case: one free shape, no components."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.IGESCAFControl import IGESCAFControl_Writer
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    body = tool.AddShape(BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape(), False)
    TDataStd_Name.Set_s(body, TCollection_ExtendedString("body"))
    writer = IGESCAFControl_Writer()
    assert writer.Transfer(doc)
    assert writer.Write(str(path))


def _write_two_free_iges(path: Path) -> None:
    """Write two top-level bodies, including one non-identity free placement."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Trsf, gp_Vec
    from OCP.IGESCAFControl import IGESCAFControl_Writer
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    first = tool.AddShape(BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(), False)
    second = tool.AddShape(BRepPrimAPI_MakeBox(20.0, 10.0, 10.0).Shape(), False)
    TDataStd_Name.Set_s(first, TCollection_ExtendedString("body"))
    TDataStd_Name.Set_s(second, TCollection_ExtendedString("body"))
    placement = gp_Trsf()
    placement.SetTranslation(gp_Vec(50.0, 0.0, 0.0))
    located_second = TDF_Label()
    assert tool.SetLocation(second, TopLoc_Location(placement), located_second)
    TDataStd_Name.Set_s(located_second, TCollection_ExtendedString("body"))

    writer = IGESCAFControl_Writer()
    assert writer.Transfer(doc)
    assert writer.Write(str(path))


def test_load_flat_iges_falls_back_to_one_mesh_bearing_root(tmp_path: Path) -> None:
    """Removing the IGES wrapper or treating a flat shape as an assembly breaks this."""
    iges = tmp_path / "body.iges"
    _write_flat_iges(iges)

    try:
        from cad2urdf.core.parsers.iges import StepAssembly, load_iges_assembly
    except ModuleNotFoundError:
        pytest.fail("the IGES parser wrapper is missing")

    assembly = load_iges_assembly(iges)

    assert isinstance(assembly, StepAssembly)
    assert assembly.root_id == "body"
    assert assembly.edges == []
    assert len(assembly.parts) == 1
    part = assembly.parts[0]
    assert part.id == "body"
    assert part.is_leaf
    assert isinstance(part.mesh, trimesh.Trimesh)
    assert part.mesh.extents == pytest.approx([0.01, 0.02, 0.03], rel=1e-3)


def test_load_iges_with_shapes_returns_transient_leaf_map(tmp_path: Path) -> None:
    """Dropping the accepted side-map interface or keying it differently breaks this."""
    iges = tmp_path / "body.igs"
    _write_flat_iges(iges)

    try:
        from cad2urdf.core.parsers.iges import load_iges_assembly_with_shapes
    except ModuleNotFoundError:
        pytest.fail("the IGES parser wrapper is missing")

    assembly, shapes = load_iges_assembly_with_shapes(iges)

    assert set(shapes) == {"body"}
    assert set(shapes) == {part.id for part in assembly.parts if part.is_leaf}


def test_load_iges_preserves_all_free_bodies_and_top_level_placements(
    tmp_path: Path,
) -> None:
    """Selecting only the first free label drops independent IGES bodies."""
    from cad2urdf.core.parsers.iges import load_iges_assembly_with_shapes

    iges = tmp_path / "two_bodies.iges"
    _write_two_free_iges(iges)
    assembly, shapes = load_iges_assembly_with_shapes(iges)

    assert assembly.root_id == "assembly_root"
    assert [(part.id, part.is_leaf) for part in assembly.parts] == [
        ("assembly_root", False),
        ("body", True),
        ("body_2", True),
    ]
    assert set(shapes) == {"body", "body_2"}
    assert {(edge.parent_id, edge.child_id) for edge in assembly.edges} == {
        ("assembly_root", "body"),
        ("assembly_root", "body_2"),
    }

    edge_for = {edge.child_id: edge for edge in assembly.edges}
    assert edge_for["body"].relative_pose == pytest.approx(np.eye(4))
    # IGES bakes this top-level placement into geometry on round-trip, so the
    # free-label edge is identity and the mesh carries the world position.
    assert edge_for["body_2"].relative_pose == pytest.approx(np.eye(4))

    part_for = {part.id: part for part in assembly.parts}
    assert part_for["body"].mesh is not None
    assert part_for["body_2"].mesh is not None
    assert part_for["body"].mesh.centroid == pytest.approx([0.005, 0.005, 0.005])
    assert part_for["body_2"].mesh.centroid == pytest.approx([0.06, 0.005, 0.005])
    world_second_centroid = (
        edge_for["body_2"].relative_pose
        @ [*part_for["body_2"].mesh.centroid, 1.0]
    )
    assert world_second_centroid[:3] == pytest.approx([0.06, 0.005, 0.005])
