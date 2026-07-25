from __future__ import annotations

from pathlib import Path

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
