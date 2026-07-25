from __future__ import annotations

import pytest

pytest.importorskip("OCP")  # STEP support requires the [step] extra

from pathlib import Path

import trimesh

from cad2urdf.core.parsers.step import StepAssembly, load_step_assembly, tessellate


def test_tessellate_box_to_trimesh() -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox

    shape = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape()
    mesh = tessellate(shape, unit_scale=0.001)  # mm -> m
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    # 10x20x30 mm = 0.01x0.02x0.03 m -> volume 6e-6 m^3
    assert mesh.volume == pytest.approx(6e-6, rel=1e-3)
    # Per-axis extents pin the uniform mm -> m scaling, not just total volume.
    assert mesh.extents == pytest.approx([0.01, 0.02, 0.03], rel=1e-3)


def _write_named_assembly(path: Path) -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Trsf, gp_Vec
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    base = tool.AddShape(BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(), False)
    arm = tool.AddShape(BRepPrimAPI_MakeBox(5.0, 5.0, 20.0).Shape(), False)
    TDataStd_Name.Set_s(base, TCollection_ExtendedString("base"))
    TDataStd_Name.Set_s(arm, TCollection_ExtendedString("arm"))
    asm = tool.NewShape()
    TDataStd_Name.Set_s(asm, TCollection_ExtendedString("robot"))
    t1 = gp_Trsf()
    t1.SetTranslation(gp_Vec(0.0, 0.0, 10.0))
    tool.AddComponent(asm, base, TopLoc_Location(gp_Trsf()))
    tool.AddComponent(asm, arm, TopLoc_Location(t1))
    tool.UpdateAssemblies()
    w = STEPCAFControl_Writer()
    w.Transfer(doc)
    assert w.Write(str(path)) == IFSelect_RetDone


def test_load_step_assembly_extracts_names_hierarchy_poses(tmp_path: Path) -> None:
    step = tmp_path / "robot.step"
    _write_named_assembly(step)
    asm = load_step_assembly(step)

    assert isinstance(asm, StepAssembly)
    names = {p.name for p in asm.parts}
    assert {"base", "arm"} <= names
    leaves = {p.name for p in asm.parts if p.is_leaf and p.mesh is not None}
    assert leaves == {"base", "arm"}
    # arm sits +10 mm above base -> +0.01 m after unit scaling, on some edge
    arm_edges = [e for e in asm.edges if asm_id_to_name(asm, e.child_id) == "arm"]
    assert arm_edges
    assert arm_edges[0].relative_pose[2, 3] == pytest.approx(0.01, rel=1e-3)


def asm_id_to_name(asm: StepAssembly, node_id: str) -> str:
    return next(p.name for p in asm.parts if p.id == node_id)


def test_shared_caf_walk_applies_free_label_placement_once(tmp_path: Path) -> None:
    """Synthetic-root edges own free-label placement; leaf meshes stay local."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Trsf, gp_Vec
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    from cad2urdf.core.parsers.caf import load_caf_assembly

    class GeneratedReader:
        def ReadFile(self, _path: str):  # noqa: N802 - OCP reader protocol
            return IFSelect_RetDone

        def Transfer(self, doc):  # noqa: N802 - OCP reader protocol
            tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
            first = tool.AddShape(
                BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(), False
            )
            second = tool.AddShape(
                BRepPrimAPI_MakeBox(20.0, 10.0, 10.0).Shape(), False
            )
            TDataStd_Name.Set_s(first, TCollection_ExtendedString("first"))
            placement = gp_Trsf()
            placement.SetTranslation(gp_Vec(50.0, 0.0, 0.0))
            located_second = TDF_Label()
            assert tool.SetLocation(
                second, TopLoc_Location(placement), located_second
            )
            TDataStd_Name.Set_s(
                located_second, TCollection_ExtendedString("second")
            )

    source = tmp_path / "generated.step"
    source.touch()
    assembly, _shapes = load_caf_assembly(
        reader_factory=GeneratedReader,
        path=source,
    )

    edge_for = {edge.child_id: edge for edge in assembly.edges}
    part_for = {part.id: part for part in assembly.parts}
    second_mesh = part_for["second"].mesh
    assert second_mesh is not None
    assert edge_for["second"].relative_pose[:3, 3] == pytest.approx(
        [0.05, 0.0, 0.0]
    )
    assert second_mesh.centroid == pytest.approx([0.01, 0.005, 0.005])
    second_world_centroid = edge_for["second"].relative_pose @ [
        *second_mesh.centroid,
        1.0,
    ]
    assert second_world_centroid[:3] == pytest.approx([0.06, 0.005, 0.005])
