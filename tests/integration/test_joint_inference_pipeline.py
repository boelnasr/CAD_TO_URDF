from __future__ import annotations

from dataclasses import fields, is_dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pytest

pytest.importorskip("OCP")

from cad2urdf.cli.main import _build_cad_robot
from cad2urdf.core.extract import build_robot as build_robot_module
from cad2urdf.core.extract import joint_infer
from cad2urdf.core.extract.mate_classify import classify_from_hints
from cad2urdf.core.extract.types import AssemblyEdge
from cad2urdf.core.inertia.materials import lookup
from cad2urdf.core.kinematic.model import Robot
from cad2urdf.core.parsers.caf import StepPart
from cad2urdf.core.parsers.step import load_step_assembly_with_shapes
from cad2urdf.core.urdf.emit import emit_urdf
from cad2urdf.gui.workers import step_import


def _write_concentric_sibling_assembly(path: Path) -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Trsf, gp_Vec
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    bearing = BRepPrimAPI_MakeCylinder(5.0, 30.0).Shape()
    shaft = BRepPrimAPI_MakeCylinder(5.0, 20.0).Shape()

    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    bearing_label = tool.AddShape(bearing, False)
    shaft_label = tool.AddShape(shaft, False)
    TDataStd_Name.Set_s(bearing_label, TCollection_ExtendedString("bearing"))
    TDataStd_Name.Set_s(shaft_label, TCollection_ExtendedString("shaft"))
    assembly = tool.NewShape()
    TDataStd_Name.Set_s(assembly, TCollection_ExtendedString("robot"))
    shaft_pose = gp_Trsf()
    shaft_pose.SetTranslation(gp_Vec(0.0, 0.0, 5.0))
    tool.AddComponent(assembly, bearing_label, TopLoc_Location(gp_Trsf()))
    tool.AddComponent(assembly, shaft_label, TopLoc_Location(shaft_pose))
    tool.UpdateAssemblies()

    writer = STEPCAFControl_Writer()
    writer.Transfer(doc)
    assert writer.Write(str(path)) == IFSelect_RetDone


def test_real_caf_siblings_expand_to_one_anchor_and_one_moving_body_edge(
    tmp_path: Path,
) -> None:
    step_path = tmp_path / "bearing.step"
    _write_concentric_sibling_assembly(step_path)
    assembly, shapes = load_step_assembly_with_shapes(step_path)
    try:
        inferred = joint_infer.infer_assembly_edge_hints(assembly.edges, shapes)
    finally:
        shapes.clear()

    assert {(edge.parent_id, edge.child_id) for edge in inferred} == {
        ("robot", "bearing"),
        ("bearing", "shaft"),
    }
    classified = classify_from_hints(inferred)
    assert [edge.joint_type for edge in classified] == ["fixed", "continuous"]
    assert inferred[1].relative_pose[:3, 3] == pytest.approx([0.0, 0.0, 0.005])


def test_real_caf_sibling_inference_reaches_cli_urdf(tmp_path: Path) -> None:
    from cad2urdf.cli.main import main

    step_path = tmp_path / "bearing.step"
    out = tmp_path / "bearing_description"
    _write_concentric_sibling_assembly(step_path)

    result = main(
        [
            str(step_path),
            "-o",
            str(out),
            "--robot-name",
            "bearing_robot",
            "--package-name",
            "bearing_description",
            "--no-validate",
        ]
    )

    assert result == 0
    text = (out / "urdf" / "bearing_robot.urdf").read_text()
    assert text.count('type="fixed"') == 1
    assert text.count('type="continuous"') == 1


def test_rotated_inferred_parent_axis_emits_in_joint_coordinates(tmp_path: Path) -> None:
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    parent_axis = gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0))
    outer = BRepPrimAPI_MakeCylinder(parent_axis, 10.0, 30.0).Shape()
    bore = BRepPrimAPI_MakeCylinder(parent_axis, 5.0, 30.0).Shape()
    parent_shape = BRepAlgoAPI_Cut(outer, bore).Shape()
    child_shape = BRepPrimAPI_MakeCylinder(5.0, 20.0).Shape()
    pose = np.eye(4)
    pose[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ]
    )
    pose[0, 3] = 0.005
    edge = AssemblyEdge(parent_id="parent", child_id="child", relative_pose=pose)

    inferred = joint_infer.infer_edge_hints(
        [edge],
        {"parent": parent_shape, "child": child_shape},
    )
    classified = classify_from_hints(inferred)
    assert classified[0].axis == pytest.approx([1.0, 0.0, 0.0])
    robot = build_robot_module.build_robot_from_tree(
        parts=[
            StepPart(id="parent", name="parent", mesh=None, is_leaf=False),
            StepPart(id="child", name="child", mesh=None, is_leaf=False),
        ],
        tree=classified,
        base_id="parent",
        robot_name="rotated_axis",
        mesh_dir=tmp_path / "meshes",
        material=lookup("aluminum_6061"),
    )

    assert robot.joints["child_to_parent"].axis == pytest.approx([0.0, 0.0, 1.0])
    urdf_path = tmp_path / "rotated_axis.urdf"
    emit_urdf(robot, urdf_path, package_name="rotated_axis")
    assert '<axis xyz="0 0 1"' in urdf_path.read_text()


def _contains_ocp_shape(value: object) -> bool:
    from OCP.TopoDS import TopoDS_Shape

    if isinstance(value, TopoDS_Shape):
        return True
    if is_dataclass(value) and not isinstance(value, type):
        return any(_contains_ocp_shape(getattr(value, item.name)) for item in fields(value))
    if isinstance(value, dict):
        return any(_contains_ocp_shape(item) for item in value.values())
    if isinstance(value, (tuple, list, set, frozenset)):
        return any(_contains_ocp_shape(item) for item in value)
    return False


def test_cli_infers_moving_joint_and_disposes_shapes_before_build(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from cad2urdf.core.parsers import step

    cad_path = tmp_path / "bearing.step"
    _write_concentric_sibling_assembly(cad_path)
    captured: dict[str, dict[str, Any]] = {}
    real_loader = step.load_step_assembly_with_shapes

    def capture_shapes(path: Path, **kwargs: object) -> tuple[object, dict[str, Any]]:
        assembly, shapes = real_loader(path, **kwargs)
        captured["shapes"] = shapes
        return assembly, shapes

    monkeypatch.setattr(step, "load_step_assembly_with_shapes", capture_shapes)

    real_builder = build_robot_module.build_robot_from_tree

    def build_after_disposal(**kwargs: object) -> Robot:
        assert captured["shapes"] == {}
        return real_builder(**kwargs)  # type: ignore[arg-type]

    monkeypatch.setattr(build_robot_module, "build_robot_from_tree", build_after_disposal)

    robot = _build_cad_robot(cad_path, tmp_path / "package", "bearing_robot")

    assert captured["shapes"] == {}
    assert not _contains_ocp_shape(robot)
    joint = robot.joints["shaft_to_bearing"]
    assert joint.type == "continuous"
    assert joint.axis == pytest.approx([0.0, 0.0, 1.0])

    urdf_path = tmp_path / "package" / "urdf" / "bearing_robot.urdf"
    emit_urdf(robot, urdf_path, package_name="bearing_description")
    text = urdf_path.read_text()
    assert 'type="continuous"' in text


def test_gui_job_infers_moving_joint_and_disposes_shapes_before_robot_build(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from cad2urdf.core.parsers import step

    cad_path = tmp_path / "bearing.step"
    _write_concentric_sibling_assembly(cad_path)
    captured: dict[str, dict[str, Any]] = {}
    real_loader = step.load_step_assembly_with_shapes

    def capture_shapes(path: Path, **kwargs: object) -> tuple[object, dict[str, Any]]:
        assembly, shapes = real_loader(path, **kwargs)
        captured["shapes"] = shapes
        return assembly, shapes

    monkeypatch.setattr(step, "load_step_assembly_with_shapes", capture_shapes)

    real_builder = build_robot_module.build_robot_from_tree

    def build_after_disposal(**kwargs: object) -> Robot:
        assert captured["shapes"] == {}
        return real_builder(**kwargs)  # type: ignore[arg-type]

    monkeypatch.setattr(step_import, "build_robot_from_tree", build_after_disposal)
    job = step_import.build_cad_import_job(
        path=cad_path,
        mesh_dir=tmp_path / "meshes",
        robot_name="bearing_robot",
    )

    robot = job(lambda _current, _total, _message: None)

    assert captured["shapes"] == {}
    assert not _contains_ocp_shape(robot)
    assert robot.joints["shaft_to_bearing"].type == "continuous"
