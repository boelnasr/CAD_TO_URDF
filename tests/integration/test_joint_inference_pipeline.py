from __future__ import annotations

from dataclasses import fields, is_dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import trimesh

pytest.importorskip("OCP")

from cad2urdf.cli.main import _build_cad_robot
from cad2urdf.core.extract import build_robot as build_robot_module
from cad2urdf.core.kinematic.model import Robot
from cad2urdf.core.parsers.caf import StepAssembly, StepPart
from cad2urdf.core.urdf.emit import emit_urdf
from cad2urdf.gui.workers import step_import


def _sliding_assembly() -> tuple[StepAssembly, dict[str, Any]]:
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt

    outer = BRepPrimAPI_MakeBox(gp_Pnt(-10.0, -10.0, 0.0), 20.0, 20.0, 50.0).Shape()
    tunnel = BRepPrimAPI_MakeBox(gp_Pnt(-5.0, -4.0, -1.0), 10.0, 8.0, 52.0).Shape()
    guide = BRepAlgoAPI_Cut(outer, tunnel).Shape()
    slider = BRepPrimAPI_MakeBox(gp_Pnt(-5.0, -4.0, 0.0), 10.0, 8.0, 20.0).Shape()
    pose = np.eye(4)
    pose[2, 3] = 0.015

    from cad2urdf.core.extract.types import AssemblyEdge

    assembly = StepAssembly(
        parts=[
            StepPart(
                id="guide",
                name="guide",
                mesh=trimesh.creation.box(extents=(0.02, 0.02, 0.05)),
                is_leaf=True,
            ),
            StepPart(
                id="slider",
                name="slider",
                mesh=trimesh.creation.box(extents=(0.01, 0.008, 0.02)),
                is_leaf=True,
            ),
        ],
        edges=[
            AssemblyEdge(
                parent_id="guide",
                child_id="slider",
                relative_pose=pose,
            )
        ],
        root_id="guide",
    )
    return assembly, {"guide": guide, "slider": slider}


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


def test_cli_infers_prismatic_emits_urdf_and_disposes_shapes_before_build(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from cad2urdf.core.parsers import step

    assembly, shapes = _sliding_assembly()
    cad_path = tmp_path / "slider.step"
    cad_path.touch()
    monkeypatch.setattr(
        step,
        "load_step_assembly_with_shapes",
        lambda _path, **_kwargs: (assembly, shapes),
    )

    real_builder = build_robot_module.build_robot_from_tree

    def build_after_disposal(**kwargs: object) -> Robot:
        assert shapes == {}
        return real_builder(**kwargs)  # type: ignore[arg-type]

    monkeypatch.setattr(build_robot_module, "build_robot_from_tree", build_after_disposal)

    robot = _build_cad_robot(cad_path, tmp_path / "package", "slider_robot")

    assert shapes == {}
    assert not _contains_ocp_shape(robot)
    joint = robot.joints["slider_to_guide"]
    assert joint.type == "prismatic"
    assert joint.axis == pytest.approx([0.0, 0.0, 1.0])
    assert joint.limit_lower == pytest.approx(-0.015, abs=1e-6)
    assert joint.limit_upper == pytest.approx(0.015, abs=1e-6)

    urdf_path = tmp_path / "package" / "urdf" / "slider_robot.urdf"
    emit_urdf(robot, urdf_path, package_name="slider_description")
    text = urdf_path.read_text()
    assert 'type="prismatic"' in text
    assert '<limit lower="-0.015' in text


def test_gui_job_infers_prismatic_and_disposes_shapes_before_robot_build(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from cad2urdf.core.parsers import step

    assembly, shapes = _sliding_assembly()
    cad_path = tmp_path / "slider.step"
    cad_path.touch()
    monkeypatch.setattr(
        step,
        "load_step_assembly_with_shapes",
        lambda _path, **_kwargs: (assembly, shapes),
    )

    real_builder = build_robot_module.build_robot_from_tree

    def build_after_disposal(**kwargs: object) -> Robot:
        assert shapes == {}
        return real_builder(**kwargs)  # type: ignore[arg-type]

    monkeypatch.setattr(step_import, "build_robot_from_tree", build_after_disposal)
    job = step_import.build_cad_import_job(
        path=cad_path,
        mesh_dir=tmp_path / "meshes",
        robot_name="slider_robot",
    )

    robot = job(lambda _current, _total, _message: None)

    assert shapes == {}
    assert not _contains_ocp_shape(robot)
    assert robot.joints["slider_to_guide"].type == "prismatic"
