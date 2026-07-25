from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("OCP")

from cad2urdf.core.extract.joint_infer import infer_edge_hints
from cad2urdf.core.extract.mate_classify import classify_from_hints
from cad2urdf.core.extract.types import AssemblyEdge


def _edge(relative_pose: np.ndarray | None = None) -> AssemblyEdge:
    return AssemblyEdge(
        parent_id="parent",
        child_id="child",
        relative_pose=np.eye(4) if relative_pose is None else relative_pose,
    )


def test_real_coaxial_cylindrical_faces_infer_parent_frame_axis() -> None:
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    parent_axis = gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0))
    outer = BRepPrimAPI_MakeCylinder(parent_axis, 10.0, 30.0).Shape()
    bore = BRepPrimAPI_MakeCylinder(parent_axis, 5.0, 30.0).Shape()
    parent = BRepAlgoAPI_Cut(outer, bore).Shape()
    child = BRepPrimAPI_MakeCylinder(5.0, 20.0).Shape()

    # OCP geometry is millimetres. The child pose is metres and rotates its
    # local +Z cylinder axis onto parent-frame +X before shifting it +5 mm.
    pose = np.eye(4)
    pose[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ]
    )
    pose[0, 3] = 0.005

    inferred = infer_edge_hints([_edge(pose)], {"parent": parent, "child": child})

    assert inferred[0] is not None
    assert inferred[0].mate_kind == "concentric"
    assert inferred[0].confidence >= 0.6
    assert np.abs(inferred[0].axis) == pytest.approx([1.0, 0.0, 0.0], abs=1e-7)
    classified = classify_from_hints(inferred)
    assert classified[0].joint_type == "continuous"
    assert classified[0].limit_lower is None
    assert classified[0].limit_upper is None


def test_real_rectangular_guide_infers_only_free_translation_and_travel() -> None:
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt

    outer = BRepPrimAPI_MakeBox(gp_Pnt(-10.0, -10.0, 0.0), 20.0, 20.0, 50.0).Shape()
    tunnel = BRepPrimAPI_MakeBox(gp_Pnt(-5.0, -4.0, -1.0), 10.0, 8.0, 52.0).Shape()
    parent = BRepAlgoAPI_Cut(outer, tunnel).Shape()
    child = BRepPrimAPI_MakeBox(gp_Pnt(-5.0, -4.0, 0.0), 10.0, 8.0, 20.0).Shape()
    pose = np.eye(4)
    pose[2, 3] = 0.015

    inferred = infer_edge_hints([_edge(pose)], {"parent": parent, "child": child})

    assert inferred[0].mate_kind == "sliding"
    assert inferred[0].confidence >= 0.6
    assert np.abs(inferred[0].axis) == pytest.approx([0.0, 0.0, 1.0], abs=1e-7)
    assert inferred[0].limit_lower == pytest.approx(-0.015, abs=1e-6)
    assert inferred[0].limit_upper == pytest.approx(0.015, abs=1e-6)
    classified = classify_from_hints(inferred)
    assert classified[0].joint_type == "prismatic"
    assert classified[0].limit_lower == pytest.approx(-0.015, abs=1e-6)
    assert classified[0].limit_upper == pytest.approx(0.015, abs=1e-6)


def test_single_planar_contact_is_ambiguous_and_stays_fixed() -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox

    parent = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape()
    child = BRepPrimAPI_MakeBox(5.0, 5.0, 5.0).Shape()
    pose = np.eye(4)
    pose[2, 3] = 0.01

    inferred = infer_edge_hints([_edge(pose)], {"parent": parent, "child": child})

    assert inferred[0].mate_kind in {None, "unknown"}
    assert inferred[0].confidence < 0.6
    assert classify_from_hints(inferred)[0].joint_type == "fixed"


def test_two_support_planes_meeting_only_along_an_edge_stay_fixed() -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox

    parent = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape()
    child = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape()
    pose = np.eye(4)
    pose[0, 3] = 0.01
    pose[1, 3] = 0.01
    pose[2, 3] = 0.002

    inferred = infer_edge_hints([_edge(pose)], {"parent": parent, "child": child})

    assert inferred[0].mate_kind in {None, "unknown"}
    assert inferred[0].confidence < 0.6
    assert classify_from_hints(inferred)[0].joint_type == "fixed"


@pytest.mark.parametrize(
    "shapes",
    [
        {},
        {"parent": object(), "child": object()},
    ],
)
def test_missing_or_invalid_shapes_never_raise_and_stay_fixed(shapes: dict[str, object]) -> None:
    inferred = infer_edge_hints([_edge()], shapes)

    assert inferred[0].mate_kind in {None, "unknown"}
    assert inferred[0].confidence < 0.6
    assert classify_from_hints(inferred)[0].joint_type == "fixed"
