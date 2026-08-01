from __future__ import annotations

import math

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


def test_off_origin_coaxial_cylinders_stay_fixed_and_keep_axis_point() -> None:
    """Direction-only inference would rotate around the wrong parallel line."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    shared_axis = gp_Ax2(
        gp_Pnt(0.0, 10.0, 0.0),
        gp_Dir(0.0, 0.0, 1.0),
    )
    outer = BRepPrimAPI_MakeCylinder(20.0, 30.0).Shape()
    bore = BRepPrimAPI_MakeCylinder(shared_axis, 5.0, 30.0).Shape()
    parent = BRepAlgoAPI_Cut(outer, bore).Shape()
    child = BRepPrimAPI_MakeCylinder(shared_axis, 5.0, 20.0).Shape()
    edge = _edge()

    inferred = infer_edge_hints([edge], {"parent": parent, "child": child})
    classified = classify_from_hints(inferred)

    assert inferred[0].confidence < 0.6
    assert classified[0].joint_type == "fixed"
    true_axis_point = np.array([0.0, 0.01, 0.0, 1.0])
    assert classified[0].relative_pose @ true_axis_point == pytest.approx(
        true_axis_point
    )


def test_diagonal_coaxial_cylinders_with_disjoint_axial_intervals_stay_fixed() -> None:
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

    cylinder = BRepPrimAPI_MakeCylinder(5.0, 20.0).Shape()
    rotation = gp_Trsf()
    rotation.SetRotation(
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 1.0, 0.0)),
        0.4,
    )
    parent = BRepBuilderAPI_Transform(cylinder, rotation, True).Shape()
    child = BRepBuilderAPI_Transform(cylinder, rotation, True).Shape()
    translated_axis = gp_Vec(0.0, 0.0, 1.0).Transformed(rotation)
    pose = np.eye(4)
    pose[:3, 3] = np.array(
        [translated_axis.X(), translated_axis.Y(), translated_axis.Z()]
    ) * 0.021

    inferred = infer_edge_hints([_edge(pose)], {"parent": parent, "child": child})

    assert inferred[0].mate_kind in {None, "unknown"}
    assert inferred[0].confidence < 0.6
    assert classify_from_hints(inferred)[0].joint_type == "fixed"


def test_near_tolerance_cylinder_pair_downgrades_on_worst_constraint() -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    parent = BRepPrimAPI_MakeCylinder(5.0, 20.0).Shape()
    angle = math.radians(0.9)
    child_axis = gp_Ax2(
        gp_Pnt(0.0, 0.09, 0.0),
        gp_Dir(math.sin(angle), 0.0, math.cos(angle)),
    )
    child = BRepPrimAPI_MakeCylinder(child_axis, 5.09, 20.0).Shape()

    inferred = infer_edge_hints([_edge()], {"parent": parent, "child": child})

    assert inferred[0].mate_kind == "concentric"
    assert inferred[0].confidence < 0.6
    assert classify_from_hints(inferred)[0].joint_type == "fixed"


def test_mutually_incompatible_child_cylinder_axes_stay_fixed() -> None:
    from OCP.BRep import BRep_Builder
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
    from OCP.TopoDS import TopoDS_Compound

    parent = BRepPrimAPI_MakeCylinder(5.0, 20.0).Shape()
    child = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(child)
    for y_offset_mm in (0.0, -0.101):
        axis = gp_Ax2(
            gp_Pnt(0.0, y_offset_mm, 0.0),
            gp_Dir(0.0, 0.0, 1.0),
        )
        builder.Add(child, BRepPrimAPI_MakeCylinder(axis, 5.0, 20.0).Shape())
    pose = np.eye(4)
    pose[1, 3] = 11.0e-6

    inferred = infer_edge_hints([_edge(pose)], {"parent": parent, "child": child})

    assert inferred[0].confidence < 0.6
    assert classify_from_hints(inferred)[0].joint_type == "fixed"


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


def test_two_perpendicular_one_sided_wall_contacts_stay_fixed() -> None:
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt

    x_wall = BRepPrimAPI_MakeBox(gp_Pnt(-5.0, 0.0, 0.0), 5.0, 8.0, 50.0).Shape()
    y_wall = BRepPrimAPI_MakeBox(gp_Pnt(0.0, -5.0, 0.0), 10.0, 5.0, 50.0).Shape()
    parent = BRepAlgoAPI_Fuse(x_wall, y_wall).Shape()
    child = BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, 0.0), 10.0, 8.0, 20.0).Shape()
    pose = np.eye(4)
    pose[2, 3] = 0.015

    inferred = infer_edge_hints([_edge(pose)], {"parent": parent, "child": child})

    assert inferred[0].mate_kind in {None, "unknown"}
    assert inferred[0].confidence < 0.6
    assert classify_from_hints(inferred)[0].joint_type == "fixed"


def test_disjoint_oblique_coplanar_face_bounds_stay_fixed() -> None:
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

    box = BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape()
    rotation = gp_Trsf()
    rotation.SetRotation(
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 1.0, 0.0)),
        0.4,
    )
    parent = BRepBuilderAPI_Transform(box, rotation, True).Shape()
    child = BRepBuilderAPI_Transform(box, rotation, True).Shape()
    translated_axis = gp_Vec(0.0, 0.0, 1.0).Transformed(rotation)
    pose = np.eye(4)
    pose[:3, 3] = np.array(
        [translated_axis.X(), translated_axis.Y(), translated_axis.Z()]
    ) * 0.021

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
