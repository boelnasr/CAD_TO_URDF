"""Advisory B-rep joint inference with conservative, failure-safe fallbacks.

OCP geometry is normally expressed in millimetres while ``AssemblyEdge`` poses
are expressed in metres. Probe coordinates and radii are converted to metres
before the child pose is applied, so every comparison happens in the edge's
parent frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from itertools import product
from typing import Any

import numpy as np
from numpy.typing import NDArray

from cad2urdf.core.extract.types import AssemblyEdge

ANGULAR_TOLERANCE_RAD = math.radians(1.0)
"""Maximum angular mismatch, in radians, for parallel/coaxial directions."""

LINEAR_TOLERANCE_M = 1.0e-4
"""Maximum axis or plane separation, in metres, for coincident geometry."""

RADIUS_TOLERANCE_M = 1.0e-4
"""Maximum cylindrical-radius mismatch, in metres, for concentric geometry."""

OCP_UNIT_SCALE_M = 0.001
"""Default conversion from OCP source millimetres to metres."""


@dataclass(frozen=True)
class _Bounds:
    low: NDArray[Any]
    high: NDArray[Any]


@dataclass(frozen=True)
class _Cylinder:
    point: NDArray[Any]
    direction: NDArray[Any]
    radius: float
    axial_low: float
    axial_high: float


@dataclass(frozen=True)
class _Plane:
    point: NDArray[Any]
    normal: NDArray[Any]
    bounds: _Bounds
    vertices: NDArray[Any]
    has_four_linear_edges: bool


@dataclass(frozen=True)
class _ContactDirection:
    direction: NDArray[Any]
    face_pairs: tuple[tuple[_Bounds, _Bounds], ...]
    plane_offsets: tuple[float, ...]


@dataclass(frozen=True)
class _AxisLine:
    point: NDArray[Any]
    direction: NDArray[Any]


@dataclass(frozen=True)
class _ConcentricCandidate:
    parent_axis: _AxisLine
    child_axis: _AxisLine
    confidence: float


@dataclass(frozen=True)
class _Geometry:
    cylinders: tuple[_Cylinder, ...]
    planes: tuple[_Plane, ...]


def _xyz(value: Any) -> NDArray[Any]:
    return np.array([value.X(), value.Y(), value.Z()], dtype=float)


def _unit(vector: NDArray[Any]) -> NDArray[Any]:
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= np.finfo(float).eps:
        raise ValueError("invalid zero-length geometry direction")
    return np.asarray(vector, dtype=float) / norm


def _bounds_of(shape: Any, *, unit_scale: float) -> _Bounds:
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    if box.IsVoid():
        raise ValueError("shape has no finite bounds")
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return _Bounds(
        low=np.array([xmin, ymin, zmin], dtype=float) * unit_scale,
        high=np.array([xmax, ymax, zmax], dtype=float) * unit_scale,
    )


def _extract_geometry(shape: Any, *, unit_scale: float) -> _Geometry:
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Line, GeomAbs_Plane
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_VERTEX
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    cylinders: list[_Cylinder] = []
    planes: list[_Plane] = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        adaptor = BRepAdaptor_Surface(face, True)
        if adaptor.GetType() == GeomAbs_Cylinder:
            cylinder = adaptor.Cylinder()
            axis = cylinder.Axis()
            cylinders.append(
                _Cylinder(
                    point=_xyz(axis.Location()) * unit_scale,
                    direction=_unit(_xyz(axis.Direction())),
                    radius=float(cylinder.Radius()) * unit_scale,
                    axial_low=float(adaptor.FirstVParameter()) * unit_scale,
                    axial_high=float(adaptor.LastVParameter()) * unit_scale,
                )
            )
        elif adaptor.GetType() == GeomAbs_Plane:
            plane = adaptor.Plane()
            bounds = _bounds_of(face, unit_scale=unit_scale)
            vertices: list[NDArray[Any]] = []
            vertex_explorer = TopExp_Explorer(face, TopAbs_VERTEX)
            while vertex_explorer.More():
                vertex = TopoDS.Vertex_s(vertex_explorer.Current())
                point = _xyz(BRep_Tool.Pnt_s(vertex)) * unit_scale
                if not any(
                    np.allclose(point, known, atol=LINEAR_TOLERANCE_M * 0.1)
                    for known in vertices
                ):
                    vertices.append(point)
                vertex_explorer.Next()
            edge_count = 0
            all_edges_linear = True
            edge_explorer = TopExp_Explorer(face, TopAbs_EDGE)
            while edge_explorer.More():
                edge_count += 1
                curve = BRepAdaptor_Curve(TopoDS.Edge_s(edge_explorer.Current()))
                all_edges_linear = all_edges_linear and curve.GetType() == GeomAbs_Line
                edge_explorer.Next()
            planes.append(
                _Plane(
                    point=_xyz(plane.Location()) * unit_scale,
                    normal=_unit(_xyz(plane.Axis().Direction())),
                    bounds=bounds,
                    vertices=np.asarray(vertices, dtype=float),
                    has_four_linear_edges=all_edges_linear and edge_count == 4,
                )
            )
        explorer.Next()
    return _Geometry(
        cylinders=tuple(cylinders),
        planes=tuple(planes),
    )


def _corners(bounds: _Bounds) -> NDArray[Any]:
    return np.array(
        [
            [x, y, z]
            for x, y, z in product(
                (bounds.low[0], bounds.high[0]),
                (bounds.low[1], bounds.high[1]),
                (bounds.low[2], bounds.high[2]),
            )
        ],
        dtype=float,
    )


def _transform_bounds(bounds: _Bounds, rotation: NDArray[Any], offset: NDArray[Any]) -> _Bounds:
    points = (rotation @ _corners(bounds).T).T + offset
    return _Bounds(low=np.min(points, axis=0), high=np.max(points, axis=0))


def _transform_geometry(geometry: _Geometry, pose: NDArray[Any]) -> _Geometry:
    if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
        raise ValueError("relative pose must be a finite 4x4 matrix")
    rotation = pose[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-6):
        raise ValueError("relative pose rotation must be orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-6):
        raise ValueError("relative pose rotation must be proper")
    offset = pose[:3, 3]
    return _Geometry(
        cylinders=tuple(
            _Cylinder(
                point=rotation @ item.point + offset,
                direction=_unit(rotation @ item.direction),
                radius=item.radius,
                axial_low=item.axial_low,
                axial_high=item.axial_high,
            )
            for item in geometry.cylinders
        ),
        planes=tuple(
            _Plane(
                point=rotation @ item.point + offset,
                normal=_unit(rotation @ item.normal),
                bounds=_transform_bounds(item.bounds, rotation, offset),
                vertices=(rotation @ item.vertices.T).T + offset,
                has_four_linear_edges=item.has_four_linear_edges,
            )
            for item in geometry.planes
        ),
    )


def _parallel(first: NDArray[Any], second: NDArray[Any]) -> bool:
    cosine = float(np.clip(abs(np.dot(first, second)), -1.0, 1.0))
    return math.acos(cosine) <= ANGULAR_TOLERANCE_RAD


def _axis_aligned_rectangle(plane: _Plane) -> bool:
    if not plane.has_four_linear_edges or plane.vertices.shape != (4, 3):
        return False
    normal_axis = int(np.argmax(np.abs(plane.normal)))
    if abs(float(plane.normal[normal_axis])) < math.cos(ANGULAR_TOLERANCE_RAD):
        return False
    tangent_axes = [axis for axis in range(3) if axis != normal_axis]
    corner_pairs: set[tuple[int, int]] = set()
    for vertex in plane.vertices:
        pair: list[int] = []
        for axis in tangent_axes:
            low_error = abs(float(vertex[axis] - plane.bounds.low[axis]))
            high_error = abs(float(vertex[axis] - plane.bounds.high[axis]))
            if min(low_error, high_error) > LINEAR_TOLERANCE_M:
                return False
            pair.append(0 if low_error <= high_error else 1)
        corner_pairs.add((pair[0], pair[1]))
    return len(corner_pairs) == 4


def _verified_rectangular_overlap(first: _Plane, second: _Plane) -> bool:
    if not _axis_aligned_rectangle(first) or not _axis_aligned_rectangle(second):
        return False
    normal_axis = int(np.argmax(np.abs(first.normal)))
    if normal_axis != int(np.argmax(np.abs(second.normal))):
        return False
    overlap = np.minimum(first.bounds.high, second.bounds.high) - np.maximum(
        first.bounds.low, second.bounds.low
    )
    return all(
        overlap[axis] > LINEAR_TOLERANCE_M
        for axis in range(3)
        if axis != normal_axis
    )


def _canonical_direction(direction: NDArray[Any]) -> NDArray[Any]:
    out = _unit(direction)
    index = int(np.argmax(np.abs(out)))
    return -out if out[index] < 0.0 else out


def _cylinder_interval(
    cylinder: _Cylinder,
    reference_axis: NDArray[Any],
) -> tuple[float, float]:
    endpoints = (
        cylinder.point + cylinder.direction * cylinder.axial_low,
        cylinder.point + cylinder.direction * cylinder.axial_high,
    )
    projected = [float(np.dot(point, reference_axis)) for point in endpoints]
    return min(projected), max(projected)


def _axis_lines_compatible(first: _AxisLine, second: _AxisLine) -> bool:
    if not _parallel(first.direction, second.direction):
        return False
    cross_directions = np.cross(first.direction, second.direction)
    cross_norm = float(np.linalg.norm(cross_directions))
    delta = second.point - first.point
    if cross_norm > np.finfo(float).eps:
        line_distance = abs(float(np.dot(delta, cross_directions))) / cross_norm
    else:
        line_distance = float(np.linalg.norm(np.cross(delta, first.direction)))
    return line_distance <= LINEAR_TOLERANCE_M


def _point_line_distance(point: NDArray[Any], line: _AxisLine) -> float:
    return float(np.linalg.norm(np.cross(point - line.point, line.direction)))


def _concentric_axis(
    parent: _Geometry,
    child: _Geometry,
    *,
    parent_origin: NDArray[Any],
    child_origin: NDArray[Any],
) -> tuple[NDArray[Any], float] | None:
    candidates: list[_ConcentricCandidate] = []
    for left in parent.cylinders:
        for right in child.cylinders:
            if not _parallel(left.direction, right.direction):
                continue
            angular_error = math.acos(
                float(np.clip(abs(np.dot(left.direction, right.direction)), -1.0, 1.0))
            )
            radius_error = abs(left.radius - right.radius)
            if radius_error > RADIUS_TOLERANCE_M:
                continue
            axis_error = float(np.linalg.norm(np.cross(right.point - left.point, left.direction)))
            if axis_error > LINEAR_TOLERANCE_M:
                continue
            axis = _canonical_direction(left.direction)
            parent_axis = _AxisLine(point=left.point, direction=axis)
            child_axis = _AxisLine(
                point=right.point,
                direction=_canonical_direction(right.direction),
            )
            left_low, left_high = _cylinder_interval(left, axis)
            right_low, right_high = _cylinder_interval(right, axis)
            axial_overlap = min(left_high, right_high) - max(left_low, right_low)
            shorter_span = min(left_high - left_low, right_high - right_low)
            if axial_overlap <= LINEAR_TOLERANCE_M or shorter_span <= LINEAR_TOLERANCE_M:
                continue
            constraint_score = min(
                max(0.0, 1.0 - angular_error / ANGULAR_TOLERANCE_RAD),
                max(0.0, 1.0 - radius_error / RADIUS_TOLERANCE_M),
                max(0.0, 1.0 - axis_error / LINEAR_TOLERANCE_M),
                min(1.0, axial_overlap / shorter_span),
            )
            candidates.append(
                _ConcentricCandidate(
                    parent_axis=parent_axis,
                    child_axis=child_axis,
                    confidence=constraint_score,
                )
            )
    if not candidates:
        return None

    axis_lines = [
        axis_line
        for candidate in candidates
        for axis_line in (candidate.parent_axis, candidate.child_axis)
    ]
    for index, axis_line in enumerate(axis_lines):
        if not all(
            _axis_lines_compatible(axis_line, other)
            for other in axis_lines[index + 1 :]
        ):
            return None

    # Assembly edges currently carry only a direction, not an axis-line point.
    # Keep all candidates through the ambiguity check above, then retain only
    # lines safe under either forest orientation. Filtering earlier could hide
    # a conflicting off-origin cylinder and incorrectly bless another line.
    safe_candidates = [
        candidate
        for candidate in candidates
        if _point_line_distance(parent_origin, candidate.parent_axis)
        <= LINEAR_TOLERANCE_M
        and _point_line_distance(child_origin, candidate.child_axis)
        <= LINEAR_TOLERANCE_M
    ]
    if not safe_candidates:
        return None
    best = max(safe_candidates, key=lambda candidate: candidate.confidence)
    return best.parent_axis.direction, best.confidence


def _contact_directions(parent: _Geometry, child: _Geometry) -> list[_ContactDirection]:
    directions: list[
        tuple[NDArray[Any], list[tuple[_Bounds, _Bounds]], list[float]]
    ] = []
    for left in parent.planes:
        for right in child.planes:
            if not _parallel(left.normal, right.normal):
                continue
            separation = abs(float(np.dot(right.point - left.point, left.normal)))
            if (
                separation > LINEAR_TOLERANCE_M
                or not _verified_rectangular_overlap(left, right)
            ):
                continue
            direction = _canonical_direction(left.normal)
            offset = float(np.dot(left.point, direction))
            for known, face_pairs, offsets in directions:
                if _parallel(direction, known):
                    face_pairs.append((left.bounds, right.bounds))
                    offsets.append(offset)
                    break
            else:
                directions.append(
                    (direction, [(left.bounds, right.bounds)], [offset])
                )
    return [
        _ContactDirection(
            direction=direction,
            face_pairs=tuple(face_pairs),
            plane_offsets=tuple(offsets),
        )
        for direction, face_pairs, offsets in directions
    ]


def _bilaterally_confined(contact: _ContactDirection) -> bool:
    return max(contact.plane_offsets) - min(contact.plane_offsets) > LINEAR_TOLERANCE_M


def _travel_limits(
    contacts: list[_ContactDirection],
    axis: NDArray[Any],
) -> tuple[float | None, float | None]:
    # Axis-aligned bounding projections are exact. For an oblique axis an AABB
    # overestimates travel, so leave the classifier to use its safe defaults.
    if float(np.max(np.abs(axis))) < math.cos(ANGULAR_TOLERANCE_RAD):
        return None, None
    lower_candidates: list[float] = []
    upper_candidates: list[float] = []
    for contact in contacts:
        for parent_face, child_face in contact.face_pairs:
            parent_projection = _corners(parent_face) @ axis
            child_projection = _corners(child_face) @ axis
            parent_low = float(np.min(parent_projection))
            parent_high = float(np.max(parent_projection))
            child_low = float(np.min(child_projection))
            child_high = float(np.max(child_projection))
            if child_high - child_low > parent_high - parent_low + LINEAR_TOLERANCE_M:
                return None, None
            lower_candidates.append(parent_low - child_low)
            upper_candidates.append(parent_high - child_high)
    if not lower_candidates:
        return None, None
    lower = max(lower_candidates)
    upper = min(upper_candidates)
    if lower > LINEAR_TOLERANCE_M or upper < -LINEAR_TOLERANCE_M:
        return None, None
    return lower, upper


def _infer_pair(
    edge: AssemblyEdge,
    parent_shape: Any,
    child_shape: Any,
    *,
    unit_scale: float,
) -> AssemblyEdge:
    parent = _extract_geometry(parent_shape, unit_scale=unit_scale)
    child_local = _extract_geometry(child_shape, unit_scale=unit_scale)
    pose = np.asarray(edge.relative_pose, dtype=float)
    child = _transform_geometry(child_local, pose)
    concentric = _concentric_axis(
        parent,
        child,
        parent_origin=np.zeros(3),
        child_origin=pose[:3, 3],
    )
    if concentric is not None:
        axis, confidence = concentric
        return replace(
            edge,
            mate_kind="concentric",
            confidence=confidence,
            axis=axis,
            limit_lower=None,
            limit_upper=None,
        )

    contacts = _contact_directions(parent, child)
    if (
        len(contacts) == 2
        and all(_bilaterally_confined(contact) for contact in contacts)
        and not _parallel(contacts[0].direction, contacts[1].direction)
    ):
        axis = _canonical_direction(np.cross(contacts[0].direction, contacts[1].direction))
        lower, upper = _travel_limits(contacts, axis)
        return replace(
            edge,
            mate_kind="sliding",
            confidence=0.85,
            axis=axis,
            limit_lower=lower,
            limit_upper=upper,
        )
    return replace(
        edge,
        mate_kind=None,
        confidence=0.0,
        axis=None,
        limit_lower=None,
        limit_upper=None,
    )


def infer_edge_hints(
    edges: list[AssemblyEdge],
    shapes: dict[str, Any],
    *,
    unit_scale: float = OCP_UNIT_SCALE_M,
) -> list[AssemblyEdge]:
    """Return immutable edge copies populated with conservative B-rep mate hints.

    Missing shapes, invalid geometry, unavailable OCP, and all probe errors are
    intentionally reduced to low-confidence unknown hints. Inference must never
    make CAD import fail.
    """
    inferred: list[AssemblyEdge] = []
    for edge in edges:
        parent_shape = shapes.get(edge.parent_id)
        child_shape = shapes.get(edge.child_id)
        if parent_shape is None or child_shape is None:
            inferred.append(
                replace(
                    edge,
                    mate_kind=None,
                    confidence=0.0,
                    axis=None,
                    limit_lower=None,
                    limit_upper=None,
                )
            )
            continue
        try:
            inferred.append(
                _infer_pair(
                    edge,
                    parent_shape,
                    child_shape,
                    unit_scale=unit_scale,
                )
            )
        except Exception:
            inferred.append(
                replace(
                    edge,
                    mate_kind=None,
                    confidence=0.0,
                    axis=None,
                    limit_lower=None,
                    limit_upper=None,
                )
            )
    return inferred


def _valid_moving_hint(edge: AssemblyEdge, *, min_confidence: float) -> bool:
    if edge.mate_kind not in {"concentric", "sliding"} or edge.confidence < min_confidence:
        return False
    if edge.axis is None:
        return False
    axis = np.asarray(edge.axis, dtype=float)
    return bool(
        axis.shape == (3,)
        and np.all(np.isfinite(axis))
        and float(np.linalg.norm(axis)) > np.finfo(float).eps
    )


def infer_assembly_edge_hints(
    edges: list[AssemblyEdge],
    shapes: dict[str, Any],
    *,
    unit_scale: float = OCP_UNIT_SCALE_M,
    min_confidence: float = 0.6,
) -> list[AssemblyEdge]:
    """Infer explicit body edges and expand shaped siblings into a rooted body forest.

    Real XCAF topology connects a meshless assembly frame to each shaped child.
    For siblings under the same frame, this function probes leaf-to-leaf
    candidates in the first leaf's frame, retains one original frame anchor per
    accepted moving component, and replaces the redundant frame edges with a
    deterministic maximum-confidence inferred spanning tree.
    """
    inferred_originals = infer_edge_hints(edges, shapes, unit_scale=unit_scale)
    incoming_count: dict[str, int] = {}
    siblings_by_parent: dict[str, list[AssemblyEdge]] = {}
    for edge in edges:
        incoming_count[edge.child_id] = incoming_count.get(edge.child_id, 0) + 1
        if edge.child_id in shapes:
            siblings_by_parent.setdefault(edge.parent_id, []).append(edge)

    removed_anchors: set[tuple[str, str]] = set()
    selected_body_edges: list[AssemblyEdge] = []
    for frame_id in sorted(siblings_by_parent):
        siblings = sorted(siblings_by_parent[frame_id], key=lambda edge: edge.child_id)
        siblings = [edge for edge in siblings if incoming_count[edge.child_id] == 1]
        candidates: list[AssemblyEdge] = []
        for index, first in enumerate(siblings):
            for second in siblings[index + 1 :]:
                try:
                    relative_pose = np.linalg.inv(first.relative_pose) @ second.relative_pose
                except (TypeError, ValueError, np.linalg.LinAlgError):
                    continue
                candidates.append(
                    AssemblyEdge(
                        parent_id=first.child_id,
                        child_id=second.child_id,
                        relative_pose=relative_pose,
                    )
                )
        accepted = [
            edge
            for edge in infer_edge_hints(candidates, shapes, unit_scale=unit_scale)
            if _valid_moving_hint(edge, min_confidence=min_confidence)
        ]

        parent_for: dict[str, str] = {}

        def find(node: str) -> str:
            parent_for.setdefault(node, node)
            while parent_for[node] != node:
                parent_for[node] = parent_for[parent_for[node]]
                node = parent_for[node]
            return node

        def union(first: str, second: str) -> bool:
            first_root, second_root = find(first), find(second)
            if first_root == second_root:
                return False
            parent_for[max(first_root, second_root)] = min(first_root, second_root)
            return True

        selected: list[AssemblyEdge] = []
        for edge in sorted(
            accepted,
            key=lambda item: (-item.confidence, item.parent_id, item.child_id),
        ):
            if union(edge.parent_id, edge.child_id):
                selected.append(edge)

        components: dict[str, set[str]] = {}
        for edge in selected:
            root = find(edge.parent_id)
            components.setdefault(root, set()).update((edge.parent_id, edge.child_id))
        for members in components.values():
            anchor = min(members)
            removed_anchors.update(
                (frame_id, member) for member in members if member != anchor
            )
        selected_body_edges.extend(selected)

    kept = [
        edge
        for edge in inferred_originals
        if (edge.parent_id, edge.child_id) not in removed_anchors
    ]
    return kept + sorted(
        selected_body_edges,
        key=lambda edge: (edge.parent_id, edge.child_id),
    )
