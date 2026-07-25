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
    bounds: _Bounds


@dataclass(frozen=True)
class _Plane:
    point: NDArray[Any]
    normal: NDArray[Any]
    bounds: _Bounds


@dataclass(frozen=True)
class _ContactDirection:
    direction: NDArray[Any]
    face_pairs: tuple[tuple[_Bounds, _Bounds], ...]


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
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    cylinders: list[_Cylinder] = []
    planes: list[_Plane] = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        adaptor = BRepAdaptor_Surface(face, True)
        bounds = _bounds_of(face, unit_scale=unit_scale)
        if adaptor.GetType() == GeomAbs_Cylinder:
            cylinder = adaptor.Cylinder()
            axis = cylinder.Axis()
            cylinders.append(
                _Cylinder(
                    point=_xyz(axis.Location()) * unit_scale,
                    direction=_unit(_xyz(axis.Direction())),
                    radius=float(cylinder.Radius()) * unit_scale,
                    bounds=bounds,
                )
            )
        elif adaptor.GetType() == GeomAbs_Plane:
            plane = adaptor.Plane()
            planes.append(
                _Plane(
                    point=_xyz(plane.Location()) * unit_scale,
                    normal=_unit(_xyz(plane.Axis().Direction())),
                    bounds=bounds,
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
                bounds=_transform_bounds(item.bounds, rotation, offset),
            )
            for item in geometry.cylinders
        ),
        planes=tuple(
            _Plane(
                point=rotation @ item.point + offset,
                normal=_unit(rotation @ item.normal),
                bounds=_transform_bounds(item.bounds, rotation, offset),
            )
            for item in geometry.planes
        ),
    )


def _parallel(first: NDArray[Any], second: NDArray[Any]) -> bool:
    cosine = float(np.clip(abs(np.dot(first, second)), -1.0, 1.0))
    return math.acos(cosine) <= ANGULAR_TOLERANCE_RAD


def _bounds_overlap(first: _Bounds, second: _Bounds) -> bool:
    return bool(
        np.all(
            np.minimum(first.high, second.high) + LINEAR_TOLERANCE_M
            >= np.maximum(first.low, second.low)
        )
    )


def _face_area_overlaps(first: _Bounds, second: _Bounds) -> bool:
    overlap = np.minimum(first.high, second.high) - np.maximum(first.low, second.low)
    return int(np.count_nonzero(overlap > LINEAR_TOLERANCE_M)) >= 2


def _canonical_direction(direction: NDArray[Any]) -> NDArray[Any]:
    out = _unit(direction)
    index = int(np.argmax(np.abs(out)))
    return -out if out[index] < 0.0 else out


def _concentric_axis(parent: _Geometry, child: _Geometry) -> tuple[NDArray[Any], float] | None:
    candidates: list[tuple[NDArray[Any], NDArray[Any], float]] = []
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
            if axis_error > LINEAR_TOLERANCE_M or not _bounds_overlap(left.bounds, right.bounds):
                continue
            closeness = (
                1.0
                - angular_error / (3.0 * ANGULAR_TOLERANCE_RAD)
                - radius_error / (3.0 * RADIUS_TOLERANCE_M)
                - axis_error / (3.0 * LINEAR_TOLERANCE_M)
            )
            candidates.append((left.point, _canonical_direction(left.direction), closeness))
    if not candidates:
        return None

    point, axis, _ = candidates[0]
    for other_point, other_axis, _ in candidates[1:]:
        same_line = (
            _parallel(axis, other_axis)
            and float(np.linalg.norm(np.cross(other_point - point, axis)))
            <= LINEAR_TOLERANCE_M
        )
        if not same_line:
            return None
    confidence = max(0.6, min(1.0, 0.8 + 0.2 * max(item[2] for item in candidates)))
    return axis, confidence


def _contact_directions(parent: _Geometry, child: _Geometry) -> list[_ContactDirection]:
    directions: list[tuple[NDArray[Any], list[tuple[_Bounds, _Bounds]]]] = []
    for left in parent.planes:
        for right in child.planes:
            if not _parallel(left.normal, right.normal):
                continue
            separation = abs(float(np.dot(right.point - left.point, left.normal)))
            if (
                separation > LINEAR_TOLERANCE_M
                or not _face_area_overlaps(left.bounds, right.bounds)
            ):
                continue
            direction = _canonical_direction(left.normal)
            for known, face_pairs in directions:
                if _parallel(direction, known):
                    face_pairs.append((left.bounds, right.bounds))
                    break
            else:
                directions.append((direction, [(left.bounds, right.bounds)]))
    return [
        _ContactDirection(direction=direction, face_pairs=tuple(face_pairs))
        for direction, face_pairs in directions
    ]


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
    child = _transform_geometry(child_local, np.asarray(edge.relative_pose, dtype=float))

    concentric = _concentric_axis(parent, child)
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
    if len(contacts) == 2 and not _parallel(contacts[0].direction, contacts[1].direction):
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
