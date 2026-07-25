"""Classify CAD assembly edges into conservative URDF joint hypotheses."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import NDArray

from cad2urdf.core.extract.types import AssemblyEdge
from cad2urdf.core.kinematic.model import JointType


@dataclass(frozen=True)
class ClassifiedJoint:
    """An assembly edge with a derived joint type, axis, and pose.

    Output of the fixed-default or hint-aware classifier. Consumed by the
    spanning-tree builder and Robot constructor.
    """

    parent_id: str
    child_id: str
    joint_type: JointType
    axis: NDArray[Any] = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    relative_pose: NDArray[Any] = field(default_factory=lambda: np.eye(4))
    limit_lower: float | None = None
    limit_upper: float | None = None


def classify_default_fixed(edges: list[AssemblyEdge]) -> list[ClassifiedJoint]:
    """v1-alpha: every edge -> 'fixed' joint."""
    return [
        ClassifiedJoint(
            parent_id=e.parent_id,
            child_id=e.child_id,
            joint_type="fixed",
            axis=np.array([1.0, 0.0, 0.0]),
            relative_pose=e.relative_pose,
        )
        for e in edges
    ]


def _normalized_axis(value: NDArray[Any] | None) -> NDArray[Any] | None:
    if value is None:
        return None
    axis = np.asarray(value, dtype=float)
    if axis.shape != (3,) or not np.all(np.isfinite(axis)):
        return None
    norm = float(np.linalg.norm(axis))
    if norm <= np.finfo(float).eps:
        return None
    return axis / norm


def _finite_bound(value: float | None) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return float(value)


def classify_from_hints(
    edges: list[AssemblyEdge],
    *,
    min_confidence: float = 0.6,
) -> list[ClassifiedJoint]:
    """Classify advisory mate hints, downgrading uncertainty to fixed joints."""
    out: list[ClassifiedJoint] = []
    for edge in edges:
        axis = _normalized_axis(edge.axis)
        joint_type: JointType = "fixed"
        limit_lower: float | None = None
        limit_upper: float | None = None

        if edge.confidence >= min_confidence and axis is not None:
            if edge.mate_kind == "concentric":
                if edge.limit_lower is None and edge.limit_upper is None:
                    joint_type = "continuous"
                else:
                    joint_type = "revolute"
                    lower = _finite_bound(edge.limit_lower)
                    upper = _finite_bound(edge.limit_upper)
                    limit_lower = -math.pi if lower is None else lower
                    limit_upper = math.pi if upper is None else upper
                    if limit_lower > limit_upper:
                        limit_lower, limit_upper = -math.pi, math.pi
            elif edge.mate_kind == "sliding":
                joint_type = "prismatic"
                lower = _finite_bound(edge.limit_lower)
                upper = _finite_bound(edge.limit_upper)
                if lower is not None and upper is not None and lower <= upper:
                    limit_lower, limit_upper = lower, upper
                else:
                    limit_lower, limit_upper = -0.1, 0.1

        out.append(
            ClassifiedJoint(
                parent_id=edge.parent_id,
                child_id=edge.child_id,
                joint_type=joint_type,
                axis=(
                    axis
                    if joint_type != "fixed" and axis is not None
                    else np.array([1.0, 0.0, 0.0])
                ),
                relative_pose=edge.relative_pose,
                limit_lower=limit_lower,
                limit_upper=limit_upper,
            )
        )
    return out
