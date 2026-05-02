"""Classify CAD assembly edges into URDF joint types.

v1-alpha behavior: every edge -> 'fixed' joint with axis=[1,0,0] (axis is
irrelevant for fixed joints anyway). Richer heuristics - concentric->revolute,
sliding->prismatic, etc. - deferred to v2.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import NDArray

from cad2urdf.core.extract.types import AssemblyEdge
from cad2urdf.core.kinematic.model import JointType


@dataclass(frozen=True)
class ClassifiedJoint:
    """An assembly edge with a derived joint type, axis, and pose.

    Output of `classify_default_fixed` (and any future heuristic classifiers).
    Consumed by the spanning-tree builder (Task 4.2) and the Robot constructor
    (Task 4.3).
    """

    parent_id: str
    child_id: str
    joint_type: JointType
    axis: NDArray[Any] = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    relative_pose: NDArray[Any] = field(default_factory=lambda: np.eye(4))


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
