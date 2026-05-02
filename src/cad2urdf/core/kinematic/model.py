"""Robot kinematic data model: Robot, Link, Joint, InertialOverride."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray

JointType = Literal["revolute", "prismatic", "fixed", "continuous", "floating", "planar"]

_VALID_JOINT_TYPES: frozenset[str] = frozenset(
    {"revolute", "prismatic", "fixed", "continuous", "floating", "planar"}
)


@dataclass
class InertialOverride:
    """Per-link manual inertial overrides; any field set wins over auto-computed value."""

    mass: float | None = None
    com: NDArray[Any] | None = None
    inertia: NDArray[Any] | None = None

    def __post_init__(self) -> None:
        if self.com is not None and self.com.shape != (3,):
            raise ValueError(f"com must be shape (3,), got {self.com.shape}")
        if self.inertia is not None and self.inertia.shape != (3, 3):
            raise ValueError(f"inertia must be shape (3, 3), got {self.inertia.shape}")
        if self.com is not None:
            self.com = np.asarray(self.com, dtype=float)
        if self.inertia is not None:
            self.inertia = np.asarray(self.inertia, dtype=float)
        if self.mass is not None and self.mass < 0:
            raise ValueError(f"mass must be non-negative, got {self.mass}")


@dataclass
class Link:
    """A rigid body in the robot's kinematic tree."""

    name: str
    visual_mesh_path: Path
    collision_mesh_path: Path
    material_density: float
    material_name: str
    inertial_override: InertialOverride
    origin: NDArray[Any] = field(default_factory=lambda: np.eye(4))

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("name must not be empty")
        if self.origin.shape != (4, 4):
            raise ValueError(f"origin must be shape (4, 4), got {self.origin.shape}")
        if self.material_density <= 0:
            raise ValueError(f"material_density must be positive, got {self.material_density}")


@dataclass
class Joint:
    """A constraint between two links."""

    name: str
    type: JointType
    parent: str
    child: str
    axis: NDArray[Any]
    origin: NDArray[Any] = field(default_factory=lambda: np.eye(4))
    limit_lower: float | None = None
    limit_upper: float | None = None
    effort: float | None = None
    velocity: float | None = None

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("name must not be empty")
        if self.type not in _VALID_JOINT_TYPES:
            raise ValueError(f"type must be one of {sorted(_VALID_JOINT_TYPES)}, got {self.type!r}")
        if self.parent == self.child:
            raise ValueError(f"parent and child must differ; both are {self.parent!r}")
        if self.axis.shape != (3,):
            raise ValueError(f"axis must be shape (3,), got {self.axis.shape}")
        norm = float(np.linalg.norm(self.axis))
        if not np.isclose(norm, 1.0, atol=1e-6):
            raise ValueError(f"axis must be unit-length, got |a|={norm:.6f}")
        if self.origin.shape != (4, 4):
            raise ValueError(f"origin must be shape (4, 4), got {self.origin.shape}")
        if (
            self.limit_lower is not None
            and self.limit_upper is not None
            and self.limit_lower > self.limit_upper
        ):
            raise ValueError(
                f"limit_lower ({self.limit_lower}) must be <= limit_upper ({self.limit_upper})"
            )


@dataclass
class Robot:
    """Top-level robot description."""

    name: str
    base_link: str
    links: dict[str, Link]
    joints: dict[str, Joint]

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("name must not be empty")
        if self.base_link not in self.links:
            raise ValueError(f"base_link {self.base_link!r} not in links")
        for key, link in self.links.items():
            if link.name != key:
                raise ValueError(f"links dict key {key!r} does not match Link.name {link.name!r}")
        for key, joint in self.joints.items():
            if joint.name != key:
                raise ValueError(
                    f"joints dict key {key!r} does not match Joint.name {joint.name!r}"
                )
            if joint.parent not in self.links or joint.child not in self.links:
                raise ValueError(
                    f"joint {key!r} references unknown link "
                    f"(parent={joint.parent!r}, child={joint.child!r})"
                )
