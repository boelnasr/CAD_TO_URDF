"""Robot kinematic data model: Robot, Link, Joint, InertialOverride."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray


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
