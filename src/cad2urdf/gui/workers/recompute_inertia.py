"""Worker job: recompute one link's inertial values from its mesh."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from numpy.typing import NDArray

from cad2urdf.core.inertia.compute import compute_inertial
from cad2urdf.core.kinematic.model import InertialOverride

InertiaResult = tuple[float, NDArray[Any], NDArray[Any]]


def build_recompute_job(
    *,
    mesh_path: Path,
    density: float,
    override: InertialOverride,
) -> Callable[[Callable[[int, int, str], None]], InertiaResult]:
    """Return a Worker job that loads `mesh_path` and returns mass, COM, inertia."""

    def job(report: Callable[[int, int, str], None]) -> InertiaResult:
        report(0, 2, f"loading {mesh_path.name}")
        mesh = trimesh.load(
            str(mesh_path),
            file_type=mesh_path.suffix.lower().lstrip("."),
            force="mesh",
        )
        if not isinstance(mesh, trimesh.Trimesh):
            raise ValueError(f"could not load {mesh_path} as a single mesh")

        report(1, 2, "computing inertia")
        mass, com, inertia = compute_inertial(mesh, density=density, override=override)
        return float(mass), np.asarray(com, dtype=float), np.asarray(inertia, dtype=float)

    return job
