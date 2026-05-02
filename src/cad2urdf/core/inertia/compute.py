"""Mass / center-of-mass / 3x3 inertia tensor from a mesh + density."""

from __future__ import annotations

import logging
from typing import Any

import numpy as np
import trimesh
from numpy.typing import NDArray

from cad2urdf.core.kinematic.model import InertialOverride

log = logging.getLogger(__name__)


def compute_inertial(
    mesh: trimesh.Trimesh,
    density: float,
    override: InertialOverride,
) -> tuple[float, NDArray[Any], NDArray[Any]]:
    """Return (mass, com_3, inertia_3x3). Override fields take precedence.

    Non-watertight meshes fall back to convex-hull approximation with a logged warning.
    """
    if density <= 0:
        raise ValueError(f"density must be positive, got {density}")

    if not mesh.is_watertight:
        log.warning(
            "mesh %s is non-watertight; falling back to convex hull for inertia",
            getattr(mesh, "metadata", {}).get("file_name", "<unnamed>"),
        )
        try:
            mesh = mesh.convex_hull
        except Exception as e:  # QhullError or anything else
            log.error(
                "convex hull failed for non-watertight mesh: %s. Returning zero-mass fallback.",
                e,
            )
            zero_com = np.zeros(3)
            zero_inertia = np.zeros((3, 3))
            ov = override
            return (
                ov.mass if ov.mass is not None else 0.0,
                ov.com if ov.com is not None else zero_com,
                ov.inertia if ov.inertia is not None else zero_inertia,
            )

    mesh_copy = mesh.copy()
    mesh_copy.density = density
    auto_mass = float(mesh_copy.mass)
    auto_com: NDArray[Any] = np.asarray(mesh_copy.center_mass, dtype=float).reshape(3)
    auto_inertia: NDArray[Any] = np.asarray(mesh_copy.moment_inertia, dtype=float).reshape(3, 3)

    mass = override.mass if override.mass is not None else auto_mass
    com = override.com if override.com is not None else auto_com
    inertia = override.inertia if override.inertia is not None else auto_inertia
    return mass, com, inertia
