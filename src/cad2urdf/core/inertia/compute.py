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

    Only valid closed volumes are integrated. A watertight, consistently wound
    but inside-out mesh is inverted (on a copy). Non-watertight or
    inconsistently wound meshes fall back to convex-hull approximation with a
    logged warning. The input mesh is never mutated.
    """
    if density <= 0:
        raise ValueError(f"density must be positive, got {density}")

    mesh_name = getattr(mesh, "metadata", {}).get("file_name", "<unnamed>")
    hull_reason: str | None = None
    if not mesh.is_watertight:
        hull_reason = "non-watertight"
    else:
        if mesh.is_winding_consistent and mesh.volume < 0:
            # Mirrored CAD parts are often exported inside-out; integrating
            # them as-is yields negative mass and inertia.
            mesh = mesh.copy()
            mesh.invert()
        if not mesh.is_volume:
            # Flipped triangles make trimesh's signed-volume integration
            # silently wrong (e.g. half the mass with a shifted COM).
            hull_reason = "not a valid volume (inconsistent winding)"

    if hull_reason is not None:
        log.warning(
            "mesh %s is %s; falling back to convex hull for inertia",
            mesh_name,
            hull_reason,
        )
        try:
            mesh = mesh.convex_hull
        except Exception as e:  # QhullError or anything else
            log.error(
                "convex hull failed for %s mesh: %s. Returning zero-mass fallback.",
                hull_reason,
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
