"""OBJ -> trimesh.Trimesh loader."""

from __future__ import annotations

from pathlib import Path

import trimesh


def load_obj(path: Path) -> trimesh.Trimesh:
    """Load an OBJ file as a single Trimesh. Raises on wrong extension or missing file."""
    if path.suffix.lower() != ".obj":
        raise ValueError(f"expected .obj file, got {path.suffix!r}")
    if not path.is_file():
        raise FileNotFoundError(f"OBJ file not found: {path}")

    mesh = trimesh.load(str(path), file_type="obj", force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"failed to load OBJ as a single Trimesh: {path}")
    return mesh
