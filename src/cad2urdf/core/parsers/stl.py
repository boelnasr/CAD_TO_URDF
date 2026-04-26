"""STL -> trimesh.Trimesh loader (thin, validates extension and existence)."""

from __future__ import annotations

from pathlib import Path

import trimesh


def load_stl(path: Path) -> trimesh.Trimesh:
    """Load an STL file as a single Trimesh. Raises on wrong extension or missing file."""
    if path.suffix.lower() != ".stl":
        raise ValueError(f"expected .stl file, got {path.suffix!r}")
    if not path.is_file():
        raise FileNotFoundError(f"STL file not found: {path}")

    mesh = trimesh.load(str(path), file_type="stl", force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"failed to load STL as a single Trimesh: {path}")
    return mesh
