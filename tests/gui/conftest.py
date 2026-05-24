"""Shared GUI test fixtures. Forces Qt to use the offscreen platform plugin."""

from __future__ import annotations

import os
from pathlib import Path

import pytest
import trimesh

# Must be set BEFORE PyQt6 is imported anywhere in the test session.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# Avoid the ROS launch_testing pytest plugin (lark crash) — see CLAUDE.md memory.
os.environ.pop("PYTHONPATH", None)
os.environ.pop("AMENT_PREFIX_PATH", None)

MESH_FIXTURE_DIR = Path(__file__).resolve().parents[1] / "fixtures" / "meshes"


@pytest.fixture(scope="session", autouse=True)
def _generate_mesh_fixtures() -> None:
    """Create deterministic 1x1x1 cube STLs once per test session."""
    MESH_FIXTURE_DIR.mkdir(parents=True, exist_ok=True)
    for name in ("base", "arm"):
        path = MESH_FIXTURE_DIR / f"{name}.stl"
        if not path.is_file():
            trimesh.creation.box(extents=(1.0, 1.0, 1.0)).export(str(path), file_type="stl")


@pytest.fixture
def mesh_fixture_dir() -> Path:
    return MESH_FIXTURE_DIR


@pytest.fixture
def base_stl(mesh_fixture_dir: Path) -> Path:
    return mesh_fixture_dir / "base.stl"


@pytest.fixture
def arm_stl(mesh_fixture_dir: Path) -> Path:
    return mesh_fixture_dir / "arm.stl"
