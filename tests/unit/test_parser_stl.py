from pathlib import Path

import numpy as np
import pytest
import trimesh

from cad2urdf.core.parsers.stl import load_stl


def _write_cube_stl(path: Path) -> None:
    cube = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    cube.export(str(path), file_type="stl")


def test_load_stl_returns_trimesh(tmp_path: Path) -> None:
    p = tmp_path / "cube.stl"
    _write_cube_stl(p)
    mesh = load_stl(p)
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert np.isclose(mesh.volume, 1.0)


def test_load_stl_missing_file_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        load_stl(tmp_path / "ghost.stl")


def test_load_stl_wrong_extension_raises(tmp_path: Path) -> None:
    p = tmp_path / "something.txt"
    p.write_text("not a stl")
    with pytest.raises(ValueError, match=r"expected \.stl"):
        load_stl(p)
