from pathlib import Path

import numpy as np
import pytest
import trimesh

from cad2urdf.core.parsers.obj import load_obj


def _write_cube_obj(path: Path) -> None:
    cube = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    cube.export(str(path), file_type="obj")


def test_load_obj_returns_trimesh(tmp_path: Path) -> None:
    p = tmp_path / "cube.obj"
    _write_cube_obj(p)
    mesh = load_obj(p)
    assert isinstance(mesh, trimesh.Trimesh)
    assert np.isclose(mesh.volume, 1.0, atol=1e-6)


def test_load_obj_missing_file_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        load_obj(tmp_path / "ghost.obj")


def test_load_obj_wrong_extension_raises(tmp_path: Path) -> None:
    p = tmp_path / "x.stl"
    p.write_text("nope")
    with pytest.raises(ValueError, match=r"expected \.obj"):
        load_obj(p)
