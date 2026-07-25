"""Shared XCAF assembly walk behavior."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

pytest.importorskip("OCP")

from tests.unit.test_step_parser import _write_named_assembly


def _step_reader() -> Any:
    from OCP.STEPCAFControl import STEPCAFControl_Reader

    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    return reader


def test_load_caf_assembly_returns_shapes_for_leaf_parts_only(tmp_path: Path) -> None:
    """The shared walk exposes each leaf B-rep but no assembly-frame shape."""
    from cad2urdf.core.parsers.caf import load_caf_assembly

    path = tmp_path / "robot.step"
    _write_named_assembly(path)

    assembly, shapes = load_caf_assembly(path=path, reader_factory=_step_reader)

    leaf_ids = {part.id for part in assembly.parts if part.is_leaf}
    assert {part.name for part in assembly.parts if part.is_leaf} == {"base", "arm"}
    assert set(shapes) == leaf_ids
    assert all(shape is not None for shape in shapes.values())


def test_step_wrapper_exposes_leaf_shape_side_map(tmp_path: Path) -> None:
    """The STEP convenience API retains transient leaf B-reps alongside the assembly."""
    from cad2urdf.core.parsers.step import load_step_assembly_with_shapes

    path = tmp_path / "robot.step"
    _write_named_assembly(path)

    assembly, shapes = load_step_assembly_with_shapes(path)

    assert set(shapes) == {part.id for part in assembly.parts if part.is_leaf}
