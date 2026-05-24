"""Architectural guard: nothing under src/cad2urdf/core/ may import Qt or VTK.

This protects the CLI / library install (which deliberately omits the `gui`
extra) and prevents accidental coupling between the AST and the GUI.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

CORE_ROOT = Path(__file__).resolve().parents[2] / "src" / "cad2urdf" / "core"

FORBIDDEN_TOP_MODULES = frozenset(
    {"PyQt6", "PyQt5", "PySide6", "PySide2", "vtk", "pyvista", "pyvistaqt"}
)


def _imported_top_modules(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(), filename=str(path))
    out: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                out.add(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                out.add(node.module.split(".")[0])
    return out


@pytest.mark.parametrize(
    "py_file",
    sorted(CORE_ROOT.rglob("*.py")),
    ids=lambda p: str(p.relative_to(CORE_ROOT)),
)
def test_core_file_does_not_import_qt_or_vtk(py_file: Path) -> None:
    imports = _imported_top_modules(py_file)
    leaked = imports & FORBIDDEN_TOP_MODULES
    assert not leaked, (
        f"{py_file.relative_to(CORE_ROOT)} imports forbidden GUI/VTK modules: {sorted(leaked)}. "
        "core/ must stay free of Qt/VTK so the CLI install (no `gui` extra) keeps working."
    )
