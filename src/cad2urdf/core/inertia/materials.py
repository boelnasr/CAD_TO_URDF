"""Material density table loader. Default table ships at config/materials.json."""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from functools import cache
from pathlib import Path
from types import MappingProxyType

PROJECT_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_TABLE_PATH = PROJECT_ROOT / "config" / "materials.json"


@dataclass(frozen=True)
class Material:
    """A material with density (kg/m³) and an RGBA color tuple."""

    name: str
    density_kg_m3: float
    color_rgba: tuple[float, float, float, float]


@cache
def _load_material_table_inner(path: Path | None = None) -> dict[str, Material]:
    """Internal cached loader. Returns the underlying mutable dict."""
    p = path or DEFAULT_TABLE_PATH
    if not p.is_file():
        raise FileNotFoundError(f"materials table not found: {p}")
    raw = json.loads(p.read_text())
    out: dict[str, Material] = {}
    for name, body in raw.items():
        rgba = body.get("color_rgba", [0.5, 0.5, 0.5, 1.0])
        if len(rgba) != 4:
            raise ValueError(f"color_rgba for {name!r} must have 4 elements, got {len(rgba)}")
        out[name] = Material(
            name=name,
            density_kg_m3=float(body["density_kg_m3"]),
            color_rgba=(float(rgba[0]), float(rgba[1]), float(rgba[2]), float(rgba[3])),
        )
    return out


def load_material_table(path: Path | None = None) -> Mapping[str, Material]:
    """Load the material table from JSON. Returns a read-only mapping.
    Cached per path for the lifetime of the process."""
    return MappingProxyType(_load_material_table_inner(path))


def lookup(name: str) -> Material:
    """Look up a material by name. Raises KeyError if unknown."""
    table = load_material_table()
    if name not in table:
        raise KeyError(f"unknown material {name!r}; known: {sorted(table)}")
    return table[name]


def list_materials() -> list[str]:
    """Return all known material names in sorted order."""
    return sorted(load_material_table().keys())
