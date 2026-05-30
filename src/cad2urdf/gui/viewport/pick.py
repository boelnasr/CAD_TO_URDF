"""Mesh-picking helper: maps a picked pyvista actor back to its link name."""

from __future__ import annotations

from typing import Any


def resolve_picked_link(actor: Any, actors_by_link_name: dict[str, Any]) -> str | None:
    """Return the link name whose actor matches `actor`, or None."""
    for name, a in actors_by_link_name.items():
        if a is actor:
            return name
    return None
