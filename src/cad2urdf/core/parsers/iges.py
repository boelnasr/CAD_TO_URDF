"""IGES reader wrapper around the shared lazy-imported XCAF parser.

IGES often carries less assembly structure than STEP.  When XCAF exposes one
free shape with no components, the shared walker intentionally returns that
shape as one mesh-bearing root part with no assembly edges.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from cad2urdf.core.parsers.caf import StepAssembly, StepPart, load_caf_assembly, tessellate

__all__ = [
    "StepAssembly",
    "StepPart",
    "load_iges_assembly",
    "load_iges_assembly_with_shapes",
    "tessellate",
]


def _iges_reader() -> Any:
    """Create the XCAF IGES reader configured to preserve document names."""
    from OCP.IGESCAFControl import IGESCAFControl_Reader

    reader = IGESCAFControl_Reader()
    reader.SetNameMode(True)
    return reader


def load_iges_assembly_with_shapes(
    path: Path, *, unit_scale: float = 0.001
) -> tuple[StepAssembly, dict[str, Any]]:
    """Read an IGES assembly together with transient leaf-part B-reps."""
    return load_caf_assembly(path=path, reader_factory=_iges_reader, unit_scale=unit_scale)


def load_iges_assembly(path: Path, *, unit_scale: float = 0.001) -> StepAssembly:
    """Read IGES, falling back to one part for a single flat free shape."""
    assembly, _ = load_iges_assembly_with_shapes(path, unit_scale=unit_scale)
    return assembly
