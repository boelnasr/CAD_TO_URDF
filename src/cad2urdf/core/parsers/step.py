"""STEP reader wrapper around the shared lazy-imported XCAF parser."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from cad2urdf.core.parsers.caf import StepAssembly, StepPart, load_caf_assembly, tessellate

__all__ = [
    "StepAssembly",
    "StepPart",
    "load_step_assembly",
    "load_step_assembly_with_shapes",
    "tessellate",
]


def _step_reader() -> Any:
    """Create the XCAF STEP reader configured to preserve document names."""
    from OCP.STEPCAFControl import STEPCAFControl_Reader

    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    return reader


def load_step_assembly_with_shapes(
    path: Path, *, unit_scale: float = 0.001
) -> tuple[StepAssembly, dict[str, Any]]:
    """Read a STEP assembly together with transient leaf-part B-reps."""
    return load_caf_assembly(path=path, reader_factory=_step_reader, unit_scale=unit_scale)


def load_step_assembly(path: Path, *, unit_scale: float = 0.001) -> StepAssembly:
    """Read a STEP assembly into named mesh-bearing parts and parent-child edges."""
    assembly, _ = load_step_assembly_with_shapes(path, unit_scale=unit_scale)
    return assembly
