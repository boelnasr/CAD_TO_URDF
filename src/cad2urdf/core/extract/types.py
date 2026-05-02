"""Shared data types produced by parsers and consumed by extractors.

Lives outside `core/parsers/` so it can be used without importing OCC-dependent
STEP parsing code (which lazy-imports pythonOCC-core only when needed).

Optional fields on ``AssemblyEdge`` are advisory hints populated by richer
extractors (Tasks 2.4 / 4.x). v1-alpha consumers (``classify_default_fixed``)
only use the required fields and may safely ignore the rest.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class AssemblyEdge:
    """Parent-child relationship in a CAD assembly tree.

    ``relative_pose`` is child-relative-to-parent. Optional fields capture
    metadata that future STEP/XCAF extraction will populate; v1-alpha
    consumers (``classify_default_fixed``) only use the required fields.

    Produced by STEP loaders (when ``pythonOCC-core`` is available) and by any
    other parser that exposes a hierarchical assembly. Consumed by
    ``core/extract/*`` to derive joint hypotheses.
    """

    parent_id: str
    child_id: str
    relative_pose: NDArray[Any] = field(default_factory=lambda: np.eye(4))
    # Optional metadata for richer extractors (Tasks 2.4 / 4.x):
    world_pose: NDArray[Any] | None = None  # 4x4 absolute pose; None if unknown
    source_label: str | None = None  # human-readable label from the source file
    mate_kind: str | None = None  # raw OCC/STEP mate kind hint, before classification
    confidence: float = 1.0  # 0.0-1.0; default 1.0 = "asserted by source"
