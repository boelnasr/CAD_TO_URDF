"""Shared data types produced by parsers and consumed by extractors.

Lives outside `core/parsers/` so it can be used without importing OCC-dependent
STEP parsing code (which lazy-imports pythonOCC-core only when needed).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class AssemblyEdge:
    """Parent-child relationship in a CAD assembly tree.

    Produced by STEP loaders (when `pythonOCC-core` is available) and by any
    other parser that exposes a hierarchical assembly. Consumed by
    `core/extract/*` to derive joint hypotheses.
    """

    parent_id: str
    child_id: str
    relative_pose: NDArray[Any] = field(default_factory=lambda: np.eye(4))
