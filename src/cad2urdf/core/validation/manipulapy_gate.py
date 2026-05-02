"""Run a generated URDF through ManipulaPy.URDFToSerialManipulator and report.

ManipulaPy is an OPTIONAL dep (lives in the `[urdf-io]` extra — see pyproject.toml).
When not installed, validate_urdf returns ValidationReport(ok=False) with a clear
error rather than raising ImportError.
"""

from __future__ import annotations

import traceback
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ValidationReport:
    """Result of a URDF validation pass through ManipulaPy."""

    urdf_path: Path
    ok: bool
    error: str | None = None
    trace: str | None = None


def validate_urdf(urdf_path: Path) -> ValidationReport:
    """Parse via ManipulaPy and check IK/dynamics objects construct.

    Returns a ValidationReport rather than raising. The caller decides whether
    a failed validation should block (e.g., in CLI export) or just warn.
    """
    if not urdf_path.is_file():
        return ValidationReport(
            urdf_path=urdf_path,
            ok=False,
            error=f"file not found: {urdf_path}",
        )

    try:
        from ManipulaPy.urdf_processor import URDFToSerialManipulator
    except ImportError as e:
        return ValidationReport(
            urdf_path=urdf_path,
            ok=False,
            error=(
                f"ManipulaPy not installed: {e}. "
                "Install via `pip install -e .[urdf-io]` to enable validation."
            ),
        )

    try:
        proc = URDFToSerialManipulator(str(urdf_path))
        # touch both attributes so we trigger any lazy construction
        _ = proc.serial_manipulator
        _ = proc.dynamics
        return ValidationReport(urdf_path=urdf_path, ok=True)
    except Exception as e:
        return ValidationReport(
            urdf_path=urdf_path,
            ok=False,
            error=str(e),
            trace=traceback.format_exc(),
        )
