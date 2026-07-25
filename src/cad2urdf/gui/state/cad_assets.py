"""Ownership and cleanup for generated CAD-import mesh directories."""

from __future__ import annotations

import logging
import shutil
import tempfile
from pathlib import Path

_log = logging.getLogger(__name__)


class CadAssetOwner:
    """Own generated CAD assets for one GUI/controller session.

    Only directories allocated by this instance can be activated or removed.
    A supplied ``root`` is treated as caller-owned; the default session root is
    itself temporary and is removed by :meth:`close`.
    """

    def __init__(self, *, root: Path | None = None) -> None:
        self._owns_root = root is None
        if root is None:
            root = Path(tempfile.mkdtemp(prefix="cad2urdf_cad_session_"))
        else:
            root.mkdir(parents=True, exist_ok=True)
        self._root = root.resolve()
        self._pending: set[Path] = set()
        self._active: Path | None = None
        self._closing = False

    def allocate(self) -> Path:
        """Create and track a pending per-import directory."""
        if self._closing:
            raise RuntimeError("CAD asset owner is shutting down")
        path = Path(tempfile.mkdtemp(prefix="import_", dir=self._root)).resolve()
        self._pending.add(path)
        return path

    def validate_pending(self, path: Path) -> Path:
        """Return the normalized path only if this owner allocated it."""
        normalized = path.resolve()
        if normalized not in self._pending:
            raise ValueError(f"CAD asset directory was not allocated by this owner: {path}")
        return normalized

    def activate(self, path: Path) -> None:
        """Make a pending directory active, then retire the prior active tree."""
        normalized = self.validate_pending(path)
        if self._closing:
            self.discard(normalized)
            raise RuntimeError("CAD asset owner is shutting down")
        previous = self._active
        self._pending.remove(normalized)
        self._active = normalized
        if previous is not None and previous != normalized:
            self._remove(previous)

    def discard(self, path: Path) -> None:
        """Remove a failed/cancelled pending import directory."""
        normalized = path.resolve()
        if normalized not in self._pending:
            return
        self._pending.remove(normalized)
        self._remove(normalized)
        self._finish_close()

    def retire_active(self) -> None:
        """Remove the current document's generated assets, if any."""
        previous = self._active
        self._active = None
        if previous is not None:
            self._remove(previous)

    def close(self) -> None:
        """Begin shutdown, deferring pending cleanup until their jobs finish."""
        self._closing = True
        self.retire_active()
        self._finish_close()

    def _finish_close(self) -> None:
        if self._closing and not self._pending and self._owns_root:
            self._remove(self._root)

    @staticmethod
    def _remove(path: Path) -> None:
        try:
            shutil.rmtree(path)
        except FileNotFoundError:
            return
        except OSError:
            _log.exception("failed to remove generated CAD assets at %s", path)
