"""Helpers for the two-point joint-axis pick interaction."""

from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import NDArray
from PyQt6.QtCore import QObject, pyqtSignal


def axis_from_two_points(p1: NDArray[Any], p2: NDArray[Any]) -> NDArray[Any]:
    """Return the unit vector pointing from `p1` to `p2`. Raises if coincident."""
    delta = np.asarray(p2, dtype=float) - np.asarray(p1, dtype=float)
    norm = float(np.linalg.norm(delta))
    if norm < 1e-9:
        raise ValueError(f"coincident points: {p1} == {p2}")
    return delta / norm


class TwoPointAxisCollector(QObject):
    """Accumulate two picked points and emit the unit axis when both arrive."""

    axisReady = pyqtSignal(object)  # NDArray[Any] of shape (3,)  # noqa: N815

    def __init__(self) -> None:
        super().__init__()
        self._buffer: list[NDArray[Any]] = []

    def add_point(self, point: NDArray[Any]) -> None:
        self._buffer.append(np.asarray(point, dtype=float).reshape(3))
        if len(self._buffer) == 2:
            try:
                axis = axis_from_two_points(self._buffer[0], self._buffer[1])
            except ValueError:
                self._buffer.clear()
                return
            self._buffer.clear()
            self.axisReady.emit(axis)

    def points_collected(self) -> int:
        return len(self._buffer)

    def reset(self) -> None:
        self._buffer.clear()
