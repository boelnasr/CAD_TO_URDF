"""RobotController — single mutable owner of the Robot AST for the GUI.

Mutations only happen through `apply(transform)`, where `transform` is a
pure function `Robot -> Robot` (the immutable tree.py ops). The controller
captures every successful transform on an undo stack so File > Undo works
across the whole session.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from PyQt6.QtCore import QObject, pyqtSignal

from cad2urdf.core.kinematic.model import Robot
from cad2urdf.gui.state.empty_robot import make_empty_robot

RobotTransform = Callable[[Robot], Robot]


@dataclass(frozen=True)
class _HistoryEntry:
    before: Robot
    after: Robot
    label: str


class RobotController(QObject):
    """Holds the current Robot AST and an undo/redo history."""

    robotChanged = pyqtSignal(object)  # arg: new Robot  # noqa: N815
    historyChanged = pyqtSignal(str)  # arg: label of the change just applied  # noqa: N815

    def __init__(self) -> None:
        super().__init__()
        self._robot: Robot = make_empty_robot()
        self._undo: list[_HistoryEntry] = []
        self._redo: list[_HistoryEntry] = []

    # ---- query --------------------------------------------------------------
    def current(self) -> Robot:
        return self._robot

    def can_undo(self) -> bool:
        return bool(self._undo)

    def can_redo(self) -> bool:
        return bool(self._redo)

    # ---- mutate -------------------------------------------------------------
    def replace(self, robot: Robot) -> None:
        """Hard-replace the AST (used by File > Open and Import wizard).

        Clears the undo history because the new AST is a fresh document.
        """
        self._robot = robot
        self._undo.clear()
        self._redo.clear()
        self.robotChanged.emit(self._robot)
        self.historyChanged.emit("replace")

    def apply(self, transform: RobotTransform, *, label: str) -> None:
        """Apply a pure Robot->Robot transform and remember it for undo.

        Re-raises any exception from `transform` without mutating state.
        """
        before = self._robot
        after = transform(before)  # may raise — propagated to caller
        if not isinstance(after, Robot):
            raise TypeError(f"transform must return Robot, got {type(after).__name__}")
        self._robot = after
        self._undo.append(_HistoryEntry(before=before, after=after, label=label))
        self._redo.clear()
        self.robotChanged.emit(self._robot)
        self.historyChanged.emit(label)

    def undo(self) -> None:
        if not self._undo:
            return
        entry = self._undo.pop()
        self._redo.append(entry)
        self._robot = entry.before
        self.robotChanged.emit(self._robot)
        self.historyChanged.emit(f"undo {entry.label}")

    def redo(self) -> None:
        if not self._redo:
            return
        entry = self._redo.pop()
        self._undo.append(entry)
        self._robot = entry.after
        self.robotChanged.emit(self._robot)
        self.historyChanged.emit(f"redo {entry.label}")
