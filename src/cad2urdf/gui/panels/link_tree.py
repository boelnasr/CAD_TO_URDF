"""Link tree dock: read-only QTreeView over Robot, rooted at base_link.

The dock owns a QStandardItemModel rebuilt from the AST whenever the
RobotController changes. Selection emits `linkSelected(str)` so other panels
and the viewport can stay keyed to the same link.
"""

from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QStandardItem, QStandardItemModel
from PyQt6.QtWidgets import QDockWidget, QTreeView, QWidget

from cad2urdf.core.kinematic.model import Robot
from cad2urdf.core.kinematic.tree import children_of
from cad2urdf.gui.state.controller import RobotController


class LinkTreeDock(QDockWidget):
    """Hierarchical view of links. Joints are implied by parent/child rows."""

    linkSelected = pyqtSignal(str)  # noqa: N815

    def __init__(self, controller: RobotController, parent: QWidget | None = None) -> None:
        super().__init__("Link Tree", parent)
        self._controller = controller

        self.tree_view = QTreeView(self)
        self.tree_view.setHeaderHidden(True)
        self.tree_view.setEditTriggers(QTreeView.EditTrigger.NoEditTriggers)

        self._model = QStandardItemModel(self)
        self.tree_view.setModel(self._model)
        self.setWidget(self.tree_view)

        self.tree_view.selectionModel().currentChanged.connect(self._on_current_changed)
        controller.robotChanged.connect(self._rebuild)
        self._rebuild(controller.current())

    def _rebuild(self, robot: Robot) -> None:
        self._model.clear()
        if robot.base_link not in robot.links:
            return

        root = QStandardItem(robot.base_link)
        root.setEditable(False)
        self._model.appendRow(root)
        self._populate(root, robot, robot.base_link)
        self.tree_view.expandAll()

    def _populate(self, parent_item: QStandardItem, robot: Robot, link_name: str) -> None:
        for child in children_of(robot, link_name):
            item = QStandardItem(child)
            item.setEditable(False)
            parent_item.appendRow(item)
            self._populate(item, robot, child)

    def _on_current_changed(self, current, _previous) -> None:
        name = self._model.data(current)
        if isinstance(name, str) and name:
            self.linkSelected.emit(name)
