"""Link tree dock: read-only QTreeView over Robot, rooted at base_link.

The dock owns a QStandardItemModel rebuilt from the AST whenever the
RobotController changes. Selection emits `linkSelected(str)` so other panels
and the viewport can stay keyed to the same link.
"""

from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
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

        self.tree_view.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.tree_view.customContextMenuRequested.connect(self._show_context_menu)

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

    def reparent_link(self, link_name: str, *, new_parent: str) -> None:
        """Move the joint whose child is `link_name` so that its parent is `new_parent`."""
        from cad2urdf.core.kinematic.tree import descendants_of, reparent_joint

        if link_name == new_parent:
            raise ValueError(f"cannot drop link {link_name!r} onto itself")

        robot = self._controller.current()
        if new_parent in descendants_of(robot, link_name):
            raise ValueError(f"reparenting {link_name!r} under {new_parent!r} would create a cycle")

        joint = next((j for j in robot.joints.values() if j.child == link_name), None)
        if joint is None:
            raise ValueError(f"link {link_name!r} has no parent joint to reparent")

        self._controller.apply(
            lambda r: reparent_joint(r, joint.name, new_parent=new_parent),
            label=f"reparent {link_name} under {new_parent}",
        )

    def _show_context_menu(self, pos: object) -> None:
        from PyQt6.QtGui import QAction
        from PyQt6.QtWidgets import QInputDialog, QMenu

        idx = self.tree_view.indexAt(pos)
        if not idx.isValid():
            return
        link_name = self._model.data(idx)
        if not isinstance(link_name, str):
            return

        menu = QMenu(self.tree_view)
        act = QAction(f"Reparent {link_name}…", menu)

        def _do() -> None:
            names = [k for k in self._controller.current().links if k != link_name]
            if not names:
                return
            target, ok = QInputDialog.getItem(
                self.tree_view, "Reparent", "New parent:", names, 0, False
            )
            if not ok:
                return
            try:
                self.reparent_link(link_name, new_parent=target)
            except ValueError as e:
                win = self.window()
                if win is not None and hasattr(win, "statusBar"):
                    win.statusBar().showMessage(f"reparent failed: {e}", 6000)

        act.triggered.connect(_do)
        menu.addAction(act)
        menu.exec(self.tree_view.viewport().mapToGlobal(pos))
