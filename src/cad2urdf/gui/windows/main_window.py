"""Top-level QMainWindow: docks, menus, toolbar, status strip. Owns the RobotController."""

from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QAction, QKeySequence
from PyQt6.QtWidgets import (
    QDockWidget,
    QLabel,
    QMainWindow,
    QWidget,
)

from cad2urdf.gui.state.controller import RobotController


class MainWindow(QMainWindow):
    """Shell for the cad2urdf editor: docks + central placeholder + menus."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("cad2urdf")
        self.resize(1280, 800)

        self.controller = RobotController()

        # Central placeholder — replaced by the VTK viewport in Task 7.1.
        self._central_placeholder = QLabel("3D viewport will appear here once a project is loaded.")
        self._central_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setCentralWidget(self._central_placeholder)

        self._build_docks()
        self._build_actions()
        self._build_menus()
        self._build_toolbar()

        self.statusBar().showMessage("Ready")

        self.controller.historyChanged.connect(self._on_history_changed)

    # ---- construction helpers ----------------------------------------------
    def _build_docks(self) -> None:
        # Placeholders. Real widgets land in Tasks 4.1 / 5.1 / 6.1 — each task
        # replaces the placeholder by calling dock.setWidget(...).
        self.dock_link_tree = QDockWidget("Link Tree", self)
        self.dock_link_tree.setWidget(QLabel("(link tree)"))
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.dock_link_tree)

        self.dock_joint_editor = QDockWidget("Joint Editor", self)
        self.dock_joint_editor.setWidget(QLabel("(joint editor)"))
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.dock_joint_editor)

        self.dock_inertia_editor = QDockWidget("Inertia Editor", self)
        self.dock_inertia_editor.setWidget(QLabel("(inertia editor)"))
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.dock_inertia_editor)

    def _build_actions(self) -> None:
        self.action_open = QAction("&Open Project…", self, shortcut=QKeySequence.StandardKey.Open)
        self.action_save = QAction("&Save Project…", self, shortcut=QKeySequence.StandardKey.Save)
        self.action_import = QAction("&Import Meshes…", self)
        self.action_export = QAction("&Export ROS Package…", self)
        self.action_quit = QAction("&Quit", self, shortcut=QKeySequence.StandardKey.Quit)
        self.action_quit.triggered.connect(self.close)

        self.action_undo = QAction("&Undo", self, shortcut=QKeySequence.StandardKey.Undo)
        self.action_undo.setEnabled(False)
        self.action_undo.triggered.connect(self.controller.undo)

        self.action_redo = QAction("&Redo", self, shortcut=QKeySequence.StandardKey.Redo)
        self.action_redo.setEnabled(False)
        self.action_redo.triggered.connect(self.controller.redo)

        self.action_validate = QAction("&Validate (dry run)", self)
        self.action_import_step_disabled = QAction("Import &STEP… (requires pythonOCC-core)", self)
        self.action_import_step_disabled.setEnabled(False)
        self.action_import_step_disabled.setToolTip(
            "Install pythonOCC-core via `conda env create -f environment.yml` to enable STEP."
        )

    def _build_menus(self) -> None:
        mb = self.menuBar()
        file_menu = mb.addMenu("&File")
        file_menu.addAction(self.action_open)
        file_menu.addAction(self.action_save)
        file_menu.addSeparator()
        file_menu.addAction(self.action_import)
        file_menu.addAction(self.action_import_step_disabled)
        file_menu.addSeparator()
        file_menu.addAction(self.action_quit)

        edit_menu = mb.addMenu("&Edit")
        edit_menu.addAction(self.action_undo)
        edit_menu.addAction(self.action_redo)

        view_menu = mb.addMenu("&View")
        view_menu.addAction(self.dock_link_tree.toggleViewAction())
        view_menu.addAction(self.dock_joint_editor.toggleViewAction())
        view_menu.addAction(self.dock_inertia_editor.toggleViewAction())

        export_menu = mb.addMenu("&Export")
        export_menu.addAction(self.action_export)
        export_menu.addAction(self.action_validate)

        help_menu = mb.addMenu("&Help")
        help_menu.addAction(QAction("&About cad2urdf", self))

    def _build_toolbar(self) -> None:
        tb = self.addToolBar("Main")
        tb.setObjectName("MainToolBar")
        tb.addAction(self.action_import)
        tb.addAction(self.action_export)
        tb.addSeparator()
        tb.addAction(self.action_undo)
        tb.addAction(self.action_redo)
        tb.addSeparator()
        tb.addAction(self.action_validate)

    # ---- controller signals ------------------------------------------------
    def _on_history_changed(self, label: str) -> None:
        self.action_undo.setEnabled(self.controller.can_undo())
        self.action_redo.setEnabled(self.controller.can_redo())
        self.statusBar().showMessage(f"{label}", 4000)
