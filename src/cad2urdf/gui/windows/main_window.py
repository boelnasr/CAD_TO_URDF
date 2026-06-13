"""Top-level QMainWindow: docks, menus, toolbar, status strip. Owns the RobotController."""

from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QAction, QKeySequence
from PyQt6.QtWidgets import (
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

        from cad2urdf.gui.viewport.widget import ViewportWidget

        self.viewport = ViewportWidget(self.controller, self)
        self.setCentralWidget(self.viewport)

        self._build_docks()
        self._build_actions()
        self._build_menus()
        self._build_toolbar()

        self.statusBar().showMessage("Ready")

        self.controller.historyChanged.connect(self._on_history_changed)
        self.action_validate.triggered.connect(self._on_validate_clicked)
        self.action_import.triggered.connect(self._on_import_clicked)
        self.action_export.triggered.connect(self._on_export_clicked)
        self.action_save.triggered.connect(self._on_save_clicked)
        self.action_open.triggered.connect(self._on_open_clicked)
        self._validate_worker = None

    # ---- construction helpers ----------------------------------------------
    def _build_docks(self) -> None:
        from cad2urdf.gui.panels.inertia_editor import InertiaEditorDock
        from cad2urdf.gui.panels.joint_editor import JointEditorDock
        from cad2urdf.gui.panels.link_tree import LinkTreeDock

        self.dock_link_tree = LinkTreeDock(self.controller, self)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.dock_link_tree)

        self.dock_joint_editor = JointEditorDock(self.controller, self)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.dock_joint_editor)
        self.dock_link_tree.linkSelected.connect(self.dock_joint_editor.show_link)

        self.dock_inertia_editor = InertiaEditorDock(self.controller, self)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.dock_inertia_editor)
        self.dock_link_tree.linkSelected.connect(self.dock_inertia_editor.show_link)

        # Tree selection → highlight the link in the viewport. Viewport picking
        # loops back through the tree, so both paths drive the highlight.
        self.dock_link_tree.linkSelected.connect(self.viewport.highlight_link)

        # Viewport pick → select that link in the tree (which cascades to the
        # joint + inertia editors via the existing linkSelected connections).
        self.viewport.linkPicked.connect(self._select_link_in_tree)

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

    # ---- viewport pick → tree selection ------------------------------------
    def _select_link_in_tree(self, link_name: str) -> None:
        model = self.dock_link_tree.tree_view.model()

        def _walk(parent_index) -> bool:
            for row in range(model.rowCount(parent_index)):
                idx = model.index(row, 0, parent_index)
                if model.data(idx) == link_name:
                    self.dock_link_tree.tree_view.setCurrentIndex(idx)
                    return True
                if _walk(idx):
                    return True
            return False

        from PyQt6.QtCore import QModelIndex

        _walk(QModelIndex())  # start from the invisible root

    # ---- controller signals ------------------------------------------------
    def _on_history_changed(self, label: str) -> None:
        self.action_undo.setEnabled(self.controller.can_undo())
        self.action_redo.setEnabled(self.controller.can_redo())
        self.statusBar().showMessage(f"{label}", 4000)

    # ---- import action -----------------------------------------------------
    def _on_import_clicked(self) -> None:
        from PyQt6.QtWidgets import QFileDialog, QInputDialog

        from cad2urdf.gui.dialogs.import_meshes import run_import_into_controller

        paths_str, _filter = QFileDialog.getOpenFileNames(
            self, "Import meshes", "", "Meshes (*.stl *.obj)"
        )
        if not paths_str:
            return
        robot_name, ok = QInputDialog.getText(self, "Robot name", "Robot name:", text="my_robot")
        if not ok or not robot_name.strip():
            return
        try:
            run_import_into_controller(
                controller=self.controller,
                mesh_paths=[Path(p) for p in paths_str],
                robot_name=robot_name.strip(),
            )
        except RuntimeError as e:
            self.statusBar().showMessage(f"import failed: {e}", 8000)

    # ---- validate action ---------------------------------------------------
    def _on_validate_clicked(self) -> None:
        import tempfile

        from cad2urdf.gui.workers.base import Worker
        from cad2urdf.gui.workers.validate import build_validate_job

        scratch = Path(tempfile.mkdtemp(prefix="cad2urdf_validate_"))
        robot = self.controller.current()
        worker = Worker(
            build_validate_job(
                robot=robot,
                out_dir=scratch,
                package_name=f"{robot.name}_pkg",
                urdf_relname=f"{robot.name}.urdf",
                run_manipulapy=True,
            )
        )
        worker.finished.connect(self._on_validate_done)
        worker.failed.connect(
            lambda err, _t: self.statusBar().showMessage(f"validate failed: {err}", 8000)
        )
        self.statusBar().showMessage("validating…", 8000)
        worker.start()
        self._validate_worker = worker  # keep a reference alive

    def _on_validate_done(self, report: object) -> None:
        ast_n = len(report.ast_issues)  # type: ignore[attr-defined]
        if report.manipulapy_ok is True:  # type: ignore[attr-defined]
            self.statusBar().showMessage(
                f"validation OK ({ast_n} AST notices, ManipulaPy-compatible)", 10000
            )
        elif report.manipulapy_ok is False:  # type: ignore[attr-defined]
            self.statusBar().showMessage(
                f"validation issues: {ast_n} AST notices; ManipulaPy: {report.manipulapy_error}",  # type: ignore[attr-defined]
                12000,
            )
        else:
            self.statusBar().showMessage(
                f"validation OK ({ast_n} AST notices, ManipulaPy skipped)", 10000
            )

    # ---- export action --------------------------------------------------------
    def _on_export_clicked(self) -> None:
        from PyQt6.QtWidgets import QFileDialog, QInputDialog

        from cad2urdf.gui.dialogs.export_package import run_export_into_dir

        out_str = QFileDialog.getExistingDirectory(self, "Choose output directory", "")
        if not out_str:
            return
        package_name, ok = QInputDialog.getText(
            self,
            "Package name",
            "ROS package name (lowercase, underscores):",
            text=Path(out_str).name,
        )
        if not ok or not package_name.strip():
            return
        try:
            report = run_export_into_dir(
                controller=self.controller,
                out_dir=Path(out_str),
                package_name=package_name.strip(),
                maintainer="cad2urdf-user",
                maintainer_email="user@example.com",
                run_manipulapy=True,
            )
        except RuntimeError as e:
            self.statusBar().showMessage(f"export failed: {e}", 8000)
            return
        if report.manipulapy_ok is True:
            ok_label = "ManipulaPy-compatible"
        elif report.manipulapy_ok is None:
            ok_label = "ManipulaPy: skipped"
        else:
            ok_label = f"ManipulaPy: {report.manipulapy_error}"
        self.statusBar().showMessage(f"wrote {report.urdf_path} ({ok_label})", 12000)

    # ---- project save / open ------------------------------------------------
    def _save_project_to(self, path: Path) -> None:
        from cad2urdf.core.project.save import save_project

        save_project(self.controller.current(), path)
        self.statusBar().showMessage(f"saved {path}", 6000)

    def _open_project_from(self, path: Path) -> None:
        from cad2urdf.core.project.save import load_project

        robot = load_project(path)
        self.controller.replace(robot)
        self.statusBar().showMessage(f"opened {path}", 6000)

    def _on_save_clicked(self) -> None:
        from PyQt6.QtWidgets import QFileDialog

        path_str, _ = QFileDialog.getSaveFileName(
            self, "Save project", "", "cad2urdf project (*.cad2urdf)"
        )
        if not path_str:
            return
        path = Path(path_str)
        if path.suffix != ".cad2urdf":
            path = path.with_suffix(".cad2urdf")
        try:
            self._save_project_to(path)
        except OSError as e:
            self.statusBar().showMessage(f"save failed: {e}", 8000)

    def _on_open_clicked(self) -> None:
        from PyQt6.QtWidgets import QFileDialog

        path_str, _ = QFileDialog.getOpenFileName(
            self, "Open project", "", "cad2urdf project (*.cad2urdf)"
        )
        if not path_str:
            return
        try:
            self._open_project_from(Path(path_str))
        except (OSError, ValueError) as e:
            self.statusBar().showMessage(f"open failed: {e}", 8000)
