"""Inertia editor dock: material selection and mesh-based inertial preview."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDockWidget,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QWidget,
)

from cad2urdf.core.inertia.materials import list_materials, lookup
from cad2urdf.core.kinematic.model import InertialOverride, Robot
from cad2urdf.gui.state.controller import RobotController
from cad2urdf.gui.workers.base import Worker
from cad2urdf.gui.workers.recompute_inertia import InertiaResult, build_recompute_job


class InertiaEditorDock(QDockWidget):
    """Bottom dock for material/density changes and auto inertia recomputation."""

    recomputed = pyqtSignal(float)

    def __init__(self, controller: RobotController, parent: QWidget | None = None) -> None:
        super().__init__("Inertia Editor", parent)
        self._controller = controller
        self._link_name: str | None = None
        self._worker: Worker | None = None

        body = QWidget(self)
        form = QFormLayout(body)

        self.material_combo = QComboBox()
        self.material_combo.addItems(list_materials())
        self.apply_material_button = QPushButton("Apply material")
        self.apply_material_button.clicked.connect(self._on_apply_material)

        material_row = QHBoxLayout()
        material_row.addWidget(self.material_combo)
        material_row.addWidget(self.apply_material_button)
        material_holder = QWidget()
        material_holder.setLayout(material_row)
        form.addRow("Material", material_holder)

        self.density_spin = QDoubleSpinBox()
        self.density_spin.setRange(0.001, 1e6)
        self.density_spin.setDecimals(3)
        self.density_spin.setSuffix(" kg/m^3")
        form.addRow("Density", self.density_spin)

        self.auto_mass_label = QLabel("-")
        self.auto_com_label = QLabel("-")
        self.auto_inertia_label = QLabel("-")
        form.addRow("Auto mass", self.auto_mass_label)
        form.addRow("Auto COM", self.auto_com_label)
        form.addRow("Auto inertia (diag)", self.auto_inertia_label)

        self.recompute_button = QPushButton("Recompute from mesh")
        self.recompute_button.clicked.connect(self._on_recompute)
        form.addRow(self.recompute_button)

        self.override_mass_check = QCheckBox("Override mass")
        self.override_mass_spin = QDoubleSpinBox()
        self.override_mass_spin.setRange(0.0, 1e6)
        self.override_mass_spin.setDecimals(4)
        form.addRow(self.override_mass_check, self.override_mass_spin)

        self.apply_override_button = QPushButton("Apply override")
        self.apply_override_button.clicked.connect(self._on_apply_override)
        form.addRow(self.apply_override_button)

        self.setWidget(body)
        body.setEnabled(False)
        self._body = body

        controller.robotChanged.connect(self._on_robot_changed)

    def show_link(self, link_name: str) -> None:
        link = self._controller.current().links.get(link_name)
        if link is None:
            self._clear()
            return

        self._link_name = link_name
        self.material_combo.setCurrentText(link.material_name)
        self.density_spin.setValue(link.material_density)
        override = link.inertial_override
        self.override_mass_check.setChecked(override.mass is not None)
        self.override_mass_spin.setValue(override.mass if override.mass is not None else 0.0)
        self.auto_mass_label.setText("-")
        self.auto_com_label.setText("-")
        self.auto_inertia_label.setText("-")
        self._body.setEnabled(True)

    def _clear(self) -> None:
        self._link_name = None
        self._body.setEnabled(False)

    def _on_apply_material(self) -> None:
        if self._link_name is None:
            return

        link_name = self._link_name
        material_name = self.material_combo.currentText()
        try:
            material = lookup(material_name)
        except KeyError as exc:
            self._show_status(f"material error: {exc}", 4000)
            return

        def transform(robot: Robot) -> Robot:
            old = robot.links[link_name]
            new_link = replace(
                old,
                material_density=material.density_kg_m3,
                material_name=material_name,
            )
            new_links = dict(robot.links)
            new_links[link_name] = new_link
            return Robot(
                name=robot.name,
                base_link=robot.base_link,
                links=new_links,
                joints=dict(robot.joints),
            )

        self._controller.apply(transform, label=f"material {material_name} on {link_name}")

    def _on_apply_override(self) -> None:
        if self._link_name is None:
            return

        link_name = self._link_name
        mass_value: float | None = (
            self.override_mass_spin.value() if self.override_mass_check.isChecked() else None
        )

        def transform(robot: Robot) -> Robot:
            old = robot.links[link_name]
            new_link = replace(
                old,
                inertial_override=InertialOverride(
                    mass=mass_value,
                    com=old.inertial_override.com,
                    inertia=old.inertial_override.inertia,
                ),
            )
            new_links = dict(robot.links)
            new_links[link_name] = new_link
            return Robot(
                name=robot.name,
                base_link=robot.base_link,
                links=new_links,
                joints=dict(robot.joints),
            )

        self._controller.apply(transform, label=f"override mass on {link_name}")

    def _on_recompute(self) -> None:
        if self._link_name is None:
            return

        link = self._controller.current().links[self._link_name]
        mesh_path = link.visual_mesh_path
        if not mesh_path.is_absolute():
            self._show_status("recompute needs an absolute mesh path", 6000)
            return

        self._worker = Worker(
            build_recompute_job(
                mesh_path=mesh_path,
                density=self.density_spin.value(),
                override=link.inertial_override,
            )
        )
        self._worker.finished.connect(self._on_recompute_finished)
        self._worker.failed.connect(self._on_recompute_failed)
        self.recompute_button.setEnabled(False)
        self._worker.start()

    def _on_recompute_finished(self, result: object) -> None:
        self.recompute_button.setEnabled(True)
        mass, com, inertia = result  # type: ignore[misc]
        self._set_auto_values((float(mass), np.asarray(com), np.asarray(inertia)))
        self.recomputed.emit(float(mass))

    def _set_auto_values(self, result: InertiaResult) -> None:
        mass, com, inertia = result
        self.auto_mass_label.setText(f"{mass:.4f} kg")
        self.auto_com_label.setText(np.array2string(com, precision=4))
        self.auto_inertia_label.setText(np.array2string(np.diag(inertia), precision=6))

    def _on_recompute_failed(self, error: str, _trace: str) -> None:
        self.recompute_button.setEnabled(True)
        self._show_status(f"recompute failed: {error}", 6000)

    def _on_robot_changed(self, _robot: Robot) -> None:
        if self._link_name is None:
            return
        if self._link_name in self._controller.current().links:
            self.show_link(self._link_name)
        else:
            self._clear()

    def _show_status(self, message: str, timeout_ms: int) -> None:
        status_bar = getattr(self.window(), "statusBar", None)
        if callable(status_bar):
            status_bar().showMessage(message, timeout_ms)
