"""Joint editor dock for the currently selected link's parent joint."""

from __future__ import annotations

from dataclasses import replace
from typing import cast

import numpy as np
from PyQt6.QtWidgets import (
    QComboBox,
    QDockWidget,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QPushButton,
    QWidget,
)

from cad2urdf.core.kinematic.model import Joint, JointType, Robot
from cad2urdf.core.kinematic.tree import parent_of
from cad2urdf.gui.state.controller import RobotController

_JOINT_TYPES = ("revolute", "prismatic", "fixed", "continuous", "floating", "planar")
_LIMITED_JOINT_TYPES = {"revolute", "prismatic"}


def _make_spin(lo: float, hi: float, decimals: int = 4) -> QDoubleSpinBox:
    spin = QDoubleSpinBox()
    spin.setRange(lo, hi)
    spin.setDecimals(decimals)
    spin.setSingleStep(0.01)
    return spin


class JointEditorDock(QDockWidget):
    """Form panel that edits the joint parenting the selected link."""

    def __init__(self, controller: RobotController, parent: QWidget | None = None) -> None:
        super().__init__("Joint Editor", parent)
        self._controller = controller
        self._joint_name: str | None = None

        self.form_widget = QWidget(self)
        form = QFormLayout(self.form_widget)

        self.type_combo = QComboBox()
        self.type_combo.addItems(_JOINT_TYPES)
        form.addRow("Type", self.type_combo)

        self.axis_x = _make_spin(-1.0, 1.0)
        self.axis_y = _make_spin(-1.0, 1.0)
        self.axis_z = _make_spin(-1.0, 1.0)
        axis_row = QHBoxLayout()
        for spin in (self.axis_x, self.axis_y, self.axis_z):
            axis_row.addWidget(spin)
        axis_holder = QWidget()
        axis_holder.setLayout(axis_row)
        form.addRow("Axis (xyz)", axis_holder)

        self.lower_limit = _make_spin(-1e6, 1e6)
        self.upper_limit = _make_spin(-1e6, 1e6)
        form.addRow("Limit lower", self.lower_limit)
        form.addRow("Limit upper", self.upper_limit)

        self.effort = _make_spin(0.0, 1e6)
        self.velocity = _make_spin(0.0, 1e6)
        form.addRow("Effort", self.effort)
        form.addRow("Velocity", self.velocity)

        self.apply_button = QPushButton("Apply changes")
        self.apply_button.clicked.connect(self._on_apply)
        form.addRow(self.apply_button)

        self.setWidget(self.form_widget)
        self.form_widget.setEnabled(False)

        controller.robotChanged.connect(self._on_robot_changed)

    def show_link(self, link_name: str) -> None:
        robot = self._controller.current()
        parent = parent_of(robot, link_name)
        if parent is None:
            self._clear()
            return

        match = next(
            (joint for joint in robot.joints.values() if joint.parent == parent and joint.child == link_name),
            None,
        )
        if match is None:
            self._clear()
            return

        self._joint_name = match.name
        self._populate_from(match)
        self.form_widget.setEnabled(True)

    def _clear(self) -> None:
        self._joint_name = None
        self.form_widget.setEnabled(False)

    def _populate_from(self, joint: Joint) -> None:
        self.type_combo.setCurrentText(joint.type)
        self.axis_x.setValue(float(joint.axis[0]))
        self.axis_y.setValue(float(joint.axis[1]))
        self.axis_z.setValue(float(joint.axis[2]))
        self.lower_limit.setValue(joint.limit_lower if joint.limit_lower is not None else 0.0)
        self.upper_limit.setValue(joint.limit_upper if joint.limit_upper is not None else 0.0)
        self.effort.setValue(joint.effort if joint.effort is not None else 0.0)
        self.velocity.setValue(joint.velocity if joint.velocity is not None else 0.0)

    def _on_apply(self) -> None:
        if self._joint_name is None:
            return

        joint_name = self._joint_name
        new_type = cast(JointType, self.type_combo.currentText())
        axis = np.array(
            [self.axis_x.value(), self.axis_y.value(), self.axis_z.value()],
            dtype=float,
        )
        norm = float(np.linalg.norm(axis))
        if norm == 0.0:
            self._show_status("axis must be non-zero", 4000)
            return
        axis /= norm

        lower = self.lower_limit.value()
        upper = self.upper_limit.value()
        effort = self.effort.value()
        velocity = self.velocity.value()

        def transform(robot: Robot) -> Robot:
            old = robot.joints[joint_name]
            new_joint = replace(
                old,
                type=new_type,
                axis=axis,
                limit_lower=lower if new_type in _LIMITED_JOINT_TYPES else None,
                limit_upper=upper if new_type in _LIMITED_JOINT_TYPES else None,
                effort=effort if new_type in _LIMITED_JOINT_TYPES else None,
                velocity=velocity if new_type in _LIMITED_JOINT_TYPES else None,
            )
            new_joints = dict(robot.joints)
            new_joints[joint_name] = new_joint
            return Robot(
                name=robot.name,
                base_link=robot.base_link,
                links=dict(robot.links),
                joints=new_joints,
            )

        try:
            self._controller.apply(transform, label=f"edit joint {joint_name}")
        except ValueError as exc:
            self._show_status(f"joint update rejected: {exc}", 6000)

    def _on_robot_changed(self, _robot: Robot) -> None:
        if self._joint_name is None:
            return
        joint = self._controller.current().joints.get(self._joint_name)
        if joint is None:
            self._clear()
        else:
            self._populate_from(joint)

    def _show_status(self, message: str, timeout_ms: int) -> None:
        status_bar = getattr(self.window(), "statusBar", None)
        if callable(status_bar):
            status_bar().showMessage(message, timeout_ms)
