"""Smoke: GUI package + every subpackage import cleanly under offscreen Qt."""

from __future__ import annotations

import pytest

pytestmark = pytest.mark.gui


def test_gui_package_imports() -> None:
    import cad2urdf.gui
    import cad2urdf.gui.dialogs
    import cad2urdf.gui.panels
    import cad2urdf.gui.state
    import cad2urdf.gui.viewport
    import cad2urdf.gui.windows
    import cad2urdf.gui.workers

    assert cad2urdf.gui is not None
    assert cad2urdf.gui.dialogs is not None
    assert cad2urdf.gui.panels is not None
    assert cad2urdf.gui.state is not None
    assert cad2urdf.gui.viewport is not None
    assert cad2urdf.gui.windows is not None
    assert cad2urdf.gui.workers is not None


def test_qapplication_constructs(qtbot) -> None:
    _ = qtbot  # injected by pytest-qt
    from PyQt6.QtWidgets import QApplication

    app = QApplication.instance()
    assert app is not None
