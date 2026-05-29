"""3D viewport widget — embeds pyvistaqt.QtInteractor inside a QWidget.

One pyvista actor per link; the dict is rebuilt whenever robotChanged fires.
Links whose visual_mesh_path is not absolute or not on disk are skipped
(the placeholder make_empty_robot link uses a relative path; that is fine —
the viewport just shows nothing until the user imports real meshes).

Headless / offscreen note
-------------------------
When ``pyvista.OFF_SCREEN`` is True (e.g. the ``PYVISTA_OFF_SCREEN=true``
env-var is set before pyvista is imported, as ``tests/gui/conftest.py`` does),
``QtInteractor`` cannot be used because VTK's X11 render-window integration
issues a fatal ``X_UnmapWindow`` / ``X_ConfigureWindow`` error against the
dummy window ID produced by Qt's offscreen platform plugin.  In that case the
widget falls back to a plain ``pyvista.Plotter(off_screen=True)`` which has
the same ``add_mesh`` / ``remove_actor`` / ``reset_camera`` API but does not
embed into a Qt widget hierarchy.  The QVBoxLayout is left empty in that mode,
which is fine — tests only call ``actors_by_link_name()`` and never paint.
"""

from __future__ import annotations

from typing import Any

import pyvista as pv
from PyQt6.QtWidgets import QVBoxLayout, QWidget

from cad2urdf.core.kinematic.model import Robot
from cad2urdf.gui.state.controller import RobotController


class ViewportWidget(QWidget):
    """QWidget wrapping a pyvista plotter with one actor per robot link."""

    def __init__(self, controller: RobotController, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._controller = controller
        self._actors: dict[str, Any] = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if pv.OFF_SCREEN:
            # Headless / test mode: use a plain off-screen plotter.
            # QVTKRenderWindowInteractor crashes with a fatal X11 BadWindow
            # error when Qt is using the offscreen platform plugin.
            self.plotter: Any = pv.Plotter(off_screen=True)
        else:
            # Real display: embed pyvistaqt.QtInteractor as a Qt widget.
            from pyvistaqt import QtInteractor  # deferred so tests never hit Qt-VTK interop

            self.plotter = QtInteractor(self)
            layout.addWidget(self.plotter.interactor)
            self.plotter.set_background("white")
            self.plotter.show_axes()
            self.plotter.show_grid()

        controller.robotChanged.connect(self._rebuild)
        self._rebuild(controller.current())

    # ---- public API --------------------------------------------------------

    def actors_by_link_name(self) -> dict[str, Any]:
        """Return a copy of the {link_name: actor} dict."""
        return dict(self._actors)

    # ---- private -----------------------------------------------------------

    def _rebuild(self, robot: Robot) -> None:
        """Replace all actors to match the current robot link set."""
        for actor in list(self._actors.values()):
            self.plotter.remove_actor(actor)
        self._actors.clear()

        for name, link in robot.links.items():
            path = link.visual_mesh_path
            if not path.is_absolute() or not path.is_file():
                continue
            try:
                mesh = pv.read(str(path))
            except Exception:
                continue
            actor = self.plotter.add_mesh(
                mesh, name=name, show_edges=False, color="lightgray", pickable=True
            )
            self._actors[name] = actor

        self.plotter.reset_camera()
