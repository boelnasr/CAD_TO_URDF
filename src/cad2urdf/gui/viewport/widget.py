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

import contextlib
from typing import Any

import pyvista as pv
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QVBoxLayout, QWidget

from cad2urdf.core.kinematic.model import Robot
from cad2urdf.core.kinematic.tree import link_world_transforms
from cad2urdf.gui.state.controller import RobotController
from cad2urdf.gui.viewport.style import ViewportStyle


class ViewportWidget(QWidget):
    """QWidget wrapping a pyvista plotter with one actor per robot link."""

    linkPicked = pyqtSignal(str)  # noqa: N815

    def __init__(self, controller: RobotController, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._controller = controller
        self._actors: dict[str, Any] = {}
        self._selected: str | None = None

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

        # Studio look (background gradient, lights, env map, floor, AA/SSAO).
        # Heavy GPU passes are gated on a real display inside apply_scene.
        # The return flag says whether the env map landed; without it PBR metal
        # reads flat, so links fall back to a lighter, rougher metal.
        self._env_ok = ViewportStyle.apply_scene(self.plotter, offscreen=pv.OFF_SCREEN)
        # Keep a small orientation axes widget regardless of platform.
        with contextlib.suppress(Exception):
            self.plotter.add_axes()

        controller.robotChanged.connect(self._rebuild)
        self._rebuild(controller.current())

        # Mesh picking. pyvistaqt routes left-clicks on actors through this callback.
        with contextlib.suppress(Exception):
            self.plotter.enable_mesh_picking(
                callback=self.handle_pick, show=False, show_message=False
            )

    # ---- public API --------------------------------------------------------

    def actors_by_link_name(self) -> dict[str, Any]:
        """Return a copy of the {link_name: actor} dict."""
        return dict(self._actors)

    def handle_pick(self, picked_actor: Any) -> None:
        """Resolve a picked actor to a link name and emit linkPicked."""
        from cad2urdf.gui.viewport.pick import resolve_picked_link

        name = resolve_picked_link(picked_actor, self._actors)
        if name is not None:
            self.linkPicked.emit(name)

    def highlight_link(self, name: str | None) -> None:
        """Highlight ``name`` (restyling the previous selection back to normal).

        A no-op for ``None`` and for unknown link names / links without an actor
        (e.g. placeholder links whose mesh path is not absolute). The currently
        selected link is tracked in ``self._selected`` so a rebuild can restore
        the highlight on the freshly created actor.
        """
        # Unknown link with no actor: leave the current selection untouched.
        if name is not None and name not in self._actors:
            return

        # Restore the previously highlighted actor (if it still exists).
        if self._selected is not None and self._selected != name:
            self._apply_style(self._actors.get(self._selected), self._normal_kwargs())

        if name is None:
            self._selected = None
            return

        self._apply_style(self._actors[name], ViewportStyle.highlight_kwargs())
        self._selected = name

    # ---- private -----------------------------------------------------------

    def _normal_kwargs(self) -> dict[str, Any]:
        """Styling for an unselected link: reflective metal when the environment
        map applied, the lighter/rougher fallback metal otherwise."""
        return ViewportStyle.mesh_kwargs() if self._env_ok else ViewportStyle.mesh_kwargs_no_envmap()

    @staticmethod
    def _apply_style(actor: Any, kwargs: dict[str, Any]) -> None:
        """Apply ``add_mesh`` styling kwargs to an existing actor's property."""
        if actor is None:
            return
        prop = actor.prop
        with contextlib.suppress(Exception):
            prop.color = kwargs["color"]
        with contextlib.suppress(Exception):
            prop.metallic = kwargs["metallic"]
        with contextlib.suppress(Exception):
            prop.roughness = kwargs["roughness"]

    def _rebuild(self, robot: Robot) -> None:
        """Replace all actors to match the current robot link set."""
        for actor in list(self._actors.values()):
            self.plotter.remove_actor(actor)
        self._actors.clear()

        # Forward kinematics: place each link's mesh at its world pose, matching
        # the exported URDF / RViz. Without this every mesh renders at its own
        # local origin and the links pile up on top of each other.
        transforms = link_world_transforms(robot)

        for name, link in robot.links.items():
            path = link.visual_mesh_path
            if not path.is_absolute() or not path.is_file():
                continue
            try:
                mesh = pv.read(str(path))
            except Exception:
                continue
            transform = transforms.get(name)
            if transform is not None:
                mesh = mesh.transform(transform, inplace=False)
            actor = self.plotter.add_mesh(
                mesh, name=name, pickable=True, **self._normal_kwargs()
            )
            self._actors[name] = actor

        self.plotter.reset_camera()

        # Re-apply the highlight to the still-selected link on the new actor.
        if self._selected is not None:
            actor = self._actors.get(self._selected)
            if actor is None:
                self._selected = None
            else:
                self._apply_style(actor, ViewportStyle.highlight_kwargs())
