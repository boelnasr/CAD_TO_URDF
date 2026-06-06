"""CommandDispatcher: turns a wire command dict into controller/core calls.

Pure of any socket code so it can be unit-tested headlessly. Each command is a
method named ``_cmd_<name>`` taking the ``args`` dict and returning a JSON-able
result; the public ``dispatch`` wraps them in ok/err envelopes and never raises.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from dataclasses import replace as dc_replace
from pathlib import Path
from typing import Any

from cad2urdf.core.inertia.materials import list_materials, lookup
from cad2urdf.core.kinematic.model import Robot
from cad2urdf.core.kinematic.tree import remove_link, rename_link, set_base_link
from cad2urdf.core.project.save import robot_to_payload
from cad2urdf.gui.control import protocol
from cad2urdf.gui.state.controller import RobotController
from cad2urdf.gui.workers.import_meshes import build_import_job

_log = logging.getLogger(__name__)


class CommandDispatcher:
    """Dispatch wire commands onto a RobotController + core functions."""

    def __init__(
        self,
        controller: RobotController,
        *,
        grab_png: Callable[[], bytes] | None = None,
    ) -> None:
        self._controller = controller
        # Injected so the dispatcher stays headlessly testable; the real server
        # supplies a window-grab callback.
        self._grab_png = grab_png

    def dispatch(self, message: dict[str, Any]) -> dict[str, Any]:
        """Execute one command message, returning an ok/err envelope."""
        command = message.get("command")
        if command not in protocol.COMMANDS:
            return protocol.err("unknown_command", f"no such command: {command!r}")
        raw_args = message.get("args")
        args = raw_args if isinstance(raw_args, dict) else {}
        handler = getattr(self, f"_cmd_{command}", None)
        if handler is None:  # pragma: no cover - registry/handler drift guard
            return protocol.err("not_implemented", f"command {command!r} has no handler")
        try:
            return protocol.ok(handler(args))
        except (ValueError, KeyError, FileNotFoundError, TypeError) as e:
            return protocol.err(type(e).__name__, str(e))
        except Exception as e:  # noqa: BLE001 - never let a command crash the GUI
            _log.exception("command %r failed", command)
            return protocol.err("internal_error", str(e))

    # ---- introspection ------------------------------------------------------
    def _cmd_get_robot(self, args: dict[str, Any]) -> dict[str, Any]:
        return robot_to_payload(self._controller.current())

    def _cmd_list_materials(self, args: dict[str, Any]) -> list[str]:
        return list_materials()

    def _cmd_get_history(self, args: dict[str, Any]) -> dict[str, bool]:
        return {
            "can_undo": self._controller.can_undo(),
            "can_redo": self._controller.can_redo(),
        }

    def _cmd_gui_status(self, args: dict[str, Any]) -> dict[str, Any]:
        robot = self._controller.current()
        return {"running": True, "robot_name": robot.name, "link_count": len(robot.links)}

    # ---- build --------------------------------------------------------------
    def _cmd_import_meshes(self, args: dict[str, Any]) -> dict[str, Any]:
        paths = [Path(p) for p in args["paths"]]
        robot_name = args.get("robot_name", "robot")
        job = build_import_job(paths=paths, robot_name=robot_name)
        robot = job(lambda c, t, m: None)  # run synchronously on the Qt thread
        self._controller.replace(robot)
        return robot_to_payload(self._controller.current())

    def _cmd_rename_link(self, args: dict[str, Any]) -> dict[str, Any]:
        old, new = args["old"], args["new"]
        self._controller.apply(lambda r: rename_link(r, old, new), label=f"rename {old}")
        return robot_to_payload(self._controller.current())

    def _cmd_remove_link(self, args: dict[str, Any]) -> dict[str, Any]:
        name = args["link"]
        self._controller.apply(lambda r: remove_link(r, name), label=f"remove {name}")
        return robot_to_payload(self._controller.current())

    def _cmd_set_base_link(self, args: dict[str, Any]) -> dict[str, Any]:
        name = args["link"]
        self._controller.apply(lambda r: set_base_link(r, name), label=f"base {name}")
        return robot_to_payload(self._controller.current())

    def _cmd_set_link_material(self, args: dict[str, Any]) -> dict[str, Any]:
        link_name, material_name = args["link"], args["material"]
        material = lookup(material_name)  # raises KeyError on unknown material

        def transform(robot: Robot) -> Robot:
            old = robot.links[link_name]
            new_link = dc_replace(
                old, material_density=material.density_kg_m3, material_name=material.name
            )
            new_links = {**robot.links, link_name: new_link}
            return dc_replace(robot, links=new_links, joints=dict(robot.joints))

        self._controller.apply(transform, label=f"material {link_name}")
        return robot_to_payload(self._controller.current())
