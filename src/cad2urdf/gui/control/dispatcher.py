"""CommandDispatcher: turns a wire command dict into controller/core calls.

Pure of any socket code so it can be unit-tested headlessly. Each command is a
method named ``_cmd_<name>`` taking the ``args`` dict and returning a JSON-able
result; the public ``dispatch`` wraps them in ok/err envelopes and never raises.
"""

from __future__ import annotations

import base64
import logging
from collections.abc import Callable
from dataclasses import asdict, replace as dc_replace
from pathlib import Path
from typing import Any

import numpy as np

from cad2urdf.core.config.loader import origin_from_xyz_rpy
from cad2urdf.core.inertia.materials import list_materials, lookup
from cad2urdf.core.kinematic.model import Joint, Robot
from cad2urdf.core.kinematic.tree import remove_link, rename_link, reparent_joint, set_base_link
from cad2urdf.core.project.save import load_project, robot_to_payload, save_project
from cad2urdf.gui.control import protocol
from cad2urdf.gui.state.controller import RobotController
from cad2urdf.gui.workers.export_package import build_export_job
from cad2urdf.gui.workers.import_meshes import build_import_job
from cad2urdf.gui.workers.validate import build_validate_job

_log = logging.getLogger(__name__)


class _NoWindow(RuntimeError):
    """Raised when an operation requires a live window that has not been wired."""


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
        except _NoWindow as e:
            return protocol.err("no_window", str(e))
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

    # ---- joints -------------------------------------------------------------
    def _cmd_update_joint(self, args: dict[str, Any]) -> dict[str, Any]:
        joint_name = args["joint"]
        new_parent = args.get("parent")
        jtype = args.get("type")
        axis_in = args.get("axis")
        xyz = args.get("origin_xyz")
        rpy = args.get("origin_rpy")
        if (xyz is None) != (rpy is None):
            raise ValueError("origin_xyz and origin_rpy must be provided together")

        def transform(robot: Robot) -> Robot:
            work = reparent_joint(robot, joint_name, new_parent) if new_parent else robot
            old = work.joints[joint_name]
            if axis_in is None:
                axis = old.axis
            else:
                vec = np.asarray(axis_in, dtype=float)
                norm = float(np.linalg.norm(vec))
                if norm == 0.0:
                    raise ValueError("axis must be non-zero")
                axis = vec / norm
            origin = old.origin if xyz is None else origin_from_xyz_rpy(list(xyz), list(rpy))
            new_joint = Joint(
                name=old.name,
                type=jtype or old.type,
                parent=old.parent,
                child=old.child,
                axis=axis,
                origin=origin,
                # absent key → keep old value; explicit null → clear it
                limit_lower=args.get("limit_lower", old.limit_lower),
                limit_upper=args.get("limit_upper", old.limit_upper),
                effort=args.get("effort", old.effort),
                velocity=args.get("velocity", old.velocity),
            )
            new_joints = dict(work.joints)
            new_joints[joint_name] = new_joint
            return Robot(
                name=work.name,
                base_link=work.base_link,
                links=dict(work.links),
                joints=new_joints,
            )

        self._controller.apply(transform, label=f"edit joint {joint_name}")
        return robot_to_payload(self._controller.current())

    # ---- project / export ---------------------------------------------------
    def _cmd_save_project(self, args: dict[str, Any]) -> dict[str, Any]:
        save_project(self._controller.current(), Path(args["path"]))
        return {"saved": args["path"]}

    def _cmd_open_project(self, args: dict[str, Any]) -> dict[str, Any]:
        robot = load_project(Path(args["path"]))
        self._controller.replace(robot)
        return robot_to_payload(self._controller.current())

    def _cmd_validate(self, args: dict[str, Any]) -> dict[str, Any]:
        robot = self._controller.current()
        job = build_validate_job(
            robot=robot,
            out_dir=Path(args["out_dir"]),
            package_name=args["package_name"],
            urdf_relname=f"{robot.name}.urdf",
            run_manipulapy=bool(args.get("run_manipulapy", False)),
        )
        report = job(lambda c, t, m: None)
        return {
            "ast_issues": [asdict(i) for i in report.ast_issues],
            "urdf_written": report.urdf_written,
            "urdf_path": str(report.urdf_path),
            "manipulapy_ok": report.manipulapy_ok,
            "manipulapy_error": report.manipulapy_error,
        }

    def _cmd_export_package(self, args: dict[str, Any]) -> dict[str, Any]:
        job = build_export_job(
            robot=self._controller.current(),
            out_dir=Path(args["out_dir"]),
            package_name=args["package_name"],
            maintainer=args.get("maintainer", "cad2urdf-user"),
            maintainer_email=args.get("maintainer_email", "user@example.com"),
            run_manipulapy=bool(args.get("run_manipulapy", False)),
        )
        report = job(lambda c, t, m: None)
        return {
            "urdf_path": str(report.urdf_path),
            "manipulapy_ok": report.manipulapy_ok,
            "manipulapy_error": report.manipulapy_error,
        }

    # ---- control / visual ---------------------------------------------------
    def _cmd_undo(self, args: dict[str, Any]) -> dict[str, Any]:
        self._controller.undo()
        return robot_to_payload(self._controller.current())

    def _cmd_redo(self, args: dict[str, Any]) -> dict[str, Any]:
        self._controller.redo()
        return robot_to_payload(self._controller.current())

    def _cmd_screenshot(self, args: dict[str, Any]) -> dict[str, str]:
        if self._grab_png is None:
            raise _NoWindow("no live window to screenshot")
        png = self._grab_png()
        return {"png_base64": base64.b64encode(png).decode("ascii")}
