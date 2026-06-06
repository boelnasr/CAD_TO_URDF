"""FastMCP server exposing the cad2urdf GUI over a semantic command bridge."""

from __future__ import annotations

import base64
import os
import shutil
import tempfile
from typing import Any

from mcp.server.fastmcp import FastMCP, Image

from cad2urdf.mcp.client import ControlClient
from cad2urdf.mcp.gui_process import GuiProcess


class Bridge:
    """Owns the GUI subprocess + socket client; lazily spawned on first use."""

    def __init__(self) -> None:
        self._socket_path = os.path.join(
            tempfile.mkdtemp(prefix="cad2urdf-mcp-"), "control.sock"
        )
        self._gui: GuiProcess | None = None
        self._client: ControlClient | None = None

    def _ensure_started(self) -> None:
        if self._gui is None:
            gui = GuiProcess(socket_path=self._socket_path)
            gui.start(timeout=30.0)
            self._gui = gui
            self._client = ControlClient(self._socket_path)
            self._client.connect(timeout=10.0)

    def call(self, command: str, args: dict[str, Any]) -> Any:
        """Send a command, returning its result or raising on an error envelope."""
        self._ensure_started()
        if self._client is None:  # pragma: no cover - defensive; partial startup failure
            raise RuntimeError("Bridge client not connected; startup may have failed")
        resp = self._client.send(command, args)
        if not resp.get("ok"):
            raise RuntimeError(f"{resp.get('error')}: {resp.get('detail')}")
        return resp["result"]

    def screenshot_png(self) -> bytes:
        result = self.call("screenshot", {})
        return base64.b64decode(result["png_base64"])

    def shutdown(self) -> None:
        if self._client is not None:
            self._client.close()
        if self._gui is not None:
            self._gui.stop()
        shutil.rmtree(os.path.dirname(self._socket_path), ignore_errors=True)


bridge = Bridge()
mcp = FastMCP("cad2urdf")


@mcp.tool()
def get_robot() -> Any:
    """Return the current robot as a structured payload (links, joints, tree)."""
    return bridge.call("get_robot", {})


@mcp.tool()
def list_materials() -> Any:
    """List available material names for set_link_material."""
    return bridge.call("list_materials", {})


@mcp.tool()
def get_history() -> Any:
    """Return undo/redo availability."""
    return bridge.call("get_history", {})


@mcp.tool()
def gui_status() -> Any:
    """Report whether the GUI is running and a brief robot summary."""
    return bridge.call("gui_status", {})


@mcp.tool()
def import_meshes(paths: list[str], robot_name: str = "robot") -> Any:
    """Import STL/OBJ files as links (first becomes the base, others fixed to it)."""
    return bridge.call("import_meshes", {"paths": paths, "robot_name": robot_name})


@mcp.tool()
def set_base_link(link: str) -> Any:
    """Re-root the kinematic tree at the given link."""
    return bridge.call("set_base_link", {"link": link})


@mcp.tool()
def rename_link(old: str, new: str) -> Any:
    """Rename a link and update all joints referencing it."""
    return bridge.call("rename_link", {"old": old, "new": new})


@mcp.tool()
def remove_link(link: str) -> Any:
    """Remove a link and its entire subtree."""
    return bridge.call("remove_link", {"link": link})


@mcp.tool()
def set_link_material(link: str, material: str) -> Any:
    """Set a link's material (name + density) from the material table."""
    return bridge.call("set_link_material", {"link": link, "material": material})


@mcp.tool()
def update_joint(
    joint: str,
    type: str | None = None,
    axis: list[float] | None = None,
    origin_xyz: list[float] | None = None,
    origin_rpy: list[float] | None = None,
    limit_lower: float | None = None,
    limit_upper: float | None = None,
    effort: float | None = None,
    velocity: float | None = None,
    parent: str | None = None,
) -> Any:
    """Edit a joint: type/axis/origin/limits, or reparent via `parent`.

    Provide origin_xyz and origin_rpy together (or neither). A robot is a tree, so
    joints are created by import_meshes and removed by remove_link.
    """
    args: dict[str, Any] = {"joint": joint}
    for key, value in (
        ("type", type),
        ("axis", axis),
        ("origin_xyz", origin_xyz),
        ("origin_rpy", origin_rpy),
        ("limit_lower", limit_lower),
        ("limit_upper", limit_upper),
        ("effort", effort),
        ("velocity", velocity),
        ("parent", parent),
    ):
        if value is not None:
            args[key] = value
    return bridge.call("update_joint", args)


@mcp.tool()
def save_project(path: str) -> Any:
    """Save the current robot to a .cad2urdf JSON project file."""
    return bridge.call("save_project", {"path": path})


@mcp.tool()
def open_project(path: str) -> Any:
    """Load a .cad2urdf project, replacing the current robot."""
    return bridge.call("open_project", {"path": path})


@mcp.tool()
def validate(out_dir: str, package_name: str, run_manipulapy: bool = False) -> Any:
    """Write the URDF and run AST (and optional ManipulaPy) validation."""
    return bridge.call(
        "validate",
        {"out_dir": out_dir, "package_name": package_name, "run_manipulapy": run_manipulapy},
    )


@mcp.tool()
def export_package(
    out_dir: str,
    package_name: str,
    maintainer: str = "cad2urdf-user",
    maintainer_email: str = "user@example.com",
    run_manipulapy: bool = False,
) -> Any:
    """Export the full ROS 2 package (URDF + meshes + scaffolding) to out_dir."""
    return bridge.call(
        "export_package",
        {
            "out_dir": out_dir,
            "package_name": package_name,
            "maintainer": maintainer,
            "maintainer_email": maintainer_email,
            "run_manipulapy": run_manipulapy,
        },
    )


@mcp.tool()
def undo() -> Any:
    """Undo the last change."""
    return bridge.call("undo", {})


@mcp.tool()
def redo() -> Any:
    """Redo the last undone change."""
    return bridge.call("redo", {})


@mcp.tool()
def screenshot() -> Image:
    """Capture the live GUI window as a PNG image."""
    return Image(data=bridge.screenshot_png(), format="png")


def run() -> None:
    """Entry point: serve MCP over stdio until the client disconnects."""
    try:
        mcp.run()
    finally:
        bridge.shutdown()
