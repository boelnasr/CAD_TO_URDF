"""Wire protocol shared by the GUI ControlServer and the MCP client.

Messages are newline-delimited JSON objects. A request is
``{"command": <name>, "args": {...}}``; a response is an ``ok``/``err`` envelope.
"""

from __future__ import annotations

import json
from typing import Any

# Frozen set of every command the dispatcher understands. The server rejects
# anything not listed here before dispatch.
COMMANDS: frozenset[str] = frozenset(
    {
        # introspection
        "get_robot",
        "list_materials",
        "get_history",
        "gui_status",
        # build
        "import_meshes",
        "set_base_link",
        "rename_link",
        "remove_link",
        "set_link_material",
        # joints
        "update_joint",
        # project / export
        "save_project",
        "open_project",
        "validate",
        "export_package",
        # control / visual
        "undo",
        "redo",
        "screenshot",
    }
)


class ProtocolError(ValueError):
    """Raised on malformed wire messages."""


def encode(obj: dict[str, Any]) -> str:
    """Serialize a message to a single newline-terminated JSON line."""
    return json.dumps(obj) + "\n"


def decode(line: str) -> dict[str, Any]:
    """Parse one JSON line into a dict, rejecting non-object payloads."""
    if not line.strip():
        raise ProtocolError("empty message line")
    try:
        parsed = json.loads(line)
    except json.JSONDecodeError as e:
        raise ProtocolError(f"invalid JSON: {e}") from e
    if not isinstance(parsed, dict):
        raise ProtocolError(f"message must be a JSON object, got {type(parsed).__name__}")
    return parsed


def ok(result: Any) -> dict[str, Any]:
    """Build a success envelope."""
    return {"ok": True, "result": result}


def err(error: str, detail: str) -> dict[str, Any]:
    """Build an error envelope."""
    return {"ok": False, "error": error, "detail": detail}
