from __future__ import annotations

import asyncio
from pathlib import Path

import pytest

pytest.importorskip("mcp")

from cad2urdf.gui.control import protocol
from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.mcp import server as mcp_server


def test_import_cad_is_synchronized_across_all_four_mcp_surfaces() -> None:
    """Omitting protocol, handler, registration, or README exposure breaks this."""
    assert "import_cad" in protocol.COMMANDS
    assert callable(getattr(CommandDispatcher, "_cmd_import_cad", None))
    tools = asyncio.run(mcp_server.mcp.list_tools())
    assert "import_cad" in {tool.name for tool in tools}

    readme = Path(__file__).resolve().parents[2] / "README.md"
    build_row = next(
        line for line in readme.read_text().splitlines() if line.startswith("| Build |")
    )
    assert "`import_cad`" in build_row


def test_import_cad_mcp_wrapper_sends_one_path(monkeypatch: pytest.MonkeyPatch) -> None:
    """Changing the wrapper command or payload breaks the public MCP contract."""
    calls: list[tuple[str, dict[str, object]]] = []
    monkeypatch.setattr(
        mcp_server.bridge,
        "call",
        lambda command, args: calls.append((command, args)) or {"name": "robot"},
    )

    result = mcp_server.import_cad("/tmp/robot.iges")

    assert result == {"name": "robot"}
    assert calls == [("import_cad", {"path": "/tmp/robot.iges"})]
