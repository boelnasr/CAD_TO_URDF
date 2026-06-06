import pytest

pytest.importorskip("mcp")  # requires the [mcp] extra

from cad2urdf.mcp import server as mcp_server


class _FakeClient:
    def __init__(self):
        self.sent = []

    def send(self, command, args):
        self.sent.append((command, args))
        if command == "get_robot":
            return {"ok": True, "result": {"name": "rig", "links": [], "joints": []}}
        if command == "screenshot":
            import base64

            return {"ok": True, "result": {"png_base64": base64.b64encode(b"PNG").decode()}}
        return {"ok": True, "result": {"command": command, "args": args}}


def test_call_command_returns_result(monkeypatch):
    bridge = mcp_server.Bridge()
    monkeypatch.setattr(bridge, "_client", _FakeClient())
    monkeypatch.setattr(bridge, "_ensure_started", lambda: None)
    out = bridge.call("get_robot", {})
    assert out["name"] == "rig"


def test_call_command_raises_on_error(monkeypatch):
    bridge = mcp_server.Bridge()

    class _ErrClient:
        def send(self, command, args):
            return {"ok": False, "error": "ValueError", "detail": "boom"}

    monkeypatch.setattr(bridge, "_client", _ErrClient())
    monkeypatch.setattr(bridge, "_ensure_started", lambda: None)
    with pytest.raises(RuntimeError, match="boom"):
        bridge.call("import_meshes", {"paths": []})


def test_screenshot_png_bytes(monkeypatch):
    bridge = mcp_server.Bridge()
    monkeypatch.setattr(bridge, "_client", _FakeClient())
    monkeypatch.setattr(bridge, "_ensure_started", lambda: None)
    assert bridge.screenshot_png() == b"PNG"
