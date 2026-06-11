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


def test_call_raises_when_client_none_after_start(monkeypatch):
    # Defensive branch: _ensure_started is a no-op and _client stays None.
    bridge = mcp_server.Bridge()
    monkeypatch.setattr(bridge, "_ensure_started", lambda: None)
    with pytest.raises(RuntimeError, match="startup may have failed"):
        bridge.call("get_robot", {})


def test_shutdown_idempotent_when_never_started():
    # Fresh bridge: both _gui and _client are None; shutdown must not raise.
    bridge = mcp_server.Bridge()
    bridge.shutdown()


def test_shutdown_closes_client_and_stops_gui(monkeypatch):
    import os

    class _RecordingClient:
        def __init__(self):
            self.closed = False

        def close(self):
            self.closed = True

    class _RecordingGui:
        def __init__(self):
            self.stopped = False

        def stop(self):
            self.stopped = True

    bridge = mcp_server.Bridge()
    sock_dir = os.path.dirname(bridge._socket_path)
    client = _RecordingClient()
    gui = _RecordingGui()
    monkeypatch.setattr(bridge, "_client", client)
    monkeypatch.setattr(bridge, "_gui", gui)
    bridge.shutdown()
    assert client.closed
    assert gui.stopped
    assert not os.path.exists(sock_dir)


def test_run_shuts_down_on_exception(monkeypatch):
    calls = []
    monkeypatch.setattr(mcp_server.bridge, "shutdown", lambda: calls.append(1))

    def _boom():
        raise RuntimeError("mcp crashed")

    monkeypatch.setattr(mcp_server.mcp, "run", _boom)
    with pytest.raises(RuntimeError, match="mcp crashed"):
        mcp_server.run()
    assert calls == [1]


def test_run_shuts_down_on_normal_exit(monkeypatch):
    calls = []
    monkeypatch.setattr(mcp_server.bridge, "shutdown", lambda: calls.append(1))
    monkeypatch.setattr(mcp_server.mcp, "run", lambda: None)
    mcp_server.run()
    assert calls == [1]


def _wire_bridge(monkeypatch):
    """Point the module-level bridge at a recording fake; return that fake."""
    fake = _FakeClient()
    monkeypatch.setattr(mcp_server.bridge, "_client", fake)
    monkeypatch.setattr(mcp_server.bridge, "_ensure_started", lambda: None)
    return fake


def test_update_joint_filters_none_and_keeps_provided(monkeypatch):
    fake = _wire_bridge(monkeypatch)
    mcp_server.update_joint("arm_to_base", type="revolute", axis=[0, 0, 1], effort=5.0)
    assert fake.sent[-1] == (
        "update_joint",
        {"joint": "arm_to_base", "type": "revolute", "axis": [0, 0, 1], "effort": 5.0},
    )


def test_update_joint_no_optional_args(monkeypatch):
    fake = _wire_bridge(monkeypatch)
    mcp_server.update_joint("j")
    assert fake.sent[-1] == ("update_joint", {"joint": "j"})


@pytest.mark.parametrize(
    ("invoke", "expected"),
    [
        (lambda: mcp_server.get_robot(), ("get_robot", {})),
        (lambda: mcp_server.list_materials(), ("list_materials", {})),
        (lambda: mcp_server.get_history(), ("get_history", {})),
        (lambda: mcp_server.gui_status(), ("gui_status", {})),
        (
            lambda: mcp_server.import_meshes(["a.stl"]),
            ("import_meshes", {"paths": ["a.stl"], "robot_name": "robot"}),
        ),
        (
            lambda: mcp_server.set_base_link("base"),
            ("set_base_link", {"link": "base"}),
        ),
        (
            lambda: mcp_server.rename_link("a", "b"),
            ("rename_link", {"old": "a", "new": "b"}),
        ),
        (lambda: mcp_server.remove_link("a"), ("remove_link", {"link": "a"})),
        (
            lambda: mcp_server.set_link_material("a", "steel"),
            ("set_link_material", {"link": "a", "material": "steel"}),
        ),
        (
            lambda: mcp_server.save_project("/tmp/p.cad2urdf"),
            ("save_project", {"path": "/tmp/p.cad2urdf"}),
        ),
        (
            lambda: mcp_server.open_project("/tmp/p.cad2urdf"),
            ("open_project", {"path": "/tmp/p.cad2urdf"}),
        ),
        (
            lambda: mcp_server.validate("out", "pkg"),
            (
                "validate",
                {"out_dir": "out", "package_name": "pkg", "run_manipulapy": False},
            ),
        ),
        (
            lambda: mcp_server.export_package("out", "pkg"),
            (
                "export_package",
                {
                    "out_dir": "out",
                    "package_name": "pkg",
                    "maintainer": "cad2urdf-user",
                    "maintainer_email": "user@example.com",
                    "run_manipulapy": False,
                },
            ),
        ),
        (lambda: mcp_server.undo(), ("undo", {})),
        (lambda: mcp_server.redo(), ("redo", {})),
    ],
)
def test_tool_wrapper_protocol_contract(monkeypatch, invoke, expected):
    fake = _wire_bridge(monkeypatch)
    invoke()
    assert fake.sent[-1] == expected


def test_screenshot_tool_returns_png_image(monkeypatch):
    monkeypatch.setattr(mcp_server.bridge, "screenshot_png", lambda: b"PNG")
    img = mcp_server.screenshot()
    assert img.data == b"PNG"
    assert img._format == "png"
