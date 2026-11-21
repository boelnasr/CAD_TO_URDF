import json
import socket
import threading
import time
from dataclasses import replace

from PyQt6.QtNetwork import QLocalSocket

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.control.server import ControlServer
from cad2urdf.gui.state.cad_assets import CadAssetOwner
from cad2urdf.gui.state.controller import RobotController
from cad2urdf.gui.state.empty_robot import make_empty_robot


def _pump_until(qapp, predicate, timeout=2.0):
    """Spin the Qt event loop until predicate() is true or timeout elapses.

    The server runs on the main thread, so the event loop MUST be pumped for it
    to accept connections and answer — we cannot use blocking waitFor* here.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        qapp.processEvents()
        if predicate():
            return True
        time.sleep(0.005)
    qapp.processEvents()
    return predicate()


def _connected(sock):
    return sock.state() == QLocalSocket.LocalSocketState.ConnectedState


def _request(qapp, socket_name, message):
    sock = QLocalSocket()
    sock.connectToServer(socket_name)
    assert _pump_until(qapp, lambda: _connected(sock)), "could not connect to ControlServer"
    sock.write((json.dumps(message) + "\n").encode("utf-8"))
    sock.flush()
    assert _pump_until(qapp, sock.canReadLine), "no response from ControlServer"
    line = bytes(sock.readLine()).decode("utf-8").strip()
    sock.disconnectFromServer()
    return json.loads(line)


def test_server_roundtrips_get_robot(qapp, tmp_path):
    name = str(tmp_path / "ctl.sock")
    server = ControlServer(CommandDispatcher(RobotController()), name)
    server.start()
    try:
        resp = _request(qapp, name, {"command": "get_robot", "args": {}})
        assert resp["ok"] is True
        assert resp["result"]["schema_version"] == 1
    finally:
        server.stop()


def test_server_rejects_bad_json(qapp, tmp_path):
    name = str(tmp_path / "ctl2.sock")
    server = ControlServer(CommandDispatcher(RobotController()), name)
    server.start()
    try:
        sock = QLocalSocket()
        sock.connectToServer(name)
        assert _pump_until(qapp, lambda: _connected(sock))
        sock.write(b"{not json}\n")
        sock.flush()
        assert _pump_until(qapp, sock.canReadLine)
        resp = json.loads(bytes(sock.readLine()).decode("utf-8").strip())
        assert resp["ok"] is False
        assert resp["error"] == "protocol_error"
        sock.disconnectFromServer()
    finally:
        server.stop()


def test_pipelined_command_waits_for_blocked_import_and_preserves_response_order(
    monkeypatch, qapp, tmp_path
):
    """Nested Qt events must not dispatch a later request ahead of import_cad."""
    socket_name = str(tmp_path / "ordered.sock")
    controller = RobotController()
    import_started = threading.Event()
    release_import = threading.Event()
    client_done = threading.Event()
    responses: list[dict] = []
    client_errors: list[BaseException] = []

    def blocking_builder(*, path, mesh_dir, robot_name):
        def job(_report):
            import_started.set()
            assert release_import.wait(timeout=2.0)
            return replace(make_empty_robot(), name=robot_name)

        return job

    monkeypatch.setattr(
        "cad2urdf.gui.control.dispatcher.build_cad_import_job", blocking_builder
    )
    server = ControlServer(CommandDispatcher(controller), socket_name)
    server.start()

    def client() -> None:
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.settimeout(1.0)
                sock.connect(socket_name)
                sock.sendall(
                    json.dumps(
                        {
                            "command": "import_cad",
                            "args": {"path": str(tmp_path / "robot.iges")},
                        }
                    ).encode()
                    + b"\n"
                )
                assert import_started.wait(timeout=2.0)
                sock.sendall(
                    json.dumps(
                        {
                            "command": "rename_link",
                            "args": {"old": "base", "new": "renamed"},
                        }
                    ).encode()
                    + b"\n"
                )
                time.sleep(0.05)
                release_import.set()
                data = b""
                try:
                    while data.count(b"\n") < 2:
                        chunk = sock.recv(65536)
                        if not chunk:
                            break
                        data += chunk
                except TimeoutError:
                    pass
                responses.extend(json.loads(line) for line in data.splitlines())
        except BaseException as error:
            client_errors.append(error)
        finally:
            client_done.set()

    thread = threading.Thread(target=client, daemon=True)
    thread.start()
    try:
        assert _pump_until(qapp, client_done.is_set, timeout=4.0)
        thread.join(timeout=1.0)
        assert client_errors == []
        assert [response["result"]["name"] for response in responses] == [
            "robot",
            "robot",
        ]
        assert "renamed" not in {
            link["name"] for link in responses[0]["result"]["links"]
        }
        assert "renamed" in {
            link["name"] for link in responses[1]["result"]["links"]
        }
        assert set(controller.current().links) == {"renamed"}
    finally:
        release_import.set()
        server.stop()
        controller.shutdown()


def test_shutdown_rejects_queued_commands_before_releasing_blocked_import(
    monkeypatch, qapp, tmp_path
):
    """Closing during a nested import must reject, not execute, queued work."""
    socket_name = str(tmp_path / "closing.sock")
    owned_root = tmp_path / "owned"
    controller = RobotController(cad_assets=CadAssetOwner(root=owned_root))
    dispatcher = CommandDispatcher(controller)
    import_started = threading.Event()
    release_import = threading.Event()
    later_handlers: list[str] = []
    mesh_dirs = []

    def blocking_builder(*, path, mesh_dir, robot_name):
        mesh_dirs.append(mesh_dir)

        def job(_report):
            import_started.set()
            assert release_import.wait(timeout=2.0)
            (mesh_dir / "finished.stl").write_text("finished")
            return replace(make_empty_robot(), name=robot_name)

        return job

    def forbidden_rename(_args):
        later_handlers.append("rename")
        return {}

    def forbidden_export(_args):
        later_handlers.append("export")
        return {}

    monkeypatch.setattr(
        "cad2urdf.gui.control.dispatcher.build_cad_import_job", blocking_builder
    )
    monkeypatch.setattr(dispatcher, "_cmd_rename_link", forbidden_rename)
    monkeypatch.setattr(dispatcher, "_cmd_export_package", forbidden_export)
    server = ControlServer(dispatcher, socket_name)
    server.start()
    sock = QLocalSocket()
    sock.connectToServer(socket_name)
    assert _pump_until(qapp, lambda: _connected(sock))
    requests = [
        {
            "command": "import_cad",
            "args": {"path": str(tmp_path / "robot.iges")},
        },
        {
            "command": "rename_link",
            "args": {"old": "base", "new": "renamed"},
        },
        {
            "command": "export_package",
            "args": {
                "out_dir": str(tmp_path / "out"),
                "package_name": "robot_description",
            },
        },
    ]
    sock.write(
        b"".join((json.dumps(request) + "\n").encode() for request in requests)
    )
    sock.flush()

    def close_when_import_is_blocked() -> None:
        if not import_started.is_set():
            from PyQt6.QtCore import QTimer

            QTimer.singleShot(1, close_when_import_is_blocked)
            return
        server.stop()
        controller.shutdown()
        release_import.set()

    from PyQt6.QtCore import QTimer

    QTimer.singleShot(0, close_when_import_is_blocked)
    try:
        assert _pump_until(
            qapp,
            lambda: bytes(sock.peek(sock.bytesAvailable())).count(b"\n")
            == len(requests),
            timeout=4.0,
        )
        responses = [
            json.loads(bytes(sock.readLine()).decode())
            for _request in requests
        ]
        assert len(responses) == 3
        assert all(response["ok"] is False for response in responses)
        assert {response["error"] for response in responses} == {"shutting_down"}
        assert later_handlers == []
        assert mesh_dirs and not mesh_dirs[0].exists()
        assert not any(owned_root.iterdir())
    finally:
        release_import.set()
        sock.disconnectFromServer()
        server.stop()
        controller.shutdown()


def test_server_wraps_dispatcher_exception_in_err_envelope_issue19(qapp, tmp_path):
    """A raising dispatcher must yield an err line, not an exception in the slot."""

    class _RaisingDispatcher:
        def dispatch(self, message):
            raise TypeError("unhashable type: 'list'")

        def begin_shutdown(self):
            pass

    name = str(tmp_path / "raising.sock")
    server = ControlServer(_RaisingDispatcher(), name)  # type: ignore[arg-type]
    server.start()
    try:
        resp = _request(qapp, name, {"command": ["gui_status"], "args": {}})
        assert resp["ok"] is False
        assert resp["error"] == "internal_error"
        assert "unhashable" in resp["detail"]
    finally:
        server.stop()


def test_client_disconnect_during_nested_loop_drops_response_issue20(qapp, tmp_path):
    """The client may vanish while a handler (import_cad) spins a nested loop.

    That loop used to delete the connection's QLocalSocket while its readyRead
    emission was still on the stack, then the response was written to the freed
    object: a RuntimeError in a Qt slot (GUI abort) or a segfault.
    """
    from PyQt6 import sip
    from PyQt6.QtCore import QEventLoop, QTimer

    from cad2urdf.gui.control import protocol

    name = str(tmp_path / "vanish.sock")
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    seen: dict[str, bool] = {"entered": False, "disconnect_in_loop": False}
    conns: list[QLocalSocket] = []

    class _NestedLoopDispatcher:
        """Stands in for import_cad's nested worker loop."""

        def dispatch(self, message):
            if message["command"] != "import_cad":
                return protocol.ok({"pong": True})
            seen["entered"] = True
            conns.extend(server._buffers)
            client.close()  # the MCP process dies mid-import
            loop = QEventLoop()
            deadline = time.monotonic() + 1.0

            def poll() -> None:
                if not any(c in server._buffers for c in conns):
                    seen["disconnect_in_loop"] = True
                    # A few more iterations, so a deleteLater() posted by the
                    # disconnect handler gets its chance to run in this loop.
                    QTimer.singleShot(20, loop.quit)
                elif time.monotonic() > deadline:
                    loop.quit()
                else:
                    QTimer.singleShot(5, poll)

            QTimer.singleShot(0, poll)
            loop.exec()
            return protocol.ok({"imported": True})

        def begin_shutdown(self):
            pass

    server = ControlServer(_NestedLoopDispatcher(), name)  # type: ignore[arg-type]
    server.start()
    try:
        client.connect(name)
        client.sendall(b'{"command": "import_cad", "args": {"path": "x.step"}}\n')
        assert _pump_until(qapp, lambda: seen["entered"]), "handler never ran"
        assert seen["disconnect_in_loop"], "disconnect was not handled during dispatch"
        # The vanished connection is still cleaned up once the handler returns.
        assert _pump_until(qapp, lambda: all(sip.isdeleted(c) for c in conns))

        resp = _request(qapp, name, {"command": "gui_status", "args": {}})
        assert resp == {"ok": True, "result": {"pong": True}}
    finally:
        client.close()
        server.stop()
