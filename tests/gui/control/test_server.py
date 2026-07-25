import json
import socket
import threading
import time
from dataclasses import replace

from PyQt6.QtNetwork import QLocalSocket

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.control.server import ControlServer
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
