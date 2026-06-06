import json
import time

from PyQt6.QtNetwork import QLocalSocket

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.control.server import ControlServer
from cad2urdf.gui.state.controller import RobotController


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
