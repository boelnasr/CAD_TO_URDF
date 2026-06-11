import json
import socket
import threading

import pytest

from cad2urdf.mcp.client import ControlClient


def _echo_ok_server(sock_path: str) -> None:
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(1)
    srv.settimeout(5.0)
    conn, _ = srv.accept()
    data = b""
    while b"\n" not in data:
        data += conn.recv(4096)
    request = json.loads(data.decode().strip())
    conn.sendall((json.dumps({"ok": True, "result": {"echo": request}}) + "\n").encode())
    conn.close()
    srv.close()


def test_client_send_receives_response(tmp_path):
    sock_path = str(tmp_path / "echo.sock")
    t = threading.Thread(target=_echo_ok_server, args=(sock_path,), daemon=True)
    t.start()

    client = ControlClient(sock_path)
    client.connect(timeout=2.0)
    resp = client.send("get_robot", {})
    client.close()
    t.join(timeout=2.0)

    assert resp["ok"] is True
    assert resp["result"]["echo"]["command"] == "get_robot"
    assert resp["result"]["echo"]["args"] == {}


def test_send_without_connection_raises(tmp_path):
    client = ControlClient(str(tmp_path / "nope.sock"))
    with pytest.raises(RuntimeError, match="client not connected"):
        client.send("get_robot", {})


def test_connect_twice_raises(tmp_path):
    sock_path = str(tmp_path / "echo.sock")
    t = threading.Thread(target=_echo_ok_server, args=(sock_path,), daemon=True)
    t.start()
    client = ControlClient(sock_path)
    client.connect(timeout=2.0)
    try:
        with pytest.raises(RuntimeError, match="already connected"):
            client.connect(timeout=2.0)
    finally:
        client.close()
        t.join(timeout=2.0)


def _accept_then_close_server(sock_path: str) -> None:
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(1)
    srv.settimeout(5.0)
    conn, _ = srv.accept()
    # Drain the request, then close cleanly so the client sees an EOF (not a RST).
    data = b""
    while b"\n" not in data:
        data += conn.recv(4096)
    conn.shutdown(socket.SHUT_WR)
    conn.close()
    srv.close()


def test_send_raises_when_server_closes(tmp_path):
    sock_path = str(tmp_path / "close.sock")
    t = threading.Thread(target=_accept_then_close_server, args=(sock_path,), daemon=True)
    t.start()
    client = ControlClient(sock_path)
    client.connect(timeout=2.0)
    try:
        with pytest.raises(ConnectionError, match="closed the connection"):
            client.send("get_robot", {})
    finally:
        client.close()
        t.join(timeout=2.0)


def test_close_on_never_connected_is_noop(tmp_path):
    client = ControlClient(str(tmp_path / "nope.sock"))
    client.close()  # must not raise
