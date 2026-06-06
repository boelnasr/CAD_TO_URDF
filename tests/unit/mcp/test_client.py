import json
import socket
import threading

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
