"""Blocking Unix-domain-socket client speaking the control protocol."""

from __future__ import annotations

import json
import socket
import time
from typing import Any, cast


class ControlClient:
    """One persistent connection to the GUI ControlServer."""

    def __init__(self, socket_path: str) -> None:
        self._socket_path = socket_path
        self._sock: socket.socket | None = None
        self._buffer = b""

    def connect(self, *, timeout: float = 10.0) -> None:
        """Connect, retrying until the server's socket is accepting."""
        if self._sock is not None:
            raise RuntimeError("already connected; call close() first")
        deadline = time.monotonic() + timeout
        last_err: OSError | None = None
        while time.monotonic() < deadline:
            try:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.connect(self._socket_path)
                self._sock = sock
                return
            except OSError as e:  # socket not up yet
                sock.close()
                last_err = e
                time.sleep(0.1)
        raise TimeoutError(f"could not connect to {self._socket_path!r}: {last_err}")

    def send(self, command: str, args: dict[str, Any]) -> dict[str, Any]:
        """Send one command and block for the single-line response."""
        if self._sock is None:
            raise RuntimeError("client not connected")
        payload = json.dumps({"command": command, "args": args}) + "\n"
        self._sock.sendall(payload.encode("utf-8"))
        return self._read_line()

    # NOTE: no read timeout; if the GUI process stalls, recv blocks indefinitely.
    # Acceptable for v1 — the MCP server is the sole caller and controls GUI lifecycle.
    def _read_line(self) -> dict[str, Any]:
        while b"\n" not in self._buffer:
            if self._sock is None:
                raise RuntimeError("client not connected")
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ConnectionError("control server closed the connection")
            self._buffer += chunk
        raw, _, rest = self._buffer.partition(b"\n")
        self._buffer = rest
        return cast(dict[str, Any], json.loads(raw.decode("utf-8")))

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None
