"""Spawn and supervise the GUI child process for the MCP server."""

from __future__ import annotations

import socket
import subprocess
import sys
import time


class GuiProcess:
    """Launches `cad2urdf-gui --control-socket <path>` and waits for the socket."""

    def __init__(
        self,
        socket_path: str,
        launch_cmd: list[str] | None = None,
    ) -> None:
        self._socket_path = socket_path
        # Default: run the GUI module so we don't depend on PATH for the script.
        self._launch_cmd = launch_cmd or [
            sys.executable,
            "-m",
            "cad2urdf.gui.app",
            "--control-socket",
            socket_path,
        ]
        self._proc: subprocess.Popen[bytes] | None = None

    def start(self, *, timeout: float = 30.0) -> None:
        """Spawn the child and block until its socket accepts connections."""
        if self._proc is not None:
            raise RuntimeError("GUI process already started; call stop() first")
        self._proc = subprocess.Popen(self._launch_cmd)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._proc.poll() is not None:
                code = self._proc.returncode
                self.stop()
                raise RuntimeError(f"GUI process exited early with code {code}")
            if self._socket_connectable():
                return
            time.sleep(0.1)
        self.stop()
        raise TimeoutError(f"GUI socket {self._socket_path!r} did not come up in {timeout}s")

    def _socket_connectable(self) -> bool:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(self._socket_path)
            return True
        except OSError:
            return False
        finally:
            s.close()

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def stop(self) -> None:
        if self._proc is None:
            return
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                try:
                    self._proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    pass
        self._proc = None
