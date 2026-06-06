"""ControlServer: a QLocalServer that feeds JSON-line commands to the dispatcher.

Runs entirely on the Qt main thread, so handlers may touch the controller and
widgets directly. One command per line; one response line per command. A client
may pipeline multiple commands; partial lines are buffered per connection.
"""

from __future__ import annotations

import logging

from PyQt6.QtCore import QObject
from PyQt6.QtNetwork import QLocalServer, QLocalSocket

from cad2urdf.gui.control import protocol
from cad2urdf.gui.control.dispatcher import CommandDispatcher

_log = logging.getLogger(__name__)


class ControlServer(QObject):
    """Listens on a local socket and dispatches framed JSON commands."""

    def __init__(
        self,
        dispatcher: CommandDispatcher,
        socket_name: str,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._dispatcher = dispatcher
        self._socket_name = socket_name
        self._server = QLocalServer(self)
        self._buffers: dict[QLocalSocket, bytes] = {}
        self._server.newConnection.connect(self._on_new_connection)

    def start(self) -> None:
        """Begin listening. Removes any stale socket file first."""
        QLocalServer.removeServer(self._socket_name)
        if not self._server.listen(self._socket_name):
            raise RuntimeError(
                f"ControlServer could not listen on {self._socket_name!r}: "
                f"{self._server.errorString()}"
            )
        _log.info("ControlServer listening on %s", self._socket_name)

    def stop(self) -> None:
        for conn in list(self._buffers):
            conn.disconnectFromServer()
        self._buffers.clear()
        self._server.close()
        QLocalServer.removeServer(self._socket_name)

    # ---- connection handling ------------------------------------------------
    def _on_new_connection(self) -> None:
        conn = self._server.nextPendingConnection()
        if conn is None:
            return
        self._buffers[conn] = b""
        conn.readyRead.connect(lambda c=conn: self._on_ready_read(c))
        conn.disconnected.connect(lambda c=conn: self._on_disconnected(c))

    def _on_disconnected(self, conn: QLocalSocket) -> None:
        self._buffers.pop(conn, None)
        conn.deleteLater()

    def _on_ready_read(self, conn: QLocalSocket) -> None:
        if conn not in self._buffers:
            return
        self._buffers[conn] += bytes(conn.readAll())
        while b"\n" in self._buffers[conn]:
            raw, _, rest = self._buffers[conn].partition(b"\n")
            self._buffers[conn] = rest
            self._handle_line(conn, raw)

    def _handle_line(self, conn: QLocalSocket, raw: bytes) -> None:
        try:
            message = protocol.decode(raw.decode("utf-8"))
        except (protocol.ProtocolError, UnicodeDecodeError) as e:
            response = protocol.err("protocol_error", str(e))
        else:
            response = self._dispatcher.dispatch(message)
        conn.write(protocol.encode(response).encode("utf-8"))
        conn.flush()
