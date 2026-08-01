"""ControlServer: a QLocalServer that feeds JSON-line commands to the dispatcher.

Runs entirely on the Qt main thread, so handlers may touch the controller and
widgets directly. One command per line; one response line per command. A client
may pipeline multiple commands; partial lines are buffered per connection.
"""

from __future__ import annotations

import logging
from collections import deque

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
        self._pending: deque[tuple[QLocalSocket, bytes]] = deque()
        self._dispatching = False
        self._stopping = False
        self._server.newConnection.connect(self._on_new_connection)

    def start(self) -> None:
        """Begin listening. Removes any stale socket file first."""
        self._stopping = False
        QLocalServer.removeServer(self._socket_name)
        if not self._server.listen(self._socket_name):
            raise RuntimeError(
                f"ControlServer could not listen on {self._socket_name!r}: "
                f"{self._server.errorString()}"
            )
        _log.info("ControlServer listening on %s", self._socket_name)

    def stop(self) -> None:
        """Stop accepting work and reject anything queued behind a handler.

        A CAD handler runs a nested Qt event loop while its worker is active.
        Close can therefore arrive while :meth:`_drain_pending` is on the
        stack. In that case connections stay alive until the active request and
        all already-framed queued requests receive FIFO shutdown errors.
        """
        if self._stopping:
            return
        self._stopping = True
        self._dispatcher.begin_shutdown()
        self._server.close()
        QLocalServer.removeServer(self._socket_name)
        for conn in list(self._buffers):
            if conn.bytesAvailable():
                self._collect_available(conn)
        if not self._dispatching:
            self._reject_pending()
            self._finish_stop()

    def _reject_pending(self) -> None:
        while self._pending:
            conn, _raw = self._pending.popleft()
            if conn in self._buffers:
                self._write_response(
                    conn,
                    protocol.err(
                        "shutting_down",
                        "GUI is shutting down; command was not executed",
                    ),
                )

    def _finish_stop(self) -> None:
        for conn in list(self._buffers):
            conn.flush()
            conn.disconnectFromServer()
        self._buffers.clear()

    @staticmethod
    def _write_response(conn: QLocalSocket, response: dict[str, object]) -> None:
        conn.write(protocol.encode(response).encode("utf-8"))
        conn.flush()

    # ---- connection handling ------------------------------------------------
    def _on_new_connection(self) -> None:
        conn = self._server.nextPendingConnection()
        if conn is None:
            return
        if self._stopping:
            conn.disconnectFromServer()
            conn.deleteLater()
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
        self._collect_available(conn)
        if self._stopping:
            if not self._dispatching:
                self._reject_pending()
                self._finish_stop()
            return
        self._drain_pending()

    def _collect_available(self, conn: QLocalSocket) -> None:
        """Read all currently available bytes and enqueue every complete line."""
        self._buffers[conn] += bytes(conn.readAll())
        while b"\n" in self._buffers[conn]:
            raw, _, rest = self._buffers[conn].partition(b"\n")
            self._buffers[conn] = rest
            self._pending.append((conn, raw))

    def _drain_pending(self) -> None:
        """Dispatch complete requests FIFO, without nested-loop reentrancy."""
        if self._dispatching:
            return
        if self._stopping:
            self._reject_pending()
            self._finish_stop()
            return
        self._dispatching = True
        try:
            while self._pending:
                conn, raw = self._pending.popleft()
                if conn not in self._buffers:
                    continue
                response = self._response_for(raw)
                if self._stopping:
                    response = protocol.err(
                        "shutting_down",
                        "GUI is shutting down; command was not completed",
                    )
                self._write_response(conn, response)
                if self._stopping:
                    self._reject_pending()
                    break
                # A nested event loop may coalesce readyRead while this slot is
                # active. Poll after the response so those bytes cannot strand.
                for open_conn in list(self._buffers):
                    if open_conn.bytesAvailable():
                        self._collect_available(open_conn)
        finally:
            self._dispatching = False
            if self._stopping:
                self._finish_stop()

    def _response_for(self, raw: bytes) -> dict[str, object]:
        try:
            message = protocol.decode(raw.decode("utf-8"))
        except (protocol.ProtocolError, UnicodeDecodeError) as e:
            return protocol.err("protocol_error", str(e))
        return self._dispatcher.dispatch(message)
