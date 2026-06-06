"""QApplication entrypoint. Exposed as the `cad2urdf-gui` console script."""

from __future__ import annotations

import argparse
import sys

from PyQt6.QtWidgets import QApplication, QWidget


def parse_args(argv: list[str]) -> argparse.Namespace:
    """Parse cad2urdf-gui CLI args."""
    parser = argparse.ArgumentParser(prog="cad2urdf-gui")
    parser.add_argument(
        "--control-socket",
        default=None,
        help="If set, start an MCP control server on this local socket name.",
    )
    return parser.parse_args(argv)


def _grab_png(window: QWidget) -> bytes:
    """Grab the window to PNG bytes (used by the screenshot command)."""
    from PyQt6.QtCore import QBuffer, QIODevice

    pixmap = window.grab()
    image = pixmap.toImage()
    buffer = QBuffer()
    buffer.open(QIODevice.OpenModeFlag.WriteOnly)
    image.save(buffer, "PNG")
    return bytes(buffer.data())


def main(argv: list[str] | None = None) -> int:
    raw = argv if argv is not None else sys.argv[1:]
    ns = parse_args(raw)

    app = QApplication(sys.argv[:1])
    try:
        from cad2urdf.gui.windows.main_window import MainWindow
    except ImportError as e:
        print(f"cad2urdf-gui: MainWindow not available ({e}).", file=sys.stderr)
        return 1

    win = MainWindow()
    win.show()

    server = None
    if ns.control_socket:
        from cad2urdf.gui.control.dispatcher import CommandDispatcher
        from cad2urdf.gui.control.server import ControlServer

        dispatcher = CommandDispatcher(win.controller, grab_png=lambda: _grab_png(win))
        server = ControlServer(dispatcher, ns.control_socket)
        server.start()

    try:
        return app.exec()
    finally:
        if server is not None:
            server.stop()


if __name__ == "__main__":
    raise SystemExit(main())
