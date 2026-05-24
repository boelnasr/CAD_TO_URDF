"""QApplication entrypoint. Exposed as the `cad2urdf-gui` console script."""

from __future__ import annotations

import sys

from PyQt6.QtWidgets import QApplication


def main(argv: list[str] | None = None) -> int:
    app = QApplication(argv if argv is not None else sys.argv)
    # MainWindow import is deferred until Task 3.1 lands. For now we just exit
    # cleanly so the entrypoint is callable from CI (--exit-after-init).
    from cad2urdf.gui.windows.main_window import MainWindow

    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
