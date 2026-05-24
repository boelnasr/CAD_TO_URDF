"""QApplication entrypoint. Exposed as the `cad2urdf-gui` console script."""

from __future__ import annotations

import sys

from PyQt6.QtWidgets import QApplication


def main(argv: list[str] | None = None) -> int:
    app = QApplication(argv if argv is not None else sys.argv)
    try:
        from cad2urdf.gui.windows.main_window import MainWindow
    except ImportError as e:
        # MainWindow lands in Task 3.1 of the v1 GUI plan. Until then, surface
        # the situation honestly instead of crashing with ModuleNotFoundError.
        print(
            f"cad2urdf-gui: MainWindow not available yet ({e}). "
            "This entry point becomes functional once Task 3.1 of the v1 GUI plan ships.",
            file=sys.stderr,
        )
        return 1

    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
