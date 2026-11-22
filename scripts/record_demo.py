"""Record the README demo GIF: build the KUKA LBR iiwa14 in the cad2urdf GUI.

Drives the real MainWindow in-process through CommandDispatcher (the same
command layer the MCP server uses), renders offscreen, and stitches captioned
frames into an animated GIF. Joint origins, axes and limits are read from the
shipped example URDF, so the recording always reflects what the GUI does today.

Run from the repo root (Pillow is needed only by this script, not by cad2urdf):

    uv run --with pillow python scripts/record_demo.py --out docs/assets/demo.gif

No display is required: Qt and PyVista are switched to offscreen rendering.
"""

from __future__ import annotations

import argparse
import io
import os
import shutil
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# Must be set before PyQt6 / pyvista are imported (same as tests/gui/conftest.py).
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("PYVISTA_OFF_SCREEN", "true")

import numpy as np  # noqa: E402
from PyQt6.QtCore import QBuffer, QIODevice, Qt  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as e:  # pragma: no cover - dev tool
    raise SystemExit(
        "record_demo.py needs Pillow: uv run --with pillow python scripts/record_demo.py"
    ) from e

REPO = Path(__file__).resolve().parents[1]
EXAMPLE = REPO / "examples" / "iiwa14" / "iiwa14_description"
WINDOW_PX = (1280, 800)
# Fixed framing for the ~1.3 m tall arm (metres, world frame, z up), so the
# view stays still while links snap into place instead of re-zooming per edit.
CAMERA = [(2.1, -2.3, 1.35), (0.0, 0.0, 0.62), (0.0, 0.0, 1.0)]
FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/Library/Fonts/Arial Bold.ttf",
    "C:/Windows/Fonts/arialbd.ttf",
)


@dataclass(frozen=True)
class JointSpec:
    """One URDF joint: origin in metres / radians (fixed-axis rpy), limits in rad."""

    name: str
    parent: str
    child: str
    xyz: list[float]
    rpy: list[float]
    axis: list[float]
    lower: float
    upper: float
    effort: float
    velocity: float


def read_joints(urdf: Path) -> list[JointSpec]:
    def floats(el: ET.Element | None, attr: str, default: str) -> list[float]:
        raw = default if el is None else el.get(attr, default)
        return [float(v) for v in raw.split()]

    specs = []
    for j in ET.parse(urdf).getroot().iter("joint"):
        limit = j.find("limit")
        assert limit is not None, f"joint {j.get('name')} has no <limit>"
        specs.append(
            JointSpec(
                name=str(j.get("name")),
                parent=str(j.find("parent").get("link")),  # type: ignore[union-attr]
                child=str(j.find("child").get("link")),  # type: ignore[union-attr]
                xyz=floats(j.find("origin"), "xyz", "0 0 0"),
                rpy=floats(j.find("origin"), "rpy", "0 0 0"),
                axis=floats(j.find("axis"), "xyz", "1 0 0"),
                lower=float(limit.get("lower", "0")),
                upper=float(limit.get("upper", "0")),
                effort=float(limit.get("effort", "0")),
                velocity=float(limit.get("velocity", "0")),
            )
        )
    return specs


def load_font(size: int) -> Any:
    for path in FONT_CANDIDATES:
        if Path(path).is_file():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default(size=size)


class Recorder:
    """Grabs the window, pastes the VTK render into the viewport, adds a caption."""

    def __init__(self, app: QApplication, win: Any, width: int) -> None:
        self.app, self.win, self.width = app, win, width
        self.frames: list[Image.Image] = []
        self.durations: list[int] = []
        self.font = load_font(30)
        self.small = load_font(21)

    def _grab_window(self) -> Image.Image:
        buf = QBuffer()
        buf.open(QIODevice.OpenModeFlag.WriteOnly)
        self.win.grab().save(buf, "PNG")
        return Image.open(io.BytesIO(bytes(buf.data()))).convert("RGB")

    def capture(self, caption: str, hold_ms: int, panel: list[str] | None = None) -> None:
        self.app.processEvents()
        frame = self._grab_window()
        vp = self.win.viewport
        # Offscreen, the viewport is a bare pyvista Plotter that Qt cannot grab,
        # so its render is pasted over the viewport's place in the window.
        top_left = vp.mapTo(self.win, vp.rect().topLeft())
        size = (vp.width(), vp.height())
        shot = vp.plotter.screenshot(window_size=size, return_img=True)
        frame.paste(Image.fromarray(np.asarray(shot)).convert("RGB"), (top_left.x(), top_left.y()))
        if panel:
            self._panel(frame, panel, origin=(top_left.x() + 18, top_left.y() + 18))
        frame = self._caption(frame, caption)
        scaled = frame.resize(
            (self.width, round(frame.height * self.width / frame.width)), Image.Resampling.LANCZOS
        )
        self.frames.append(scaled)
        self.durations.append(hold_ms)

    def _caption(self, frame: Image.Image, text: str) -> Image.Image:
        # A strip below the window, so the caption never hides real UI.
        bar_h = 64
        canvas = Image.new("RGB", (frame.width, frame.height + bar_h), (17, 24, 39))
        canvas.paste(frame, (0, 0))
        draw = ImageDraw.Draw(canvas)
        draw.text((28, frame.height + bar_h / 2), text, font=self.font, fill=(255, 255, 255), anchor="lm")
        return canvas

    def _panel(self, frame: Image.Image, lines: list[str], origin: tuple[int, int]) -> None:
        draw = ImageDraw.Draw(frame, "RGBA")
        line_h = 28
        w = max(int(draw.textlength(line, font=self.small)) for line in lines) + 36
        x, y = origin
        draw.rounded_rectangle(
            (x, y, x + w, y + line_h * len(lines) + 24), radius=10, fill=(17, 24, 39, 220)
        )
        for i, line in enumerate(lines):
            draw.text((x + 18, y + 12 + i * line_h), line, font=self.small, fill=(229, 231, 235))

    def save(self, out: Path) -> None:
        out.parent.mkdir(parents=True, exist_ok=True)
        # One adaptive palette per frame, no dithering: crisp UI text and a
        # smaller file than error-diffused gradients.
        pal = [f.quantize(colors=160, dither=Image.Dither.NONE) for f in self.frames]
        pal[0].save(
            out, save_all=True, append_images=pal[1:], duration=self.durations, loop=0, optimize=True
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=REPO / "docs" / "assets" / "demo.gif")
    parser.add_argument("--width", type=int, default=960, help="GIF width in px")
    args = parser.parse_args()

    app = QApplication([])
    from cad2urdf.gui.control.dispatcher import CommandDispatcher
    from cad2urdf.gui.windows.main_window import MainWindow

    win = MainWindow()
    win.resize(*WINDOW_PX)
    win.show()
    # Give the 3D view the room; the inertia dock only needs a few rows.
    win.resizeDocks([win.dock_inertia_editor], [210], Qt.Orientation.Vertical)
    dispatch = CommandDispatcher(win.controller).dispatch
    rec = Recorder(app, win, args.width)

    def run(command: str, **cmd_args: Any) -> Any:
        resp = dispatch({"command": command, "args": cmd_args})
        if not resp["ok"]:
            raise RuntimeError(f"{command} failed: {resp['error']}: {resp.get('detail')}")
        return resp["result"]

    def frame_camera() -> None:
        win.viewport.plotter.camera_position = CAMERA

    joints = read_joints(EXAMPLE / "urdf" / "iiwa14.urdf")
    links = [joints[0].parent] + [j.child for j in joints]

    with tempfile.TemporaryDirectory(prefix="cad2urdf_demo_") as tmp_str:
        tmp = Path(tmp_str)
        meshes = []
        for link in links:
            shutil.copy(EXAMPLE / "meshes" / "visual" / f"{link}.stl", tmp / f"{link}.stl")
            meshes.append(str(tmp / f"{link}.stl"))

        rec.capture("cad2urdf  ·  from CAD meshes to a ROS 2 package", 1400)

        run("import_meshes", paths=meshes, robot_name="iiwa14")
        frame_camera()
        rec.capture("① Import one STL per link  (8 links, all at the origin)", 1800)

        for j in joints:
            # Imported links all hang off the base as '<child>_to_<base>'.
            run(
                "update_joint",
                joint=f"{j.child}_to_{links[0]}",
                parent=j.parent,
                type="revolute",
                axis=j.axis,
                origin_xyz=j.xyz,
                origin_rpy=j.rpy,
                limit_lower=j.lower,
                limit_upper=j.upper,
                effort=j.effort,
                velocity=j.velocity,
            )
            win._select_link_in_tree(j.child)
            frame_camera()
            deg = np.degrees(j.upper)
            rec.capture(f"② Chain {j.parent} → {j.child}:  revolute, ±{deg:.0f}°", 750)

        win._select_link_in_tree(links[-1])
        rec.capture("② The arm assembles at its forward-kinematic pose", 1500)

        cam = win.viewport.plotter.camera
        for _ in range(24):
            cam.Azimuth(15.0)
            rec.capture("③ Inspect it in the 3D viewport", 90)

        win._select_link_in_tree("link_4")
        frame_camera()
        inertia = win.dock_inertia_editor
        inertia.recompute_button.click()  # runs the real recompute worker thread
        deadline = time.monotonic() + 30.0
        while inertia.auto_mass_label.text() == "-" and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(0.02)
        if inertia.auto_mass_label.text() == "-":
            raise RuntimeError("inertia recompute did not finish within 30 s")
        rec.capture("④ Mass, COM and inertia computed from mesh + material", 2400)

        out_pkg = tmp / "iiwa14_description"
        run("export_package", out_dir=str(out_pkg), package_name="iiwa14_description")
        written = sorted(
            str(p.relative_to(out_pkg)) for p in out_pkg.rglob("*") if p.is_file() and "meshes" not in p.parts
        )
        n_meshes = sum(1 for _ in (out_pkg / "meshes").rglob("*.stl"))
        rec.capture(
            "⑤ Export a ready-to-launch ROS 2 package",
            3200,
            panel=["iiwa14_description/", *[f"  {w}" for w in written], f"  meshes/  ({n_meshes} STL)"],
        )

    rec.save(args.out)
    print(f"wrote {args.out} ({args.out.stat().st_size / 1e6:.1f} MB, {len(rec.frames)} frames)")
    win.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
