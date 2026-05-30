"""File > Save / Open round-trips a Robot via core/project/save.py."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Joint, Link, Robot

pytestmark = pytest.mark.gui


def _robot() -> Robot:
    return Robot(
        name="r",
        base_link="a",
        links={
            "a": Link(
                name="a",
                visual_mesh_path=Path("meshes/visual/a.stl"),
                collision_mesh_path=Path("meshes/collision/a.stl"),
                material_density=2700.0,
                material_name="aluminum_6061",
                inertial_override=InertialOverride(),
                origin=np.eye(4),
            ),
            "b": Link(
                name="b",
                visual_mesh_path=Path("meshes/visual/b.stl"),
                collision_mesh_path=Path("meshes/collision/b.stl"),
                material_density=2700.0,
                material_name="aluminum_6061",
                inertial_override=InertialOverride(),
                origin=np.eye(4),
            ),
        },
        joints={
            "ab": Joint(
                name="ab",
                type="fixed",
                parent="a",
                child="b",
                axis=np.array([1.0, 0.0, 0.0]),
                origin=np.eye(4),
            )
        },
    )


def test_save_then_open_round_trip(qtbot, tmp_path) -> None:
    from cad2urdf.gui.windows.main_window import MainWindow

    win = MainWindow()
    qtbot.addWidget(win)
    win.controller.replace(_robot())

    proj = tmp_path / "x.cad2urdf"
    win._save_project_to(proj)

    win2 = MainWindow()
    qtbot.addWidget(win2)
    win2._open_project_from(proj)

    r = win2.controller.current()
    assert set(r.links) == {"a", "b"}
    assert "ab" in r.joints
