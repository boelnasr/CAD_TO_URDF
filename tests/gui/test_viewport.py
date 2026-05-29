"""ViewportWidget: pyvistaqt QtInteractor with one actor per link."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Link, Robot

pytestmark = pytest.mark.gui


def _link_with_mesh(name: str, stl: Path) -> Link:
    return Link(
        name=name,
        visual_mesh_path=stl,
        collision_mesh_path=stl,
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )


def _robot(stls: list[tuple[str, Path]]) -> Robot:
    links = {name: _link_with_mesh(name, p) for name, p in stls}
    return Robot(name="r", base_link=stls[0][0], links=links, joints={})


def test_viewport_creates_one_actor_per_link(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.viewport.widget import ViewportWidget

    c = RobotController()
    c.replace(_robot([("base", base_stl), ("arm", arm_stl)]))
    vp = ViewportWidget(c)
    qtbot.addWidget(vp)
    assert set(vp.actors_by_link_name()) == {"base", "arm"}


def test_viewport_rebuilds_on_robotChanged(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.core.kinematic.tree import remove_link
    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.viewport.widget import ViewportWidget

    c = RobotController()
    c.replace(_robot([("base", base_stl), ("arm", arm_stl)]))
    vp = ViewportWidget(c)
    qtbot.addWidget(vp)
    c.apply(lambda r: remove_link(r, "arm"), label="rm arm")
    assert set(vp.actors_by_link_name()) == {"base"}


def test_viewport_skips_links_with_non_absolute_paths(qtbot) -> None:
    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.state.empty_robot import make_empty_robot
    from cad2urdf.gui.viewport.widget import ViewportWidget

    c = RobotController()
    c.replace(make_empty_robot())  # placeholder Link has a non-absolute path
    vp = ViewportWidget(c)
    qtbot.addWidget(vp)
    assert vp.actors_by_link_name() == {}
