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


def test_viewport_places_links_via_forward_kinematics(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.core.kinematic.model import Joint
    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.viewport.widget import ViewportWidget

    # base at origin; arm attached by a fixed joint offset +5.0 in z.
    origin = np.eye(4)
    origin[2, 3] = 5.0
    links = {
        "base": _link_with_mesh("base", base_stl),
        "arm": _link_with_mesh("arm", arm_stl),
    }
    joint = Joint(
        name="j",
        type="fixed",
        parent="base",
        child="arm",
        axis=np.array([1.0, 0.0, 0.0]),
        origin=origin,
    )
    c = RobotController()
    c.replace(Robot(name="r", base_link="base", links=links, joints={"j": joint}))
    vp = ViewportWidget(c)
    qtbot.addWidget(vp)

    # The arm's unit-cube actor must be lifted to ~z=5 by FK, not left at origin.
    arm_bounds = vp.actors_by_link_name()["arm"].bounds
    arm_center_z = (arm_bounds[4] + arm_bounds[5]) / 2.0
    assert 4.0 < arm_center_z < 6.0


def test_viewport_skips_links_with_non_absolute_paths(qtbot) -> None:
    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.state.empty_robot import make_empty_robot
    from cad2urdf.gui.viewport.widget import ViewportWidget

    c = RobotController()
    c.replace(make_empty_robot())  # placeholder Link has a non-absolute path
    vp = ViewportWidget(c)
    qtbot.addWidget(vp)
    assert vp.actors_by_link_name() == {}


def _color(actor):
    """Actor base colour as a pyvista ``Color`` (8-bit-quantised, comparable)."""
    import pyvista as pv

    return pv.Color(actor.prop.color)


def _expected(rgb):
    """Quantise an expected float RGB the same way VTK stores actor colours."""
    import pyvista as pv

    return pv.Color(rgb)


def test_highlight_link_changes_selected_actor_appearance(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.viewport.style import ViewportStyle
    from cad2urdf.gui.viewport.widget import ViewportWidget

    c = RobotController()
    c.replace(_robot([("base", base_stl), ("arm", arm_stl)]))
    vp = ViewportWidget(c)
    qtbot.addWidget(vp)
    actors = vp.actors_by_link_name()

    # Before any selection both links look identical (normal metal).
    assert _color(actors["arm"]) == _color(actors["base"])

    vp.highlight_link("arm")

    # The selected link now differs from a normal (unselected) link...
    assert _color(actors["arm"]) != _color(actors["base"])
    # ...and matches the highlight style, while the other link stays normal.
    assert _color(actors["arm"]) == _expected(ViewportStyle.highlight_kwargs()["color"])
    assert _color(actors["base"]) == _expected(ViewportStyle.mesh_kwargs()["color"])


def test_highlight_link_restores_on_deselect(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.viewport.style import ViewportStyle
    from cad2urdf.gui.viewport.widget import ViewportWidget

    c = RobotController()
    c.replace(_robot([("base", base_stl), ("arm", arm_stl)]))
    vp = ViewportWidget(c)
    qtbot.addWidget(vp)
    actors = vp.actors_by_link_name()

    vp.highlight_link("arm")
    assert _color(actors["arm"]) == _expected(ViewportStyle.highlight_kwargs()["color"])

    vp.highlight_link(None)  # clear selection
    assert _color(actors["arm"]) == _expected(ViewportStyle.mesh_kwargs()["color"])


def test_highlight_link_moves_highlight_to_other_link(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.viewport.style import ViewportStyle
    from cad2urdf.gui.viewport.widget import ViewportWidget

    c = RobotController()
    c.replace(_robot([("base", base_stl), ("arm", arm_stl)]))
    vp = ViewportWidget(c)
    qtbot.addWidget(vp)
    actors = vp.actors_by_link_name()

    vp.highlight_link("arm")
    vp.highlight_link("base")  # select a different link

    # The previously selected link is restored, the new one is highlighted.
    assert _color(actors["arm"]) == _expected(ViewportStyle.mesh_kwargs()["color"])
    assert _color(actors["base"]) == _expected(ViewportStyle.highlight_kwargs()["color"])


def test_highlight_link_noop_for_unknown_and_none(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.viewport.style import ViewportStyle
    from cad2urdf.gui.viewport.widget import ViewportWidget

    c = RobotController()
    c.replace(_robot([("base", base_stl), ("arm", arm_stl)]))
    vp = ViewportWidget(c)
    qtbot.addWidget(vp)
    actors = vp.actors_by_link_name()

    # Establish a real selection first, then an unknown name must NOT disturb it
    # (the dangerous regression: an unknown link clearing/moving the highlight).
    vp.highlight_link("arm")
    vp.highlight_link("does_not_exist")
    assert _color(actors["arm"]) == _expected(ViewportStyle.highlight_kwargs()["color"])
    assert _color(actors["base"]) == _expected(ViewportStyle.mesh_kwargs()["color"])

    # None clears the selection without raising.
    vp.highlight_link(None)
    assert _color(actors["arm"]) == _expected(ViewportStyle.mesh_kwargs()["color"])


def test_highlight_survives_rebuild(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.viewport.style import ViewportStyle
    from cad2urdf.gui.viewport.widget import ViewportWidget

    c = RobotController()
    c.replace(_robot([("base", base_stl), ("arm", arm_stl)]))
    vp = ViewportWidget(c)
    qtbot.addWidget(vp)

    vp.highlight_link("arm")
    # Force a rebuild (e.g. an edit fired robotChanged). New actor objects are
    # created; the highlight must be re-applied to the still-selected link.
    c.replace(_robot([("base", base_stl), ("arm", arm_stl)]))
    actors = vp.actors_by_link_name()
    assert _color(actors["arm"]) == _expected(ViewportStyle.highlight_kwargs()["color"])
    assert _color(actors["base"]) == _expected(ViewportStyle.mesh_kwargs()["color"])


def test_widget_uses_fallback_metal_when_env_map_fails(
    qtbot, base_stl, arm_stl, monkeypatch
) -> None:
    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.viewport.style import ViewportStyle
    from cad2urdf.gui.viewport.widget import ViewportWidget

    # Force the environment map to fail during widget construction; the widget
    # must then render links with the no-envmap fallback metal (lighter, rougher)
    # rather than the reflective default that would read flat without reflections.
    def _boom() -> object:
        raise RuntimeError("no env map")

    monkeypatch.setattr(ViewportStyle, "make_environment_texture", staticmethod(_boom))

    c = RobotController()
    c.replace(_robot([("base", base_stl), ("arm", arm_stl)]))
    vp = ViewportWidget(c)
    qtbot.addWidget(vp)
    actors = vp.actors_by_link_name()
    assert _color(actors["base"]) == _expected(ViewportStyle.mesh_kwargs_no_envmap()["color"])
