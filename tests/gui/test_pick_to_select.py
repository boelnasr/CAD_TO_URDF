"""Picking a mesh in the viewport selects the corresponding link in the tree."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Link, Robot

pytestmark = pytest.mark.gui


def _link(name: str, stl: Path) -> Link:
    return Link(
        name=name,
        visual_mesh_path=stl,
        collision_mesh_path=stl,
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )


def test_emits_linkPicked_when_actor_clicked(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.gui.state.controller import RobotController
    from cad2urdf.gui.viewport.widget import ViewportWidget

    c = RobotController()
    c.replace(
        Robot(
            name="r",
            base_link="base",
            links={"base": _link("base", base_stl), "arm": _link("arm", arm_stl)},
            joints={},
        )
    )
    vp = ViewportWidget(c)
    qtbot.addWidget(vp)

    received: list[str] = []
    vp.linkPicked.connect(lambda name: received.append(name))

    # Simulate a pick — call the registered picker callback directly with the
    # arm actor. (pyvistaqt's `enable_mesh_picking` ultimately calls the callback
    # with the picked mesh / actor — we exercise the same code path.)
    actor = vp.actors_by_link_name()["arm"]
    vp.handle_pick(actor)
    assert received == ["arm"]
