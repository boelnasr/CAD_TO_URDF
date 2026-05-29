"""ImportMeshesDialog: given a list of mesh paths + a robot name, seeds the controller."""

from __future__ import annotations

import pytest

pytestmark = pytest.mark.gui


def test_run_import_replaces_controller_robot(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.gui.dialogs.import_meshes import run_import_into_controller
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    with qtbot.waitSignal(c.robotChanged, timeout=8000):
        run_import_into_controller(
            controller=c, mesh_paths=[base_stl, arm_stl], robot_name="my_arm"
        )
    r = c.current()
    assert r.name == "my_arm"
    assert set(r.links) == {"base", "arm"}
