import base64

import pytest
import trimesh

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.state.controller import RobotController


@pytest.fixture()
def dispatcher(qapp, tmp_path):
    grab = lambda: b"PNGBYTES"  # noqa: E731 - injected fake window grabber
    d = CommandDispatcher(RobotController(), grab_png=grab)
    paths = []
    for name in ("base", "arm"):
        p = tmp_path / f"{name}.stl"
        trimesh.creation.box(extents=(1, 1, 1)).export(p)
        paths.append(str(p))
    d.dispatch({"command": "import_meshes", "args": {"paths": paths, "robot_name": "rig"}})
    return d


def test_undo_then_redo(dispatcher):
    dispatcher.dispatch({"command": "rename_link", "args": {"old": "arm", "new": "x"}})
    undo = dispatcher.dispatch({"command": "undo", "args": {}})
    assert undo["ok"] is True
    names = {link["name"] for link in undo["result"]["links"]}
    assert "arm" in names
    redo = dispatcher.dispatch({"command": "redo", "args": {}})
    assert redo["ok"] is True
    names = {link["name"] for link in redo["result"]["links"]}
    assert "x" in names


def test_screenshot_returns_base64_png(dispatcher):
    resp = dispatcher.dispatch({"command": "screenshot", "args": {}})
    assert resp["ok"] is True
    assert base64.b64decode(resp["result"]["png_base64"]) == b"PNGBYTES"


def test_screenshot_without_grabber_errors(qapp):
    d = CommandDispatcher(RobotController())  # no grab_png injected
    resp = d.dispatch({"command": "screenshot", "args": {}})
    assert resp["ok"] is False
    assert resp["error"] == "no_window"
