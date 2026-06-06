import pytest
import trimesh

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.state.controller import RobotController


@pytest.fixture()
def two_meshes(tmp_path):
    a = tmp_path / "base.stl"
    b = tmp_path / "arm.stl"
    trimesh.creation.box(extents=(1, 1, 1)).export(a)
    trimesh.creation.box(extents=(1, 1, 1)).export(b)
    return [str(a), str(b)]


@pytest.fixture()
def dispatcher(qapp):
    return CommandDispatcher(RobotController())


def _imported(dispatcher, two_meshes):
    return dispatcher.dispatch(
        {"command": "import_meshes", "args": {"paths": two_meshes, "robot_name": "rig"}}
    )


def test_import_meshes_builds_links(dispatcher, two_meshes):
    resp = _imported(dispatcher, two_meshes)
    assert resp["ok"] is True
    names = {link["name"] for link in resp["result"]["links"]}
    assert names == {"base", "arm"}


def test_rename_link(dispatcher, two_meshes):
    _imported(dispatcher, two_meshes)
    resp = dispatcher.dispatch(
        {"command": "rename_link", "args": {"old": "arm", "new": "shoulder"}}
    )
    names = {link["name"] for link in resp["result"]["links"]}
    assert "shoulder" in names and "arm" not in names


def test_set_link_material_changes_density(dispatcher, two_meshes):
    _imported(dispatcher, two_meshes)
    resp = dispatcher.dispatch(
        {"command": "set_link_material", "args": {"link": "arm", "material": "steel_1018"}}
    )
    arm = next(link for link in resp["result"]["links"] if link["name"] == "arm")
    assert arm["material_name"] == "steel_1018"
    assert arm["material_density"] == pytest.approx(7850.0)


def test_remove_link(dispatcher, two_meshes):
    _imported(dispatcher, two_meshes)
    resp = dispatcher.dispatch({"command": "remove_link", "args": {"link": "arm"}})
    names = {link["name"] for link in resp["result"]["links"]}
    assert names == {"base"}


def test_set_base_link(dispatcher, two_meshes):
    _imported(dispatcher, two_meshes)
    resp = dispatcher.dispatch({"command": "set_base_link", "args": {"link": "arm"}})
    assert resp["result"]["base_link"] == "arm"


def test_import_rejects_missing_args(dispatcher):
    resp = dispatcher.dispatch({"command": "import_meshes", "args": {}})
    assert resp["ok"] is False
