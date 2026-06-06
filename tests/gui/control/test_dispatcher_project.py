import pytest
import trimesh

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.state.controller import RobotController


@pytest.fixture()
def dispatcher(qapp, tmp_path):
    d = CommandDispatcher(RobotController())
    paths = []
    for name in ("base", "arm"):
        p = tmp_path / f"{name}.stl"
        trimesh.creation.box(extents=(1, 1, 1)).export(p)
        paths.append(str(p))
    d.dispatch({"command": "import_meshes", "args": {"paths": paths, "robot_name": "rig"}})
    return d


def test_save_then_open_roundtrip(dispatcher, tmp_path):
    proj = tmp_path / "rig.cad2urdf"
    save = dispatcher.dispatch({"command": "save_project", "args": {"path": str(proj)}})
    assert save["ok"] is True
    assert proj.is_file()

    fresh = CommandDispatcher(RobotController())
    opened = fresh.dispatch({"command": "open_project", "args": {"path": str(proj)}})
    assert opened["result"]["name"] == "rig"


def test_validate_writes_urdf(dispatcher, tmp_path):
    out = tmp_path / "val"
    resp = dispatcher.dispatch(
        {
            "command": "validate",
            "args": {"out_dir": str(out), "package_name": "rig_desc", "run_manipulapy": False},
        }
    )
    assert resp["ok"] is True
    assert resp["result"]["urdf_written"] is True
    import json
    json.dumps(resp)  # result must be wire-serializable


def test_open_project_missing_file_returns_err(tmp_path):
    d = CommandDispatcher(RobotController())
    resp = d.dispatch(
        {"command": "open_project", "args": {"path": str(tmp_path / "nope.cad2urdf")}}
    )
    assert resp["ok"] is False
    assert "FileNotFoundError" in resp["error"]


def test_export_package_writes_files(dispatcher, tmp_path):
    out = tmp_path / "pkg"
    resp = dispatcher.dispatch(
        {
            "command": "export_package",
            "args": {
                "out_dir": str(out),
                "package_name": "rig_desc",
                "run_manipulapy": False,
            },
        }
    )
    assert resp["ok"] is True
    assert (out / "package.xml").is_file()
    assert (out / "urdf" / "rig.urdf").is_file()
