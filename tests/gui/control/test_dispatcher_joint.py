import numpy as np
import pytest
import trimesh

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.state.controller import RobotController


@pytest.fixture()
def dispatcher(qapp, tmp_path):
    d = CommandDispatcher(RobotController())
    paths = []
    for name in ("base", "a", "b"):
        p = tmp_path / f"{name}.stl"
        trimesh.creation.box(extents=(1, 1, 1)).export(p)
        paths.append(str(p))
    d.dispatch({"command": "import_meshes", "args": {"paths": paths, "robot_name": "rig"}})
    return d


def _joint(resp, name):
    return next(j for j in resp["result"]["joints"] if j["name"] == name)


def test_update_joint_type_and_axis(dispatcher):
    # import wires a_to_base + b_to_base as fixed.
    resp = dispatcher.dispatch(
        {
            "command": "update_joint",
            "args": {
                "joint": "a_to_base",
                "type": "revolute",
                "axis": [0.0, 0.0, 2.0],
                "limit_lower": -1.0,
                "limit_upper": 1.0,
                "effort": 10.0,
                "velocity": 1.0,
            },
        }
    )
    j = _joint(resp, "a_to_base")
    assert j["type"] == "revolute"
    assert j["axis"] == pytest.approx([0.0, 0.0, 1.0])  # normalized
    assert j["limit_lower"] == -1.0


def test_update_joint_origin(dispatcher):
    resp = dispatcher.dispatch(
        {
            "command": "update_joint",
            "args": {
                "joint": "a_to_base",
                "origin_xyz": [1.0, 2.0, 3.0],
                "origin_rpy": [0.0, 0.0, 0.0],
            },
        }
    )
    j = _joint(resp, "a_to_base")
    assert np.allclose(np.array(j["origin"])[:3, 3], [1.0, 2.0, 3.0])


def test_update_joint_reparent_builds_chain(dispatcher):
    # Move b under a so the tree becomes base -> a -> b.
    resp = dispatcher.dispatch(
        {"command": "update_joint", "args": {"joint": "b_to_base", "parent": "a"}}
    )
    j = _joint(resp, "b_to_base")
    assert j["parent"] == "a"


def test_update_joint_origin_requires_both(dispatcher):
    resp = dispatcher.dispatch(
        {"command": "update_joint", "args": {"joint": "a_to_base", "origin_xyz": [1, 0, 0]}}
    )
    assert resp["ok"] is False


def test_update_joint_unknown_name(dispatcher):
    resp = dispatcher.dispatch(
        {"command": "update_joint", "args": {"joint": "nonexistent"}}
    )
    assert resp["ok"] is False


def test_update_joint_reparent_and_edit_combined(dispatcher):
    resp = dispatcher.dispatch(
        {
            "command": "update_joint",
            "args": {
                "joint": "b_to_base",
                "parent": "a",
                "type": "revolute",
                "axis": [0.0, 1.0, 0.0],
            },
        }
    )
    j = next(j for j in resp["result"]["joints"] if j["name"] == "b_to_base")
    assert j["parent"] == "a"
    assert j["type"] == "revolute"
