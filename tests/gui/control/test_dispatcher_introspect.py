import pytest

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.state.controller import RobotController


@pytest.fixture()
def dispatcher(qapp):
    return CommandDispatcher(RobotController())


def test_unknown_command_returns_error(dispatcher):
    resp = dispatcher.dispatch({"command": "nope", "args": {}})
    assert resp["ok"] is False
    assert resp["error"] == "unknown_command"


def test_get_robot_returns_payload(dispatcher):
    resp = dispatcher.dispatch({"command": "get_robot", "args": {}})
    assert resp["ok"] is True
    assert resp["result"]["schema_version"] == 1
    assert "links" in resp["result"]


def test_list_materials_includes_aluminum(dispatcher):
    resp = dispatcher.dispatch({"command": "list_materials", "args": {}})
    assert resp["ok"] is True
    assert "aluminum_6061" in resp["result"]


def test_get_history_reports_flags(dispatcher):
    resp = dispatcher.dispatch({"command": "get_history", "args": {}})
    assert resp["result"] == {"can_undo": False, "can_redo": False}


def test_gui_status_ok(dispatcher):
    resp = dispatcher.dispatch({"command": "gui_status", "args": {}})
    result = resp["result"]
    assert result["running"] is True
    assert result["robot_name"] == "untitled"
    assert result["link_count"] == 1
