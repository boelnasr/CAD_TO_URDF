import pytest

from cad2urdf.gui.control import protocol


def test_known_command_names_include_core_groups():
    for name in ("get_robot", "import_meshes", "update_joint", "export_package", "screenshot"):
        assert name in protocol.COMMANDS
    assert len(protocol.COMMANDS) == 17


def test_encode_decode_roundtrip():
    line = protocol.encode({"command": "get_robot", "args": {}})
    assert line.endswith("\n")
    assert protocol.decode(line) == {"command": "get_robot", "args": {}}


def test_decode_rejects_non_object():
    with pytest.raises(protocol.ProtocolError):
        protocol.decode("[1, 2, 3]\n")


def test_ok_and_err_envelopes():
    assert protocol.ok({"x": 1}) == {"ok": True, "result": {"x": 1}}
    env = protocol.err("bad_command", "nope")
    assert env == {"ok": False, "error": "bad_command", "detail": "nope"}


def test_ok_allows_none_result():
    assert protocol.ok(None) == {"ok": True, "result": None}


def test_decode_rejects_empty_line():
    with pytest.raises(protocol.ProtocolError, match="empty message"):
        protocol.decode("\n")
