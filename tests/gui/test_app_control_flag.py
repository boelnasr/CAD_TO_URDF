import sys

import pytest

from cad2urdf.gui.app import parse_args


def test_parse_args_default_socket_is_none():
    ns = parse_args([])
    assert ns.control_socket is None


def test_parse_args_reads_control_socket():
    ns = parse_args(["--control-socket", "/tmp/x.sock"])
    assert ns.control_socket == "/tmp/x.sock"


@pytest.mark.gui
def test_sigterm_removes_cad_session_root_issue28(tmp_path, monkeypatch):
    """GuiProcess.stop() sends SIGTERM; the GUI must still run its cleanup."""
    from cad2urdf.mcp.gui_process import GuiProcess

    # The child inherits TMPDIR, so its CadAssetOwner session root lands here.
    monkeypatch.setenv("TMPDIR", str(tmp_path))
    sock_path = str(tmp_path / "s.sock")
    gui = GuiProcess(
        socket_path=sock_path,
        launch_cmd=[sys.executable, "-m", "cad2urdf.gui.app", "--control-socket", sock_path],
    )
    gui.start(timeout=60.0)
    try:
        # Precondition: the live GUI owns a session root under TMPDIR.
        assert list(tmp_path.glob("cad2urdf_cad_session_*"))
    finally:
        gui.stop()

    assert list(tmp_path.glob("cad2urdf_cad_session_*")) == []
