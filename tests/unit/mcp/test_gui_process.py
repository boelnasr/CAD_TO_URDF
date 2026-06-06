import socket
import sys
import textwrap

import pytest

from cad2urdf.mcp.gui_process import GuiProcess


def test_gui_process_spawns_and_waits(tmp_path):
    sock_path = str(tmp_path / "fake.sock")
    fake = tmp_path / "fake_gui.py"
    # Fake GUI: bind the socket then sleep so the spawner sees it become connectable.
    fake.write_text(
        textwrap.dedent(
            f"""
            import socket, time
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.bind({sock_path!r})
            s.listen(1)
            time.sleep(30)
            """
        )
    )

    proc = GuiProcess(socket_path=sock_path, launch_cmd=[sys.executable, str(fake)])
    proc.start(timeout=5.0)
    try:
        assert proc.is_running()
        # The socket is connectable.
        c = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        c.connect(sock_path)
        c.close()
    finally:
        proc.stop()
    assert not proc.is_running()


def test_gui_process_raises_on_early_exit(tmp_path):
    sock_path = str(tmp_path / "never.sock")
    fake = tmp_path / "crash_gui.py"
    fake.write_text("import sys; sys.exit(3)")
    proc = GuiProcess(socket_path=sock_path, launch_cmd=[sys.executable, str(fake)])
    with pytest.raises(RuntimeError, match="exited early"):
        proc.start(timeout=5.0)
    assert not proc.is_running()
