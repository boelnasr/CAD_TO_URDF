import os
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


def test_stop_is_noop_before_start(tmp_path):
    proc = GuiProcess(socket_path=str(tmp_path / "nope.sock"))
    proc.stop()  # no _proc yet; must not raise
    assert not proc.is_running()


def _fake_gui_proc(tmp_path):
    sock_path = str(tmp_path / "fake.sock")
    fake = tmp_path / "fake_gui.py"
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
    return GuiProcess(socket_path=sock_path, launch_cmd=[sys.executable, str(fake)])


def test_double_start_raises(tmp_path):
    proc = _fake_gui_proc(tmp_path)
    proc.start(timeout=5.0)
    try:
        with pytest.raises(RuntimeError, match="already started"):
            proc.start(timeout=5.0)
    finally:
        proc.stop()
    proc.stop()  # second stop is harmless
    assert not proc.is_running()


def test_child_stdout_kept_off_mcp_stdio_stream_issue14(tmp_path, capfd):
    # fd 1 of cad2urdf-mcp is the JSON-RPC channel; native-library chatter from
    # the GUI child (OCCT, VTK, ...) must never land on it.
    sock_path = str(tmp_path / "noisy.sock")
    fake = tmp_path / "noisy_gui.py"
    fake.write_text(
        textwrap.dedent(
            f"""
            import socket, sys, time
            print("noise-from-gui-child", flush=True)
            # Reading stdin must not consume the parent's JSON-RPC input.
            sys.stdout.write("stdin=" + repr(sys.stdin.read(5)) + "\\n")
            sys.stdout.flush()
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.bind({sock_path!r})
            s.listen(1)
            time.sleep(30)
            """
        )
    )
    # pytest already points fd 0 at /dev/null, which would hide an inherited
    # stdin; put readable bytes there so inheriting it shows up as a leak.
    read_end, write_end = os.pipe()
    os.write(write_end, b"LEAK!")
    os.close(write_end)
    saved_stdin = os.dup(0)
    os.dup2(read_end, 0)
    os.close(read_end)
    try:
        proc = GuiProcess(socket_path=sock_path, launch_cmd=[sys.executable, str(fake)])
        proc.start(timeout=5.0)
        proc.stop()
    finally:
        os.dup2(saved_stdin, 0)
        os.close(saved_stdin)

    captured = capfd.readouterr()
    assert "noise-from-gui-child" not in captured.out
    assert "stdin=" not in captured.out
    # Redirected to stderr rather than discarded, so it stays diagnosable.
    assert "noise-from-gui-child" in captured.err
    assert "stdin=''" in captured.err
