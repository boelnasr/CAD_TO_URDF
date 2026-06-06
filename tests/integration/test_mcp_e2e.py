import pytest
import trimesh

from cad2urdf.mcp.client import ControlClient
from cad2urdf.mcp.gui_process import GuiProcess


@pytest.mark.slow
def test_import_edit_export_end_to_end(tmp_path, monkeypatch):
    # Run the spawned GUI headless. PYVISTA_OFF_SCREEN is required in addition to
    # the Qt platform: PyVista's QtInteractor otherwise makes a fatal X11 call
    # even under the offscreen platform. monkeypatch restores both after the test.
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setenv("PYVISTA_OFF_SCREEN", "true")

    sock_path = str(tmp_path / "e2e.sock")
    base = tmp_path / "base.stl"
    arm = tmp_path / "arm.stl"
    trimesh.creation.box(extents=(1, 1, 1)).export(base)
    trimesh.creation.box(extents=(1, 1, 1)).export(arm)

    gui = GuiProcess(socket_path=sock_path)
    gui.start(timeout=60.0)
    client = ControlClient(sock_path)
    try:
        client.connect(timeout=10.0)

        imp = client.send(
            "import_meshes",
            {"paths": [str(base), str(arm)], "robot_name": "rig"},
        )
        assert imp["ok"] is True

        upd = client.send(
            "update_joint",
            {
                "joint": "arm_to_base",
                "type": "revolute",
                "axis": [0.0, 0.0, 1.0],
                "limit_lower": -1.0,
                "limit_upper": 1.0,
                "effort": 5.0,
                "velocity": 1.0,
            },
        )
        assert upd["ok"] is True
        edited = next(j for j in upd["result"]["joints"] if j["name"] == "arm_to_base")
        assert edited["type"] == "revolute"

        out = tmp_path / "pkg"
        exp = client.send(
            "export_package",
            {"out_dir": str(out), "package_name": "rig_desc", "run_manipulapy": False},
        )
        assert exp["ok"] is True
        assert (out / "package.xml").is_file()
        urdf = out / "urdf" / "rig.urdf"
        assert urdf.is_file()
        assert 'type="revolute"' in urdf.read_text()
    finally:
        client.close()
        gui.stop()
