"""ImportMeshesDialog: given a list of mesh paths + a robot name, seeds the controller."""

from __future__ import annotations

from pathlib import Path

import pytest

pytestmark = pytest.mark.gui


def test_run_import_replaces_controller_robot(qtbot, base_stl, arm_stl) -> None:
    from cad2urdf.gui.dialogs.import_meshes import run_import_into_controller
    from cad2urdf.gui.state.controller import RobotController

    c = RobotController()
    with qtbot.waitSignal(c.robotChanged, timeout=8000):
        run_import_into_controller(
            controller=c, mesh_paths=[base_stl, arm_stl], robot_name="my_arm"
        )
    r = c.current()
    assert r.name == "my_arm"
    assert set(r.links) == {"base", "arm"}


def test_failed_cad_import_removes_generated_assets_but_not_source(
    monkeypatch: pytest.MonkeyPatch, qapp, tmp_path: Path
) -> None:
    """A failed GUI worker must retire its generated tree and preserve user input."""
    from cad2urdf.gui.dialogs import import_meshes
    from cad2urdf.gui.state.controller import RobotController

    try:
        from cad2urdf.gui.state.cad_assets import CadAssetOwner
    except ModuleNotFoundError:
        pytest.fail("generated CAD assets do not have an explicit owner")

    source = tmp_path / "source.iges"
    source.write_text("user input")
    owned_root = tmp_path / "owned"
    controller = RobotController(cad_assets=CadAssetOwner(root=owned_root))

    def failing_builder(*, path: Path, mesh_dir: Path, robot_name: str):
        def job(_report):
            (mesh_dir / "partial.stl").write_text("partial")
            raise ValueError("broken CAD")

        return job

    monkeypatch.setattr(import_meshes, "build_cad_import_job", failing_builder)

    with pytest.raises(RuntimeError, match="broken CAD"):
        import_meshes.run_cad_import_into_controller(
            controller=controller,
            cad_path=source,
            robot_name="robot",
        )

    assert source.read_text() == "user input"
    assert list(owned_root.iterdir()) == []
