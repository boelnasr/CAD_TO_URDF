"""Lifecycle coverage for generated CAD meshes owned by the current session."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from cad2urdf.gui.state.controller import RobotController
from cad2urdf.gui.state.empty_robot import make_empty_robot

pytestmark = pytest.mark.gui


def _owner(root: Path):
    try:
        from cad2urdf.gui.state.cad_assets import CadAssetOwner
    except ModuleNotFoundError:
        pytest.fail("generated CAD assets do not have an explicit owner")
    return CadAssetOwner(root=root)


def test_superseded_cad_assets_are_removed_after_robot_replacement(
    qapp, tmp_path: Path
) -> None:
    """Deleting before robotChanged would race the viewport; never deleting would leak."""
    owner = _owner(tmp_path / "owned")
    controller = RobotController(cad_assets=owner)

    first = controller.create_cad_import_dir()
    (first / "first.stl").write_text("first")
    controller.replace_with_cad_assets(
        replace(make_empty_robot(), name="first"), asset_dir=first
    )

    second = controller.create_cad_import_dir()
    (second / "second.stl").write_text("second")
    old_exists_during_replace: list[bool] = []
    controller.robotChanged.connect(
        lambda _robot: old_exists_during_replace.append(first.exists())
    )
    controller.replace_with_cad_assets(
        replace(make_empty_robot(), name="second"), asset_dir=second
    )

    assert old_exists_during_replace == [True]
    assert not first.exists()
    assert second.exists()


def test_non_cad_document_replacement_retires_active_cad_assets(
    qapp, tmp_path: Path
) -> None:
    """Opening/importing another document must not leave the prior CAD tree behind."""
    owner = _owner(tmp_path / "owned")
    controller = RobotController(cad_assets=owner)
    assets = controller.create_cad_import_dir()
    (assets / "body.stl").write_text("mesh")
    controller.replace_with_cad_assets(make_empty_robot(), asset_dir=assets)

    existed_during_replace: list[bool] = []
    controller.robotChanged.connect(
        lambda _robot: existed_during_replace.append(assets.exists())
    )
    controller.replace(replace(make_empty_robot(), name="other"))

    assert existed_during_replace == [True]
    assert not assets.exists()


def test_controller_shutdown_removes_remaining_owned_assets(qapp, tmp_path: Path) -> None:
    """Exiting the GUI must retire the current document's generated mesh tree."""
    owner = _owner(tmp_path / "owned")
    controller = RobotController(cad_assets=owner)
    assets = controller.create_cad_import_dir()
    (assets / "body.stl").write_text("mesh")
    controller.replace_with_cad_assets(make_empty_robot(), asset_dir=assets)

    controller.shutdown()

    assert not assets.exists()


def test_owner_rejects_user_directory_as_generated_assets(qapp, tmp_path: Path) -> None:
    """A caller must never be able to make replacement cleanup own user data."""
    owner = _owner(tmp_path / "owned")
    controller = RobotController(cad_assets=owner)
    user_dir = tmp_path / "user-output"
    user_dir.mkdir()
    source = user_dir / "source.iges"
    source.write_text("user data")

    with pytest.raises(ValueError, match="not allocated"):
        controller.replace_with_cad_assets(make_empty_robot(), asset_dir=user_dir)

    assert source.read_text() == "user data"
