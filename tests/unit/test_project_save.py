from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import (
    InertialOverride,
    Joint,
    Link,
    Robot,
)
from cad2urdf.core.project.save import load_project, save_project


def _robot() -> Robot:
    return Robot(
        name="r",
        base_link="base",
        links={
            "base": Link(
                name="base",
                visual_mesh_path=Path("meshes/visual/base.stl"),
                collision_mesh_path=Path("meshes/collision/base.stl"),
                material_density=2700.0,
                material_name="aluminum_6061",
                inertial_override=InertialOverride(mass=2.0),
                origin=np.eye(4),
            ),
            "tip": Link(
                name="tip",
                visual_mesh_path=Path("meshes/visual/tip.stl"),
                collision_mesh_path=Path("meshes/collision/tip.stl"),
                material_density=2700.0,
                material_name="aluminum_6061",
                inertial_override=InertialOverride(),
                origin=np.eye(4),
            ),
        },
        joints={
            "j1": Joint(
                name="j1",
                type="revolute",
                parent="base",
                child="tip",
                axis=np.array([0.0, 0.0, 1.0]),
                origin=np.eye(4),
                limit_lower=-1.0,
                limit_upper=1.0,
            )
        },
    )


def test_save_then_load_yields_equal_robot(tmp_path: Path) -> None:
    out = tmp_path / "proj.cad2urdf"
    save_project(_robot(), out)
    loaded = load_project(out)
    assert loaded.name == "r"
    assert sorted(loaded.links.keys()) == ["base", "tip"]
    assert loaded.joints["j1"].limit_lower == -1.0
    assert loaded.links["base"].inertial_override.mass == 2.0
    assert np.allclose(loaded.joints["j1"].axis, np.array([0.0, 0.0, 1.0]))


def test_load_missing_file_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="project file not found"):
        load_project(tmp_path / "ghost.cad2urdf")


def test_load_unsupported_schema_raises(tmp_path: Path) -> None:
    p = tmp_path / "future.cad2urdf"
    p.write_text('{"schema_version": 99, "name": "r"}')
    with pytest.raises(ValueError, match="unsupported schema_version"):
        load_project(p)


def test_save_creates_parent_dirs(tmp_path: Path) -> None:
    out = tmp_path / "deep" / "nested" / "proj.cad2urdf"
    save_project(_robot(), out)
    assert out.is_file()


def test_v1_payload_loads_directly_without_migration(tmp_path: Path) -> None:
    """Sanity: with no migrations registered, v1 payload round-trips with no transform."""
    out = tmp_path / "p.cad2urdf"
    save_project(_robot(), out)
    # File was written with current SCHEMA_VERSION, so loading is identity.
    loaded = load_project(out)
    assert loaded.name == "r"


def test_save_preserves_full_inertial_override(tmp_path: Path) -> None:
    out = tmp_path / "proj.cad2urdf"
    base = Link(
        name="base",
        visual_mesh_path=Path("m/base.stl"),
        collision_mesh_path=Path("m/base.stl"),
        material_density=1.0,
        material_name="m",
        inertial_override=InertialOverride(
            mass=3.0,
            com=np.array([0.1, 0.2, 0.3]),
            inertia=np.eye(3) * 0.5,
        ),
        origin=np.eye(4),
    )
    r = Robot(name="r", base_link="base", links={"base": base}, joints={})
    save_project(r, out)
    loaded = load_project(out)
    ov = loaded.links["base"].inertial_override
    assert ov.mass == 3.0
    assert np.allclose(ov.com, np.array([0.1, 0.2, 0.3]))
    assert np.allclose(ov.inertia, np.eye(3) * 0.5)


def test_save_bundles_existing_meshes_and_loads_them_relative_to_project(
    tmp_path: Path,
) -> None:
    source = tmp_path / "session" / "body.stl"
    source.parent.mkdir()
    source.write_text("temporary tessellation")
    robot = _robot()
    robot.links["base"] = replace(
        robot.links["base"],
        visual_mesh_path=source,
        collision_mesh_path=source,
    )
    project = tmp_path / "saved" / "robot.cad2urdf"

    save_project(robot, project)
    source.unlink()
    loaded = load_project(project)

    visual = loaded.links["base"].visual_mesh_path
    collision = loaded.links["base"].collision_mesh_path
    assert visual == collision
    assert visual is not None
    assert visual.read_text() == "temporary tessellation"
    assert visual.parent == project.parent / "robot_assets"


def test_failed_json_commit_restores_previous_asset_bundle(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    first = tmp_path / "first.stl"
    first.write_text("first mesh")
    robot = _robot()
    robot.links["base"] = replace(
        robot.links["base"], visual_mesh_path=first, collision_mesh_path=first
    )
    project = tmp_path / "robot.cad2urdf"
    save_project(robot, project)

    second = tmp_path / "second.stl"
    second.write_text("second mesh")
    robot.links["base"] = replace(
        robot.links["base"], visual_mesh_path=second, collision_mesh_path=second
    )
    original_replace = Path.replace

    def fail_json_commit(source: Path, target: Path) -> Path:
        if source.suffix == ".tmp" and target == project:
            raise OSError("simulated JSON commit failure")
        return original_replace(source, target)

    monkeypatch.setattr(Path, "replace", fail_json_commit)

    with pytest.raises(OSError, match="JSON commit failure"):
        save_project(robot, project)

    loaded = load_project(project)
    visual = loaded.links["base"].visual_mesh_path
    assert visual is not None
    assert visual.read_text() == "first mesh"
    assert not list(tmp_path.glob(".*_backup_*"))
