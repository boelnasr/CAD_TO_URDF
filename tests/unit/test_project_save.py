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
from cad2urdf.core.kinematic.tree import remove_link, rename_link
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


def test_load_relative_project_path_yields_absolute_mesh_paths_issue10(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """MCP open_project may pass a cwd-relative path; mesh paths must still be absolute."""
    source = tmp_path / "body.stl"
    source.write_text("mesh bytes")
    robot = _robot()
    for name in ("base", "tip"):
        robot.links[name] = replace(
            robot.links[name], visual_mesh_path=source, collision_mesh_path=source
        )
    save_project(robot, tmp_path / "p.cad2urdf")
    monkeypatch.chdir(tmp_path)

    loaded = load_project(Path("p.cad2urdf"))

    for link in loaded.links.values():
        for mesh in (link.visual_mesh_path, link.collision_mesh_path):
            assert mesh is not None
            assert mesh.is_absolute(), mesh
            assert mesh.is_file(), mesh


_STAR_MESHES = {"a_base": "box mesh", "b_arm": "cylinder mesh", "c_tool": "ball mesh"}


def _star_robot(src: Path) -> Robot:
    """a_base with two fixed children, each link bundling distinct mesh bytes."""
    src.mkdir()
    links: dict[str, Link] = {}
    for name, content in _STAR_MESHES.items():
        mesh = src / f"{name}.stl"
        mesh.write_text(content)
        links[name] = Link(
            name=name,
            visual_mesh_path=mesh,
            collision_mesh_path=mesh,
            material_density=2700.0,
            material_name="aluminum_6061",
            inertial_override=InertialOverride(),
        )
    x = np.array([1.0, 0.0, 0.0])
    return Robot(
        name="r",
        base_link="a_base",
        links=links,
        joints={
            "j1": Joint(name="j1", type="fixed", parent="a_base", child="b_arm", axis=x),
            "j2": Joint(name="j2", type="fixed", parent="a_base", child="c_tool", axis=x),
        },
    )


def _mesh_contents(robot: Robot) -> dict[str, str]:
    out: dict[str, str] = {}
    for name, link in robot.links.items():
        assert link.visual_mesh_path is not None
        out[name] = link.visual_mesh_path.read_text() if link.visual_mesh_path.is_file() else ""
    return out


def test_resave_after_rename_keeps_live_mesh_paths_issue3(tmp_path: Path) -> None:
    """GUI keeps the in-memory Robot after Save; its asset paths must not be re-pointed."""
    project = tmp_path / "p.cad2urdf"
    save_project(_star_robot(tmp_path / "src"), project)
    # File > Open, then a rename that changes the sorted link order.
    live = rename_link(load_project(project), "b_arm", "z_arm")

    save_project(live, project)

    expected = {
        "a_base": "box mesh",
        "c_tool": "ball mesh",
        "z_arm": "cylinder mesh",
    }
    assert _mesh_contents(live) == expected
    assert _mesh_contents(load_project(project)) == expected


def test_resave_after_remove_keeps_live_mesh_paths_issue3(tmp_path: Path) -> None:
    project = tmp_path / "p.cad2urdf"
    save_project(_star_robot(tmp_path / "src"), project)
    live = remove_link(load_project(project), "b_arm")

    save_project(live, project)

    expected = {"a_base": "box mesh", "c_tool": "ball mesh"}
    assert _mesh_contents(live) == expected
    assert _mesh_contents(load_project(project)) == expected


def test_save_refuses_to_replace_user_asset_dir_issue6(tmp_path: Path) -> None:
    """A '<stem>_assets' folder cad2urdf did not create must survive a save untouched."""
    user_dir = tmp_path / "robot_assets"
    user_dir.mkdir()
    mesh = user_dir / "base.stl"
    mesh.write_text("user mesh")
    (user_dir / "notes.txt").write_text("calibration notes")
    robot = _robot()
    robot.links["base"] = replace(
        robot.links["base"], visual_mesh_path=mesh, collision_mesh_path=mesh
    )

    with pytest.raises(FileExistsError, match="robot_assets"):
        save_project(robot, tmp_path / "robot.cad2urdf")

    assert sorted(p.name for p in user_dir.iterdir()) == ["base.stl", "notes.txt"]
    assert mesh.read_text() == "user mesh"
    # No project file and no staging leftovers beside it.
    assert sorted(p.name for p in tmp_path.iterdir()) == ["robot_assets"]


def test_resave_replaces_asset_dir_created_by_cad2urdf_issue6(tmp_path: Path) -> None:
    first = tmp_path / "first.stl"
    first.write_text("first mesh")
    second = tmp_path / "second.stl"
    second.write_text("second mesh")
    robot = _robot()
    project = tmp_path / "robot.cad2urdf"
    robot.links["base"] = replace(
        robot.links["base"], visual_mesh_path=first, collision_mesh_path=first
    )
    save_project(robot, project)
    robot.links["base"] = replace(
        robot.links["base"], visual_mesh_path=second, collision_mesh_path=second
    )

    save_project(robot, project)

    visual = load_project(project).links["base"].visual_mesh_path
    assert visual is not None
    assert visual.read_text() == "second mesh"


def test_resave_accepts_legacy_numbered_asset_dir_issue6(tmp_path: Path) -> None:
    """Bundles written before the ownership marker hold only NNNN.<ext> files."""
    legacy = tmp_path / "robot_assets"
    legacy.mkdir()
    (legacy / "0001.stl").write_text("base mesh")
    (legacy / "0002.stl").write_text("tip mesh")
    robot = _robot()
    for name, filename in (("base", "0001.stl"), ("tip", "0002.stl")):
        robot.links[name] = replace(
            robot.links[name],
            visual_mesh_path=legacy / filename,
            collision_mesh_path=legacy / filename,
        )
    project = tmp_path / "robot.cad2urdf"

    save_project(robot, project)

    loaded = load_project(project)
    assert _mesh_contents(loaded) == {"base": "base mesh", "tip": "tip mesh"}
