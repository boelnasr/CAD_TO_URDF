from pathlib import Path

import pytest

from cad2urdf.core.urdf.package import scaffold_ros_package


def test_scaffold_creates_expected_files(tmp_path: Path) -> None:
    out = tmp_path / "my_robot_description"
    scaffold_ros_package(
        out_dir=out,
        package_name="my_robot_description",
        urdf_relpath=Path("urdf/my_robot.urdf"),
        maintainer_name="boelnasr",
        maintainer_email="boelnasr@example.com",
    )
    assert (out / "package.xml").is_file()
    assert (out / "CMakeLists.txt").is_file()
    assert (out / "launch" / "display.launch.py").is_file()
    assert (out / "rviz" / "display.rviz").is_file()
    assert (out / "urdf").is_dir()
    assert (out / "meshes" / "visual").is_dir()
    assert (out / "meshes" / "collision").is_dir()


def test_package_xml_contains_name_and_maintainer(tmp_path: Path) -> None:
    out = tmp_path / "pkg"
    scaffold_ros_package(
        out_dir=out,
        package_name="pkg",
        urdf_relpath=Path("urdf/r.urdf"),
        maintainer_name="me",
        maintainer_email="me@e.com",
    )
    xml = (out / "package.xml").read_text()
    assert "<name>pkg</name>" in xml
    assert '<maintainer email="me@e.com">me</maintainer>' in xml


def test_launch_file_references_correct_urdf(tmp_path: Path) -> None:
    out = tmp_path / "pkg"
    scaffold_ros_package(
        out_dir=out,
        package_name="pkg",
        urdf_relpath=Path("urdf/foo.urdf"),
        maintainer_name="m",
        maintainer_email="m@e.com",
    )
    launch = (out / "launch" / "display.launch.py").read_text()
    assert "urdf/foo.urdf" in launch
    assert "pkg" in launch  # references the package


def test_cmakelists_uses_package_name(tmp_path: Path) -> None:
    out = tmp_path / "pkg"
    scaffold_ros_package(
        out_dir=out,
        package_name="my_test_pkg",
        urdf_relpath=Path("urdf/r.urdf"),
        maintainer_name="m",
        maintainer_email="m@e.com",
    )
    cml = (out / "CMakeLists.txt").read_text()
    assert "project(my_test_pkg)" in cml


# ---------------------------------------------------------------------------
# Validation tests — invalid package names
# ---------------------------------------------------------------------------


def test_scaffold_rejects_invalid_package_name(tmp_path: Path) -> None:
    """Uppercase + hyphen package name must raise ValueError."""
    with pytest.raises(ValueError, match="invalid ROS package name"):
        scaffold_ros_package(
            out_dir=tmp_path / "out",
            package_name="My-Pkg",
            urdf_relpath=Path("urdf/r.urdf"),
            maintainer_name="m",
            maintainer_email="m@e.com",
        )


def test_scaffold_rejects_uppercase_package_name(tmp_path: Path) -> None:
    """All-uppercase package name must raise ValueError."""
    with pytest.raises(ValueError, match="invalid ROS package name"):
        scaffold_ros_package(
            out_dir=tmp_path / "out",
            package_name="MYPKG",
            urdf_relpath=Path("urdf/r.urdf"),
            maintainer_name="m",
            maintainer_email="m@e.com",
        )


def test_scaffold_rejects_package_name_starting_with_digit(tmp_path: Path) -> None:
    """Package name starting with a digit must raise ValueError."""
    with pytest.raises(ValueError, match="invalid ROS package name"):
        scaffold_ros_package(
            out_dir=tmp_path / "out",
            package_name="1pkg",
            urdf_relpath=Path("urdf/r.urdf"),
            maintainer_name="m",
            maintainer_email="m@e.com",
        )


# ---------------------------------------------------------------------------
# Validation tests — invalid urdf_relpath
# ---------------------------------------------------------------------------


def test_scaffold_rejects_absolute_urdf_relpath(tmp_path: Path) -> None:
    """An absolute urdf_relpath must raise ValueError."""
    with pytest.raises(ValueError, match="must be relative"):
        scaffold_ros_package(
            out_dir=tmp_path / "out",
            package_name="mypkg",
            urdf_relpath=Path("/abs/path/r.urdf"),
            maintainer_name="m",
            maintainer_email="m@e.com",
        )


def test_scaffold_rejects_urdf_relpath_with_dotdot(tmp_path: Path) -> None:
    """A urdf_relpath containing '..' must raise ValueError."""
    with pytest.raises(ValueError, match="must not contain '\\.\\.'"):
        scaffold_ros_package(
            out_dir=tmp_path / "out",
            package_name="mypkg",
            urdf_relpath=Path("../etc/r.urdf"),
            maintainer_name="m",
            maintainer_email="m@e.com",
        )


# ---------------------------------------------------------------------------
# RViz base_link_name test
# ---------------------------------------------------------------------------


def test_rviz_config_uses_base_link_name(tmp_path: Path) -> None:
    """RViz config Fixed Frame must reflect the supplied base_link_name."""
    out = tmp_path / "pkg"
    scaffold_ros_package(
        out_dir=out,
        package_name="mypkg",
        urdf_relpath=Path("urdf/r.urdf"),
        maintainer_name="m",
        maintainer_email="m@e.com",
        base_link_name="cube_a",
    )
    rviz = (out / "rviz" / "display.rviz").read_text()
    assert "Fixed Frame: cube_a" in rviz
