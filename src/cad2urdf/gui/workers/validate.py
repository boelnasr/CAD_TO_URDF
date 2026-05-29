"""Validate worker: write URDF to a scratch dir, validate AST + optionally ManipulaPy.

Follows the same closure pattern as build_import_job / build_recompute_job.

Mesh paths in the Robot AST may be absolute (as stored by the GUI after import).
The worker copies each mesh into the scratch package and rewrites link paths to
relative package:// URIs before calling emit_urdf — matching the CLI's approach.
"""

from __future__ import annotations

import shutil
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from cad2urdf.core.kinematic.model import Link, Robot
from cad2urdf.core.kinematic.validate import ValidationIssue, validate_robot
from cad2urdf.core.urdf.emit import emit_urdf
from cad2urdf.core.urdf.package import scaffold_ros_package
from cad2urdf.core.validation.manipulapy_gate import validate_urdf


@dataclass(frozen=True)
class ValidateReport:
    ast_issues: list[ValidationIssue] = field(default_factory=list)
    urdf_written: bool = False
    urdf_path: Path | None = None
    manipulapy_ok: bool | None = None
    manipulapy_error: str | None = None


def _relativise_links(robot: Robot, out_dir: Path) -> Robot:
    """Copy absolute mesh files into out_dir and return a Robot with relative mesh paths.

    Links whose paths are already relative are left untouched (they are assumed
    to be valid package:// relative paths already).  Absolute paths are copied
    into meshes/visual/ and meshes/collision/ mirroring the CLI's layout.
    """
    visual_dir = out_dir / "meshes" / "visual"
    collision_dir = out_dir / "meshes" / "collision"
    visual_dir.mkdir(parents=True, exist_ok=True)
    collision_dir.mkdir(parents=True, exist_ok=True)

    new_links: dict[str, Link] = {}
    for name, link in robot.links.items():
        visual = link.visual_mesh_path
        collision = link.collision_mesh_path

        if visual.is_absolute():
            dest = visual_dir / visual.name
            if not dest.is_file():
                shutil.copy2(visual, dest)
            visual = Path("meshes/visual") / visual.name

        if collision.is_absolute():
            dest = collision_dir / collision.name
            if not dest.is_file():
                shutil.copy2(collision, dest)
            collision = Path("meshes/collision") / collision.name

        new_links[name] = Link(
            name=link.name,
            visual_mesh_path=visual,
            collision_mesh_path=collision,
            material_density=link.material_density,
            material_name=link.material_name,
            inertial_override=link.inertial_override,
            origin=link.origin,
        )

    return Robot(
        name=robot.name,
        base_link=robot.base_link,
        links=new_links,
        joints=robot.joints,
    )


def build_validate_job(
    *,
    robot: Robot,
    out_dir: Path,
    package_name: str,
    urdf_relname: str,
    run_manipulapy: bool,
) -> Callable[[Callable[[int, int, str], None]], ValidateReport]:
    def job(report: Callable[[int, int, str], None]) -> ValidateReport:
        report(0, 3, "validating AST")
        ast_issues = validate_robot(robot)

        report(1, 3, "writing URDF")
        out_dir.mkdir(parents=True, exist_ok=True)
        scaffold_ros_package(
            out_dir=out_dir,
            package_name=package_name,
            urdf_relpath=Path("urdf") / urdf_relname,
            maintainer_name="cad2urdf-gui",
            maintainer_email="gui@example.com",
            base_link_name=robot.base_link,
        )

        # Rewrite absolute mesh paths to relative package paths so emit_urdf
        # can produce valid package:// URIs (raises on absolute paths).
        robot_rel = _relativise_links(robot, out_dir)

        urdf_path = out_dir / "urdf" / urdf_relname
        emit_urdf(robot_rel, urdf_path, package_name=package_name)

        if not run_manipulapy:
            report(3, 3, "skipped ManipulaPy")
            return ValidateReport(ast_issues=ast_issues, urdf_written=True, urdf_path=urdf_path)

        report(2, 3, "running ManipulaPy gate")
        manipulapy = validate_urdf(urdf_path)
        return ValidateReport(
            ast_issues=ast_issues,
            urdf_written=True,
            urdf_path=urdf_path,
            manipulapy_ok=manipulapy.ok,
            manipulapy_error=manipulapy.error,
        )

    return job
