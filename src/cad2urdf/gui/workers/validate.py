"""Validate worker: write URDF to a scratch dir, validate AST + optionally ManipulaPy.

Follows the same closure pattern as build_import_job / build_recompute_job.

Mesh paths in the Robot AST may be absolute (as stored by the GUI after import).
The worker materializes each link into the scratch package exactly as the export
job does (link-named meshes, baked inertia, frame links untouched), so the URDF
that gets validated is the URDF that export would write.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from cad2urdf.core.kinematic.model import Robot
from cad2urdf.core.kinematic.validate import ValidationIssue, validate_robot
from cad2urdf.core.urdf.emit import emit_urdf
from cad2urdf.core.urdf.package import scaffold_ros_package
from cad2urdf.core.validation.manipulapy_gate import validate_urdf
from cad2urdf.gui.workers.export_package import _materialize_link_into_pkg


@dataclass(frozen=True)
class ValidateReport:
    ast_issues: list[ValidationIssue] = field(default_factory=list)
    urdf_written: bool = False
    urdf_path: Path | None = None
    manipulapy_ok: bool | None = None
    manipulapy_error: str | None = None


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

        # Share export's materialization: relative paths alone would make
        # emit_urdf drop every <inertial>, and the dynamics gate would then
        # validate a massless robot that export never writes.
        baked = Robot(
            name=robot.name,
            base_link=robot.base_link,
            links={
                n: _materialize_link_into_pkg(link, out_dir) for n, link in robot.links.items()
            },
            joints=robot.joints,
        )

        urdf_path = out_dir / "urdf" / urdf_relname
        emit_urdf(baked, urdf_path, package_name=package_name)

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
