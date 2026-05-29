"""Export job: write a full ROS 2 package mirroring the CLI's pipeline."""

from __future__ import annotations

import shutil
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from cad2urdf.core.inertia.compute import compute_inertial
from cad2urdf.core.kinematic.model import InertialOverride, Link, Robot
from cad2urdf.core.urdf.emit import emit_urdf
from cad2urdf.core.urdf.package import scaffold_ros_package
from cad2urdf.core.validation.manipulapy_gate import validate_urdf


@dataclass(frozen=True)
class ExportReport:
    urdf_path: Path
    manipulapy_ok: bool | None
    manipulapy_error: str | None


def _materialize_link_into_pkg(link: Link, out_dir: Path) -> Link:
    """Copy link's source mesh into the package's meshes/ dirs and bake inertia.

    Mirrors the CLI's _load_mesh_to_link + _populate_inertia two-step:
    1. Export visual STL → meshes/visual/<name>.stl
    2. Copy visual STL → meshes/collision/<name>.stl
    3. Compute mass/COM/inertia from the written STL
    4. Return a new Link with package-relative paths + fully-populated InertialOverride

    Links whose visual_mesh_path is not an absolute file are left untouched
    (they are already package-relative or missing).
    """
    import trimesh

    visual_src = link.visual_mesh_path
    if not visual_src.is_absolute() or not visual_src.is_file():
        return link  # already package-relative or missing — leave untouched

    visual_rel = Path("meshes/visual") / f"{link.name}.stl"
    collision_rel = Path("meshes/collision") / f"{link.name}.stl"
    visual_dst = out_dir / visual_rel
    collision_dst = out_dir / collision_rel
    visual_dst.parent.mkdir(parents=True, exist_ok=True)
    collision_dst.parent.mkdir(parents=True, exist_ok=True)

    mesh = trimesh.load(
        str(visual_src), file_type=visual_src.suffix.lower().lstrip("."), force="mesh"
    )
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"could not load {visual_src} as a single mesh")
    mesh.export(str(visual_dst), file_type="stl")
    shutil.copyfile(visual_dst, collision_dst)

    mass, com, inertia = compute_inertial(
        mesh, density=link.material_density, override=link.inertial_override
    )
    return Link(
        name=link.name,
        visual_mesh_path=visual_rel,
        collision_mesh_path=collision_rel,
        material_density=link.material_density,
        material_name=link.material_name,
        inertial_override=InertialOverride(mass=mass, com=com, inertia=inertia),
        origin=link.origin,
    )


def build_export_job(
    *,
    robot: Robot,
    out_dir: Path,
    package_name: str,
    maintainer: str,
    maintainer_email: str,
    run_manipulapy: bool,
) -> Callable[[Callable[[int, int, str], None]], ExportReport]:
    """Return a job closure compatible with Worker(job)."""

    def job(report: Callable[[int, int, str], None]) -> ExportReport:
        report(0, 4, "scaffolding ROS package")
        out_dir.mkdir(parents=True, exist_ok=True)
        scaffold_ros_package(
            out_dir=out_dir,
            package_name=package_name,
            urdf_relpath=Path("urdf") / f"{robot.name}.urdf",
            maintainer_name=maintainer,
            maintainer_email=maintainer_email,
            base_link_name=robot.base_link,
        )

        report(1, 4, "copying meshes + computing inertia")
        new_links: dict[str, Link] = {
            n: _materialize_link_into_pkg(link, out_dir) for n, link in robot.links.items()
        }
        baked = Robot(
            name=robot.name,
            base_link=robot.base_link,
            links=new_links,
            joints=robot.joints,
        )

        report(2, 4, "writing URDF")
        urdf_path = out_dir / "urdf" / f"{robot.name}.urdf"
        emit_urdf(baked, urdf_path, package_name=package_name)

        if not run_manipulapy:
            report(4, 4, "skipped ManipulaPy")
            return ExportReport(
                urdf_path=urdf_path,
                manipulapy_ok=None,
                manipulapy_error=None,
            )

        report(3, 4, "running ManipulaPy gate")
        result = validate_urdf(urdf_path)
        return ExportReport(
            urdf_path=urdf_path,
            manipulapy_ok=result.ok,
            manipulapy_error=result.error,
        )

    return job
