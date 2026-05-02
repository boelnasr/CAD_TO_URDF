"""cad2urdf CLI: input meshes + joint config → ROS 2 URDF package.

v1-alpha: STL/OBJ inputs only. STEP support requires conda + pythonOCC-core
(see README's Option A install path).
"""

from __future__ import annotations

import argparse
import logging
import shutil
import sys
from pathlib import Path

import numpy as np

from cad2urdf.core.config.loader import (
    JointsConfig,
    load_joints_config,
    origin_from_xyz_rpy,
)
from cad2urdf.core.inertia.compute import compute_inertial
from cad2urdf.core.inertia.materials import lookup
from cad2urdf.core.kinematic.model import (
    InertialOverride,
    Joint,
    Link,
    Robot,
)
from cad2urdf.core.kinematic.validate import validate_robot
from cad2urdf.core.parsers.obj import load_obj
from cad2urdf.core.parsers.stl import load_stl
from cad2urdf.core.urdf.emit import emit_urdf
from cad2urdf.core.urdf.package import scaffold_ros_package
from cad2urdf.core.validation.manipulapy_gate import validate_urdf

log = logging.getLogger("cad2urdf")

STEP_NOT_SUPPORTED_MSG = (
    "STEP / IGES inputs require pythonOCC-core (conda-only). "
    "Install via `conda env create -f environment.yml` and rerun. "
    "v1-alpha pip-install path supports STL/OBJ only."
)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="cad2urdf", description="Convert CAD assemblies to ROS 2 URDF packages."
    )
    p.add_argument("inputs", nargs="+", type=Path, help="Input STL/OBJ files (one per link).")
    p.add_argument("-o", "--out", type=Path, required=True, help="Output package directory.")
    p.add_argument("--joints", type=Path, required=True, help="Joints YAML config.")
    p.add_argument(
        "--robot-name",
        type=str,
        help="Override robot name (default: from joints config).",
    )
    p.add_argument(
        "--package-name",
        type=str,
        help="ROS package name (default: <out_dir>.name).",
    )
    p.add_argument(
        "--maintainer",
        type=str,
        default="cad2urdf-user",
        help="Maintainer name for package.xml.",
    )
    p.add_argument(
        "--maintainer-email",
        type=str,
        default="user@example.com",
        help="Maintainer email for package.xml.",
    )
    p.add_argument(
        "--no-validate",
        action="store_true",
        help="Skip ManipulaPy post-emit validation (useful when manipulapy not installed).",
    )
    p.add_argument("-v", "--verbose", action="count", default=0, help="-v info, -vv debug.")
    return p


def _load_mesh_to_link(
    inp: Path,
    out_dir: Path,
    cfg: JointsConfig,
) -> tuple[Link, Path]:
    """Load one mesh input, copy/export it into the package, return Link + abs visual path."""
    visual_dir = out_dir / "meshes" / "visual"
    collision_dir = out_dir / "meshes" / "collision"
    visual_dir.mkdir(parents=True, exist_ok=True)
    collision_dir.mkdir(parents=True, exist_ok=True)

    suffix = inp.suffix.lower()
    if suffix in {".step", ".stp"}:
        raise ValueError(STEP_NOT_SUPPORTED_MSG)

    name = inp.stem
    if suffix == ".stl":
        mesh = load_stl(inp)
    elif suffix == ".obj":
        mesh = load_obj(inp)
    else:
        raise ValueError(f"unsupported input format: {inp.suffix!r} (use .stl / .obj)")

    # Always export to STL inside the package (URDF mesh references prefer STL).
    visual_stl = visual_dir / f"{name}.stl"
    mesh.export(str(visual_stl), file_type="stl")
    (collision_dir / f"{name}.stl").write_bytes(visual_stl.read_bytes())

    material_name = cfg.materials.get(name, "aluminum_6061")
    mat = lookup(material_name)
    link = Link(
        name=name,
        visual_mesh_path=Path("meshes/visual") / f"{name}.stl",
        collision_mesh_path=Path("meshes/collision") / f"{name}.stl",
        material_density=mat.density_kg_m3,
        material_name=material_name,
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )
    return link, visual_stl


def _build_joints(cfg: JointsConfig) -> dict[str, Joint]:
    out: dict[str, Joint] = {}
    for js in cfg.joints:
        axis = np.asarray(js.axis, dtype=float)
        norm = float(np.linalg.norm(axis))
        if norm == 0.0:
            raise ValueError(f"joint {js.name!r}: axis cannot be the zero vector")
        out[js.name] = Joint(
            name=js.name,
            type=js.type,  # type: ignore[arg-type]
            parent=js.parent,
            child=js.child,
            axis=axis / norm,
            origin=origin_from_xyz_rpy(js.origin_xyz, js.origin_rpy),
            limit_lower=js.limit_lower,
            limit_upper=js.limit_upper,
            effort=js.effort,
            velocity=js.velocity,
        )
    return out


def _populate_inertia(link: Link, abs_visual_path: Path) -> Link:
    """Compute mass/COM/inertia from the on-disk STL and bake into a new InertialOverride.

    This lets the URDF emitter write the <inertial> block via the override-only path
    (which doesn't need absolute mesh paths in the Link), so we can do a single emit pass.
    """
    import trimesh as _trimesh  # local import; trimesh already in core deps

    mesh = _trimesh.load(str(abs_visual_path), file_type="stl", force="mesh")
    if not isinstance(mesh, _trimesh.Trimesh):
        # Couldn't load — keep the existing override (which may already be empty).
        return link
    mass, com, inertia = compute_inertial(
        mesh,
        density=link.material_density,
        override=link.inertial_override,
    )
    return Link(
        name=link.name,
        visual_mesh_path=link.visual_mesh_path,
        collision_mesh_path=link.collision_mesh_path,
        material_density=link.material_density,
        material_name=link.material_name,
        inertial_override=InertialOverride(mass=mass, com=com, inertia=inertia),
        origin=link.origin,
    )


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    logging.basicConfig(
        level=[logging.WARNING, logging.INFO, logging.DEBUG][min(args.verbose, 2)],
        format="%(levelname)s %(name)s: %(message)s",
    )

    try:
        cfg = load_joints_config(args.joints)
    except (FileNotFoundError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    if args.robot_name:
        cfg = JointsConfig(
            robot_name=args.robot_name,
            base_link=cfg.base_link,
            joints=cfg.joints,
            materials=cfg.materials,
        )
    package_name = args.package_name or args.out.name

    # ------------------------------------------------------------------
    # Rollback guard: track whether WE created the output directory.
    # If any step after the first disk write fails, remove what we wrote.
    # NOTE (Track B coordination): scaffold_ros_package call below already
    # includes base_link_name=cfg.base_link — do not duplicate that arg here.
    # ------------------------------------------------------------------
    out_dir_existed_before = args.out.exists()
    write_started = False

    def _cleanup_on_error() -> None:
        """Remove partially-written output directory if WE created it."""
        if write_started and not out_dir_existed_before and args.out.exists():
            try:
                shutil.rmtree(args.out)
                log.info("rolled back partial write to %s", args.out)
            except Exception as exc:
                log.warning("cleanup failed: %s", exc)

    try:
        args.out.mkdir(parents=True, exist_ok=True)
        write_started = True

        # Phase 1: load each mesh, write into the package, build Link records (relative paths).
        rel_links: dict[str, Link] = {}
        abs_paths: dict[str, Path] = {}
        for inp in args.inputs:
            try:
                link, abs_visual = _load_mesh_to_link(inp, args.out, cfg)
            except ValueError as e:
                print(f"error: {e}", file=sys.stderr)
                _cleanup_on_error()
                return 2
            rel_links[link.name] = link
            abs_paths[link.name] = abs_visual

        # Phase 2: build joints + Robot AST.
        try:
            joints = _build_joints(cfg)
            robot = Robot(
                name=cfg.robot_name,
                base_link=cfg.base_link,
                links=rel_links,
                joints=joints,
            )
        except ValueError as e:
            print(f"error: {e}", file=sys.stderr)
            _cleanup_on_error()
            return 2

        # Phase 3: structural validation (issues are warnings, not blockers).
        issues = validate_robot(robot)
        for i in issues:
            log.warning("validation issue: %s on %s — %s", i.kind, i.target, i.detail)

        # Phase 4: pre-compute inertia from on-disk STLs and bake into each link's
        # InertialOverride. Single-pass emit: no more absolute-path round trip; the
        # emitter takes the override-only path and writes a complete <inertial> block.
        links_with_inertia: dict[str, Link] = {
            n: _populate_inertia(rel_links[n], abs_paths[n]) for n in rel_links
        }
        robot_final = Robot(
            name=robot.name,
            base_link=robot.base_link,
            links=links_with_inertia,
            joints=robot.joints,
        )

        urdf_path = args.out / "urdf" / f"{cfg.robot_name}.urdf"

        # Phase 5: scaffold ROS package files (skeleton lands first so emit writes into it).
        scaffold_ros_package(
            out_dir=args.out,
            package_name=package_name,
            urdf_relpath=Path("urdf") / f"{cfg.robot_name}.urdf",
            maintainer_name=args.maintainer,
            maintainer_email=args.maintainer_email,
            base_link_name=cfg.base_link,
        )

        # Phase 6: emit URDF with package:// URIs (final, ROS-correct paths).
        emit_urdf(robot_final, urdf_path, package_name=package_name)

        print(f"wrote {urdf_path}")

        # Phase 7: optional ManipulaPy validation.
        if not args.no_validate:
            report = validate_urdf(urdf_path)
            if report.ok:
                print(f"ManipulaPy-compatible: {urdf_path}")
            else:
                print(
                    f"warning: ManipulaPy validation failed: {report.error}",
                    file=sys.stderr,
                )
                # don't exit nonzero — validation is informative, not blocking

    except Exception as e:
        # Safety net: catches unexpected failures (OSError on disk write, etc.)
        # that the per-phase except-ValueError blocks above do not cover.
        _cleanup_on_error()
        print(f"error: {e}", file=sys.stderr)
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
