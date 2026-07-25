"""cad2urdf CLI: input meshes + joint config → ROS 2 URDF package.

STL/OBJ inputs (one mesh per link, joints from YAML) or one CAD assembly
(.step/.stp/.iges/.igs; each part becomes a fixed-jointed link).
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

CAD_ASSEMBLY_SUFFIXES = frozenset({".step", ".stp", ".iges", ".igs"})
CAD_MULTI_INPUT_MSG = (
    "CAD assemblies (.step/.stp/.iges/.igs) must be passed as a single input "
    "file (the whole assembly becomes one Robot); do not mix CAD with other meshes."
)
# Private compatibility for callers that imported the old STEP-specific name.
STEP_MULTI_INPUT_MSG = CAD_MULTI_INPUT_MSG


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="cad2urdf", description="Convert CAD assemblies to ROS 2 URDF packages."
    )
    p.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help=(
            "Input STL/OBJ files (one per link), or one CAD assembly "
            "(.step/.stp/.iges/.igs)."
        ),
    )
    p.add_argument("-o", "--out", type=Path, required=True, help="Output package directory.")
    p.add_argument(
        "--joints",
        type=Path,
        help="Joints YAML config (required for STL/OBJ inputs; ignored for CAD assemblies).",
    )
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
    if suffix in CAD_ASSEMBLY_SUFFIXES:
        # CAD is handled by the dedicated single-input route in main(); reaching
        # here means it was mixed with other meshes, which is unsupported.
        raise ValueError(CAD_MULTI_INPUT_MSG)

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


def _build_cad_robot(inp: Path, out_dir: Path, robot_name: str) -> Robot:
    """Import a STEP/IGES assembly into the package emit/scaffold flow.

    `build_robot_from_tree` writes per-part STLs with *absolute* visual/collision
    paths; the emitter requires *package-relative* paths (`package://` URIs). We
    therefore build into the package's `meshes/visual` dir, mirror each mesh into
    `meshes/collision`, bake inertia from the on-disk STL, and rewrite each mesh
    Link to relative paths — matching the STL pipeline's emit contract. Frame
    (meshless) links pass through untouched.
    """
    try:
        from cad2urdf.core.extract.build_robot import build_robot_from_tree
        from cad2urdf.core.extract.mate_classify import classify_default_fixed
        from cad2urdf.core.extract.spanning_tree import build_spanning_tree
    except ImportError as e:  # pragma: no cover - exercised only without OCP
        raise ValueError(
            "CAD import requires OCP. Install it with: pip install -e '.[step]'"
        ) from e

    suffix = inp.suffix.lower()
    if suffix in {".step", ".stp"}:
        from cad2urdf.core.parsers.step import load_step_assembly

        loader = load_step_assembly
    elif suffix in {".iges", ".igs"}:
        from cad2urdf.core.parsers.iges import load_iges_assembly

        loader = load_iges_assembly
    else:
        raise ValueError(f"unsupported CAD assembly format: {inp.suffix!r}")

    visual_dir = out_dir / "meshes" / "visual"
    collision_dir = out_dir / "meshes" / "collision"
    visual_dir.mkdir(parents=True, exist_ok=True)
    collision_dir.mkdir(parents=True, exist_ok=True)

    try:
        asm = loader(inp)
    except ImportError as e:  # pragma: no cover - exercised only without OCP
        raise ValueError(
            "CAD import requires OCP. Install it with: pip install -e '.[step]'"
        ) from e
    tree = (
        build_spanning_tree(classify_default_fixed(asm.edges), base_id=asm.root_id)
        if asm.edges
        else []
    )
    raw = build_robot_from_tree(
        parts=asm.parts,
        tree=tree,
        base_id=asm.root_id,
        robot_name=robot_name,
        mesh_dir=visual_dir,
        material=lookup("aluminum_6061"),
    )

    links: dict[str, Link] = {}
    for link in raw.links.values():
        if link.visual_mesh_path is None:
            links[link.name] = link  # frame link: nothing on disk to rewrite
            continue
        abs_visual = link.visual_mesh_path
        rel = Path("meshes/visual") / abs_visual.name
        # Mirror the visual STL into the collision tree (matching the STL pipeline).
        (collision_dir / abs_visual.name).write_bytes(abs_visual.read_bytes())
        rel_link = Link(
            name=link.name,
            visual_mesh_path=rel,
            collision_mesh_path=Path("meshes/collision") / abs_visual.name,
            material_density=link.material_density,
            material_name=link.material_name,
            inertial_override=link.inertial_override,
            origin=link.origin,
        )
        links[link.name] = _populate_inertia(rel_link, abs_visual)

    return Robot(name=raw.name, base_link=raw.base_link, links=links, joints=raw.joints)


def _build_step_robot(inp: Path, out_dir: Path, robot_name: str) -> Robot:
    """Compatibility wrapper for the former STEP-specific private helper."""
    return _build_cad_robot(inp, out_dir, robot_name)


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

    if link.material_density is None:
        # No density (e.g. a meshless frame link) — nothing to auto-compute.
        return link
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

    cad_inputs = [
        path for path in args.inputs if path.suffix.lower() in CAD_ASSEMBLY_SUFFIXES
    ]
    is_cad_assembly = len(args.inputs) == 1 and len(cad_inputs) == 1
    if cad_inputs and not is_cad_assembly:
        print(f"error: {CAD_MULTI_INPUT_MSG}", file=sys.stderr)
        return 2

    cfg: JointsConfig | None = None
    if not is_cad_assembly:
        if args.joints is None:
            print("error: --joints is required for STL/OBJ inputs", file=sys.stderr)
            return 2
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

        if is_cad_assembly:
            # CAD path: a single assembly produces the whole Robot (links + joints).
            # _build_cad_robot writes per-part STLs into the package and returns a
            # Robot with package-relative mesh paths + baked inertia, ready to emit.
            robot_name = args.robot_name or args.inputs[0].stem
            try:
                robot_final = _build_cad_robot(args.inputs[0], args.out, robot_name)
            except (ValueError, FileNotFoundError) as e:
                print(f"error: {e}", file=sys.stderr)
                _cleanup_on_error()
                return 2
            base_link = robot_final.base_link
        else:
            assert cfg is not None  # guaranteed by the CAD assembly branch above
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
            robot_name = cfg.robot_name
            base_link = cfg.base_link

        # Phase 3: structural validation (issues are warnings, not blockers).
        issues = validate_robot(robot_final)
        for i in issues:
            log.warning("validation issue: %s on %s — %s", i.kind, i.target, i.detail)

        urdf_path = args.out / "urdf" / f"{robot_name}.urdf"

        # Phase 5: scaffold ROS package files (skeleton lands first so emit writes into it).
        scaffold_ros_package(
            out_dir=args.out,
            package_name=package_name,
            urdf_relpath=Path("urdf") / f"{robot_name}.urdf",
            maintainer_name=args.maintainer,
            maintainer_email=args.maintainer_email,
            base_link_name=base_link,
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
