"""Worker job: STEP/IGES file -> Robot (XCAF assembly pipeline)."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from cad2urdf.core.extract.build_robot import build_robot_from_tree
from cad2urdf.core.extract.joint_infer import infer_edge_hints
from cad2urdf.core.extract.mate_classify import classify_default_fixed, classify_from_hints
from cad2urdf.core.extract.spanning_tree import build_spanning_tree
from cad2urdf.core.inertia.materials import lookup
from cad2urdf.core.kinematic.model import Robot


def build_cad_import_job(
    *, path: Path, mesh_dir: Path, robot_name: str
) -> Callable[[Callable[[int, int, str], None]], Robot]:
    """Return a job suitable for `Worker(job).start()`.

    The job reads a CAD assembly, infers conservative joint hints, builds a
    spanning tree, and assembles a Robot writing per-part STLs into `mesh_dir`.
    Progress is reported once per stage.
    """
    suffix = path.suffix.lower()
    if suffix in {".step", ".stp"}:
        from cad2urdf.core.parsers.step import load_step_assembly_with_shapes

        loader = load_step_assembly_with_shapes
    elif suffix in {".iges", ".igs"}:
        from cad2urdf.core.parsers.iges import load_iges_assembly_with_shapes

        loader = load_iges_assembly_with_shapes
    else:
        raise ValueError(f"unsupported CAD assembly format: {path.suffix!r}")

    def job(report: Callable[[int, int, str], None]) -> Robot:
        report(1, 3, f"reading {path.name}")
        asm, shapes = loader(path)
        try:
            classified = (
                classify_from_hints(infer_edge_hints(asm.edges, shapes))
                if shapes
                else classify_default_fixed(asm.edges)
            )
        finally:
            shapes.clear()
            del shapes
        report(2, 3, "building tree")
        tree = (
            build_spanning_tree(classified, base_id=asm.root_id)
            if asm.edges
            else []
        )
        report(3, 3, "assembling robot")
        return build_robot_from_tree(
            parts=asm.parts,
            tree=tree,
            base_id=asm.root_id,
            robot_name=robot_name,
            mesh_dir=mesh_dir,
            material=lookup("aluminum_6061"),
        )

    return job


# Compatibility for existing callers while the public surface moves to CAD.
build_step_import_job = build_cad_import_job
