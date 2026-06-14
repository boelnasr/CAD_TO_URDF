"""Worker job: STEP file -> Robot (XCAF assembly pipeline)."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from cad2urdf.core.extract.build_robot import build_robot_from_tree
from cad2urdf.core.extract.mate_classify import classify_default_fixed
from cad2urdf.core.extract.spanning_tree import build_spanning_tree
from cad2urdf.core.inertia.materials import lookup
from cad2urdf.core.kinematic.model import Robot
from cad2urdf.core.parsers.step import load_step_assembly


def build_step_import_job(
    *, path: Path, mesh_dir: Path, robot_name: str
) -> Callable[[Callable[[int, int, str], None]], Robot]:
    """Return a job suitable for `Worker(job).start()`.

    The job reads a STEP assembly, builds a spanning tree of fixed joints, and
    assembles a Robot writing per-part STLs into `mesh_dir`. Progress is reported
    once per stage.
    """

    def job(report: Callable[[int, int, str], None]) -> Robot:
        report(1, 3, f"reading {path.name}")
        asm = load_step_assembly(path)
        report(2, 3, "building tree")
        tree = build_spanning_tree(classify_default_fixed(asm.edges), base_id=asm.root_id)
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
