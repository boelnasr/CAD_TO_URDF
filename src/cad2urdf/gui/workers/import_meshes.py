"""Build a worker job that loads N mesh files and returns a seed Robot AST.

The seed AST has one Link per mesh (named after the file stem). The first
mesh in the list becomes base_link; every other mesh is attached with a
fixed joint named `<child>_to_<base>`. The user reshapes the tree afterwards
in the link tree dock.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import numpy as np

from cad2urdf.core.inertia.materials import lookup
from cad2urdf.core.kinematic.model import (
    InertialOverride,
    Joint,
    Link,
    Robot,
)
from cad2urdf.core.parsers.obj import load_obj
from cad2urdf.core.parsers.stl import load_stl

STEP_NOT_SUPPORTED = (
    "STEP / IGES inputs require pythonOCC-core (conda-only). "
    "Install via `conda env create -f environment.yml` and rerun. "
    "v1 GUI supports STL/OBJ only."
)


def _load_one(path: Path) -> tuple[str, Path]:
    """Validate and load a single mesh. Returns (link_name, absolute_path).

    Raises ValueError on unsupported extensions (including STEP/IGES) and
    FileNotFoundError on missing files.
    """
    suffix = path.suffix.lower()
    if suffix in {".step", ".stp", ".iges", ".igs"}:
        raise ValueError(STEP_NOT_SUPPORTED)
    if suffix == ".stl":
        load_stl(path)
    elif suffix == ".obj":
        load_obj(path)
    else:
        raise ValueError(f"unsupported mesh extension {suffix!r} (use .stl or .obj)")
    return path.stem, path.resolve()


def build_import_job(
    *, paths: list[Path], robot_name: str
) -> Callable[[Callable[[int, int, str], None]], Robot]:
    """Return a job suitable for `Worker(job).start()`.

    The job is a closure capturing `paths` + `robot_name`; the returned
    callable is what `Worker._JobRunner.run()` invokes. Progress is reported
    once per mesh.
    """

    def job(report: Callable[[int, int, str], None]) -> Robot:
        if not paths:
            raise ValueError("no input mesh paths provided")
        total = len(paths)
        link_records: list[tuple[str, Path]] = []
        for i, p in enumerate(paths, start=1):
            report(i, total, f"loading {p.name}")
            link_records.append(_load_one(p))

        # All names must be unique (file stems collide → require user to rename).
        names = [name for name, _ in link_records]
        if len(set(names)) != len(names):
            dupes = sorted({n for n in names if names.count(n) > 1})
            raise ValueError(f"duplicate link names from file stems: {dupes}")

        material = lookup("aluminum_6061")
        links: dict[str, Link] = {}
        for name, abs_path in link_records:
            links[name] = Link(
                name=name,
                visual_mesh_path=abs_path,
                collision_mesh_path=abs_path,
                material_density=material.density_kg_m3,
                material_name=material.name,
                inertial_override=InertialOverride(),
                origin=np.eye(4),
            )

        base_name = names[0]
        joints: dict[str, Joint] = {}
        for child in names[1:]:
            jn = f"{child}_to_{base_name}"
            joints[jn] = Joint(
                name=jn,
                type="fixed",
                parent=base_name,
                child=child,
                axis=np.array([1.0, 0.0, 0.0]),
                origin=np.eye(4),
            )

        return Robot(name=robot_name, base_link=base_name, links=links, joints=joints)

    return job
