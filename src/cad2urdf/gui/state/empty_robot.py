"""Factory for the placeholder Robot shown when no project has been loaded yet."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from cad2urdf.core.kinematic.model import InertialOverride, Link, Robot


def make_empty_robot() -> Robot:
    """Return a one-link Robot named 'untitled' that satisfies all invariants.

    The GUI uses this as the initial AST so panels always have something
    consistent to render before the user imports meshes.
    """
    base = Link(
        name="base",
        visual_mesh_path=Path("meshes/visual/base.stl"),
        collision_mesh_path=Path("meshes/collision/base.stl"),
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )
    return Robot(name="untitled", base_link="base", links={"base": base}, joints={})
