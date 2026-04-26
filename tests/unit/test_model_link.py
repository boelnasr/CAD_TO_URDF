from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Link


def test_link_constructs_with_required_fields() -> None:
    link = Link(
        name="base",
        visual_mesh_path=Path("meshes/base.stl"),
        collision_mesh_path=Path("meshes/base.stl"),
        material_density=2700.0,
        material_name="aluminum",
        inertial_override=InertialOverride(),
        origin=np.eye(4),
    )
    assert link.name == "base"
    assert link.material_density == 2700.0


def test_link_rejects_non_4x4_origin() -> None:
    with pytest.raises(ValueError, match="origin must be shape"):
        Link(
            name="bad",
            visual_mesh_path=Path("x"),
            collision_mesh_path=Path("x"),
            material_density=1.0,
            material_name="m",
            inertial_override=InertialOverride(),
            origin=np.eye(3),
        )


def test_link_rejects_empty_name() -> None:
    with pytest.raises(ValueError, match="name must not be empty"):
        Link(
            name="",
            visual_mesh_path=Path("x"),
            collision_mesh_path=Path("x"),
            material_density=1.0,
            material_name="m",
            inertial_override=InertialOverride(),
            origin=np.eye(4),
        )


def test_link_rejects_non_positive_density() -> None:
    with pytest.raises(ValueError, match="material_density must be positive"):
        Link(
            name="bad",
            visual_mesh_path=Path("x"),
            collision_mesh_path=Path("x"),
            material_density=0.0,
            material_name="m",
            inertial_override=InertialOverride(),
            origin=np.eye(4),
        )
