import numpy as np
import trimesh

from cad2urdf.core.inertia.compute import compute_inertial
from cad2urdf.core.kinematic.model import InertialOverride


def test_unit_cube_aluminum_mass() -> None:
    cube = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    mass, _, _ = compute_inertial(cube, density=2700.0, override=InertialOverride())
    assert np.isclose(mass, 2700.0, rtol=1e-6)


def test_unit_cube_aluminum_inertia_diagonal() -> None:
    cube = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    _, _, inertia = compute_inertial(cube, density=2700.0, override=InertialOverride())
    # for a uniform cube of mass m and side s, I_xx = m * s^2 / 6
    expected_diag = 2700.0 * (1.0**2) / 6.0
    assert np.allclose(np.diag(inertia), expected_diag, rtol=1e-3)


def test_override_mass_wins_over_auto() -> None:
    cube = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    mass, _, _ = compute_inertial(cube, density=2700.0, override=InertialOverride(mass=5.0))
    assert mass == 5.0


def test_partial_override_only_replaces_set_fields() -> None:
    cube = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    custom_com = np.array([0.1, 0.2, 0.3])
    mass, com, _ = compute_inertial(cube, density=2700.0, override=InertialOverride(com=custom_com))
    assert np.allclose(com, custom_com)
    assert np.isclose(mass, 2700.0)


def test_negative_density_raises() -> None:
    import pytest

    cube = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    with pytest.raises(ValueError, match="density must be positive"):
        compute_inertial(cube, density=-1.0, override=InertialOverride())


def test_non_watertight_falls_back_to_convex_hull(caplog) -> None:
    import logging

    # open mesh: tetrahedron with one face removed (4 verts, 3 faces -> non-watertight,
    # but enough points for scipy QHull to build a 3D convex hull)
    verts = np.array(
        [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
        dtype=float,
    )
    faces = np.array([[0, 1, 2], [0, 1, 3], [0, 2, 3]])
    open_mesh = trimesh.Trimesh(vertices=verts, faces=faces)
    assert not open_mesh.is_watertight

    with caplog.at_level(logging.WARNING):
        mass, _, _ = compute_inertial(open_mesh, density=1000.0, override=InertialOverride())

    assert mass >= 0.0
    assert any("non-watertight" in rec.message for rec in caplog.records)
