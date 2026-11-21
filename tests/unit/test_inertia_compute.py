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


def test_convex_hull_failure_returns_zero_fallback(caplog) -> None:
    """Degenerate mesh (3 colinear points) causes QhullError; function must not raise."""
    import logging
    import unittest.mock as mock

    # A single triangle (3 coplanar/colinear verts) — convex_hull will fail because
    # scipy QHull cannot build a 3-D hull from coplanar/degenerate input.
    verts = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float)
    faces = np.array([[0, 1, 2]])
    bad_mesh = trimesh.Trimesh(vertices=verts, faces=faces)
    assert not bad_mesh.is_watertight

    # Force convex_hull to raise regardless of trimesh version behaviour.
    with (
        mock.patch.object(
            type(bad_mesh),
            "convex_hull",
            new_callable=lambda: property(
                lambda self: (_ for _ in ()).throw(Exception("QhullError: degenerate"))
            ),
        ),
        caplog.at_level(logging.ERROR),
    ):
        mass, com, inertia = compute_inertial(bad_mesh, density=1000.0, override=InertialOverride())

    assert mass == 0.0
    assert np.array_equal(com, np.zeros(3))
    assert np.array_equal(inertia, np.zeros((3, 3)))
    assert any("convex hull failed" in rec.message for rec in caplog.records)


def test_compute_does_not_mutate_input_mesh_density() -> None:
    """compute_inertial must not alter the caller's mesh.density."""
    cube = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    cube.density = 1000.0
    compute_inertial(cube, density=2700.0, override=InertialOverride())
    assert cube.density == 1000.0


def test_inside_out_watertight_mesh_is_inverted_not_negative_issue21() -> None:
    """A mirrored/inside-out export must give the true positive mass, not -m."""
    cube = trimesh.creation.box(extents=(0.1, 0.1, 0.1))
    inside_out = cube.copy()
    inside_out.invert()
    faces_before = inside_out.faces.copy()
    assert inside_out.is_watertight and inside_out.volume < 0

    mass, com, inertia = compute_inertial(inside_out, density=2700.0, override=InertialOverride())

    assert np.isclose(mass, 2.7, rtol=1e-6)
    assert np.allclose(com, 0.0, atol=1e-12)
    assert np.allclose(np.diag(inertia), 2.7 * 0.1**2 / 6.0, rtol=1e-3)
    # The caller's mesh must keep its own winding.
    assert np.array_equal(inside_out.faces, faces_before)


def test_inconsistent_winding_falls_back_to_convex_hull_issue21(caplog) -> None:
    """Watertight but inconsistently wound meshes must not be integrated as-is."""
    import logging

    cube = trimesh.creation.box(extents=(0.1, 0.1, 0.1))
    faces = cube.faces.copy()
    faces[:2] = faces[:2, ::-1]
    flipped = trimesh.Trimesh(cube.vertices, faces, process=False)
    assert flipped.is_watertight and not flipped.is_winding_consistent

    with caplog.at_level(logging.WARNING):
        mass, com, _ = compute_inertial(flipped, density=2700.0, override=InertialOverride())

    assert np.isclose(mass, 2.7, rtol=1e-6)
    assert np.allclose(com, 0.0, atol=1e-12)
    assert any("convex hull" in rec.message for rec in caplog.records)
