import numpy as np

from cad2urdf.core.kinematic.model import InertialOverride


def test_default_override_is_all_none() -> None:
    o = InertialOverride()
    assert o.mass is None
    assert o.com is None
    assert o.inertia is None


def test_override_accepts_partial_fields() -> None:
    o = InertialOverride(mass=1.5)
    assert o.mass == 1.5
    assert o.com is None


def test_override_validates_com_shape() -> None:
    import pytest

    with pytest.raises(ValueError, match="com must be shape"):
        InertialOverride(com=np.zeros(2))


def test_override_validates_inertia_shape() -> None:
    import pytest

    with pytest.raises(ValueError, match="inertia must be shape"):
        InertialOverride(inertia=np.zeros((2, 2)))
