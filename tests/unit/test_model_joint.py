import numpy as np
import pytest

from cad2urdf.core.kinematic.model import Joint


def test_joint_constructs_revolute() -> None:
    j = Joint(
        name="shoulder",
        type="revolute",
        parent="base",
        child="link_1",
        axis=np.array([0.0, 0.0, 1.0]),
        origin=np.eye(4),
        limit_lower=-3.14,
        limit_upper=3.14,
        effort=100.0,
        velocity=2.0,
    )
    assert j.type == "revolute"
    assert j.limit_lower == -3.14


def test_joint_rejects_non_unit_axis() -> None:
    with pytest.raises(ValueError, match="axis must be unit-length"):
        Joint(
            name="bad",
            type="revolute",
            parent="a",
            child="b",
            axis=np.array([0.0, 0.0, 2.0]),
            origin=np.eye(4),
        )


def test_joint_rejects_invalid_type() -> None:
    with pytest.raises(ValueError, match="type must be one of"):
        Joint(
            name="bad",
            type="weird",  # type: ignore[arg-type]
            parent="a",
            child="b",
            axis=np.array([1.0, 0.0, 0.0]),
            origin=np.eye(4),
        )


def test_joint_rejects_self_loop() -> None:
    with pytest.raises(ValueError, match="parent and child must differ"):
        Joint(
            name="loop",
            type="fixed",
            parent="x",
            child="x",
            axis=np.array([1.0, 0.0, 0.0]),
            origin=np.eye(4),
        )


def test_joint_rejects_inverted_limits() -> None:
    with pytest.raises(ValueError, match=r"limit_lower .* must be <="):
        Joint(
            name="bad",
            type="revolute",
            parent="a",
            child="b",
            axis=np.array([1.0, 0.0, 0.0]),
            origin=np.eye(4),
            limit_lower=1.0,
            limit_upper=-1.0,
        )


def test_joint_rejects_empty_name() -> None:
    with pytest.raises(ValueError, match="name must not be empty"):
        Joint(
            name="",
            type="fixed",
            parent="a",
            child="b",
            axis=np.array([1.0, 0.0, 0.0]),
            origin=np.eye(4),
        )


def test_joint_rejects_non_3_axis_shape() -> None:
    with pytest.raises(ValueError, match="axis must be shape"):
        Joint(
            name="bad",
            type="revolute",
            parent="a",
            child="b",
            axis=np.array([1.0, 0.0]),
            origin=np.eye(4),
        )
