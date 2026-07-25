from __future__ import annotations

import builtins
import importlib
import math
import sys

import numpy as np
import pytest

from cad2urdf.core.extract.mate_classify import classify_from_hints
from cad2urdf.core.extract.types import AssemblyEdge


def _edge(**hints: object) -> AssemblyEdge:
    return AssemblyEdge(
        parent_id="parent",
        child_id="child",
        relative_pose=np.eye(4),
        **hints,
    )


def test_joint_infer_module_import_does_not_require_ocp(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_import = builtins.__import__

    def reject_ocp(name: str, *args: object, **kwargs: object) -> object:
        if name == "OCP" or name.startswith("OCP."):
            raise ImportError("OCP intentionally unavailable")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", reject_ocp)
    sys.modules.pop("cad2urdf.core.extract.joint_infer", None)

    module = importlib.import_module("cad2urdf.core.extract.joint_infer")

    assert callable(module.infer_edge_hints)


def test_high_confidence_unbounded_concentric_hint_is_continuous() -> None:
    joints = classify_from_hints(
        [_edge(mate_kind="concentric", confidence=0.9, axis=np.array([0.0, 0.0, 4.0]))]
    )

    assert joints[0].joint_type == "continuous"
    assert joints[0].axis == pytest.approx([0.0, 0.0, 1.0])
    assert joints[0].limit_lower is None
    assert joints[0].limit_upper is None


def test_bounded_concentric_hint_is_revolute_and_preserves_limits() -> None:
    joints = classify_from_hints(
        [
            _edge(
                mate_kind="concentric",
                confidence=0.9,
                axis=np.array([1.0, 0.0, 0.0]),
                limit_lower=-0.75,
                limit_upper=1.25,
            )
        ]
    )

    assert joints[0].joint_type == "revolute"
    assert joints[0].limit_lower == pytest.approx(-0.75)
    assert joints[0].limit_upper == pytest.approx(1.25)


@pytest.mark.parametrize(
    ("limit_lower", "limit_upper", "expected_lower", "expected_upper"),
    [
        (-0.5, None, -0.5, math.pi),
        (None, 0.5, -math.pi, 0.5),
    ],
)
def test_partially_known_concentric_bounds_use_symmetric_defaults(
    limit_lower: float | None,
    limit_upper: float | None,
    expected_lower: float,
    expected_upper: float,
) -> None:
    joints = classify_from_hints(
        [
            _edge(
                mate_kind="concentric",
                confidence=0.9,
                axis=np.array([1.0, 0.0, 0.0]),
                limit_lower=limit_lower,
                limit_upper=limit_upper,
            )
        ]
    )

    assert joints[0].joint_type == "revolute"
    assert joints[0].limit_lower == pytest.approx(expected_lower)
    assert joints[0].limit_upper == pytest.approx(expected_upper)


def test_high_confidence_sliding_hint_is_prismatic_with_trustworthy_extents() -> None:
    joints = classify_from_hints(
        [
            _edge(
                mate_kind="sliding",
                confidence=0.8,
                axis=np.array([0.0, 2.0, 0.0]),
                limit_lower=-0.025,
                limit_upper=0.075,
            )
        ]
    )

    assert joints[0].joint_type == "prismatic"
    assert joints[0].axis == pytest.approx([0.0, 1.0, 0.0])
    assert joints[0].limit_lower == pytest.approx(-0.025)
    assert joints[0].limit_upper == pytest.approx(0.075)


def test_sliding_hint_without_trustworthy_extents_uses_safe_metre_defaults() -> None:
    joints = classify_from_hints(
        [_edge(mate_kind="sliding", confidence=0.8, axis=np.array([0.0, 1.0, 0.0]))]
    )

    assert joints[0].joint_type == "prismatic"
    assert joints[0].limit_lower == pytest.approx(-0.1)
    assert joints[0].limit_upper == pytest.approx(0.1)


@pytest.mark.parametrize(
    "edge",
    [
        _edge(mate_kind="concentric", confidence=0.59, axis=np.array([0.0, 0.0, 1.0])),
        _edge(mate_kind="sliding", confidence=0.9, axis=None),
        _edge(mate_kind="concentric", confidence=0.9, axis=np.zeros(3)),
        _edge(mate_kind="concentric", confidence=0.9, axis=np.array([np.nan, 0.0, 1.0])),
        _edge(mate_kind="unknown", confidence=1.0, axis=np.array([1.0, 0.0, 0.0])),
    ],
)
def test_uncertain_or_invalid_hints_downgrade_to_fixed(edge: AssemblyEdge) -> None:
    joints = classify_from_hints([edge])

    assert joints[0].joint_type == "fixed"
    assert joints[0].axis == pytest.approx([1.0, 0.0, 0.0])
    assert joints[0].limit_lower is None
    assert joints[0].limit_upper is None


def test_default_confidence_threshold_accepts_exactly_point_six() -> None:
    joints = classify_from_hints(
        [_edge(mate_kind="concentric", confidence=0.6, axis=np.array([0.0, 0.0, 1.0]))]
    )

    assert joints[0].joint_type == "continuous"
