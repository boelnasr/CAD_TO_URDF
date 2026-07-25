import numpy as np
import pytest

from cad2urdf.core.extract.mate_classify import ClassifiedJoint
from cad2urdf.core.extract.spanning_tree import (
    SpanningTreeError,
    build_spanning_tree,
)


def _cj(parent: str, child: str) -> ClassifiedJoint:
    return ClassifiedJoint(
        parent_id=parent,
        child_id=child,
        joint_type="fixed",
        axis=np.array([1.0, 0.0, 0.0]),
        relative_pose=np.eye(4),
    )


def test_chain_of_three_yields_unchanged_tree() -> None:
    edges = [_cj("a", "b"), _cj("b", "c")]
    out = build_spanning_tree(edges, base_id="a")
    assert len(out) == 2
    assert out[0].parent_id == "a"
    assert out[1].parent_id == "b"


def test_cycle_is_broken_with_one_edge_dropped() -> None:
    edges = [_cj("a", "b"), _cj("b", "c"), _cj("c", "a")]
    out = build_spanning_tree(edges, base_id="a")
    # spanning tree has |V|-1 = 2 edges
    assert len(out) == 2


def test_disconnected_subgraph_raises() -> None:
    edges = [_cj("a", "b"), _cj("c", "d")]  # two components
    with pytest.raises(SpanningTreeError, match="not reachable from base"):
        build_spanning_tree(edges, base_id="a")


def test_unknown_base_raises() -> None:
    edges = [_cj("a", "b")]
    with pytest.raises(SpanningTreeError, match=r"base_id .* not found"):
        build_spanning_tree(edges, base_id="ghost")


def test_empty_edge_list_with_unknown_base_raises() -> None:
    with pytest.raises(SpanningTreeError, match=r"base_id .* not found"):
        build_spanning_tree([], base_id="a")


def test_reversed_edge_inverts_pose_and_negates_axis() -> None:
    # Single edge p -> c with translation [1, 0, 0] along axis [1, 0, 0].
    # Build BFS rooted at c (the natural child) — the edge must reverse.
    pose = np.eye(4)
    pose[0, 3] = 1.0  # translate child 1m in +x relative to parent

    edge = ClassifiedJoint(
        parent_id="p",
        child_id="c",
        joint_type="revolute",
        axis=np.array([1.0, 0.0, 0.0]),
        relative_pose=pose,
    )
    out = build_spanning_tree([edge], base_id="c")
    assert len(out) == 1
    e = out[0]
    assert e.parent_id == "c"
    assert e.child_id == "p"
    # Inverted pose: parent is now at [-1, 0, 0] relative to child.
    expected_inv = np.eye(4)
    expected_inv[0, 3] = -1.0
    assert np.allclose(e.relative_pose, expected_inv)
    # Axis negated.
    assert np.allclose(e.axis, np.array([-1.0, 0.0, 0.0]))


def test_reversed_edge_rotates_axis_and_preserves_asymmetric_limits() -> None:
    # R maps child +X into parent +Y. Reversal must first express the original
    # parent-frame +Y axis in the new parent (old child), then negate it.
    pose = np.eye(4)
    pose[:3, :3] = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    edge = ClassifiedJoint(
        parent_id="p",
        child_id="c",
        joint_type="prismatic",
        axis=np.array([0.0, 1.0, 0.0]),
        relative_pose=pose,
        limit_lower=-0.02,
        limit_upper=0.08,
    )

    reversed_edge = build_spanning_tree([edge], base_id="c")[0]

    assert reversed_edge.axis == pytest.approx([-1.0, 0.0, 0.0])
    assert reversed_edge.limit_lower == pytest.approx(-0.02)
    assert reversed_edge.limit_upper == pytest.approx(0.08)

    # With the reversed axis, retaining q's parameterization makes each
    # endpoint transform exactly the inverse of the original endpoint.
    for coordinate in (-0.02, 0.08):
        original_joint_axis = edge.relative_pose[:3, :3].T @ edge.axis
        reversed_joint_axis = (
            reversed_edge.relative_pose[:3, :3].T @ reversed_edge.axis
        )
        original_motion = np.eye(4)
        original_motion[:3, 3] = original_joint_axis * coordinate
        reversed_motion = np.eye(4)
        reversed_motion[:3, 3] = reversed_joint_axis * coordinate
        original_endpoint = edge.relative_pose @ original_motion
        reversed_endpoint = reversed_edge.relative_pose @ reversed_motion
        assert reversed_endpoint == pytest.approx(np.linalg.inv(original_endpoint))


def test_sibling_body_forest_can_reverse_an_edge_without_remapping_limits() -> None:
    """A high-confidence sibling forest may root through an edge's child."""
    edges = [
        ClassifiedJoint(
            parent_id="frame",
            child_id="a",
            joint_type="fixed",
            axis=np.array([1.0, 0.0, 0.0]),
            relative_pose=np.eye(4),
        ),
        ClassifiedJoint(
            parent_id="a",
            child_id="c",
            joint_type="fixed",
            axis=np.array([1.0, 0.0, 0.0]),
            relative_pose=np.eye(4),
        ),
        ClassifiedJoint(
            parent_id="b",
            child_id="c",
            joint_type="prismatic",
            axis=np.array([0.0, 1.0, 0.0]),
            relative_pose=np.eye(4),
            limit_lower=-0.03,
            limit_upper=0.09,
        ),
    ]

    tree = build_spanning_tree(edges, base_id="frame")
    reversed_edge = next(edge for edge in tree if edge.child_id == "b")

    assert (reversed_edge.parent_id, reversed_edge.child_id) == ("c", "b")
    assert reversed_edge.axis == pytest.approx([0.0, -1.0, 0.0])
    assert reversed_edge.limit_lower == pytest.approx(-0.03)
    assert reversed_edge.limit_upper == pytest.approx(0.09)
