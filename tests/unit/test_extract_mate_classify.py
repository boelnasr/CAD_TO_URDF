import numpy as np

from cad2urdf.core.extract.mate_classify import (
    ClassifiedJoint,
    classify_default_fixed,
)
from cad2urdf.core.extract.types import AssemblyEdge


def test_default_classification_is_fixed() -> None:
    edges = [
        AssemblyEdge(parent_id="root", child_id="a", relative_pose=np.eye(4)),
        AssemblyEdge(parent_id="root", child_id="b", relative_pose=np.eye(4)),
    ]
    out = classify_default_fixed(edges)
    assert len(out) == 2
    assert all(isinstance(c, ClassifiedJoint) for c in out)
    assert all(c.joint_type == "fixed" for c in out)
    assert {c.parent_id for c in out} == {"root"}
    assert {c.child_id for c in out} == {"a", "b"}


def test_default_classification_preserves_relative_pose() -> None:
    pose = np.eye(4)
    pose[0, 3] = 1.5
    edges = [AssemblyEdge(parent_id="p", child_id="c", relative_pose=pose)]
    out = classify_default_fixed(edges)
    assert np.allclose(out[0].relative_pose, pose)


def test_classify_empty_list_returns_empty_list() -> None:
    out = classify_default_fixed([])
    assert out == []
