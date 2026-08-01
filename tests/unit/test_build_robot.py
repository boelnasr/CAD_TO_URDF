from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh

from cad2urdf.core.extract.build_robot import build_robot_from_tree
from cad2urdf.core.extract.mate_classify import ClassifiedJoint
from cad2urdf.core.inertia.materials import lookup
from cad2urdf.core.parsers.step import StepPart


def _box() -> trimesh.Trimesh:
    return trimesh.creation.box(extents=(0.01, 0.01, 0.01))


def test_build_robot_frame_root_with_mesh_children(tmp_path: Path) -> None:
    parts = [
        StepPart(id="robot", name="robot", mesh=None, is_leaf=False),
        StepPart(id="base", name="base", mesh=_box(), is_leaf=True),
        StepPart(id="arm", name="arm", mesh=_box(), is_leaf=True),
    ]
    pose = np.eye(4)
    pose[2, 3] = 0.01
    tree = [
        ClassifiedJoint("robot", "base", "fixed", np.array([1.0, 0, 0]), np.eye(4)),
        ClassifiedJoint("robot", "arm", "fixed", np.array([1.0, 0, 0]), pose),
    ]
    robot = build_robot_from_tree(
        parts=parts,
        tree=tree,
        base_id="robot",
        robot_name="r",
        mesh_dir=tmp_path,
        material=lookup("aluminum_6061"),
    )
    assert robot.base_link == "robot"
    assert robot.links["robot"].visual_mesh_path is None  # frame
    assert robot.links["base"].visual_mesh_path is not None  # mesh
    # Builder writes ABSOLUTE STL paths; the CLI remaps them to package-relative.
    assert robot.links["base"].visual_mesh_path.is_absolute()
    assert (tmp_path / "base.stl").is_file()
    assert robot.joints["arm_to_robot"].origin[2, 3] == 0.01


def test_build_robot_propagates_classified_joint_limits(tmp_path: Path) -> None:
    parts = [
        StepPart(id="base", name="base", mesh=_box(), is_leaf=True),
        StepPart(id="slider", name="slider", mesh=_box(), is_leaf=True),
    ]
    tree = [
        ClassifiedJoint(
            parent_id="base",
            child_id="slider",
            joint_type="prismatic",
            axis=np.array([0.0, 0.0, 1.0]),
            relative_pose=np.eye(4),
            limit_lower=-0.03,
            limit_upper=0.07,
        )
    ]

    robot = build_robot_from_tree(
        parts=parts,
        tree=tree,
        base_id="base",
        robot_name="r",
        mesh_dir=tmp_path,
        material=lookup("aluminum_6061"),
    )

    assert robot.joints["slider_to_base"].limit_lower == -0.03
    assert robot.joints["slider_to_base"].limit_upper == 0.07
    assert robot.joints["slider_to_base"].effort == 1.0
    assert robot.joints["slider_to_base"].velocity == 1.0
