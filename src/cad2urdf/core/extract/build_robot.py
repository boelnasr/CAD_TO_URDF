"""Turn a classified spanning tree + parts into a Robot (Task 4.3).

Leaf parts get a written STL + material; assembly nodes become meshless "frame"
links. Each classified tree edge becomes a joint. ``StepPart`` assigns
``id == name`` (both the sanitized, de-duplicated, URDF-valid name), so link
lookups by name are unambiguous.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from cad2urdf.core.extract.mate_classify import ClassifiedJoint
from cad2urdf.core.inertia.materials import Material
from cad2urdf.core.kinematic.model import InertialOverride, Joint, Link, Robot
from cad2urdf.core.parsers.step import StepPart

_LIMITED_JOINT_TYPES = frozenset({"revolute", "prismatic"})
_DEFAULT_CAD_JOINT_EFFORT = 1.0
_DEFAULT_CAD_JOINT_VELOCITY = 1.0


def build_robot_from_tree(
    *,
    parts: list[StepPart],
    tree: list[ClassifiedJoint],
    base_id: str,
    robot_name: str,
    mesh_dir: Path,
    material: Material,
) -> Robot:
    """Build a Robot from parts + a classified spanning tree.

    Leaf parts get a written STL + material; assembly nodes become meshless frame
    links. Each tree edge becomes a classified joint with the edge's relative pose.
    """
    by_id = {p.id: p for p in parts}
    node_ids = {base_id} | {e.parent_id for e in tree} | {e.child_id for e in tree}

    mesh_dir.mkdir(parents=True, exist_ok=True)
    links: dict[str, Link] = {}
    for nid in node_ids:
        part = by_id[nid]
        if part.mesh is not None:
            stl = (mesh_dir / f"{part.name}.stl").resolve()
            part.mesh.export(str(stl), file_type="stl")
            links[part.name] = Link(
                name=part.name,
                visual_mesh_path=stl,
                collision_mesh_path=stl,
                material_density=material.density_kg_m3,
                material_name=material.name,
                inertial_override=InertialOverride(),
            )
        else:
            links[part.name] = Link(
                name=part.name,
                visual_mesh_path=None,
                collision_mesh_path=None,
                material_density=None,
                material_name=None,
                inertial_override=InertialOverride(),
            )

    joints: dict[str, Joint] = {}
    for e in tree:
        pname, cname = by_id[e.parent_id].name, by_id[e.child_id].name
        jn = f"{cname}_to_{pname}"
        needs_limits = e.joint_type in _LIMITED_JOINT_TYPES
        origin = np.asarray(e.relative_pose, dtype=float)
        axis_parent = np.asarray(e.axis, dtype=float)
        axis_joint = origin[:3, :3].T @ axis_parent
        joints[jn] = Joint(
            name=jn,
            type=e.joint_type,
            parent=pname,
            child=cname,
            axis=axis_joint,
            origin=origin,
            limit_lower=e.limit_lower,
            limit_upper=e.limit_upper,
            # Static CAD geometry cannot infer actuator capability. Small,
            # usable placeholders keep emitted URDF valid until users replace
            # them with actuator-specific values in the editor/configuration.
            effort=_DEFAULT_CAD_JOINT_EFFORT if needs_limits else None,
            velocity=_DEFAULT_CAD_JOINT_VELOCITY if needs_limits else None,
        )

    return Robot(
        name=robot_name,
        base_link=by_id[base_id].name,
        links=links,
        joints=joints,
    )
