"""Robot AST → URDF 1.0 XML. Stdlib only; deterministic ordering."""

from __future__ import annotations

from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

import numpy as np
import trimesh
from numpy.typing import NDArray

from cad2urdf.core.inertia.compute import compute_inertial
from cad2urdf.core.kinematic.model import Joint, Link, Robot


def _format_xyz(v: NDArray[Any]) -> str:
    return " ".join(f"{x:g}" for x in v.tolist())


def _origin_xyz_rpy(m: NDArray[Any]) -> tuple[str, str]:
    """Decompose a 4x4 homogeneous transform into URDF xyz + roll-pitch-yaw strings."""
    xyz = m[:3, 3]
    rot = m[:3, :3]
    pitch = float(np.arcsin(-rot[2, 0]))
    if abs(np.cos(pitch)) > 1e-6:
        roll = float(np.arctan2(rot[2, 1], rot[2, 2]))
        yaw = float(np.arctan2(rot[1, 0], rot[0, 0]))
    else:
        roll = float(np.arctan2(-rot[1, 2], rot[1, 1]))
        yaw = 0.0
    return _format_xyz(xyz), f"{roll:g} {pitch:g} {yaw:g}"


def _emit_inertial(parent: ET.Element, link: Link) -> None:
    """Emit <inertial> block.

    Auto-computes from mesh if path is absolute and loadable; otherwise emits
    override-only (or skips if no override at all).
    """
    visual = link.visual_mesh_path
    if visual.is_absolute() and visual.is_file():
        mesh = trimesh.load(str(visual), file_type="stl", force="mesh")
        if not isinstance(mesh, trimesh.Trimesh):
            return
        mass, com, inertia = compute_inertial(
            mesh, density=link.material_density, override=link.inertial_override
        )
    else:
        ov = link.inertial_override
        if ov.mass is None or ov.com is None or ov.inertia is None:
            return  # nothing to emit; URDF treats missing inertial as a default unit mass
        mass, com, inertia = ov.mass, ov.com, ov.inertia

    el = ET.SubElement(parent, "inertial")
    ET.SubElement(el, "origin", attrib={"xyz": _format_xyz(com), "rpy": "0 0 0"})
    ET.SubElement(el, "mass", attrib={"value": f"{mass:g}"})
    ET.SubElement(
        el,
        "inertia",
        attrib={
            "ixx": f"{inertia[0, 0]:g}",
            "ixy": f"{inertia[0, 1]:g}",
            "ixz": f"{inertia[0, 2]:g}",
            "iyy": f"{inertia[1, 1]:g}",
            "iyz": f"{inertia[1, 2]:g}",
            "izz": f"{inertia[2, 2]:g}",
        },
    )


def _mesh_uri(package_name: str, rel: Path) -> str:
    return f"package://{package_name}/{rel.as_posix()}"


def _emit_link(parent: ET.Element, link: Link, package_name: str) -> None:
    el = ET.SubElement(parent, "link", attrib={"name": link.name})
    _emit_inertial(el, link)

    visual = ET.SubElement(el, "visual")
    ET.SubElement(visual, "origin", attrib={"xyz": "0 0 0", "rpy": "0 0 0"})
    geom = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geom,
        "mesh",
        attrib={"filename": _mesh_uri(package_name, link.visual_mesh_path)},
    )
    ET.SubElement(visual, "material", attrib={"name": link.material_name})

    collision = ET.SubElement(el, "collision")
    ET.SubElement(collision, "origin", attrib={"xyz": "0 0 0", "rpy": "0 0 0"})
    cgeom = ET.SubElement(collision, "geometry")
    ET.SubElement(
        cgeom,
        "mesh",
        attrib={"filename": _mesh_uri(package_name, link.collision_mesh_path)},
    )


def _emit_joint(parent: ET.Element, joint: Joint) -> None:
    el = ET.SubElement(parent, "joint", attrib={"name": joint.name, "type": joint.type})
    xyz, rpy = _origin_xyz_rpy(joint.origin)
    ET.SubElement(el, "origin", attrib={"xyz": xyz, "rpy": rpy})
    ET.SubElement(el, "parent", attrib={"link": joint.parent})
    ET.SubElement(el, "child", attrib={"link": joint.child})
    if joint.type != "fixed":
        ET.SubElement(el, "axis", attrib={"xyz": _format_xyz(joint.axis)})
    if joint.type in {"revolute", "prismatic"}:
        ET.SubElement(
            el,
            "limit",
            attrib={
                "lower": f"{joint.limit_lower if joint.limit_lower is not None else 0:g}",
                "upper": f"{joint.limit_upper if joint.limit_upper is not None else 0:g}",
                "effort": f"{joint.effort if joint.effort is not None else 0:g}",
                "velocity": f"{joint.velocity if joint.velocity is not None else 0:g}",
            },
        )


def emit_urdf(robot: Robot, out_path: Path, *, package_name: str) -> None:
    """Write `robot` as URDF 1.0 to `out_path`. Deterministic by sorted link/joint name."""
    root = ET.Element("robot", attrib={"name": robot.name})

    # one <material> per unique material_name (URDF requires materials be declared
    # at the robot level)
    seen: set[str] = set()
    for link in robot.links.values():
        if link.material_name in seen:
            continue
        seen.add(link.material_name)
        ET.SubElement(root, "material", attrib={"name": link.material_name})

    for name in sorted(robot.links):
        _emit_link(root, robot.links[name], package_name)
    for name in sorted(robot.joints):
        _emit_joint(root, robot.joints[name])

    ET.indent(root, space="  ")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(out_path, encoding="utf-8", xml_declaration=True)
