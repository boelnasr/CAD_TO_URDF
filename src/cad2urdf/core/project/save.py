"""Save/load Robot AST as JSON (.cad2urdf project file)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from cad2urdf.core.kinematic.model import (
    InertialOverride,
    Joint,
    Link,
    Robot,
)

SCHEMA_VERSION = 1


def _link_to_dict(link: Link) -> dict[str, Any]:
    ov = link.inertial_override
    return {
        "name": link.name,
        "visual_mesh_path": str(link.visual_mesh_path),
        "collision_mesh_path": str(link.collision_mesh_path),
        "material_density": link.material_density,
        "material_name": link.material_name,
        "origin": link.origin.tolist(),
        "inertial_override": {
            "mass": ov.mass,
            "com": ov.com.tolist() if ov.com is not None else None,
            "inertia": ov.inertia.tolist() if ov.inertia is not None else None,
        },
    }


def _link_from_dict(d: dict[str, Any]) -> Link:
    ov_d = d["inertial_override"]
    return Link(
        name=d["name"],
        visual_mesh_path=Path(d["visual_mesh_path"]),
        collision_mesh_path=Path(d["collision_mesh_path"]),
        material_density=d["material_density"],
        material_name=d["material_name"],
        inertial_override=InertialOverride(
            mass=ov_d.get("mass"),
            com=np.asarray(ov_d["com"]) if ov_d.get("com") is not None else None,
            inertia=np.asarray(ov_d["inertia"]) if ov_d.get("inertia") is not None else None,
        ),
        origin=np.asarray(d["origin"]),
    )


def _joint_to_dict(joint: Joint) -> dict[str, Any]:
    return {
        "name": joint.name,
        "type": joint.type,
        "parent": joint.parent,
        "child": joint.child,
        "axis": joint.axis.tolist(),
        "origin": joint.origin.tolist(),
        "limit_lower": joint.limit_lower,
        "limit_upper": joint.limit_upper,
        "effort": joint.effort,
        "velocity": joint.velocity,
    }


def _joint_from_dict(d: dict[str, Any]) -> Joint:
    return Joint(
        name=d["name"],
        type=d["type"],
        parent=d["parent"],
        child=d["child"],
        axis=np.asarray(d["axis"]),
        origin=np.asarray(d["origin"]),
        limit_lower=d.get("limit_lower"),
        limit_upper=d.get("limit_upper"),
        effort=d.get("effort"),
        velocity=d.get("velocity"),
    )


def save_project(robot: Robot, path: Path) -> None:
    """Serialize ``robot`` to a .cad2urdf JSON file at ``path``."""
    payload = {
        "schema_version": SCHEMA_VERSION,
        "name": robot.name,
        "base_link": robot.base_link,
        "links": [_link_to_dict(robot.links[k]) for k in sorted(robot.links)],
        "joints": [_joint_to_dict(robot.joints[k]) for k in sorted(robot.joints)],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True))


def load_project(path: Path) -> Robot:
    """Load a Robot from a .cad2urdf JSON file."""
    if not path.is_file():
        raise FileNotFoundError(f"project file not found: {path}")
    payload = json.loads(path.read_text())
    if payload.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            f"unsupported schema_version: {payload.get('schema_version')!r} "
            f"(expected {SCHEMA_VERSION})"
        )
    return Robot(
        name=payload["name"],
        base_link=payload["base_link"],
        links={d["name"]: _link_from_dict(d) for d in payload["links"]},
        joints={d["name"]: _joint_from_dict(d) for d in payload["joints"]},
    )
