"""Save/load Robot AST as JSON (.cad2urdf project file)."""

from __future__ import annotations

import json
import shutil
import tempfile
from collections.abc import Callable
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

# Migration registry: maps (from_version, to_version) -> function that transforms payload dict.
# When SCHEMA_VERSION is bumped to 2, add entry (1, 2): _migrate_v1_to_v2.
_MIGRATIONS: dict[tuple[int, int], Callable[[dict[str, Any]], dict[str, Any]]] = {}


def _migrate_payload(payload: dict[str, Any]) -> dict[str, Any]:
    """Walk migrations from payload's schema_version up to SCHEMA_VERSION.

    Returns a payload dict at SCHEMA_VERSION, or raises ValueError if no
    migration path exists.
    """
    current = payload.get("schema_version")
    if current == SCHEMA_VERSION:
        return payload
    if not isinstance(current, int):
        raise ValueError(
            f"unsupported schema_version: {current!r} (expected {SCHEMA_VERSION}). "
            "schema_version must be an integer."
        )

    while current != SCHEMA_VERSION:
        next_version = current + 1
        migration = _MIGRATIONS.get((current, next_version))
        if migration is None:
            raise ValueError(
                f"unsupported schema_version: {current} (expected {SCHEMA_VERSION}). "
                f"No migration registered for {current} -> {next_version}. "
                f"Add an entry to _MIGRATIONS in {__file__}."
            )
        payload = migration(payload)
        current = next_version

    return payload


def _link_to_dict(link: Link) -> dict[str, Any]:
    ov = link.inertial_override
    return {
        "name": link.name,
        "visual_mesh_path": str(link.visual_mesh_path) if link.visual_mesh_path else None,
        "collision_mesh_path": str(link.collision_mesh_path) if link.collision_mesh_path else None,
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
    vmp = d["visual_mesh_path"]
    cmp = d["collision_mesh_path"]
    return Link(
        name=d["name"],
        visual_mesh_path=Path(vmp) if vmp else None,
        collision_mesh_path=Path(cmp) if cmp else None,
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


def robot_to_payload(robot: Robot) -> dict[str, Any]:
    """Serialize a Robot to a plain JSON-able dict at the current SCHEMA_VERSION."""
    return {
        "schema_version": SCHEMA_VERSION,
        "name": robot.name,
        "base_link": robot.base_link,
        "links": [_link_to_dict(robot.links[k]) for k in sorted(robot.links)],
        "joints": [_joint_to_dict(robot.joints[k]) for k in sorted(robot.joints)],
    }


def save_project(robot: Robot, path: Path) -> None:
    """Serialize ``robot`` and bundle every available mesh beside the project.

    Generated CAD tessellations live in a GUI-owned temporary directory.  A
    project must therefore take its own copy rather than retaining those
    session paths.  Missing paths are left unchanged for backwards
    compatibility with projects whose relative assets are supplied later.
    """
    payload = robot_to_payload(robot)
    path.parent.mkdir(parents=True, exist_ok=True)
    asset_dir = path.parent / f"{path.stem}_assets"
    staging = Path(tempfile.mkdtemp(prefix=f".{asset_dir.name}_", dir=path.parent))
    copied: dict[Path, Path] = {}
    copied_any = False
    try:
        for link in payload["links"]:
            for field in ("visual_mesh_path", "collision_mesh_path"):
                raw = link[field]
                if raw is None:
                    continue
                source = Path(raw)
                if not source.is_file():
                    continue
                source = source.resolve()
                relative = copied.get(source)
                if relative is None:
                    suffix = source.suffix or ".mesh"
                    relative = Path(f"{len(copied) + 1:04d}{suffix}")
                    shutil.copy2(source, staging / relative)
                    copied[source] = relative
                link[field] = str(Path(asset_dir.name) / relative)
                copied_any = True

        if copied_any:
            if asset_dir.exists():
                shutil.rmtree(asset_dir)
            staging.replace(asset_dir)
        else:
            shutil.rmtree(staging)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    path.write_text(json.dumps(payload, indent=2, sort_keys=True))


def load_project(path: Path) -> Robot:
    """Load a Robot from a .cad2urdf JSON file."""
    if not path.is_file():
        raise FileNotFoundError(f"project file not found: {path}")
    payload = json.loads(path.read_text())
    payload = _migrate_payload(payload)
    robot = Robot(
        name=payload["name"],
        base_link=payload["base_link"],
        links={d["name"]: _link_from_dict(d) for d in payload["links"]},
        joints={d["name"]: _joint_from_dict(d) for d in payload["joints"]},
    )
    # Project mesh references are relative to the project, not the process's
    # current working directory.  Absolute paths from older schema-v1 files
    # continue to work unchanged.
    for link in robot.links.values():
        for field in ("visual_mesh_path", "collision_mesh_path"):
            mesh_path = getattr(link, field)
            if mesh_path is not None and not mesh_path.is_absolute():
                object.__setattr__(link, field, path.parent / mesh_path)
    return robot
