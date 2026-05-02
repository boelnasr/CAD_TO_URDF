"""Load the joints+materials YAML user-configuration files."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from numpy.typing import NDArray


@dataclass
class JointSpec:
    name: str
    type: str
    parent: str
    child: str
    axis: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0])
    origin_xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    origin_rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    limit_lower: float | None = None
    limit_upper: float | None = None
    effort: float | None = None
    velocity: float | None = None


@dataclass
class JointsConfig:
    robot_name: str
    base_link: str
    joints: list[JointSpec]
    materials: dict[str, str] = field(default_factory=dict)


def _get_limit(j: dict[str, Any], field_name: str) -> float | None:
    lim = j.get("limits", {})
    val = lim.get(field_name)
    return float(val) if val is not None else None


def load_joints_config(path: Path) -> JointsConfig:
    if not path.is_file():
        raise FileNotFoundError(f"joints config not found: {path}")
    raw = yaml.safe_load(path.read_text())

    joints = [
        JointSpec(
            name=j["name"],
            type=j["type"],
            parent=j["parent"],
            child=j["child"],
            axis=list(j.get("axis", [1.0, 0.0, 0.0])),
            origin_xyz=list(j.get("origin", {}).get("xyz", [0.0, 0.0, 0.0])),
            origin_rpy=list(j.get("origin", {}).get("rpy", [0.0, 0.0, 0.0])),
            limit_lower=_get_limit(j, "lower"),
            limit_upper=_get_limit(j, "upper"),
            effort=_get_limit(j, "effort"),
            velocity=_get_limit(j, "velocity"),
        )
        for j in raw.get("joints", [])
    ]
    return JointsConfig(
        robot_name=raw["robot_name"],
        base_link=raw["base_link"],
        joints=joints,
        materials=dict(raw.get("materials", {})),
    )


def origin_from_xyz_rpy(xyz: list[float], rpy: list[float]) -> NDArray[Any]:
    """Build a 4x4 from xyz translation + RPY (URDF fixed-axis roll-pitch-yaw)."""
    cr, sr = np.cos(rpy[0]), np.sin(rpy[0])
    cp, sp = np.cos(rpy[1]), np.sin(rpy[1])
    cy, sy = np.cos(rpy[2]), np.sin(rpy[2])
    rot = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    out = np.eye(4)
    out[:3, :3] = rot
    out[:3, 3] = xyz
    return out
