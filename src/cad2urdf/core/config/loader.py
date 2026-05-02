"""Load the joints+materials YAML user-configuration files."""

from __future__ import annotations

import math
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


# ---------------------------------------------------------------------------
# Internal validation helpers
# ---------------------------------------------------------------------------


def _require(d: dict[str, Any], key: str, type_: type, ctx: str) -> Any:
    if key not in d:
        raise ValueError(f"{ctx}: missing required field {key!r}")
    val = d[key]
    if not isinstance(val, type_):
        raise ValueError(f"{ctx}: field {key!r} must be {type_.__name__}, got {type(val).__name__}")
    return val


def _validate_axis(val: Any, ctx: str) -> list[float]:
    if not isinstance(val, list) or len(val) != 3:
        raise ValueError(f"{ctx}: 'axis' must be a 3-element list, got {val!r}")
    try:
        result = [float(x) for x in val]
    except (TypeError, ValueError) as e:
        raise ValueError(f"{ctx}: 'axis' elements must be numeric: {e}") from e
    for i, v in enumerate(result):
        if not math.isfinite(v):
            raise ValueError(f"{ctx}: 'axis[{i}]' must be finite, got {v}")
    return result


def _validate_xyz_or_rpy(val: Any, field_name: str, ctx: str) -> list[float]:
    if not isinstance(val, list) or len(val) != 3:
        raise ValueError(f"{ctx}: 'origin.{field_name}' must be a 3-element list, got {val!r}")
    try:
        result = [float(x) for x in val]
    except (TypeError, ValueError) as e:
        raise ValueError(f"{ctx}: 'origin.{field_name}' elements must be numeric: {e}") from e
    for i, v in enumerate(result):
        if not math.isfinite(v):
            raise ValueError(f"{ctx}: 'origin.{field_name}[{i}]' must be finite, got {v}")
    return result


# ---------------------------------------------------------------------------
# Public helpers
# ---------------------------------------------------------------------------


def _get_limit(j: dict[str, Any], field_name: str) -> float | None:
    lim = j.get("limits", {})
    val = lim.get(field_name)
    if val is None:
        return None
    try:
        result = float(val)
    except (TypeError, ValueError) as e:
        raise ValueError(f"limits.{field_name!r} must be numeric: {e}") from e
    if not math.isfinite(result):
        raise ValueError(f"limits.{field_name!r} must be finite, got {result}")
    return result


def load_joints_config(path: Path) -> JointsConfig:
    if not path.is_file():
        raise FileNotFoundError(f"joints config not found: {path}")
    raw = yaml.safe_load(path.read_text())

    # 1. Top-level must be a mapping.
    if not isinstance(raw, dict):
        raise ValueError(
            f"joints config at {path} must be a YAML mapping, got {type(raw).__name__}"
        )

    # 2. Required top-level string fields.
    robot_name = _require(raw, "robot_name", str, f"config {path}")
    base_link = _require(raw, "base_link", str, f"config {path}")

    # 3. joints must be a list (may be absent — empty robot is valid).
    raw_joints = raw.get("joints", [])
    if not isinstance(raw_joints, list):
        raise ValueError(f"config {path}: 'joints' must be a list, got {type(raw_joints).__name__}")

    # 4. Validate each joint entry.
    joints: list[JointSpec] = []
    for i, j in enumerate(raw_joints):
        if not isinstance(j, dict):
            raise ValueError(
                f"config {path}: joints[{i}] must be a mapping, got {type(j).__name__}"
            )
        ctx = f"config {path}: joints[{i}]"
        name = _require(j, "name", str, ctx)
        ctx = f"{ctx} ({name!r})"
        jtype = _require(j, "type", str, ctx)
        parent = _require(j, "parent", str, ctx)
        child = _require(j, "child", str, ctx)
        axis = _validate_axis(j.get("axis", [1.0, 0.0, 0.0]), ctx)
        origin = j.get("origin", {})
        if not isinstance(origin, dict):
            raise ValueError(f"{ctx}: 'origin' must be a mapping, got {type(origin).__name__}")
        origin_xyz = _validate_xyz_or_rpy(origin.get("xyz", [0.0, 0.0, 0.0]), "xyz", ctx)
        origin_rpy = _validate_xyz_or_rpy(origin.get("rpy", [0.0, 0.0, 0.0]), "rpy", ctx)
        limits = j.get("limits", {})
        if not isinstance(limits, dict):
            raise ValueError(f"{ctx}: 'limits' must be a mapping, got {type(limits).__name__}")
        joints.append(
            JointSpec(
                name=name,
                type=jtype,
                parent=parent,
                child=child,
                axis=axis,
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                limit_lower=_get_limit(j, "lower"),
                limit_upper=_get_limit(j, "upper"),
                effort=_get_limit(j, "effort"),
                velocity=_get_limit(j, "velocity"),
            )
        )

    # 5. materials must be a dict[str, str] if present.
    materials = raw.get("materials", {})
    if not isinstance(materials, dict):
        raise ValueError(
            f"config {path}: 'materials' must be a mapping, got {type(materials).__name__}"
        )
    for k, v in materials.items():
        if not isinstance(k, str) or not isinstance(v, str):
            raise ValueError(f"config {path}: 'materials' must map str to str, got {k!r} -> {v!r}")

    return JointsConfig(
        robot_name=robot_name,
        base_link=base_link,
        joints=joints,
        materials=dict(materials),
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
