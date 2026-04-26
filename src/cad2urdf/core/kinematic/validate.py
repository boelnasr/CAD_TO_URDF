"""Static validation of a Robot AST. Returns a list of issues (empty = valid)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from cad2urdf.core.kinematic.model import Robot
from cad2urdf.core.kinematic.tree import children_of, parent_of

IssueKind = Literal["dangling_link", "cycle", "multi_parent", "missing_base"]


@dataclass(frozen=True)
class ValidationIssue:
    """A single validation finding. Empty list of these = robot is valid."""

    kind: IssueKind
    target: str
    detail: str


def validate_robot(robot: Robot) -> list[ValidationIssue]:
    """Return all issues found in `robot`. Empty list = valid.

    Note: cycles are difficult to construct through the public Robot/tree API
    (constructor + tree.py validators reject most cycle-creation paths), so
    `_check_cycles` is largely defensive and rarely fires.
    """
    issues: list[ValidationIssue] = []
    issues.extend(_check_base(robot))
    issues.extend(_check_dangling(robot))
    issues.extend(_check_multi_parent(robot))
    issues.extend(_check_cycles(robot))
    return issues


def _check_base(robot: Robot) -> list[ValidationIssue]:
    if robot.base_link not in robot.links:
        return [
            ValidationIssue(
                kind="missing_base",
                target=robot.base_link,
                detail=f"base_link {robot.base_link!r} is not a known link",
            )
        ]
    return []


def _check_dangling(robot: Robot) -> list[ValidationIssue]:
    """Links not reachable from base_link via parent->child traversal."""
    if robot.base_link not in robot.links:
        return []
    reachable: set[str] = {robot.base_link}
    stack = [robot.base_link]
    while stack:
        cur = stack.pop()
        for c in children_of(robot, cur):
            if c not in reachable:
                reachable.add(c)
                stack.append(c)
    return [
        ValidationIssue(
            kind="dangling_link",
            target=name,
            detail=f"link {name!r} is not reachable from base_link {robot.base_link!r}",
        )
        for name in robot.links
        if name not in reachable
    ]


def _check_multi_parent(robot: Robot) -> list[ValidationIssue]:
    parent_count: dict[str, list[str]] = {}
    for j in robot.joints.values():
        parent_count.setdefault(j.child, []).append(j.parent)
    return [
        ValidationIssue(
            kind="multi_parent",
            target=child,
            detail=f"link {child!r} has multiple parents: {parents}",
        )
        for child, parents in parent_count.items()
        if len(parents) > 1
    ]


def _check_cycles(robot: Robot) -> list[ValidationIssue]:
    """Walk parent-of upward from each link; revisit = cycle.

    Emits one issue per link found to be part of a cycle. So a 3-cycle a->b->c->a
    will produce 3 issues, one targeting each link. This is intentional - it
    surfaces every link the user needs to inspect.
    """
    issues: list[ValidationIssue] = []
    for start in robot.links:
        seen = {start}
        cur: str | None = start
        depth = 0
        while True:
            cur = parent_of(robot, cur) if cur is not None else None
            depth += 1
            if cur is None:
                break
            if cur in seen:
                issues.append(
                    ValidationIssue(
                        kind="cycle",
                        target=start,
                        detail=f"cycle detected involving link {start!r}",
                    )
                )
                break
            if depth > len(robot.links) + 1:
                issues.append(
                    ValidationIssue(
                        kind="cycle",
                        target=start,
                        detail=f"unbounded ancestor chain from {start!r}",
                    )
                )
                break
            seen.add(cur)
    return issues
