"""Pure-function tree operations on Robot. All return a new Robot (immutable style)."""

from __future__ import annotations

from copy import deepcopy

from cad2urdf.core.kinematic.model import Joint, Link, Robot


def children_of(robot: Robot, link_name: str) -> list[str]:
    """Direct children of a link, in deterministic name order."""
    out = [j.child for j in robot.joints.values() if j.parent == link_name]
    return sorted(out)


def parent_of(robot: Robot, link_name: str) -> str | None:
    """Direct parent of a link, or None if it is the base or unparented."""
    for j in robot.joints.values():
        if j.child == link_name:
            return j.parent
    return None


def descendants_of(robot: Robot, link_name: str) -> set[str]:
    """All transitive descendants (including self)."""
    seen: set[str] = {link_name}
    stack = [link_name]
    while stack:
        cur = stack.pop()
        for c in children_of(robot, cur):
            if c not in seen:
                seen.add(c)
                stack.append(c)
    return seen


def add_link(robot: Robot, link: Link, joint: Joint) -> Robot:
    """Append a link + the joint that connects it to an existing parent."""
    if link.name in robot.links:
        raise ValueError(f"link {link.name!r} already exists")
    if joint.name in robot.joints:
        raise ValueError(f"joint {joint.name!r} already exists")
    if joint.parent not in robot.links:
        raise ValueError(f"parent link {joint.parent!r} not in robot")
    if joint.child != link.name:
        raise ValueError(f"joint.child {joint.child!r} != link.name {link.name!r}")

    new = deepcopy(robot)
    new.links[link.name] = link
    new.joints[joint.name] = joint
    return new


def remove_link(robot: Robot, link_name: str) -> Robot:
    """Remove a link and its entire subtree (all descendants)."""
    if link_name == robot.base_link:
        raise ValueError(f"cannot remove base_link {link_name!r}")
    if link_name not in robot.links:
        raise ValueError(f"link {link_name!r} not in robot")

    doomed = descendants_of(robot, link_name)
    new = deepcopy(robot)
    for name in doomed:
        new.links.pop(name, None)
    new.joints = {
        jn: j for jn, j in new.joints.items() if j.parent not in doomed and j.child not in doomed
    }
    return new


def reparent_joint(robot: Robot, joint_name: str, new_parent: str) -> Robot:
    """Change a joint's parent link. Caller is responsible for cycle-avoidance."""
    if joint_name not in robot.joints:
        raise ValueError(f"joint {joint_name!r} not in robot")
    if new_parent not in robot.links:
        raise ValueError(f"new parent {new_parent!r} not in robot")

    new = deepcopy(robot)
    j = new.joints[joint_name]
    new.joints[joint_name] = Joint(
        name=j.name,
        type=j.type,
        parent=new_parent,
        child=j.child,
        axis=j.axis,
        origin=j.origin,
        limit_lower=j.limit_lower,
        limit_upper=j.limit_upper,
        effort=j.effort,
        velocity=j.velocity,
    )
    return new
