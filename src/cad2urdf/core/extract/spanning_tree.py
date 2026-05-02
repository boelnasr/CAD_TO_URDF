"""Build a kinematic spanning tree from a possibly-cyclic classified-edge graph."""

from __future__ import annotations

from collections import deque

from cad2urdf.core.extract.mate_classify import ClassifiedJoint


class SpanningTreeError(Exception):
    """Raised when the input graph cannot be turned into a single spanning tree."""


def _all_nodes(edges: list[ClassifiedJoint]) -> set[str]:
    nodes: set[str] = set()
    for e in edges:
        nodes.add(e.parent_id)
        nodes.add(e.child_id)
    return nodes


def build_spanning_tree(
    edges: list[ClassifiedJoint],
    *,
    base_id: str,
) -> list[ClassifiedJoint]:
    """BFS from `base_id`. First edge that introduces a new node wins; cycles dropped.

    May flip an edge's direction if BFS traversal requires it (the underlying
    graph is treated as undirected for connectivity, then re-directed so the
    base sits at the root).
    """
    nodes = _all_nodes(edges)
    if base_id not in nodes:
        raise SpanningTreeError(f"base_id {base_id!r} not found in edges")

    # adjacency: undirected (we may need to flip an edge to make base the root)
    adj: dict[str, list[tuple[str, ClassifiedJoint, bool]]] = {n: [] for n in nodes}
    for e in edges:
        adj[e.parent_id].append((e.child_id, e, False))  # forward
        adj[e.child_id].append((e.parent_id, e, True))  # reverse

    out: list[ClassifiedJoint] = []
    seen: set[str] = {base_id}
    queue: deque[str] = deque([base_id])
    while queue:
        cur = queue.popleft()
        for neighbor, original, was_reversed in adj[cur]:
            if neighbor in seen:
                continue
            seen.add(neighbor)
            queue.append(neighbor)
            if was_reversed:
                # flip parent/child to make BFS root the parent
                out.append(
                    ClassifiedJoint(
                        parent_id=cur,
                        child_id=neighbor,
                        joint_type=original.joint_type,
                        axis=original.axis,
                        relative_pose=original.relative_pose,
                    )
                )
            else:
                out.append(original)

    unreached = nodes - seen
    if unreached:
        raise SpanningTreeError(f"nodes not reachable from base {base_id!r}: {sorted(unreached)}")
    return out
