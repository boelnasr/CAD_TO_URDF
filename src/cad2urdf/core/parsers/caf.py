"""Shared XCAF assembly walking and transient B-rep tessellation helpers.

OCP is intentionally imported only inside function bodies, keeping the core
parser package importable without the optional CAD dependency installed.

``load_caf_assembly`` returns B-reps only in its side map.  Consumers must use
that map for immediate inference, then discard it before creating or retaining
Robot, controller, project, or socket state.  B-reps are deliberately absent
from the serializable ``StepAssembly`` AST.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np
import trimesh
from numpy.typing import NDArray

from cad2urdf.core.extract.types import AssemblyEdge

_OCP_HINT = "CAD import requires OCP. Install it with: pip install -e '.[step]'"


def tessellate(shape: Any, *, unit_scale: float = 0.001) -> trimesh.Trimesh:
    """Triangulate an OCCT shape into a trimesh, scaling vertices by unit_scale."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    BRepMesh_IncrementalMesh(shape, 0.5, False, 0.3, True)
    verts: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            base = len(verts)
            # OCCT stores a face's triangulation in its underlying surface's
            # orientation; a REVERSED face must be re-wound or its triangles
            # point inward and the signed volume becomes placement-dependent.
            reversed_face = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                verts.append((p.X() * unit_scale, p.Y() * unit_scale, p.Z() * unit_scale))
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                if reversed_face:
                    b, c = c, b
                faces.append((base + a - 1, base + b - 1, base + c - 1))
        exp.Next()
    if not faces:
        raise ValueError("shape produced no triangles (empty or non-solid geometry)")
    return trimesh.Trimesh(vertices=np.array(verts), faces=np.array(faces), process=True)


@dataclass(frozen=True)
class StepPart:
    """A node in a STEP/XCAF assembly tree."""

    id: str
    name: str
    mesh: trimesh.Trimesh | None
    is_leaf: bool


@dataclass(frozen=True)
class StepAssembly:
    """The result of walking a CAD assembly: parts, parent->child edges, and root."""

    parts: list[StepPart]
    edges: list[AssemblyEdge]
    root_id: str


def _sanitize(name: str, used: set[str]) -> str:
    """Make ``name`` a URDF-valid, unique identifier (anonymous -> ``part``)."""
    cleaned = "".join(c if (c.isalnum() or c == "_") else "_" for c in name).strip("_")
    cleaned = cleaned or "part"
    candidate = cleaned
    n = 2
    while candidate in used:
        candidate = f"{cleaned}_{n}"
        n += 1
    used.add(candidate)
    return candidate


def _name_of(label: Any) -> str:
    """Read a label's ``TDataStd_Name``, falling back to ``"part"``."""
    from OCP.TDataStd import TDataStd_Name

    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return str(attr.Get().ToExtString())
    return "part"


def _pose_of(label: Any, tool: Any, unit_scale: float) -> NDArray[Any]:
    """Return a 4x4 child-relative-to-parent pose, scaled from mm to m."""
    trsf = tool.GetLocation_s(label).Transformation()
    out = np.eye(4)
    for r in range(3):
        for c in range(4):
            out[r, c] = trsf.Value(r + 1, c + 1)
    out[:3, 3] *= unit_scale
    return out


def load_caf_assembly(
    *,
    reader_factory: Callable[[], Any],
    path: Path,
    unit_scale: float = 0.001,
) -> tuple[StepAssembly, dict[str, Any]]:
    """Load an XCAF-backed CAD assembly and leaf B-reps using ``reader_factory``.

    The returned shape map is transient and keyed solely by leaf ``StepPart.id``
    values.  It is non-serializable and intended solely for immediate joint
    inference; discard it before building a ``Robot`` or crossing a process/UI
    boundary.  The XCAF document exists only in this function's scope, so the
    map is the only OCP state that escapes the parser.
    """
    try:
        from OCP.IFSelect import IFSelect_RetDone
        from OCP.TCollection import TCollection_ExtendedString
        from OCP.TDF import TDF_Label, TDF_LabelSequence
        from OCP.TDocStd import TDocStd_Document
        from OCP.XCAFDoc import XCAFDoc_DocumentTool
    except ImportError as e:  # pragma: no cover - exercised only without OCP
        raise ImportError(_OCP_HINT) from e

    if not path.is_file():
        raise FileNotFoundError(f"CAD file not found: {path}")

    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    XCAFDoc_DocumentTool.Set_s(doc.Main())
    reader = reader_factory()
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise ValueError(f"failed to read CAD file: {path}")
    reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    parts: list[StepPart] = []
    edges: list[AssemblyEdge] = []
    shapes: dict[str, Any] = {}
    used: set[str] = set()
    def visit(label: Any, *, localize_shape: bool = False) -> str:
        # Deliberately create a node for each visit. XCAF component labels are
        # occurrences: several components may refer to the same definition but
        # have different placements, and a Robot needs one link per occurrence.
        node_id = _sanitize(_name_of(label), used)
        is_assembly = tool.IsAssembly_s(label)
        mesh = None
        if not is_assembly:
            shape = tool.GetShape_s(label)
            if localize_shape:
                # A top-level free shape may carry a placement directly on its
                # label. The synthetic root edge owns that placement, so
                # tessellate and infer from the unlocated body to avoid applying
                # it once in the mesh and again in the kinematic tree.
                from OCP.TopLoc import TopLoc_Location

                shape = shape.Located(TopLoc_Location())
            shapes[node_id] = shape
            try:
                mesh = tessellate(shape, unit_scale=unit_scale)
            except ValueError as e:
                # Sketches and free curves are common in CAD exports; name the
                # offending part so the user knows what to remove.
                raise ValueError(
                    f"{path}: part {node_id!r} has no tessellable geometry"
                ) from e
        parts.append(StepPart(id=node_id, name=node_id, mesh=mesh, is_leaf=not is_assembly))
        if is_assembly:
            components = TDF_LabelSequence()
            tool.GetComponents_s(label, components)
            for i in range(1, components.Length() + 1):
                component = components.Value(i)
                referred = TDF_Label()
                tool.GetReferredShape_s(component, referred)
                child_id = visit(referred)
                edges.append(
                    AssemblyEdge(
                        parent_id=node_id,
                        child_id=child_id,
                        relative_pose=_pose_of(component, tool, unit_scale),
                    )
                )
        return node_id

    free = TDF_LabelSequence()
    tool.GetFreeShapes(free)
    if free.Length() == 0:
        raise ValueError(f"CAD file has no shapes: {path}")
    if free.Length() == 1:
        # Preserve the long-standing flat/single-root behavior: the free shape
        # itself is the root, including any placement embedded in its mesh.
        root_id = visit(free.Value(1))
    else:
        # Multiple independent free shapes have no common XCAF parent. Create
        # one explicit frame so every body survives as a reachable Robot link.
        # Reserving its ID first also makes collisions with source names stable.
        root_id = _sanitize("assembly_root", used)
        parts.append(StepPart(id=root_id, name=root_id, mesh=None, is_leaf=False))
        for i in range(1, free.Length() + 1):
            label = free.Value(i)
            child_id = visit(label, localize_shape=True)
            edges.append(
                AssemblyEdge(
                    parent_id=root_id,
                    child_id=child_id,
                    relative_pose=_pose_of(label, tool, unit_scale),
                )
            )
    assembly = StepAssembly(parts=parts, edges=edges, root_id=root_id)
    del doc
    return assembly, shapes
