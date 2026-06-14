"""STEP (.stp/.step) -> Robot assembly via OCP (cadquery-ocp). Lazy-imported.

OCP is the pip-installable OCCT binding (the ``[step]`` extra). Importing this
module without OCP installed never crashes the base package: every OCP symbol is
imported inside a function body, so a missing wheel only fails when a STEP
operation is actually requested (with a clear, actionable error message).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from numpy.typing import NDArray

from cad2urdf.core.extract.types import AssemblyEdge

_OCP_HINT = "STEP import requires OCP. Install it with: pip install -e '.[step]'"


def tessellate(shape: Any, *, unit_scale: float = 0.001) -> trimesh.Trimesh:
    """Triangulate an OCCT shape into a trimesh, scaling vertices by unit_scale.

    STEP geometry is typically in millimetres; ``unit_scale=0.001`` converts to
    the metres URDF expects.
    """
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_FACE
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
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                verts.append((p.X() * unit_scale, p.Y() * unit_scale, p.Z() * unit_scale))
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                faces.append((base + a - 1, base + b - 1, base + c - 1))
        exp.Next()
    if not faces:
        raise ValueError("shape produced no triangles (empty or non-solid geometry)")
    return trimesh.Trimesh(vertices=np.array(verts), faces=np.array(faces), process=True)


@dataclass(frozen=True)
class StepPart:
    """A node in a STEP/XCAF assembly tree.

    ``id`` and ``name`` are equal here: both are the sanitized, de-duplicated,
    URDF-valid name assigned during the walk. ``mesh`` is ``None`` for assembly
    nodes (they become meshless "frame" links); leaf parts carry a tessellated
    mesh in metres. ``is_leaf`` is ``True`` for a part (non-assembly) shape.
    """

    id: str
    name: str
    mesh: trimesh.Trimesh | None
    is_leaf: bool


@dataclass(frozen=True)
class StepAssembly:
    """The result of walking a STEP file: parts, parent->child edges, and root."""

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
    """4x4 child-relative-to-parent pose for a component, scaled mm -> m."""
    trsf = tool.GetLocation_s(label).Transformation()
    out = np.eye(4)
    for r in range(3):
        for c in range(4):
            out[r, c] = trsf.Value(r + 1, c + 1)  # OCC matrices are 1-indexed
    out[:3, 3] *= unit_scale  # translation mm -> m
    return out


def load_step_assembly(path: Path, *, unit_scale: float = 0.001) -> StepAssembly:
    """Read a STEP assembly into named parts + parent->child edges.

    Walks the XCAF document from ``GetFreeShapes``, recursing assemblies via
    ``GetComponents_s`` and resolving each component to its referred part label.
    Leaf parts are tessellated to a trimesh (in metres); assembly nodes carry no
    mesh. Component poses (``GetLocation_s``) become each edge's ``relative_pose``
    with translations converted mm -> m by ``unit_scale``.
    """
    try:
        from OCP.IFSelect import IFSelect_RetDone
        from OCP.STEPCAFControl import STEPCAFControl_Reader
        from OCP.TCollection import TCollection_ExtendedString
        from OCP.TDF import TDF_Label, TDF_LabelSequence
        from OCP.TDocStd import TDocStd_Document
        from OCP.XCAFApp import XCAFApp_Application
        from OCP.XCAFDoc import XCAFDoc_DocumentTool
    except ImportError as e:  # pragma: no cover - exercised only without OCP
        raise ImportError(_OCP_HINT) from e

    if not path.is_file():
        raise FileNotFoundError(f"STEP file not found: {path}")

    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise ValueError(f"failed to read STEP file: {path}")
    reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    parts: list[StepPart] = []
    edges: list[AssemblyEdge] = []
    used: set[str] = set()
    id_for: dict[str, str] = {}  # label entry -> assigned id (sanitized name)

    def entry(label: Any) -> str:
        from OCP.TCollection import TCollection_AsciiString
        from OCP.TDF import TDF_Tool

        s = TCollection_AsciiString()
        TDF_Tool.Entry_s(label, s)
        return str(s.ToCString())

    def visit(label: Any) -> str:
        key = entry(label)
        if key in id_for:
            return id_for[key]
        node_id = _sanitize(_name_of(label), used)
        id_for[key] = node_id
        is_asm = tool.IsAssembly_s(label)
        mesh = None if is_asm else tessellate(tool.GetShape_s(label), unit_scale=unit_scale)
        parts.append(StepPart(id=node_id, name=node_id, mesh=mesh, is_leaf=not is_asm))
        if is_asm:
            comps = TDF_LabelSequence()
            tool.GetComponents_s(label, comps)
            for i in range(1, comps.Length() + 1):
                comp = comps.Value(i)
                referred = TDF_Label()
                tool.GetReferredShape_s(comp, referred)
                child_id = visit(referred)
                edges.append(
                    AssemblyEdge(
                        parent_id=node_id,
                        child_id=child_id,
                        relative_pose=_pose_of(comp, tool, unit_scale),
                    )
                )
        return node_id

    free = TDF_LabelSequence()
    tool.GetFreeShapes(free)
    if free.Length() == 0:
        raise ValueError(f"STEP file has no shapes: {path}")
    root_id = visit(free.Value(1))
    return StepAssembly(parts=parts, edges=edges, root_id=root_id)
