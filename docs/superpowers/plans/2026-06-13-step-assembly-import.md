# STEP Assembly Import Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Import a STEP CAD assembly (`.stp`/`.step`) into a multi-link `Robot` — named parts, hierarchy, and poses become frame + mesh links joined by fixed joints.

**Architecture:** A new OCP-based parser (`core/parsers/step.py`, lazy import) reads the XCAF assembly into parts + `AssemblyEdge`s; the existing `classify_default_fixed` → `build_spanning_tree` stages run; a new `core/extract/build_robot.py` turns the tree into a `Robot` (writing per-part STLs). The `Link` model gains optional mesh/material so sub-assembly nodes become meshless "frame" links. CLI + GUI route STEP through this pipeline.

**Tech Stack:** Python 3.10, OCP (`cadquery-ocp`), trimesh, numpy, PyQt6 (GUI wiring), pytest (`importorskip("OCP")`).

**Spec:** [[2026-06-13-step-assembly-import-design]]

**Run prefix (all test commands):**
`unset PYTHONPATH AMENT_PREFIX_PATH && source .venv/bin/activate &&` — GUI tests additionally need `export QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true`. OCP is already installed in `.venv`.

---

## File Structure

- `src/cad2urdf/core/kinematic/model.py` — modify `Link` (optional mesh/material) + `__post_init__`.
- `src/cad2urdf/core/urdf/emit.py` — modify `_emit_link` / `_emit_inertial` for meshless links.
- `src/cad2urdf/core/project/save.py` — modify `_link_to_dict` / `_link_from_dict` for `None` paths.
- `src/cad2urdf/gui/viewport/widget.py` — guard `None` mesh path in `_rebuild`.
- `src/cad2urdf/core/parsers/step.py` — **new**: `StepPart`, `StepAssembly`, `load_step_assembly`, `tessellate`.
- `src/cad2urdf/core/extract/build_robot.py` — **new**: `build_robot_from_tree`.
- `src/cad2urdf/cli/main.py` — route `.step`/`.stp` to the STEP pipeline.
- `src/cad2urdf/gui/workers/step_import.py` — **new**: `build_step_import_job`.
- `src/cad2urdf/gui/windows/main_window.py` — enable Import STEP action + dialog.
- `src/cad2urdf/gui/workers/import_meshes.py` — drop the STEP branch in `_load_one`.
- `pyproject.toml`, `README.md`, `environment.yml` — `[step]` extra + docs.
- Tests: `tests/unit/test_frame_links.py`, `tests/unit/test_step_parser.py`, `tests/unit/test_build_robot.py`, `tests/integration/test_step_pipeline.py`, plus additions to `tests/gui/`.

---

## Task 1: `Link` supports meshless frame links

**Files:**
- Modify: `src/cad2urdf/core/kinematic/model.py` (the `Link` dataclass)
- Test: `tests/unit/test_frame_links.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_frame_links.py
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Link


def test_frame_link_allows_no_mesh_or_material() -> None:
    link = Link(
        name="frame",
        visual_mesh_path=None,
        collision_mesh_path=None,
        material_density=None,
        material_name=None,
        inertial_override=InertialOverride(),
    )
    assert link.visual_mesh_path is None
    assert link.material_density is None


def test_mesh_link_still_rejects_nonpositive_density() -> None:
    with pytest.raises(ValueError, match="material_density"):
        Link(
            name="part",
            visual_mesh_path=Path("m/part.stl"),
            collision_mesh_path=Path("m/part.stl"),
            material_density=0.0,
            material_name="aluminum_6061",
            inertial_override=InertialOverride(),
        )
```

- [ ] **Step 2: Run test to verify it fails**

Run: `<prefix> python -m pytest tests/unit/test_frame_links.py -q`
Expected: FAIL — `test_frame_link_allows_no_mesh_or_material` raises (current `Link` requires `Path` and `material_density <= 0` check fails on `None`).

- [ ] **Step 3: Implement — make mesh/material optional**

In `model.py`, change the `Link` field types and `__post_init__`:

```python
@dataclass
class Link:
    """A rigid body in the robot's kinematic tree.

    A link with no mesh (``visual_mesh_path is None``) is a massless "frame"
    link — e.g. a sub-assembly node from a STEP import. Material may be None
    when there is no mesh.
    """

    name: str
    visual_mesh_path: Path | None
    collision_mesh_path: Path | None
    material_density: float | None
    material_name: str | None
    inertial_override: InertialOverride
    origin: NDArray[Any] = field(default_factory=lambda: np.eye(4))

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("name must not be empty")
        if self.origin.shape != (4, 4):
            raise ValueError(f"origin must be shape (4, 4), got {self.origin.shape}")
        if self.material_density is not None and self.material_density <= 0:
            raise ValueError(f"material_density must be positive, got {self.material_density}")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `<prefix> python -m pytest tests/unit/test_frame_links.py -q`
Expected: PASS (2 passed)

- [ ] **Step 5: Run the full core suite (no regressions)**

Run: `<prefix> python -m pytest tests/unit -q`
Expected: PASS (existing mesh links still construct; optional fields are backward-compatible)

- [ ] **Step 6: Commit**

```bash
git add src/cad2urdf/core/kinematic/model.py tests/unit/test_frame_links.py
git commit -m "feat(model): allow meshless frame links (optional mesh/material)"
```

---

## Task 2: Emitter writes a bare `<link>` for frame links

**Files:**
- Modify: `src/cad2urdf/core/urdf/emit.py` (`_emit_inertial`, `_emit_link`)
- Test: `tests/unit/test_frame_links.py` (add)

- [ ] **Step 1: Write the failing test**

```python
# add to tests/unit/test_frame_links.py
from xml.etree import ElementTree as ET

from cad2urdf.core.kinematic.model import Joint, Robot
from cad2urdf.core.urdf.emit import emit_urdf


def _frame(name: str) -> Link:
    return Link(name, None, None, None, None, InertialOverride())


def test_emit_frame_link_has_no_visual(tmp_path: Path) -> None:
    base = _frame("root")
    arm = Link(
        "arm",
        tmp_path / "arm.stl",
        tmp_path / "arm.stl",
        2700.0,
        "aluminum_6061",
        InertialOverride(mass=1.0, com=np.zeros(3), inertia=np.eye(3)),
    )
    robot = Robot(
        name="r",
        base_link="root",
        links={"root": base, "arm": arm},
        joints={
            "arm_to_root": Joint(
                name="arm_to_root", type="fixed", parent="root", child="arm",
                axis=np.array([1.0, 0.0, 0.0]),
            )
        },
    )
    out = tmp_path / "r.urdf"
    emit_urdf(robot, out, package_name="r")
    tree = ET.parse(out)
    links = {el.get("name"): el for el in tree.getroot().findall("link")}
    assert links["root"].find("visual") is None
    assert links["root"].find("inertial") is None
    assert links["arm"].find("visual") is not None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `<prefix> python -m pytest tests/unit/test_frame_links.py::test_emit_frame_link_has_no_visual -q`
Expected: FAIL — current `_emit_link` calls `_mesh_uri` on a `None` path (AttributeError / emits a visual).

- [ ] **Step 3: Implement — guard the meshless case**

In `emit.py`, top of `_emit_inertial`:

```python
def _emit_inertial(parent: ET.Element, link: Link) -> None:
    visual = link.visual_mesh_path
    if visual is None:
        return  # frame link: no geometry, no inertial
    if visual.is_absolute() and visual.is_file() and link.material_density is not None:
        mesh = trimesh.load(str(visual), file_type="stl", force="mesh")
        # ... unchanged ...
```

(Keep the rest of `_emit_inertial`; just add the `None`/`material_density is None` guards so it only auto-computes when a mesh + density exist.)

In `_emit_link`, wrap the visual/collision emission:

```python
def _emit_link(parent: ET.Element, link: Link, package_name: str) -> None:
    el = ET.SubElement(parent, "link", attrib={"name": link.name})
    _emit_inertial(el, link)
    if link.visual_mesh_path is None or link.collision_mesh_path is None:
        return  # frame link: bare <link/>
    visual = ET.SubElement(el, "visual")
    # ... unchanged visual/collision body ...
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `<prefix> python -m pytest tests/unit/test_frame_links.py -q`
Expected: PASS

- [ ] **Step 5: Run URDF emit suite (no regressions)**

Run: `<prefix> python -m pytest tests/unit/test_urdf_emit.py -q`
Expected: PASS

- [ ] **Step 6: Commit**

```bash
git add src/cad2urdf/core/urdf/emit.py tests/unit/test_frame_links.py
git commit -m "feat(urdf): emit a bare <link> for meshless frame links"
```

---

## Task 3: Project I/O + viewport handle `None` mesh paths

**Files:**
- Modify: `src/cad2urdf/core/project/save.py` (`_link_to_dict`, `_link_from_dict`)
- Modify: `src/cad2urdf/gui/viewport/widget.py` (`_rebuild` guard)
- Test: `tests/unit/test_frame_links.py` (add a round-trip test)

- [ ] **Step 1: Write the failing test**

```python
# add to tests/unit/test_frame_links.py
from cad2urdf.core.project.save import load_project, save_project


def test_frame_link_project_roundtrip(tmp_path: Path) -> None:
    robot = Robot(
        name="r",
        base_link="root",
        links={"root": _frame("root")},
        joints={},
    )
    p = tmp_path / "r.cad2urdf"
    save_project(robot, p)
    loaded = load_project(p)
    assert loaded.links["root"].visual_mesh_path is None
    assert loaded.links["root"].material_density is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `<prefix> python -m pytest tests/unit/test_frame_links.py::test_frame_link_project_roundtrip -q`
Expected: FAIL — `_link_to_dict` calls `str(link.visual_mesh_path)` → `"None"`, and `_link_from_dict` wraps it back as `Path("None")`.

- [ ] **Step 3: Implement — serialize None**

In `save.py` `_link_to_dict`:

```python
    return {
        "name": link.name,
        "visual_mesh_path": str(link.visual_mesh_path) if link.visual_mesh_path else None,
        "collision_mesh_path": str(link.collision_mesh_path) if link.collision_mesh_path else None,
        "material_density": link.material_density,
        "material_name": link.material_name,
        "origin": link.origin.tolist(),
        "inertial_override": { ... unchanged ... },
    }
```

In `_link_from_dict`:

```python
    vmp = d["visual_mesh_path"]
    cmp = d["collision_mesh_path"]
    return Link(
        name=d["name"],
        visual_mesh_path=Path(vmp) if vmp else None,
        collision_mesh_path=Path(cmp) if cmp else None,
        material_density=d["material_density"],
        material_name=d["material_name"],
        inertial_override=InertialOverride( ... unchanged ... ),
        origin=np.asarray(d["origin"]),
    )
```

- [ ] **Step 4: Guard the viewport**

In `widget.py` `_rebuild`, change the skip check (the mesh path is now `Path | None`):

```python
        for name, link in robot.links.items():
            path = link.visual_mesh_path
            if path is None or not path.is_absolute() or not path.is_file():
                continue
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `<prefix> python -m pytest tests/unit/test_frame_links.py -q`
Then GUI: `<prefix> export QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true && python -m pytest tests/gui/test_viewport.py -q`
Expected: PASS both

- [ ] **Step 6: Commit**

```bash
git add src/cad2urdf/core/project/save.py src/cad2urdf/gui/viewport/widget.py tests/unit/test_frame_links.py
git commit -m "feat: persist and render meshless frame links (None paths)"
```

---

## Task 4: STEP tessellation — `tessellate(shape) -> trimesh`

**Files:**
- Create: `src/cad2urdf/core/parsers/step.py`
- Test: `tests/unit/test_step_parser.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_step_parser.py
from __future__ import annotations

import pytest

pytest.importorskip("OCP")  # STEP support requires the [step] extra

import trimesh

from cad2urdf.core.parsers.step import tessellate


def test_tessellate_box_to_trimesh() -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox

    shape = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape()
    mesh = tessellate(shape, unit_scale=0.001)  # mm -> m
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    # 10x20x30 mm = 0.01x0.02x0.03 m -> volume 6e-6 m^3
    assert mesh.volume == pytest.approx(6e-6, rel=1e-3)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `<prefix> python -m pytest tests/unit/test_step_parser.py -q`
Expected: FAIL — `cad2urdf.core.parsers.step` does not exist.

- [ ] **Step 3: Implement `tessellate`**

```python
# src/cad2urdf/core/parsers/step.py
"""STEP (.stp/.step) -> Robot assembly via OCP (cadquery-ocp). Lazy-imported.

OCP is the pip-installable OCCT binding (the [step] extra). Importing this module
without OCP installed raises a clear, actionable error.
"""
from __future__ import annotations

from typing import Any

import numpy as np
import trimesh
from numpy.typing import NDArray

_OCP_HINT = "STEP import requires OCP. Install it with: pip install -e '.[step]'"


def tessellate(shape: Any, *, unit_scale: float = 0.001) -> trimesh.Trimesh:
    """Triangulate an OCCT shape into a trimesh, scaling vertices by unit_scale.

    STEP geometry is typically in millimetres; unit_scale=0.001 converts to the
    metres URDF expects.
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `<prefix> python -m pytest tests/unit/test_step_parser.py -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/cad2urdf/core/parsers/step.py tests/unit/test_step_parser.py
git commit -m "feat(step): tessellate OCCT shapes to trimesh (mm->m)"
```

---

## Task 5: STEP XCAF walk — `load_step_assembly`

**Files:**
- Modify: `src/cad2urdf/core/parsers/step.py` (add `StepPart`, `StepAssembly`, `_new_doc`, `_sanitize`, `load_step_assembly`, a fixture writer for tests)
- Test: `tests/unit/test_step_parser.py` (add)

- [ ] **Step 1: Write the failing test (with an OCP-authored fixture)**

```python
# add to tests/unit/test_step_parser.py
from pathlib import Path

from cad2urdf.core.parsers.step import load_step_assembly


def _write_named_assembly(path: Path) -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Trsf, gp_Vec
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    base = tool.AddShape(BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(), False)
    arm = tool.AddShape(BRepPrimAPI_MakeBox(5.0, 5.0, 20.0).Shape(), False)
    TDataStd_Name.Set_s(base, TCollection_ExtendedString("base"))
    TDataStd_Name.Set_s(arm, TCollection_ExtendedString("arm"))
    asm = tool.NewShape()
    TDataStd_Name.Set_s(asm, TCollection_ExtendedString("robot"))
    t1 = gp_Trsf()
    t1.SetTranslation(gp_Vec(0.0, 0.0, 10.0))
    tool.AddComponent(asm, base, TopLoc_Location(gp_Trsf()))
    tool.AddComponent(asm, arm, TopLoc_Location(t1))
    tool.UpdateAssemblies()
    w = STEPCAFControl_Writer()
    w.Transfer(doc)
    assert w.Write(str(path)) == IFSelect_RetDone


def test_load_step_assembly_extracts_names_hierarchy_poses(tmp_path: Path) -> None:
    step = tmp_path / "robot.step"
    _write_named_assembly(step)
    asm = load_step_assembly(step)

    names = {p.name for p in asm.parts}
    assert {"base", "arm"} <= names
    leaves = {p.name for p in asm.parts if p.is_leaf and p.mesh is not None}
    assert leaves == {"base", "arm"}
    # arm sits +10 mm above base -> +0.01 m after unit scaling, on some edge
    arm_edges = [e for e in asm.edges if asm_id_to_name(asm, e.child_id) == "arm"]
    assert arm_edges
    assert arm_edges[0].relative_pose[2, 3] == pytest.approx(0.01, rel=1e-3)


def asm_id_to_name(asm, node_id: str) -> str:
    return next(p.name for p in asm.parts if p.id == node_id)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `<prefix> python -m pytest tests/unit/test_step_parser.py::test_load_step_assembly_extracts_names_hierarchy_poses -q`
Expected: FAIL — `load_step_assembly` / `StepPart` not defined.

- [ ] **Step 3: Implement `load_step_assembly`**

Add to `step.py`:

```python
from dataclasses import dataclass, field
from pathlib import Path

from cad2urdf.core.extract.types import AssemblyEdge


@dataclass(frozen=True)
class StepPart:
    id: str
    name: str
    mesh: trimesh.Trimesh | None
    is_leaf: bool


@dataclass(frozen=True)
class StepAssembly:
    parts: list[StepPart]
    edges: list[AssemblyEdge]
    root_id: str


def _sanitize(name: str, used: set[str]) -> str:
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
    from OCP.TDataStd import TDataStd_Name

    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return str(attr.Get().ToExtString())
    return "part"


def _pose_of(label: Any, tool: Any, unit_scale: float) -> NDArray[Any]:
    trsf = tool.GetLocation_s(label).Transformation()
    out = np.eye(4)
    for r in range(3):
        for c in range(4):
            out[r, c] = trsf.Value(r + 1, c + 1)
    out[:3, 3] *= unit_scale  # translation mm -> m
    return out


def load_step_assembly(path: Path, *, unit_scale: float = 0.001) -> StepAssembly:
    """Read a STEP assembly into named parts + parent->child edges."""
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
        from OCP.TDF import TDF_Tool
        from OCP.TCollection import TCollection_AsciiString

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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `<prefix> python -m pytest tests/unit/test_step_parser.py -q`
Expected: PASS. (If `_pose_of` translation is off by 1000x, the `unit_scale` on the translation is missing — it is applied above.)

- [ ] **Step 5: Commit**

```bash
git add src/cad2urdf/core/parsers/step.py tests/unit/test_step_parser.py
git commit -m "feat(step): XCAF assembly walk -> parts + AssemblyEdges"
```

---

## Task 6: `build_robot_from_tree`

**Files:**
- Create: `src/cad2urdf/core/extract/build_robot.py`
- Test: `tests/unit/test_build_robot.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_build_robot.py
from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh

from cad2urdf.core.extract.build_robot import build_robot_from_tree
from cad2urdf.core.extract.mate_classify import ClassifiedJoint
from cad2urdf.core.inertia.materials import lookup
from cad2urdf.core.parsers.step import StepPart


def _box() -> trimesh.Trimesh:
    return trimesh.creation.box(extents=(0.01, 0.01, 0.01))


def test_build_robot_frame_root_with_mesh_children(tmp_path: Path) -> None:
    parts = [
        StepPart(id="robot", name="robot", mesh=None, is_leaf=False),
        StepPart(id="base", name="base", mesh=_box(), is_leaf=True),
        StepPart(id="arm", name="arm", mesh=_box(), is_leaf=True),
    ]
    pose = np.eye(4)
    pose[2, 3] = 0.01
    tree = [
        ClassifiedJoint("robot", "base", "fixed", np.array([1.0, 0, 0]), np.eye(4)),
        ClassifiedJoint("robot", "arm", "fixed", np.array([1.0, 0, 0]), pose),
    ]
    robot = build_robot_from_tree(
        parts=parts, tree=tree, base_id="robot", robot_name="r",
        mesh_dir=tmp_path, material=lookup("aluminum_6061"),
    )
    assert robot.base_link == "robot"
    assert robot.links["robot"].visual_mesh_path is None       # frame
    assert robot.links["base"].visual_mesh_path is not None     # mesh
    assert (tmp_path / "base.stl").is_file()
    assert robot.joints["arm_to_robot"].origin[2, 3] == 0.01
```

- [ ] **Step 2: Run test to verify it fails**

Run: `<prefix> python -m pytest tests/unit/test_build_robot.py -q`
Expected: FAIL — `build_robot.py` does not exist.

- [ ] **Step 3: Implement `build_robot_from_tree`**

```python
# src/cad2urdf/core/extract/build_robot.py
"""Turn a classified spanning tree + parts into a Robot (Task 4.3)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from cad2urdf.core.extract.mate_classify import ClassifiedJoint
from cad2urdf.core.inertia.materials import Material
from cad2urdf.core.kinematic.model import InertialOverride, Joint, Link, Robot
from cad2urdf.core.parsers.step import StepPart


def build_robot_from_tree(
    *,
    parts: list[StepPart],
    tree: list[ClassifiedJoint],
    base_id: str,
    robot_name: str,
    mesh_dir: Path,
    material: Material,
) -> Robot:
    """Build a Robot. Leaf parts get a written STL + material; assembly nodes
    become meshless frame links. Each tree edge becomes a fixed joint."""
    by_id = {p.id: p for p in parts}
    node_ids = {base_id} | {e.parent_id for e in tree} | {e.child_id for e in tree}

    mesh_dir.mkdir(parents=True, exist_ok=True)
    links: dict[str, Link] = {}
    for nid in node_ids:
        part = by_id[nid]
        if part.mesh is not None:
            stl = (mesh_dir / f"{part.name}.stl").resolve()
            part.mesh.export(str(stl), file_type="stl")
            links[part.name] = Link(
                name=part.name,
                visual_mesh_path=stl,
                collision_mesh_path=stl,
                material_density=material.density_kg_m3,
                material_name=material.name,
                inertial_override=InertialOverride(),
            )
        else:
            links[part.name] = Link(
                name=part.name, visual_mesh_path=None, collision_mesh_path=None,
                material_density=None, material_name=None,
                inertial_override=InertialOverride(),
            )

    joints: dict[str, Joint] = {}
    for e in tree:
        pname, cname = by_id[e.parent_id].name, by_id[e.child_id].name
        jn = f"{cname}_to_{pname}"
        joints[jn] = Joint(
            name=jn, type=e.joint_type, parent=pname, child=cname,
            axis=np.asarray(e.axis, dtype=float), origin=np.asarray(e.relative_pose, dtype=float),
        )

    return Robot(
        name=robot_name, base_link=by_id[base_id].name, links=links, joints=joints,
    )
```

> Note: `node id` and `name` are equal here (the parser assigns `name` as the id), so link lookups by name are unambiguous. Confirm `Material` exposes `density_kg_m3` and `name` (it does — see `core/inertia/materials.py`).

- [ ] **Step 4: Run test to verify it passes**

Run: `<prefix> python -m pytest tests/unit/test_build_robot.py -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/cad2urdf/core/extract/build_robot.py tests/unit/test_build_robot.py
git commit -m "feat(extract): build_robot_from_tree (parts + tree -> Robot)"
```

---

## Task 7: End-to-end STEP pipeline + CLI wiring

**Files:**
- Modify: `src/cad2urdf/cli/main.py` (route `.step`/`.stp`)
- Test: `tests/integration/test_step_pipeline.py`

- [ ] **Step 1: Write the failing integration test**

```python
# tests/integration/test_step_pipeline.py
from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("OCP")

from cad2urdf.core.extract.mate_classify import classify_default_fixed
from cad2urdf.core.extract.spanning_tree import build_spanning_tree
from cad2urdf.core.extract.build_robot import build_robot_from_tree
from cad2urdf.core.inertia.materials import lookup
from cad2urdf.core.parsers.step import load_step_assembly
from cad2urdf.core.urdf.emit import emit_urdf
from tests.unit.test_step_parser import _write_named_assembly  # reuse fixture writer


def test_step_to_urdf(tmp_path: Path) -> None:
    step = tmp_path / "robot.step"
    _write_named_assembly(step)
    asm = load_step_assembly(step)
    tree = build_spanning_tree(classify_default_fixed(asm.edges), base_id=asm.root_id)
    robot = build_robot_from_tree(
        parts=asm.parts, tree=tree, base_id=asm.root_id, robot_name="robot",
        mesh_dir=tmp_path / "meshes", material=lookup("aluminum_6061"),
    )
    assert set(robot.links) == {"robot", "base", "arm"}
    out = tmp_path / "robot.urdf"
    emit_urdf(robot, out, package_name="robot")
    assert out.is_file()
    assert "<joint" in out.read_text()
```

- [ ] **Step 2: Run test to verify it fails / passes the pipeline**

Run: `<prefix> python -m pytest tests/integration/test_step_pipeline.py -q`
Expected: PASS once Tasks 4–6 are in (this wires existing stages; if it fails, the failure pinpoints the integration gap).

- [ ] **Step 3: Add the CLI route**

In `cli/main.py`, replace the `.step`/`.stp` rejection (around the `if suffix in {".step", ".stp"}: raise` block) with a STEP branch that runs the pipeline and emits. Read the surrounding `convert`/main flow first; the STEP branch produces a `Robot` (via the four pipeline calls above with `mesh_dir` = the output package's `meshes/visual` dir) and then continues through the existing emit/scaffold path. Keep a clean OCP-missing message:

```python
    suffix = inp.suffix.lower()
    if suffix in {".step", ".stp"}:
        try:
            from cad2urdf.core.parsers.step import load_step_assembly
            from cad2urdf.core.extract.mate_classify import classify_default_fixed
            from cad2urdf.core.extract.spanning_tree import build_spanning_tree
            from cad2urdf.core.extract.build_robot import build_robot_from_tree
        except ImportError:
            raise SystemExit("STEP import requires OCP. Install with: pip install -e '.[step]'")
        asm = load_step_assembly(inp)
        tree = build_spanning_tree(classify_default_fixed(asm.edges), base_id=asm.root_id)
        robot = build_robot_from_tree(
            parts=asm.parts, tree=tree, base_id=asm.root_id, robot_name=name,
            mesh_dir=meshes_visual_dir, material=lookup("aluminum_6061"),
        )
        # ...continue to the existing emit/scaffold path with `robot`...
```

(Wire `robot` into the existing emission flow rather than the per-mesh path; `meshes_visual_dir` is the output package visual-mesh directory already computed in `main`.)

- [ ] **Step 4: Run the integration + CLI tests**

Run: `<prefix> python -m pytest tests/integration/test_step_pipeline.py tests/unit -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/cad2urdf/cli/main.py tests/integration/test_step_pipeline.py
git commit -m "feat(cli): import STEP assemblies (.stp/.step)"
```

---

## Task 8: GUI wiring — Import STEP

**Files:**
- Create: `src/cad2urdf/gui/workers/step_import.py`
- Modify: `src/cad2urdf/gui/windows/main_window.py` (enable action + dialog + handler)
- Modify: `src/cad2urdf/gui/workers/import_meshes.py` (`_load_one`: drop STEP branch)
- Test: `tests/gui/test_step_import_worker.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/gui/test_step_import_worker.py
from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("OCP")
pytestmark = pytest.mark.gui

from cad2urdf.core.kinematic.model import Robot
from cad2urdf.gui.workers.step_import import build_step_import_job
from tests.unit.test_step_parser import _write_named_assembly


def test_step_import_job_builds_robot(tmp_path: Path) -> None:
    step = tmp_path / "robot.step"
    _write_named_assembly(step)
    job = build_step_import_job(path=step, mesh_dir=tmp_path / "m", robot_name="robot")
    robot = job(lambda c, t, m: None)
    assert isinstance(robot, Robot)
    assert {"base", "arm"} <= set(robot.links)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `<prefix> export QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true && python -m pytest tests/gui/test_step_import_worker.py -q`
Expected: FAIL — `step_import` module missing.

- [ ] **Step 3: Implement the worker (mirrors `build_import_job`)**

```python
# src/cad2urdf/gui/workers/step_import.py
"""Worker job: STEP file -> Robot (XCAF assembly pipeline)."""
from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from cad2urdf.core.extract.build_robot import build_robot_from_tree
from cad2urdf.core.extract.mate_classify import classify_default_fixed
from cad2urdf.core.extract.spanning_tree import build_spanning_tree
from cad2urdf.core.inertia.materials import lookup
from cad2urdf.core.kinematic.model import Robot
from cad2urdf.core.parsers.step import load_step_assembly


def build_step_import_job(
    *, path: Path, mesh_dir: Path, robot_name: str
) -> Callable[[Callable[[int, int, str], None]], Robot]:
    def job(report: Callable[[int, int, str], None]) -> Robot:
        report(1, 3, f"reading {path.name}")
        asm = load_step_assembly(path)
        report(2, 3, "building tree")
        tree = build_spanning_tree(classify_default_fixed(asm.edges), base_id=asm.root_id)
        report(3, 3, "assembling robot")
        return build_robot_from_tree(
            parts=asm.parts, tree=tree, base_id=asm.root_id, robot_name=robot_name,
            mesh_dir=mesh_dir, material=lookup("aluminum_6061"),
        )

    return job
```

- [ ] **Step 4: Run the worker test**

Run: `<prefix> export QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true && python -m pytest tests/gui/test_step_import_worker.py -q`
Expected: PASS

- [ ] **Step 5: Wire the GUI action**

In `main_window.py`: rename `action_import_step_disabled` → `action_import_step`, enable it, connect to a new `_on_import_step_clicked` that opens `QFileDialog.getOpenFileName(..., "STEP (*.step *.stp)")`, asks robot name (like `_on_import_clicked`), runs `build_step_import_job` through a `Worker` + local `QEventLoop` (mirror `dialogs/import_meshes.run_import_into_controller`), using a `tempfile.mkdtemp()` mesh dir, then `controller.replace(robot)`. On `RuntimeError`, show `import failed: {e}` in the status bar.

In `import_meshes.py` `_load_one`: remove the `.step/.stp/.iges/.igs -> STEP_NOT_SUPPORTED` branch (STEP now has its own path); keep the generic "unsupported extension" branch for everything non-STL/OBJ.

- [ ] **Step 6: Run the GUI suite**

Run: `<prefix> export QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true && python -m pytest tests/gui -q`
Expected: PASS

- [ ] **Step 7: Commit**

```bash
git add src/cad2urdf/gui/workers/step_import.py src/cad2urdf/gui/windows/main_window.py src/cad2urdf/gui/workers/import_meshes.py tests/gui/test_step_import_worker.py
git commit -m "feat(gui): Import STEP action via the XCAF assembly pipeline"
```

---

## Task 9: Dependency + docs

**Files:**
- Modify: `pyproject.toml` (`[step]` extra)
- Modify: `README.md` (STEP now supported)
- Modify: `environment.yml` (note OCP/pip path)

- [ ] **Step 1: Add the extra**

In `pyproject.toml` `[project.optional-dependencies]`:

```toml
# STEP/.stp import via OCP (pip-installable OCCT binding). Heavy wheel.
step = [
    "cadquery-ocp>=7.7",
]
```

- [ ] **Step 2: README**

- Update the GUI section's "Mesh inputs only" note and Status/Roadmap: STEP assembly import is supported via `pip install -e ".[step]"` (each part becomes a link, fixed joints, re-typed in the editor).
- Bump the test badge to the new count (run `<prefix> python -m pytest -q | tail -1` to get it).

- [ ] **Step 3: environment.yml**

Add a comment that STEP works either via conda `pythonocc-core` or the pip `[step]` extra (`cadquery-ocp`); the code targets the OCP API.

- [ ] **Step 4: Full gate**

Run: `<prefix> export QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true && python -m pytest -q && python -m mypy src/cad2urdf/core src/cad2urdf/gui && python -m ruff check src/cad2urdf tests`
Expected: all pass. (Add `cadquery-ocp` to the mypy override list in `pyproject.toml` if its untyped API trips mypy strict — mirror the existing `trimesh.*`/`OCC.*` overrides; add `OCP.*`.)

- [ ] **Step 5: Commit**

```bash
git add pyproject.toml README.md environment.yml
git commit -m "build(step): add [step] extra (cadquery-ocp) + document STEP import"
```

---

## Self-Review notes

- **Spec coverage:** engine/lazy-import (T4–5, T9), XCAF names/hierarchy/poses (T5), classify+spanning reuse (T7–8), build_robot (T6), frame links model+emitter+IO (T1–3), units mm→m (T4–5), CLI+GUI wiring + rejection removal (T7–8), `[step]` extra + docs (T9), `importorskip` guards (T4–8). All covered.
- **Type consistency:** `StepPart(id,name,mesh,is_leaf)`, `StepAssembly(parts,edges,root_id)`, `build_robot_from_tree(parts,tree,base_id,robot_name,mesh_dir,material)`, `ClassifiedJoint(parent_id,child_id,joint_type,axis,relative_pose)` used consistently across T5–8. `Material.density_kg_m3`/`.name` per `core/inertia/materials.py`.
- **Open verification (flag during execution):** the OCC location matrix read uses `trsf.Value(row,col)` (1-indexed, rows 1–3, cols 1–4) — verify the arm pose test passes; `tool.GetShape_s(label)` returns the part shape for tessellation — verify in T5.
