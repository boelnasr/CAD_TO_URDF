# cad2urdf v1.0 — IGES Support + Joint Inference Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **This is a planning document — do not deviate from the architecture below without flagging the decision.**

**Goal:** Ship cad2urdf v1.0, defined by the two README roadmap pillars:
1. **IGES support** (`.iges`/`.igs`) alongside the shipped STEP path.
2. **Richer assembly mate extraction** — auto-classify `concentric → revolute`, `sliding → prismatic`, with a safe `unknown → fixed` fallback, replacing the v0.2 every-edge-fixed stub.

**Architecture (Approach A, decided 2026-06-14):** Extract the XCAF document walk shared by STEP and IGES into a new `core/parsers/caf.py`; STEP and IGES become thin wrappers differing only in the CAF reader class (`STEPCAFControl_Reader` vs `IGESCAFControl_Reader`). The B-rep `TopoDS_Shape` handle per part is carried in a **side-map keyed by `part_id`** that lives OUTSIDE the frozen, serializable `Robot`/`StepPart` AST (OCP handles are not JSON/pickle-serializable). A new `core/extract/joint_infer.py` consumes that side-map to populate `AssemblyEdge` hint fields (`mate_kind`, `axis`, `confidence`); `mate_classify.py` gains a hint-aware classifier that reads those hints. The pure non-OCP pipeline (`spanning_tree`, `build_robot`) is unchanged.

**Rejected alternatives:** B (duplicate the XCAF walk per format — violates DRY, two drift surfaces); C (store `TopoDS_Shape` on the dataclass — breaks `frozen`/serializable invariant and `.cad2urdf` round-trip).

**Tech Stack:** Python 3.10 (no 3.11+ syntax), OCP (`cadquery-ocp`, lazy-imported, conda `pythonocc-core` alternative), trimesh, numpy, PyQt6 (GUI wiring), MCP SDK (MCP wiring), pytest (`importorskip("OCP")`).

**Spec:** [[2026-06-14-cad-to-urdf-v1.0-iges-joint-inference-design]] (write this design doc as Task 0 if it does not yet exist; otherwise this plan stands alone)

**Run prefix (all test/lint/type commands):**
`unset PYTHONPATH AMENT_PREFIX_PATH && source .venv/bin/activate &&` — GUI tests additionally need `export QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true`. mypy is strict on `core/`/`gui/`/`mcp/`. OCP is installed in the dev `.venv` and via `[step]`.

---

## Current state (verified 2026-06-14)

- STEP import ships in CLI (`cli/main.py: _build_step_robot`) and GUI (`gui/workers/step_import.py`, `gui/windows/main_window.py: _on_import_step_clicked`). **MCP has NO STEP command** — `import_step` is absent from `protocol.py` COMMANDS, `dispatcher.py`, `mcp/server.py`, and the README tool table.
- `core/parsers/step.py: load_step_assembly` walks XCAF and **discards every `TopoDS_Shape` after `tessellate`** (line ~172). Those handles are exactly what joint inference needs.
- `core/extract/types.py: AssemblyEdge` already carries optional `mate_kind`, `world_pose`, `confidence` hint fields — nothing populates them yet.
- `core/extract/mate_classify.py: classify_default_fixed` maps every edge to `fixed` (the stub to replace).
- `core/extract/spanning_tree.py` and `build_robot.py` are built, working, and OCP-free — must remain unchanged in behavior.
- The CLI already rejects IGES with `IGES_NOT_SUPPORTED_MSG`; the GUI file dialog filters to `*.step *.stp`.
- `pyproject.toml` `[step]` extra = `cadquery-ocp>=7.7`; CI installs `[dev,gui,step,mcp]` and runs pytest under `xvfb`.

---

## File Structure

- `src/cad2urdf/core/parsers/caf.py` — **new**: shared XCAF walk; `load_caf_assembly(reader_factory, path, ...)` returning parts + edges + root **and a B-rep side-map**.
- `src/cad2urdf/core/parsers/step.py` — modify: `load_step_assembly` becomes a thin wrapper delegating to `caf.py` with `STEPCAFControl_Reader`. Keep `StepPart`/`StepAssembly`/`tessellate` exported (back-compat).
- `src/cad2urdf/core/parsers/iges.py` — **new**: thin wrapper delegating to `caf.py` with `IGESCAFControl_Reader`.
- `src/cad2urdf/core/extract/types.py` — modify (additive only): an `AssemblyShapes` side-map type if a struct is preferred over a bare dict.
- `src/cad2urdf/core/extract/joint_infer.py` — **new**: B-rep geometry heuristics → populate `AssemblyEdge` hints.
- `src/cad2urdf/core/extract/mate_classify.py` — modify: add `classify_from_hints(edges)` reading `mate_kind`/`axis`/`confidence`; keep `classify_default_fixed` as the no-hint fallback.
- `src/cad2urdf/cli/main.py` — modify: accept `.iges`/`.igs`; route STEP+IGES through one CAF path; run joint inference; drop the IGES-rejection message.
- `src/cad2urdf/gui/workers/step_import.py` — modify/rename to a generic `caf_import` job (or add `build_iges_import_job`) feeding joint inference.
- `src/cad2urdf/gui/dialogs/import_meshes.py` — modify: widen STEP file filter to include `*.iges *.igs`; generic CAF import entry point.
- `src/cad2urdf/gui/windows/main_window.py` — modify: rename action to "Import CAD assembly (STEP/IGES)" or add an IGES action.
- `src/cad2urdf/gui/control/protocol.py` — modify: add `import_step` (+ `import_iges` or a unified `import_cad`) to the `COMMANDS` frozenset.
- `src/cad2urdf/gui/control/dispatcher.py` — modify: add `_cmd_import_step` / `_cmd_import_iges` handlers.
- `src/cad2urdf/mcp/server.py` — modify: add the matching `@mcp.tool()` wrapper(s).
- `README.md` — modify: roadmap → v1.0 shipped; IGES install/usage; new MCP tool(s) row; joint-inference algorithm + Mermaid.
- `pyproject.toml` — no new extra (IGES uses the same OCP wheel as STEP); add new pytest markers if needed.
- Tests: `tests/unit/test_caf_parser.py`, `tests/unit/test_iges_parser.py`, `tests/unit/test_joint_infer.py`, `tests/unit/test_mate_classify_hints.py`, `tests/integration/test_iges_pipeline.py`, `tests/integration/test_joint_inference_pipeline.py`, additions to `tests/gui/` and an MCP wiring test.

---

## Task 0: (Optional) write the design doc

**Files:** `docs/superpowers/specs/2026-06-14-cad-to-urdf-v1.0-iges-joint-inference-design.md`

- [ ] Capture Approach A, the rejected B/C alternatives, the joint-inference heuristic interface, and the scope decisions from the "Scope" section below. Link it from this plan and the README. Skip if the team prefers this plan as the single source.

**Acceptance:** Design file exists and is wiki-linked, OR the team explicitly elects to proceed plan-only.

---

## Task 1: Shared CAF walk (`core/parsers/caf.py`) — refactor STEP onto it with zero regression

**Files:**
- New: `src/cad2urdf/core/parsers/caf.py`
- Modify: `src/cad2urdf/core/parsers/step.py`
- Test: `tests/unit/test_caf_parser.py`, existing `tests/unit/test_step_parser.py` (must stay green unchanged)

- [ ] **Step 1 — Characterization first.** Run the existing STEP suite and record the pass count as the regression baseline:
  `unset PYTHONPATH AMENT_PREFIX_PATH && source .venv/bin/activate && pytest tests/unit/test_step_parser.py tests/integration/test_step_pipeline.py -q`
- [ ] **Step 2 — Write `caf.py`.** Lift the XCAF walk from `step.py` (`_name_of`, `_pose_of`, `_sanitize`, `entry`, `visit`, the `GetFreeShapes` driver) into `load_caf_assembly(*, reader_factory, path, unit_scale=0.001)`. The ONLY format-specific input is `reader_factory`, a `Callable[[], <CAFReader>]` the caller supplies. All OCP imports stay inside function bodies (lazy). Return both the existing `StepAssembly`-shaped result AND the new B-rep side-map (see Task 3). Keep `tessellate` here or in a shared `meshing` helper.
- [ ] **Step 3 — Rewrite `step.py` as a wrapper.** `load_step_assembly` calls `load_caf_assembly` passing a `STEPCAFControl_Reader` factory (with `SetNameMode(True)`). Re-export `StepPart`, `StepAssembly`, `tessellate` from their new home for import back-compat (`build_robot.py` and `gui/workers/step_import.py` import `StepPart` from `core.parsers.step`).
- [ ] **Step 4 — mypy + lint.** OCP modules trigger `import-untyped`; reuse the existing `[[tool.mypy.overrides]]` OCP ignore pattern from `step.py` so `caf.py` is clean under strict.
- [ ] **Step 5 — Regression gate.** Re-run Step 1's command; pass count must be **identical**. Add `tests/unit/test_caf_parser.py` exercising the walk via a STEP fixture (`importorskip("OCP")`).

**Acceptance:**
- `core/parsers/caf.py` exists, is OCP-free at import time, Qt-free, mypy-strict clean.
- `test_step_parser.py` + `test_step_pipeline.py` pass with the same count as the Step 1 baseline (no regression).
- `step.py` contains no duplicated walk logic — only the reader factory + re-exports.

---

## Task 2: IGES wrapper (`core/parsers/iges.py`) + CLI/GUI/MCP wiring

**Files:**
- New: `src/cad2urdf/core/parsers/iges.py`
- Test: `tests/unit/test_iges_parser.py`, `tests/integration/test_iges_pipeline.py`

- [ ] **Step 1 — `iges.py` wrapper.** `load_iges_assembly(path, *, unit_scale=0.001)` delegates to `load_caf_assembly` with an `IGESCAFControl_Reader` factory. IGES has weaker named-assembly structure than STEP — if `GetFreeShapes` returns a single flat shape with no components, document and test that fallback (one root + N leaf parts, or one part). Use the same `_OCP_HINT` import-error message.
- [ ] **Step 2 — CLI wiring.** In `cli/main.py`: extend the `is_step` detection to a shared `is_cad_assembly` check covering `{.step,.stp,.iges,.igs}`; route both through one `_build_cad_robot` (generalize `_build_step_robot`, selecting the loader by suffix). Delete `IGES_NOT_SUPPORTED_MSG` and its raise in `_load_mesh_to_link`. Keep the single-assembly-input rule (no mixing).
- [ ] **Step 3 — GUI wiring.** Widen the import dialog filter to `STEP/IGES (*.step *.stp *.iges *.igs)`; generalize `build_step_import_job` → `build_cad_import_job(path, ...)` that picks the loader by suffix; rename the menu action to "Import CAD assembly…" (keep the STEP label as an alias if simpler).
- [ ] **Step 4 — MCP wiring (FOUR-PLACE SYNC — see CLAUDE.md).** Add the new command in ALL of:
  1. `gui/control/protocol.py` — add to the `COMMANDS` frozenset (e.g. `import_step`, `import_iges`, or a single `import_cad` taking a path; **prefer one `import_cad` tool** to keep the surface small).
  2. `gui/control/dispatcher.py` — add `_cmd_import_cad` routing to the controller via the same `RobotController.replace`/`apply` path the GUI uses (so it lands on the undo stack and renders live).
  3. `mcp/server.py` — add the `@mcp.tool()` wrapper calling `bridge.call("import_cad", {...})`.
  4. `README.md` — add the row to the MCP tool table (Build group).
- [ ] **Step 5 — Tests.** `test_iges_parser.py` (unit, `importorskip("OCP")`, IGES fixture); `test_iges_pipeline.py` (CLI end-to-end IGES → ROS package); an MCP wiring test asserting `import_cad ∈ COMMANDS`, a `_cmd_import_cad` exists, and the MCP tool is registered (the four places stay in sync).

**Acceptance:**
- `cad2urdf assembly.iges -o out` produces a valid ROS package (URDF + meshes + scaffold).
- The GUI imports an IGES assembly; MCP `import_cad` drives the same path live with undo.
- A test fails loudly if any of the four MCP sync points is missing.
- IGES rejection message is gone from the codebase.

---

## Task 3: B-rep side-map — handles survive parse → inference without touching the AST

**Files:**
- Modify: `src/cad2urdf/core/parsers/caf.py` (produce the side-map)
- Optional: `src/cad2urdf/core/extract/types.py` (a typed wrapper)
- Test: `tests/unit/test_caf_parser.py` (side-map keys match part ids; AST stays serializable)

- [ ] **Step 1 — Define the carrier.** `load_caf_assembly` returns `(assembly, shapes)` where `shapes: dict[str, Any]` maps `part_id → TopoDS_Shape` (only leaf parts; assembly nodes have none). Type the value `Any` (OCP untyped) and document it as transient/non-serializable. The `StepAssembly`/`Robot` AST is **unchanged** — handles never land on a dataclass field.
- [ ] **Step 2 — Lifetime contract.** Document and enforce: the side-map is consumed by joint inference (Task 4) **immediately after parse, before `build_robot`**, then dropped. It never enters `.cad2urdf` save/load, never crosses the GUI worker→controller boundary as state, never crosses the MCP socket. The worker runs inference inside the job and emits only the resulting `Robot` + classified edges.
- [ ] **Step 3 — Memory note.** `TopoDS_Shape` handles reference the loaded OCCT B-rep (can be large). The side-map and the originating XCAF `doc` must go out of scope once inference completes — add an explicit `del`/scope boundary in the loader/worker and assert no handle is retained on the `Robot`.
- [ ] **Step 4 — Serialization guard test.** Build a `Robot` from a STEP/IGES import, `save_project` + `load_project` it, and assert success (no OCP handle leaked into the AST). Assert `shapes` keys == leaf `part_id`s.

**Acceptance:**
- No `TopoDS_Shape` (or any OCP type) appears on any `core/kinematic` or parser dataclass field.
- `.cad2urdf` round-trip of an imported assembly succeeds.
- Side-map is keyed by `part_id`, covers exactly the leaf parts, and is dropped after inference.

---

## Task 4: Joint inference (`core/extract/joint_infer.py`) + hint-aware classifier

**Files:**
- New: `src/cad2urdf/core/extract/joint_infer.py`
- Modify: `src/cad2urdf/core/extract/mate_classify.py`
- Test: `tests/unit/test_joint_infer.py`, `tests/unit/test_mate_classify_hints.py`, `tests/integration/test_joint_inference_pipeline.py`

Heuristic interface (testable, fallback-safe):

```
infer_edge_hints(
    edges: list[AssemblyEdge],
    shapes: dict[str, TopoDS_Shape],   # the Task-3 side-map
) -> list[AssemblyEdge]                 # same edges, hints populated (immutable copies)
```

- [ ] **Step 1 — Geometry probes (lazy OCP).** For a parent/child shape pair, extract candidate cylindrical/planar faces (`BRepAdaptor_Surface` → `GeomAbs_Cylinder` / `GeomAbs_Plane`). All OCP imports inside function bodies.
- [ ] **Step 2 — Concentric → revolute.** If parent and child share a coaxial cylindrical-face pair (axes collinear within tolerance), set `mate_kind="concentric"`, `axis = cylinder axis` (in the edge's parent frame), `confidence` from axis-alignment + radius match.
- [ ] **Step 3 — Sliding → prismatic.** If they share parallel planar faces / a prismatic contact with a free translational direction and NO shared rotation axis, set `mate_kind="sliding"`, `axis = slide direction`, `confidence` accordingly.
- [ ] **Step 4 — Fallback.** Anything ambiguous/unknown → leave `mate_kind=None` (or `"unknown"`), `confidence` low. Never raise — inference is advisory.
- [ ] **Step 5 — Tolerances as named constants.** Angular/linear/radius tolerances are module-level named constants with docstrings, tunable without touching logic.
- [ ] **Step 6 — `classify_from_hints`.** In `mate_classify.py`: `concentric → revolute` (default symmetric limits, e.g. `±π`, configurable), `sliding → prismatic` (translation limits from face extents or a sane default), else `fixed`. Carry `axis` and `relative_pose` into `ClassifiedJoint`. Below a `min_confidence` threshold, downgrade to `fixed` (conservative). Keep `classify_default_fixed` for the no-hint / no-OCP path.
- [ ] **Step 7 — Wire into the pipeline.** CLI `_build_cad_robot` and the GUI CAF job call `infer_edge_hints(asm.edges, shapes)` then `classify_from_hints(...)` (falling back to `classify_default_fixed` if `shapes` is empty). `build_spanning_tree` consumes the classified edges unchanged.
- [ ] **Step 8 — Tests.** Unit-test `joint_infer` with synthetic OCP shapes (or fixtures) for the three cases + fallback (`importorskip("OCP")`). Unit-test `classify_from_hints` purely (synthetic `AssemblyEdge`s with hints — NO OCP needed). Integration-test a known assembly fixture asserting at least one `revolute`/`prismatic` is inferred and the URDF emits.

**Acceptance:**
- Synthetic concentric pair → `revolute` with correct axis; sliding pair → `prismatic`; ambiguous → `fixed` with low confidence.
- `classify_from_hints` is testable with zero OCP (pure dataclass in/out).
- Low-confidence edges downgrade to `fixed` (no silently-wrong moving joints).
- Existing STEP fixtures that previously emitted all-fixed still emit valid URDF (no crash if inference finds nothing).

---

## Task 5: Testing, CI, and docs hardening

**Files:** `pyproject.toml`, `.github/workflows/ci.yml`, `README.md`, fixtures under `tests/`.

- [ ] **Step 1 — Fixtures.** Add/generate small IGES + STEP fixtures with a known revolute (concentric) and prismatic (sliding) mate. Prefer generating them in a fixture helper (OCP/cadquery) over committing large binaries; gate with `importorskip("OCP")` and a `needs_step_fixture`-style marker.
- [ ] **Step 2 — OCP-not-on-PyPI / CI.** CI already installs `[dev,gui,step,mcp]` (pulls `cadquery-ocp`) and runs under `xvfb`. Confirm the IGES path is exercised. If a future pure-pip-without-OCP matrix leg is wanted, ensure all OCP-dependent tests `importorskip("OCP")` so the no-OCP leg stays green. **The pure non-OCP `mate_classify` hint path must have coverage that runs without OCP.**
- [ ] **Step 3 — mypy strict.** New modules (`caf.py`, `iges.py`, `joint_infer.py`) clean under strict; reuse the OCP `import-untyped` override.
- [ ] **Step 4 — Full gate.** `unset PYTHONPATH AMENT_PREFIX_PATH && pytest && mypy && ruff check` all green (GUI tests with offscreen env).
- [ ] **Step 5 — README.** Roadmap: move IGES + auto-classification to "shipped v1.0". Install: note IGES uses the same `[step]`/OCP path (no new extra). GUI/CLI: document IGES inputs and that joints are now auto-typed (review/confirm in editor). MCP: add the `import_cad` tool row. Add a joint-inference algorithm subsection with a **Mermaid** flowchart (concentric→revolute / sliding→prismatic / else→fixed). Bump status badge to v1.0.

**Acceptance:** CI green on a matrix that exercises STEP + IGES + GUI + MCP; README accurately reflects v1.0 capabilities and the new tool; mypy/ruff clean.

---

## Scope recommendation (the key open question)

The user asked whether v1.0 should also include: macOS/Windows packaging, a stable version-bump/release process, docs polish, and extra joint types (continuous/planar). YAGNI-informed call:

**INCLUDE in v1.0 (serves the two pillars or is the cost of calling it "1.0"):**
- **Version bump + release hygiene** — `0.2.0a0 → 1.0.0` (drop the alpha tag), CHANGELOG, git tag, README badge. Calling something "v1.0" while the version string says `a0` is incoherent. Low cost, high signal. **Include.**
- **`continuous` joint type from inference** — a `concentric` mate with NO travel limit is naturally `continuous`, and the joint type already exists in the YAML schema (README line 330). It is a near-free output of the same concentric heuristic (revolute-without-limits). **Include as a sub-case of Task 4**, gated on confidence.
- **Targeted docs polish** — only the IGES + joint-inference surface (Task 5 Step 5). Broad docs rewrite is out. **Include the targeted slice.**

**DEFER (real scope, not required by the two pillars):**
- **macOS/Windows packaging** — README already parks this at "v-future". OCP wheels + Qt + VTK cross-platform packaging is a large, orthogonal effort with its own CI matrix. **Defer.**
- **`planar` joint type** — requires 2-DOF mate detection (two independent slide axes) with no clean single-mate geometric signature; high ambiguity, low real-world CAD frequency. **Defer to v2** (LLM/ML priors are better suited).
- **MCP `import_iges` as a separate tool** — prefer ONE `import_cad` tool dispatching by extension over two near-identical tools. Keeps the four-place sync surface minimal. **Fold into Task 2.**

**Reasoning:** The pillars are IGES + auto-classification. Version/release hygiene and the IGES/inference docs are the irreducible "make it a real 1.0" tax. `continuous` rides for free on the concentric heuristic. Everything else (packaging, planar, multi-tool MCP surface) is independent scope that dilutes the release and is already roadmapped later. Ship the two pillars clean.

---

## Sequencing & risk

**Order:** Task 1 (shared CAF, regression-gated) → Task 3 (side-map, since the CAF walk must emit it) → Task 2 (IGES wrapper + wiring, exercises the shared walk) → Task 4 (joint inference, the highest-value/highest-risk work) → Task 5 (CI/docs/version). Task 1 and 3 are tightly coupled (the walk produces the side-map) and should land together. Task 0 (design doc) is optional and front-loaded.

```mermaid
flowchart LR
    T0[Task 0 design optional] --> T1[Task 1 shared CAF walk]
    T1 --> T3[Task 3 B-rep side-map]
    T3 --> T2[Task 2 IGES + CLI/GUI/MCP wiring]
    T2 --> T4[Task 4 joint inference + hint classifier]
    T4 --> T5[Task 5 CI / docs / version bump]
```

**Top 3 risks:**
1. **Joint-inference accuracy (highest).** Geometric mate detection from B-reps is genuinely hard; real CAD has noise, sub-shapes, and ambiguous mates. *Mitigation:* conservative `min_confidence` downgrade to `fixed` — wrong-but-safe beats wrong-and-moving; treat output as advisory hints the user confirms in the editor; ship the three clear cases (concentric/sliding/unknown) and defer the rest. The pipeline already works with all-fixed, so inference only adds value, never blocks.
2. **B-rep lifetime / memory.** `TopoDS_Shape` handles and the XCAF `doc` retain the full loaded model; leaking them into the `Robot`, the GUI controller, or the MCP boundary would balloon memory and break serialization. *Mitigation:* Task 3's explicit scope boundary + the `.cad2urdf` round-trip guard test + the "no OCP type on any dataclass" assertion.
3. **OCP in CI / not on PyPI as a pure wheel.** IGES + inference depend on `cadquery-ocp`; CI already pulls it, but any reviewer on a pure-pip-no-OCP setup, or a future no-OCP CI leg, must not break. *Mitigation:* `importorskip("OCP")` on every OCP-touching test; keep `classify_from_hints` pure (OCP-free, fully testable) so the classification logic has coverage independent of the geometry probes; keep all OCP imports lazy/in-function so `core` import stays clean.

**Open decisions to resolve before execution:**
- One unified MCP `import_cad` tool vs separate `import_step`/`import_iges` (plan recommends unified).
- Default joint limits for inferred revolute/prismatic (symmetric `±π` / face-extent vs a fixed default) — needs a product call.
- `min_confidence` threshold value for the fixed-downgrade — start conservative (e.g. 0.6) and tune against fixtures.
- Whether to emit `continuous` for limitless concentric mates (plan recommends yes, gated on confidence).
