# Task 2 report — IGES wrapper and CLI/GUI/MCP wiring

## Status

Implemented and verified. The binding product decision that CAD import is a
fresh document replacement takes precedence over the stale brief sentence about
undo: GUI and MCP CAD imports use `RobotController.replace`, which clears undo
and redo history.

## Implementation

- Added `core/parsers/iges.py`, symmetric with the STEP wrapper:
  - `load_iges_assembly(path) -> StepAssembly`
  - `load_iges_assembly_with_shapes(path) -> (StepAssembly, shapes)`
  - lazy `IGESCAFControl_Reader` import with name mode enabled
  - documented the weak-structure fallback: one free shape with no components
    becomes one mesh-bearing root with no edges
  - the transient B-rep map is returned only by the explicit `with_shapes`
    interface and is discarded by all Task 2 consumers
- Generalized CLI assembly routing to exactly `.step`, `.stp`, `.iges`, and
  `.igs`.
  - one CAD assembly is accepted without joints YAML
  - CAD/mesh mixing and multiple CAD inputs are rejected by the single-input
    boundary
  - `_build_cad_robot` selects STEP or IGES by suffix
  - the old `_build_step_robot` remains as a compatibility wrapper
  - the obsolete IGES rejection constant and behavior were removed
  - a legitimate flat one-part IGES bypasses the edge-only spanning-tree
    builder with an empty tree
- Generalized the GUI worker to `build_cad_import_job`; the old
  `build_step_import_job` is an exact compatibility alias.
- Added the GUI CAD action and `STEP/IGES (*.step *.stp *.iges *.igs)` picker,
  while retaining the old action attribute and slot as compatibility aliases.
- Added `import_cad(path)` through all four MCP synchronization surfaces:
  protocol registry, dispatcher handler, MCP tool registration/wrapper, and
  README Build table.
- Dispatcher CAD import runs the shared worker synchronously, then calls
  `RobotController.replace`; the resulting fresh document renders through the
  normal controller signal and has empty undo/redo history.
- Preserved the existing `classify_default_fixed` path. No joint inference or
  side-map retention/passing was added.

## Files

Production:

- `src/cad2urdf/core/parsers/iges.py`
- `src/cad2urdf/cli/main.py`
- `src/cad2urdf/gui/workers/step_import.py`
- `src/cad2urdf/gui/dialogs/import_meshes.py`
- `src/cad2urdf/gui/windows/main_window.py`
- `src/cad2urdf/gui/control/protocol.py`
- `src/cad2urdf/gui/control/dispatcher.py`
- `src/cad2urdf/mcp/server.py`
- `README.md` (MCP Build row only)

Tests:

- `tests/unit/test_iges_parser.py`
- `tests/integration/test_iges_pipeline.py`
- `tests/unit/test_import_cad_wiring.py`
- `tests/gui/test_step_import_worker.py`
- `tests/gui/control/test_dispatcher_build.py`
- `tests/gui/test_main_window.py`
- `tests/unit/control/test_protocol.py`

## TDD evidence

### Primary RED

Command:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH .venv/bin/pytest \
  tests/unit/test_iges_parser.py \
  tests/integration/test_iges_pipeline.py \
  tests/unit/test_import_cad_wiring.py \
  tests/unit/control/test_protocol.py -q -ra
```

Result before production changes: **8 failed, 5 passed**.

The failures were the intended missing behavior:

- IGES parser module absent (2)
- `.iges` and `.igs` incorrectly routed to the joints-YAML path (2)
- mixed IGES input emitted the obsolete rejection instead of the single-input
  boundary (1)
- `import_cad` absent from synchronized MCP surfaces (2)
- protocol registry missing `import_cad` (1)

### Primary GREEN

The same command after implementation: **13 passed**.

### MCP single-argument RED/GREEN

Command:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH .venv/bin/pytest \
  tests/unit/test_import_cad_wiring.py::test_import_cad_mcp_wrapper_sends_one_path \
  -q -ra
```

RED: **1 failed** because the wrapper still sent an extra `robot_name` field.

GREEN command:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH .venv/bin/pytest \
  tests/unit/test_import_cad_wiring.py -q -ra
```

Result: **2 passed**.

### Focused GUI and regression GREEN

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
  .venv/bin/pytest \
  tests/gui/test_step_import_worker.py \
  tests/gui/control/test_dispatcher_build.py -q -ra
```

Result: **10 passed**.

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
  .venv/bin/pytest \
  tests/gui/test_main_window.py::test_cad_import_action_and_dialog_cover_step_and_iges \
  -q -ra
```

Result: **1 passed**.

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH .venv/bin/pytest \
  tests/integration/test_step_pipeline.py \
  tests/gui/test_step_import_worker.py \
  tests/gui/control/test_dispatcher_build.py \
  tests/unit/test_import_cad_wiring.py \
  tests/unit/control/test_protocol.py -q -ra
```

Result: **21 passed**.

## Full verification

The initial full-suite offscreen attempt reached the pre-existing GUI
`test_e2e_smoke` and hit a GLX context error. Setting VTK's documented runtime
window override to its software offscreen backend made the complete gate
deterministic:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
  VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
  .venv/bin/pytest -q -ra
```

Result: **328 passed, 1 skipped**. The only skip is the optional ManipulaPy
dependency (`No module named 'ManipulaPy'`).

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH .venv/bin/ruff check .
```

Result: `All checks passed!`

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  .venv/bin/mypy src/cad2urdf/core src/cad2urdf/gui src/cad2urdf/mcp
```

Result: `Success: no issues found in 64 source files`

`git diff --check` also passed, and `rg` found no
`IGES_NOT_SUPPORTED_MSG`/obsolete IGES rejection in source or tests.

## Self-review

- Scope: no joint inference, release metadata, plan, or broader README content
  changed.
- Compatibility: STEP parser/CLI/GUI tests remain green; both old private CLI
  helper and GUI builder names remain callable.
- Core isolation: all OCP imports remain inside function bodies; the full
  core-no-Qt/VTK test matrix includes the new parser and passes.
- State boundary: neither CLI, GUI worker, dispatcher, MCP, controller, project,
  nor socket payload retains or receives the transient shape map.
- History semantics: a focused dispatcher test seeds undo history, imports CAD,
  and proves both undo and redo are cleared.
- MCP drift guard: one focused test checks the protocol command, dispatcher
  handler, FastMCP registration, and README Build row together.
- Mutation check:
  - wrong IGES suffix routing fails both parametrized CLI cases
  - removing the flat fallback/empty-tree handling fails parser and pipeline
  - selecting STEP for IGES fails the worker test
  - using `apply` instead of `replace` fails the history assertions
  - dropping any MCP sync surface fails the four-place wiring test

## Concerns

- IGES/XCAF commonly collapses named assemblies into one compound free shape.
  This implementation intentionally follows the accepted v1 fallback and emits
  that compound as one mesh-bearing link; it does not attempt topology-based
  decomposition.
- GUI/MCP imports keep their generated mesh temp directory for the lifetime of
  the process because live Robot links reference those files. This matches the
  existing STEP GUI behavior.
