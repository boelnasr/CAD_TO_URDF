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

---

## Fix round 1/5 — background MCP import and generated-asset ownership

### Review findings addressed

1. MCP `import_cad` no longer executes OCP parsing, tessellation, and STL export
   on the dispatcher/Qt calling thread. `run_worker_job` uses the existing
   `Worker`/`QThread` mechanism and a local `QEventLoop`: the heavy callable runs
   in the worker thread while Qt continues processing events, then the
   dispatcher resumes on the Qt thread, calls `RobotController.replace`, and
   returns the same synchronous JSON envelope to the socket client.
2. Added `CadAssetOwner`, an explicit per-controller/session owner for generated
   CAD mesh trees.
   - only directories allocated by that owner can be activated or deleted
   - failed GUI and MCP imports discard their pending partial tree
   - successful imports replace/emit the new Robot first, then delete the
     superseded active CAD tree
   - ordinary fresh-document replacement (mesh import/project open) retires the
     prior CAD tree after emitting the replacement
   - controller/window/application shutdown removes active and pending owned
     trees
   - source CAD paths and caller-owned output directories are never adopted
3. README now states that CAD import is a fresh-document exception which clears
   undo/redo history.

The prior concern above that generated meshes remain for the full process is
superseded by this fix: only the current CAD document's generated tree remains,
and it is removed at document replacement or shutdown.

### Saved-project durability assessment

The current project format (`SCHEMA_VERSION = 1`) is JSON-only.
`core/project/save.py::_link_to_dict` serializes `visual_mesh_path` and
`collision_mesh_path` as path strings, while `save_project` writes only that
JSON payload; it does not copy or bundle mesh bytes. Consequently, a project
saved from a live CAD import points at session-owned absolute mesh paths. The
required shutdown cleanup makes those paths unavailable in a later process.

Making such a project durable requires a product/schema decision (for example,
a project bundle/directory format, asset copying rules, relative-path
resolution, collision policy, and migration from schema v1). That is beyond
Task 2 and was not silently introduced here. The strongest safe in-scope
behavior is implemented: deterministic ownership and cleanup during the live
session, with no risk of deleting user-owned input/output directories.

### Files changed in this fix round

- New: `src/cad2urdf/gui/state/cad_assets.py`
- Updated:
  - `src/cad2urdf/gui/state/controller.py`
  - `src/cad2urdf/gui/workers/base.py`
  - `src/cad2urdf/gui/control/dispatcher.py`
  - `src/cad2urdf/gui/dialogs/import_meshes.py`
  - `src/cad2urdf/gui/windows/main_window.py`
  - `src/cad2urdf/gui/app.py`
  - `README.md`
- Tests:
  - `tests/gui/test_cad_asset_owner.py`
  - `tests/gui/control/test_dispatcher_build.py`
  - `tests/gui/control/test_dispatcher_project.py`
  - `tests/gui/test_import_dialog.py`
  - `tests/gui/test_main_window.py`

### Strict TDD evidence

Primary fix-round RED:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
  VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
  .venv/bin/pytest \
  tests/gui/control/test_dispatcher_build.py::test_import_cad_runs_heavy_job_off_dispatch_thread \
  tests/gui/test_cad_asset_owner.py \
  tests/gui/test_import_dialog.py::test_failed_cad_import_removes_generated_assets_but_not_source \
  tests/gui/control/test_dispatcher_project.py::test_failed_mcp_cad_import_removes_generated_assets_but_not_source \
  -q -ra
```

Result before production changes: **7 failed**.

- the heavy job thread ID equalled the dispatcher/main thread ID
- all four ownership lifecycle tests failed because no explicit owner existed
- GUI and MCP failed-import cleanup tests failed because no owner existed

Primary fix-round GREEN: the same command completed with **7 passed**.

Shutdown RED:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
  VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
  .venv/bin/pytest \
  tests/gui/test_main_window.py::test_closing_window_removes_current_generated_cad_assets \
  -q -ra
```

Result before `MainWindow.closeEvent` ownership cleanup: **1 failed**, because
the active generated directory still existed after close.

Shutdown GREEN: the same command completed with **1 passed**.

### Focused and full verification

Focused amended/relevant gate:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
  VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
  .venv/bin/pytest \
  tests/unit/test_iges_parser.py tests/integration/test_iges_pipeline.py \
  tests/unit/test_import_cad_wiring.py tests/unit/control/test_protocol.py \
  tests/gui/test_cad_asset_owner.py tests/gui/test_import_dialog.py \
  tests/gui/test_step_import_worker.py tests/gui/test_main_window.py \
  tests/gui/test_worker_base.py tests/gui/control/test_dispatcher_build.py \
  tests/gui/control/test_dispatcher_project.py tests/gui/control/test_server.py \
  tests/gui/test_controller.py tests/gui/test_project_io_menu.py -q -ra
```

Result: **54 passed**.

Full suite:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
  VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
  .venv/bin/pytest -q -ra
```

Result: **336 passed, 1 skipped**. The one skip remains the optional
ManipulaPy dependency.

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH .venv/bin/ruff check .
```

Result: `All checks passed!`

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  .venv/bin/mypy src/cad2urdf/core src/cad2urdf/gui src/cad2urdf/mcp
```

Result: `Success: no issues found in 65 source files`

### Fix-round self-review

- Thread boundary: the test records Python thread IDs from the heavy job and
  controller signal, proving the job is off the dispatch thread and replacement
  is on it.
- Wire compatibility: `dispatch` remains synchronous and returns the existing
  `{ok,result}` / `{ok,error,detail}` envelope. `Worker` retains its public
  string-valued `failed` signal while the blocking adapter rethrows the original
  exception type for dispatcher error classification.
- Cleanup ordering: tests assert the old tree exists inside `robotChanged` and
  is absent after replacement returns.
- Ownership safety: activation rejects a user-created directory before Robot
  mutation, and failure tests prove source files remain unchanged.
- State boundary: ownership lives beside the controller, not inside `Robot`,
  project JSON, or MCP/socket payloads; the transient CAF shape map remains
  unrelated and discarded.
- Scope: no joint inference, release metadata, project schema, or plan changes.

---

## Fix round 2/5 — FIFO control dispatch and shutdown-safe pending leases

### Review findings addressed

1. `ControlServer` now serializes complete request lines through one explicit
   FIFO. While a dispatch is active, nested `readyRead` callbacks may read and
   enqueue later commands but cannot dispatch them. The server writes the
   current response before draining the next request, preserving documented
   pipelining and controller state order.
2. After writing a response, the drain polls `bytesAvailable()` on live
   connections. This is necessary because Qt may coalesce a `readyRead` emitted
   during the nested event loop; without the poll, bytes can remain unread after
   the original dispatch returns.
3. Generated CAD directories now act as pending job leases during shutdown.
   `CadAssetOwner.close()` marks the owner closing and retires active assets, but
   leaves pending directories in place. When the worker returns, controller
   adoption is rejected, the pending lease is discarded, and final owner-root
   cleanup completes. New allocations/replacements are rejected after shutdown.
4. The behavior is covered through both the blocking MCP adapter path and the
   GUI/window-close path. In both tests the worker writes successfully after
   shutdown was requested, proving no use-after-delete.

### Files changed in this fix round

- `src/cad2urdf/gui/control/server.py`
- `src/cad2urdf/gui/state/cad_assets.py`
- `src/cad2urdf/gui/state/controller.py`
- `tests/gui/control/test_server.py`
- `tests/gui/control/test_dispatcher_build.py`
- `tests/gui/test_import_dialog.py`

### Strict TDD evidence

Primary RED:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
  VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
  .venv/bin/pytest \
  tests/gui/control/test_server.py::test_pipelined_command_waits_for_blocked_import_and_preserves_response_order \
  tests/gui/control/test_dispatcher_build.py::test_dispatcher_shutdown_defers_pending_cleanup_until_worker_stops \
  tests/gui/test_import_dialog.py::test_window_close_defers_pending_cleanup_until_gui_worker_stops \
  -q -ra
```

Result before production changes: **3 failed**.

- the real socket client did not receive the ordered two-response sequence
- dispatcher shutdown deleted the pending tree and the worker failed writing
  `finished.stl` with `FileNotFoundError`
- window close caused the same use-after-delete failure in the GUI path

First queue iteration: **2 passed, 1 failed**. Shutdown coordination was fixed,
but the socket test received only the import response. This proved that merely
gating nested dispatch can strand coalesced unread `QLocalSocket` bytes.

Final GREEN after adding the post-response `bytesAvailable()` collection: the
same command completed with **3 passed**.

### Focused and full verification

Focused regression gate:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
  VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
  .venv/bin/pytest \
  tests/gui/control/test_server.py \
  tests/gui/control/test_dispatcher_build.py \
  tests/gui/control/test_dispatcher_project.py \
  tests/gui/test_import_dialog.py \
  tests/gui/test_cad_asset_owner.py \
  tests/gui/test_main_window.py \
  tests/gui/test_worker_base.py \
  tests/unit/test_import_cad_wiring.py -q -ra
```

Result: **37 passed**.

Full suite:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
  VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
  .venv/bin/pytest -q -ra
```

Result: **339 passed, 1 skipped**. The skip remains the optional ManipulaPy
dependency.

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH .venv/bin/ruff check .
```

Result: `All checks passed!`

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  .venv/bin/mypy src/cad2urdf/core src/cad2urdf/gui src/cad2urdf/mcp
```

Result: `Success: no issues found in 65 source files`

### Fix-round self-review

- Real boundary test: the pipelining test uses a real Unix socket client and
  `QLocalServer`, sends `rename_link` only after the import worker has blocked,
  and asserts both wire response order and final controller state.
- No stranded input: complete lines already read are FIFO queued; data that
  arrived during nested dispatch is explicitly collected after the current
  response, before the next dispatch.
- Disconnect safety: queued requests for connections no longer present in the
  live buffer map are skipped, and `stop()` clears both buffers and the queue.
- Shutdown safety: active assets can retire immediately, but pending assets
  remain until their worker returns. Both GUI and MCP tests perform a real file
  write after close/shutdown, then assert final cleanup.
- Ownership safety is unchanged: only directories allocated and tracked by
  `CadAssetOwner` can enter the pending/active lifecycle.
- Protocol surface, synchronous response envelopes, Task 2 parser/CLI behavior,
  and Task 4/release scope remain unchanged.
