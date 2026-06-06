# GUI MCP Server Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let an MCP client (Claude) launch the real cad2urdf PyQt6/VTK GUI and drive it end-to-end — import meshes, edit the robot, validate, export a ROS package, and screenshot the live window — through a semantic command bridge.

**Architecture:** Two processes. A plain-Python **MCP server** (`cad2urdf-mcp`) spawns the GUI as a child and speaks newline-delimited JSON over a Unix-domain socket. Inside the GUI, a `ControlServer` (`QLocalServer`, on the Qt main thread) hands each command to a `CommandDispatcher` that mutates the existing `RobotController` via the existing pure `core/kinematic/tree.py` transforms — so every change renders live and gets undo/redo for free.

**Tech Stack:** Python 3.10, PyQt6 (`QLocalServer`/`QLocalSocket`), the `mcp` SDK (FastMCP), stdlib `socket`/`subprocess`, pytest + pytest-qt.

**Spec:** [[2026-06-06-gui-mcp-server-design]]

---

## Conventions for every task

- Always run tools with ROS leakage stripped: prefix test/type commands with
  `unset PYTHONPATH AMENT_PREFIX_PATH &&`.
- GUI/Qt tests must run offscreen: `QT_QPA_PLATFORM=offscreen`.
- Commit messages end with the trailer:
  `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`
- Work happens on branch `feat/gui-mcp-server` (already created).

## File structure

| File | Responsibility | Qt? |
|---|---|---|
| `src/cad2urdf/core/project/save.py` (modify) | Extract public `robot_to_payload(robot) -> dict` (reused by `get_robot`). | No |
| `src/cad2urdf/core/kinematic/tree.py` (modify) | Add `rename_link` and `set_base_link` pure transforms. | No |
| `src/cad2urdf/gui/control/__init__.py` (create) | Package marker. | No |
| `src/cad2urdf/gui/control/protocol.py` (create) | Command-name registry + JSON-lines framing helpers. | No |
| `src/cad2urdf/gui/control/dispatcher.py` (create) | `CommandDispatcher`: command dict → controller/core → result dict. | Touches controller, no socket |
| `src/cad2urdf/gui/control/server.py` (create) | `ControlServer`: `QLocalServer` wrapping the dispatcher. | Yes |
| `src/cad2urdf/gui/app.py` (modify) | Parse `--control-socket`, start `ControlServer`. | Yes |
| `src/cad2urdf/mcp/__init__.py` (create) | Package marker. | No |
| `src/cad2urdf/mcp/client.py` (create) | Stdlib-socket JSON-lines client. | No |
| `src/cad2urdf/mcp/gui_process.py` (create) | Spawn GUI child + wait-for-socket + terminate. | No |
| `src/cad2urdf/mcp/server.py` (create) | FastMCP server + tool definitions. | No |
| `src/cad2urdf/mcp/__main__.py` (create) | `cad2urdf-mcp` entry point. | No |
| `pyproject.toml` (modify) | Add `[mcp]` extra + console script. | — |

---

## Task 1: Public `robot_to_payload` in save.py

**Files:**
- Modify: `src/cad2urdf/core/project/save.py`
- Test: `tests/unit/test_project_save_payload.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_project_save_payload.py
from pathlib import Path

import numpy as np

from cad2urdf.core.kinematic.model import InertialOverride, Joint, Link, Robot
from cad2urdf.core.project.save import robot_to_payload


def _robot() -> Robot:
    link = Link(
        name="base",
        visual_mesh_path=Path("/tmp/base.stl"),
        collision_mesh_path=Path("/tmp/base.stl"),
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
    )
    return Robot(name="r", base_link="base", links={"base": link}, joints={})


def test_robot_to_payload_has_schema_and_links():
    payload = robot_to_payload(_robot())
    assert payload["schema_version"] == 1
    assert payload["name"] == "r"
    assert payload["base_link"] == "base"
    assert payload["links"][0]["name"] == "base"
    assert payload["joints"] == []
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/test_project_save_payload.py -v`
Expected: FAIL — `ImportError: cannot import name 'robot_to_payload'`.

- [ ] **Step 3: Add `robot_to_payload` and make `save_project` use it**

In `src/cad2urdf/core/project/save.py`, add this function directly above `save_project` (after `_joint_from_dict`):

```python
def robot_to_payload(robot: Robot) -> dict[str, Any]:
    """Serialize a Robot to a plain JSON-able dict at the current SCHEMA_VERSION."""
    return {
        "schema_version": SCHEMA_VERSION,
        "name": robot.name,
        "base_link": robot.base_link,
        "links": [_link_to_dict(robot.links[k]) for k in sorted(robot.links)],
        "joints": [_joint_to_dict(robot.joints[k]) for k in sorted(robot.joints)],
    }
```

Then replace the body of `save_project` so it reuses the new helper (DRY):

```python
def save_project(robot: Robot, path: Path) -> None:
    """Serialize ``robot`` to a .cad2urdf JSON file at ``path``."""
    payload = robot_to_payload(robot)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/test_project_save_payload.py tests/unit -k project -v`
Expected: PASS (new test plus existing save/load tests still green).

- [ ] **Step 5: Type-check + commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/core/project/save.py tests/unit/test_project_save_payload.py
git commit -m "refactor(project): extract public robot_to_payload helper

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: `rename_link` tree transform

**Files:**
- Modify: `src/cad2urdf/core/kinematic/tree.py`
- Test: `tests/unit/test_tree_rename_reroot.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_tree_rename_reroot.py
from pathlib import Path

import numpy as np
import pytest

from cad2urdf.core.kinematic.model import InertialOverride, Joint, Link, Robot
from cad2urdf.core.kinematic.tree import rename_link, set_base_link


def _link(name: str) -> Link:
    return Link(
        name=name,
        visual_mesh_path=Path(f"/tmp/{name}.stl"),
        collision_mesh_path=Path(f"/tmp/{name}.stl"),
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
    )


def _chain() -> Robot:
    # base -> a -> b, all fixed joints at identity origins.
    j1 = Joint(name="j1", type="fixed", parent="base", child="a", axis=np.array([1.0, 0, 0]))
    j2 = Joint(name="j2", type="fixed", parent="a", child="b", axis=np.array([1.0, 0, 0]))
    return Robot(
        name="r",
        base_link="base",
        links={"base": _link("base"), "a": _link("a"), "b": _link("b")},
        joints={"j1": j1, "j2": j2},
    )


def test_rename_link_updates_links_and_joints():
    out = rename_link(_chain(), "a", "shoulder")
    assert "shoulder" in out.links
    assert "a" not in out.links
    assert out.joints["j1"].child == "shoulder"
    assert out.joints["j2"].parent == "shoulder"


def test_rename_base_link_updates_base():
    out = rename_link(_chain(), "base", "root")
    assert out.base_link == "root"


def test_rename_link_rejects_existing_name():
    with pytest.raises(ValueError, match="already exists"):
        rename_link(_chain(), "a", "b")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/test_tree_rename_reroot.py -k rename -v`
Expected: FAIL — `ImportError: cannot import name 'rename_link'`.

- [ ] **Step 3: Implement `rename_link`**

Append to `src/cad2urdf/core/kinematic/tree.py`:

```python
def rename_link(robot: Robot, old: str, new: str) -> Robot:
    """Rename a link and fix up every joint and the base reference."""
    if old not in robot.links:
        raise ValueError(f"link {old!r} not in robot")
    if not new:
        raise ValueError("new link name must not be empty")
    if new in robot.links:
        raise ValueError(f"link {new!r} already exists")

    work = deepcopy(robot)
    link = work.links.pop(old)
    link.name = new
    work.links[new] = link

    new_joints: dict[str, Joint] = {}
    for jn, j in work.joints.items():
        new_joints[jn] = Joint(
            name=j.name,
            type=j.type,
            parent=new if j.parent == old else j.parent,
            child=new if j.child == old else j.child,
            axis=j.axis,
            origin=j.origin,
            limit_lower=j.limit_lower,
            limit_upper=j.limit_upper,
            effort=j.effort,
            velocity=j.velocity,
        )
    base = new if work.base_link == old else work.base_link
    return Robot(name=work.name, base_link=base, links=work.links, joints=new_joints)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/test_tree_rename_reroot.py -k rename -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/core/kinematic/tree.py tests/unit/test_tree_rename_reroot.py
git commit -m "feat(tree): rename_link transform

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: `set_base_link` (re-root) tree transform

**Files:**
- Modify: `src/cad2urdf/core/kinematic/tree.py`
- Test: `tests/unit/test_tree_rename_reroot.py` (add cases)

> **Caveat (document in the docstring):** re-rooting reverses each joint along the
> path from the new base to the old base, inverting the 4x4 origin. This is exact
> for `fixed` joints (the import default). For revolute/prismatic joints the axis
> vector is preserved, so the sign convention of motion may flip — re-root before
> assigning joint types/axes, which is the intended workflow.

- [ ] **Step 1: Write the failing test (append to the same file)**

```python
def test_set_base_link_reroots_chain():
    # base -> a -> b ; make b the new base.
    out = set_base_link(_chain(), "b")
    assert out.base_link == "b"
    # j2 used to be a->b; now b->a.
    assert out.joints["j2"].parent == "b"
    assert out.joints["j2"].child == "a"
    # j1 used to be base->a; now a->base.
    assert out.joints["j1"].parent == "a"
    assert out.joints["j1"].child == "base"


def test_set_base_link_noop_when_same():
    out = set_base_link(_chain(), "base")
    assert out.base_link == "base"
    assert out.joints["j1"].parent == "base"


def test_set_base_link_rejects_unknown():
    with pytest.raises(ValueError, match="not in robot"):
        set_base_link(_chain(), "ghost")


def test_set_base_link_inverts_origin():
    r = _chain()
    moved = r.joints["j2"].origin.copy()
    moved[:3, 3] = [1.0, 2.0, 3.0]
    r.joints["j2"].origin = moved
    out = set_base_link(r, "b")
    # reversed edge origin is the matrix inverse: translation negates for pure translation.
    assert np.allclose(out.joints["j2"].origin[:3, 3], [-1.0, -2.0, -3.0])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/test_tree_rename_reroot.py -k set_base -v`
Expected: FAIL — `ImportError: cannot import name 'set_base_link'`.

- [ ] **Step 3: Implement `set_base_link`**

Append to `src/cad2urdf/core/kinematic/tree.py` (it already imports `numpy`? No — add `import numpy as np` to the imports at the top if absent; check the existing import block and only add if missing):

```python
def set_base_link(robot: Robot, new_base: str) -> Robot:
    """Re-root the tree at ``new_base`` by reversing joints along the path to the old base.

    Exact for fixed joints; for actuated joints the axis vector is preserved
    (motion sign may flip). Re-root before assigning joint types/axes.
    """
    if new_base not in robot.links:
        raise ValueError(f"link {new_base!r} not in robot")
    if new_base == robot.base_link:
        return deepcopy(robot)

    # Path from new_base up to the current base via parent links.
    path: list[str] = []
    cur: str | None = new_base
    while cur is not None:
        path.append(cur)
        cur = parent_of(robot, cur)
    if path[-1] != robot.base_link:
        raise ValueError(f"{new_base!r} is not connected to base {robot.base_link!r}")

    work = deepcopy(robot)
    for i in range(len(path) - 1):
        child, parent = path[i], path[i + 1]
        jname = next(
            jn for jn, j in work.joints.items() if j.parent == parent and j.child == child
        )
        j = work.joints[jname]
        work.joints[jname] = Joint(
            name=j.name,
            type=j.type,
            parent=child,
            child=parent,
            axis=j.axis,
            origin=np.linalg.inv(j.origin),
            limit_lower=j.limit_lower,
            limit_upper=j.limit_upper,
            effort=j.effort,
            velocity=j.velocity,
        )
    return Robot(name=work.name, base_link=new_base, links=work.links, joints=work.joints)
```

If `numpy` is not yet imported in `tree.py`, add at the top with the other imports:

```python
import numpy as np
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/test_tree_rename_reroot.py -v`
Expected: PASS (all rename + set_base tests).

- [ ] **Step 5: Commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/core/kinematic/tree.py tests/unit/test_tree_rename_reroot.py
git commit -m "feat(tree): set_base_link re-root transform

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Control protocol (command registry + framing)

**Files:**
- Create: `src/cad2urdf/gui/control/__init__.py`
- Create: `src/cad2urdf/gui/control/protocol.py`
- Test: `tests/unit/control/test_protocol.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/control/test_protocol.py
import pytest

from cad2urdf.gui.control import protocol


def test_known_command_names_include_core_groups():
    for name in ("get_robot", "import_meshes", "update_joint", "export_package", "screenshot"):
        assert name in protocol.COMMANDS


def test_encode_decode_roundtrip():
    line = protocol.encode({"command": "get_robot", "args": {}})
    assert line.endswith("\n")
    assert protocol.decode(line) == {"command": "get_robot", "args": {}}


def test_decode_rejects_non_object():
    with pytest.raises(protocol.ProtocolError):
        protocol.decode("[1, 2, 3]\n")


def test_ok_and_err_envelopes():
    assert protocol.ok({"x": 1}) == {"ok": True, "result": {"x": 1}}
    env = protocol.err("bad_command", "nope")
    assert env == {"ok": False, "error": "bad_command", "detail": "nope"}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/control/test_protocol.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'cad2urdf.gui.control'`.

- [ ] **Step 3: Create the package + protocol**

Create empty `src/cad2urdf/gui/control/__init__.py`:

```python
"""Control bridge: in-GUI command server driven by the MCP process."""
```

Create `src/cad2urdf/gui/control/protocol.py`:

```python
"""Wire protocol shared by the GUI ControlServer and the MCP client.

Messages are newline-delimited JSON objects. A request is
``{"command": <name>, "args": {...}}``; a response is an ``ok``/``err`` envelope.
"""

from __future__ import annotations

import json
from typing import Any

# Frozen set of every command the dispatcher understands. The server rejects
# anything not listed here before dispatch.
COMMANDS: frozenset[str] = frozenset(
    {
        # introspection
        "get_robot",
        "list_materials",
        "get_history",
        "gui_status",
        # build
        "import_meshes",
        "set_base_link",
        "rename_link",
        "remove_link",
        "set_link_material",
        # joints
        "update_joint",
        # project / export
        "save_project",
        "open_project",
        "validate",
        "export_package",
        # control / visual
        "undo",
        "redo",
        "screenshot",
    }
)


class ProtocolError(ValueError):
    """Raised on malformed wire messages."""


def encode(obj: dict[str, Any]) -> str:
    """Serialize a message to a single newline-terminated JSON line."""
    return json.dumps(obj) + "\n"


def decode(line: str) -> dict[str, Any]:
    """Parse one JSON line into a dict, rejecting non-object payloads."""
    try:
        parsed = json.loads(line)
    except json.JSONDecodeError as e:
        raise ProtocolError(f"invalid JSON: {e}") from e
    if not isinstance(parsed, dict):
        raise ProtocolError(f"message must be a JSON object, got {type(parsed).__name__}")
    return parsed


def ok(result: Any) -> dict[str, Any]:
    """Build a success envelope."""
    return {"ok": True, "result": result}


def err(error: str, detail: str) -> dict[str, Any]:
    """Build an error envelope."""
    return {"ok": False, "error": error, "detail": detail}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/control/test_protocol.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/gui/control/__init__.py src/cad2urdf/gui/control/protocol.py tests/unit/control/__init__.py tests/unit/control/test_protocol.py 2>/dev/null; git add tests/unit/control
git commit -m "feat(control): wire protocol — command registry + JSON-lines framing

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

(If pytest complains about test package discovery, create empty `tests/unit/control/__init__.py`.)

---

## Task 5: Dispatcher — introspection commands

**Files:**
- Create: `src/cad2urdf/gui/control/dispatcher.py`
- Test: `tests/gui/control/test_dispatcher_introspect.py`

> `RobotController` is a `QObject`; instantiating it is fine headless but tests use
> the pytest-qt `qapp` fixture to guarantee a `QApplication` exists.

- [ ] **Step 1: Write the failing test**

```python
# tests/gui/control/test_dispatcher_introspect.py
import pytest

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.state.controller import RobotController


@pytest.fixture()
def dispatcher(qapp):
    return CommandDispatcher(RobotController())


def test_unknown_command_returns_error(dispatcher):
    resp = dispatcher.dispatch({"command": "nope", "args": {}})
    assert resp["ok"] is False
    assert resp["error"] == "unknown_command"


def test_get_robot_returns_payload(dispatcher):
    resp = dispatcher.dispatch({"command": "get_robot", "args": {}})
    assert resp["ok"] is True
    assert resp["result"]["schema_version"] == 1
    assert "links" in resp["result"]


def test_list_materials_includes_aluminum(dispatcher):
    resp = dispatcher.dispatch({"command": "list_materials", "args": {}})
    assert resp["ok"] is True
    assert "aluminum_6061" in resp["result"]


def test_get_history_reports_flags(dispatcher):
    resp = dispatcher.dispatch({"command": "get_history", "args": {}})
    assert resp["result"] == {"can_undo": False, "can_redo": False}


def test_gui_status_ok(dispatcher):
    resp = dispatcher.dispatch({"command": "gui_status", "args": {}})
    assert resp["result"]["running"] is True
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/control/test_dispatcher_introspect.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'cad2urdf.gui.control.dispatcher'`.

- [ ] **Step 3: Create the dispatcher with introspection commands**

Create `src/cad2urdf/gui/control/dispatcher.py`:

```python
"""CommandDispatcher: turns a wire command dict into controller/core calls.

Pure of any socket code so it can be unit-tested headlessly. Each command is a
method named ``_cmd_<name>`` taking the ``args`` dict and returning a JSON-able
result; the public ``dispatch`` wraps them in ok/err envelopes and never raises.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any

from cad2urdf.core.inertia.materials import list_materials
from cad2urdf.core.project.save import robot_to_payload
from cad2urdf.gui.control import protocol
from cad2urdf.gui.state.controller import RobotController

_log = logging.getLogger(__name__)


class CommandDispatcher:
    """Dispatch wire commands onto a RobotController + core functions."""

    def __init__(
        self,
        controller: RobotController,
        *,
        grab_png: Callable[[], bytes] | None = None,
    ) -> None:
        self._controller = controller
        # Injected so the dispatcher stays headlessly testable; the real server
        # supplies a window-grab callback.
        self._grab_png = grab_png

    def dispatch(self, message: dict[str, Any]) -> dict[str, Any]:
        """Execute one command message, returning an ok/err envelope."""
        command = message.get("command")
        if command not in protocol.COMMANDS:
            return protocol.err("unknown_command", f"no such command: {command!r}")
        args = message.get("args") or {}
        handler = getattr(self, f"_cmd_{command}", None)
        if handler is None:  # pragma: no cover - registry/handler drift guard
            return protocol.err("not_implemented", f"command {command!r} has no handler")
        try:
            return protocol.ok(handler(args))
        except (ValueError, KeyError, FileNotFoundError, TypeError) as e:
            return protocol.err(type(e).__name__, str(e))
        except Exception as e:  # noqa: BLE001 - never let a command crash the GUI
            _log.exception("command %r failed", command)
            return protocol.err("internal_error", str(e))

    # ---- introspection ------------------------------------------------------
    def _cmd_get_robot(self, args: dict[str, Any]) -> dict[str, Any]:
        return robot_to_payload(self._controller.current())

    def _cmd_list_materials(self, args: dict[str, Any]) -> list[str]:
        return list_materials()

    def _cmd_get_history(self, args: dict[str, Any]) -> dict[str, bool]:
        return {
            "can_undo": self._controller.can_undo(),
            "can_redo": self._controller.can_redo(),
        }

    def _cmd_gui_status(self, args: dict[str, Any]) -> dict[str, Any]:
        robot = self._controller.current()
        return {"running": True, "robot_name": robot.name, "link_count": len(robot.links)}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/control/test_dispatcher_introspect.py -v`
Expected: PASS (5 tests). Create empty `tests/gui/control/__init__.py` if discovery complains.

- [ ] **Step 5: Commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/gui/control/dispatcher.py tests/gui/control
git commit -m "feat(control): dispatcher with introspection commands

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Dispatcher — build commands

**Files:**
- Modify: `src/cad2urdf/gui/control/dispatcher.py`
- Test: `tests/gui/control/test_dispatcher_build.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/gui/control/test_dispatcher_build.py
from pathlib import Path

import pytest
import trimesh

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.state.controller import RobotController


@pytest.fixture()
def two_meshes(tmp_path):
    a = tmp_path / "base.stl"
    b = tmp_path / "arm.stl"
    trimesh.creation.box(extents=(1, 1, 1)).export(a)
    trimesh.creation.box(extents=(1, 1, 1)).export(b)
    return [str(a), str(b)]


@pytest.fixture()
def dispatcher(qapp):
    return CommandDispatcher(RobotController())


def _imported(dispatcher, two_meshes):
    return dispatcher.dispatch(
        {"command": "import_meshes", "args": {"paths": two_meshes, "robot_name": "rig"}}
    )


def test_import_meshes_builds_links(dispatcher, two_meshes):
    resp = _imported(dispatcher, two_meshes)
    assert resp["ok"] is True
    names = {link["name"] for link in resp["result"]["links"]}
    assert names == {"base", "arm"}


def test_rename_link(dispatcher, two_meshes):
    _imported(dispatcher, two_meshes)
    resp = dispatcher.dispatch(
        {"command": "rename_link", "args": {"old": "arm", "new": "shoulder"}}
    )
    names = {link["name"] for link in resp["result"]["links"]}
    assert "shoulder" in names and "arm" not in names


def test_set_link_material_changes_density(dispatcher, two_meshes):
    _imported(dispatcher, two_meshes)
    resp = dispatcher.dispatch(
        {"command": "set_link_material", "args": {"link": "arm", "material": "steel_1018"}}
    )
    arm = next(link for link in resp["result"]["links"] if link["name"] == "arm")
    assert arm["material_name"] == "steel_1018"
    assert arm["material_density"] == pytest.approx(7850.0)


def test_remove_link(dispatcher, two_meshes):
    _imported(dispatcher, two_meshes)
    resp = dispatcher.dispatch({"command": "remove_link", "args": {"link": "arm"}})
    names = {link["name"] for link in resp["result"]["links"]}
    assert names == {"base"}


def test_set_base_link(dispatcher, two_meshes):
    _imported(dispatcher, two_meshes)
    resp = dispatcher.dispatch({"command": "set_base_link", "args": {"link": "arm"}})
    assert resp["result"]["base_link"] == "arm"


def test_import_rejects_missing_args(dispatcher):
    resp = dispatcher.dispatch({"command": "import_meshes", "args": {}})
    assert resp["ok"] is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/control/test_dispatcher_build.py -v`
Expected: FAIL — build commands have no handlers (`unknown`/`not_implemented` envelopes → assertion failures).

- [ ] **Step 3: Add build command handlers**

In `src/cad2urdf/gui/control/dispatcher.py`, extend the imports near the top:

```python
from pathlib import Path

from cad2urdf.core.inertia.materials import list_materials, lookup
from cad2urdf.core.kinematic.model import Link, Robot
from cad2urdf.core.kinematic.tree import remove_link, rename_link, set_base_link
from cad2urdf.gui.workers.import_meshes import build_import_job
```

(Keep the existing `robot_to_payload` / `protocol` / `RobotController` imports; merge the `list_materials` import line.)

Add a small helper and the handlers as methods of `CommandDispatcher`:

```python
    # ---- build --------------------------------------------------------------
    def _cmd_import_meshes(self, args: dict[str, Any]) -> dict[str, Any]:
        paths = [Path(p) for p in args["paths"]]
        robot_name = args.get("robot_name", "robot")
        job = build_import_job(paths=paths, robot_name=robot_name)
        robot = job(lambda c, t, m: None)  # run synchronously on the Qt thread
        self._controller.replace(robot)
        return robot_to_payload(self._controller.current())

    def _cmd_rename_link(self, args: dict[str, Any]) -> dict[str, Any]:
        old, new = args["old"], args["new"]
        self._controller.apply(lambda r: rename_link(r, old, new), label=f"rename {old}")
        return robot_to_payload(self._controller.current())

    def _cmd_remove_link(self, args: dict[str, Any]) -> dict[str, Any]:
        name = args["link"]
        self._controller.apply(lambda r: remove_link(r, name), label=f"remove {name}")
        return robot_to_payload(self._controller.current())

    def _cmd_set_base_link(self, args: dict[str, Any]) -> dict[str, Any]:
        name = args["link"]
        self._controller.apply(lambda r: set_base_link(r, name), label=f"base {name}")
        return robot_to_payload(self._controller.current())

    def _cmd_set_link_material(self, args: dict[str, Any]) -> dict[str, Any]:
        link_name, material_name = args["link"], args["material"]
        material = lookup(material_name)  # raises KeyError on unknown material

        def transform(robot: Robot) -> Robot:
            old = robot.links[link_name]
            new_links = dict(robot.links)
            new_links[link_name] = Link(
                name=old.name,
                visual_mesh_path=old.visual_mesh_path,
                collision_mesh_path=old.collision_mesh_path,
                material_density=material.density_kg_m3,
                material_name=material.name,
                inertial_override=old.inertial_override,
                origin=old.origin,
            )
            return Robot(
                name=robot.name,
                base_link=robot.base_link,
                links=new_links,
                joints=dict(robot.joints),
            )

        self._controller.apply(transform, label=f"material {link_name}")
        return robot_to_payload(self._controller.current())
```

> Note: `lookup` raises `KeyError` for unknown materials, already caught by
> `dispatch` and turned into an error envelope.

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/control/test_dispatcher_build.py -v`
Expected: PASS (6 tests).

- [ ] **Step 5: Commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/gui/control/dispatcher.py tests/gui/control/test_dispatcher_build.py
git commit -m "feat(control): dispatcher build commands (import/rename/remove/base/material)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: Dispatcher — `update_joint`

**Files:**
- Modify: `src/cad2urdf/gui/control/dispatcher.py`
- Test: `tests/gui/control/test_dispatcher_joint.py`

> A `Robot` is a tree: each non-base link has exactly one incoming joint. So
> joints are *created* by `import_meshes`, *removed* by `remove_link`, and
> *edited/restructured* by `update_joint`. `update_joint` takes a joint name plus
> any subset of: `type`, `axis` (3 floats), `origin_xyz`+`origin_rpy` (both or
> neither), `limit_lower/upper`, `effort`, `velocity`, `parent` (reparent).

- [ ] **Step 1: Write the failing test**

```python
# tests/gui/control/test_dispatcher_joint.py
from pathlib import Path

import numpy as np
import pytest
import trimesh

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.state.controller import RobotController


@pytest.fixture()
def dispatcher(qapp, tmp_path):
    d = CommandDispatcher(RobotController())
    paths = []
    for name in ("base", "a", "b"):
        p = tmp_path / f"{name}.stl"
        trimesh.creation.box(extents=(1, 1, 1)).export(p)
        paths.append(str(p))
    d.dispatch({"command": "import_meshes", "args": {"paths": paths, "robot_name": "rig"}})
    return d


def _joint(resp, name):
    return next(j for j in resp["result"]["joints"] if j["name"] == name)


def test_update_joint_type_and_axis(dispatcher):
    # import wires a_to_base + b_to_base as fixed.
    resp = dispatcher.dispatch(
        {
            "command": "update_joint",
            "args": {
                "joint": "a_to_base",
                "type": "revolute",
                "axis": [0.0, 0.0, 2.0],
                "limit_lower": -1.0,
                "limit_upper": 1.0,
                "effort": 10.0,
                "velocity": 1.0,
            },
        }
    )
    j = _joint(resp, "a_to_base")
    assert j["type"] == "revolute"
    assert j["axis"] == pytest.approx([0.0, 0.0, 1.0])  # normalized
    assert j["limit_lower"] == -1.0


def test_update_joint_origin(dispatcher):
    resp = dispatcher.dispatch(
        {
            "command": "update_joint",
            "args": {
                "joint": "a_to_base",
                "origin_xyz": [1.0, 2.0, 3.0],
                "origin_rpy": [0.0, 0.0, 0.0],
            },
        }
    )
    j = _joint(resp, "a_to_base")
    assert np.allclose(np.array(j["origin"])[:3, 3], [1.0, 2.0, 3.0])


def test_update_joint_reparent_builds_chain(dispatcher):
    # Move b under a so the tree becomes base -> a -> b.
    resp = dispatcher.dispatch(
        {"command": "update_joint", "args": {"joint": "b_to_base", "parent": "a"}}
    )
    j = _joint(resp, "b_to_base")
    assert j["parent"] == "a"


def test_update_joint_origin_requires_both(dispatcher):
    resp = dispatcher.dispatch(
        {"command": "update_joint", "args": {"joint": "a_to_base", "origin_xyz": [1, 0, 0]}}
    )
    assert resp["ok"] is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/control/test_dispatcher_joint.py -v`
Expected: FAIL — no `update_joint` handler.

- [ ] **Step 3: Add `update_joint`**

In `src/cad2urdf/gui/control/dispatcher.py`, extend imports:

```python
import numpy as np

from cad2urdf.core.config.loader import origin_from_xyz_rpy
from cad2urdf.core.kinematic.model import Joint, Link, Robot
from cad2urdf.core.kinematic.tree import remove_link, rename_link, reparent_joint, set_base_link
```

(Merge with the existing `model`/`tree` import lines — add `Joint`, `reparent_joint`.)

Add the handler method:

```python
    # ---- joints -------------------------------------------------------------
    def _cmd_update_joint(self, args: dict[str, Any]) -> dict[str, Any]:
        joint_name = args["joint"]
        new_parent = args.get("parent")
        jtype = args.get("type")
        axis_in = args.get("axis")
        xyz = args.get("origin_xyz")
        rpy = args.get("origin_rpy")
        if (xyz is None) != (rpy is None):
            raise ValueError("origin_xyz and origin_rpy must be provided together")

        def transform(robot: Robot) -> Robot:
            work = reparent_joint(robot, joint_name, new_parent) if new_parent else robot
            old = work.joints[joint_name]
            if axis_in is None:
                axis = old.axis
            else:
                vec = np.asarray(axis_in, dtype=float)
                norm = float(np.linalg.norm(vec))
                if norm == 0.0:
                    raise ValueError("axis must be non-zero")
                axis = vec / norm
            origin = old.origin if xyz is None else origin_from_xyz_rpy(list(xyz), list(rpy))
            new_joint = Joint(
                name=old.name,
                type=jtype or old.type,
                parent=old.parent,
                child=old.child,
                axis=axis,
                origin=origin,
                limit_lower=args.get("limit_lower", old.limit_lower),
                limit_upper=args.get("limit_upper", old.limit_upper),
                effort=args.get("effort", old.effort),
                velocity=args.get("velocity", old.velocity),
            )
            new_joints = dict(work.joints)
            new_joints[joint_name] = new_joint
            return Robot(
                name=work.name,
                base_link=work.base_link,
                links=dict(work.links),
                joints=new_joints,
            )

        self._controller.apply(transform, label=f"edit joint {joint_name}")
        return robot_to_payload(self._controller.current())
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/control/test_dispatcher_joint.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/gui/control/dispatcher.py tests/gui/control/test_dispatcher_joint.py
git commit -m "feat(control): dispatcher update_joint (type/axis/origin/limits/reparent)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 8: Dispatcher — project I/O, validate, export

**Files:**
- Modify: `src/cad2urdf/gui/control/dispatcher.py`
- Test: `tests/gui/control/test_dispatcher_project.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/gui/control/test_dispatcher_project.py
from pathlib import Path

import pytest
import trimesh

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.state.controller import RobotController


@pytest.fixture()
def dispatcher(qapp, tmp_path):
    d = CommandDispatcher(RobotController())
    paths = []
    for name in ("base", "arm"):
        p = tmp_path / f"{name}.stl"
        trimesh.creation.box(extents=(1, 1, 1)).export(p)
        paths.append(str(p))
    d.dispatch({"command": "import_meshes", "args": {"paths": paths, "robot_name": "rig"}})
    return d


def test_save_then_open_roundtrip(dispatcher, tmp_path):
    proj = tmp_path / "rig.cad2urdf"
    save = dispatcher.dispatch({"command": "save_project", "args": {"path": str(proj)}})
    assert save["ok"] is True
    assert proj.is_file()

    fresh = CommandDispatcher(RobotController())
    opened = fresh.dispatch({"command": "open_project", "args": {"path": str(proj)}})
    assert opened["result"]["name"] == "rig"


def test_validate_writes_urdf(dispatcher, tmp_path):
    out = tmp_path / "val"
    resp = dispatcher.dispatch(
        {
            "command": "validate",
            "args": {"out_dir": str(out), "package_name": "rig_desc", "run_manipulapy": False},
        }
    )
    assert resp["ok"] is True
    assert resp["result"]["urdf_written"] is True


def test_export_package_writes_files(dispatcher, tmp_path):
    out = tmp_path / "pkg"
    resp = dispatcher.dispatch(
        {
            "command": "export_package",
            "args": {
                "out_dir": str(out),
                "package_name": "rig_desc",
                "run_manipulapy": False,
            },
        }
    )
    assert resp["ok"] is True
    assert (out / "package.xml").is_file()
    assert (out / "urdf" / "rig.urdf").is_file()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/control/test_dispatcher_project.py -v`
Expected: FAIL — no project/validate/export handlers.

- [ ] **Step 3: Add the handlers**

In `src/cad2urdf/gui/control/dispatcher.py`, extend imports:

```python
from cad2urdf.core.project.save import load_project, robot_to_payload, save_project
from cad2urdf.gui.workers.export_package import build_export_job
from cad2urdf.gui.workers.validate import build_validate_job
```

(Merge `load_project`/`save_project` into the existing `save` import line.)

Add the handlers:

```python
    # ---- project / export ---------------------------------------------------
    def _cmd_save_project(self, args: dict[str, Any]) -> dict[str, Any]:
        save_project(self._controller.current(), Path(args["path"]))
        return {"saved": args["path"]}

    def _cmd_open_project(self, args: dict[str, Any]) -> dict[str, Any]:
        robot = load_project(Path(args["path"]))
        self._controller.replace(robot)
        return robot_to_payload(self._controller.current())

    def _cmd_validate(self, args: dict[str, Any]) -> dict[str, Any]:
        robot = self._controller.current()
        job = build_validate_job(
            robot=robot,
            out_dir=Path(args["out_dir"]),
            package_name=args["package_name"],
            urdf_relname=f"{robot.name}.urdf",
            run_manipulapy=bool(args.get("run_manipulapy", False)),
        )
        report = job(lambda c, t, m: None)
        return {
            "ast_issues": list(report.ast_issues),
            "urdf_written": report.urdf_written,
            "urdf_path": str(report.urdf_path),
            "manipulapy_ok": report.manipulapy_ok,
            "manipulapy_error": report.manipulapy_error,
        }

    def _cmd_export_package(self, args: dict[str, Any]) -> dict[str, Any]:
        job = build_export_job(
            robot=self._controller.current(),
            out_dir=Path(args["out_dir"]),
            package_name=args["package_name"],
            maintainer=args.get("maintainer", "cad2urdf-user"),
            maintainer_email=args.get("maintainer_email", "user@example.com"),
            run_manipulapy=bool(args.get("run_manipulapy", False)),
        )
        report = job(lambda c, t, m: None)
        return {
            "urdf_path": str(report.urdf_path),
            "manipulapy_ok": report.manipulapy_ok,
            "manipulapy_error": report.manipulapy_error,
        }
```

> Verify field names against `ValidateReport` (`ast_issues`, `urdf_written`,
> `urdf_path`, `manipulapy_ok`, `manipulapy_error`) and `ExportReport`
> (`urdf_path`, `manipulapy_ok`, `manipulapy_error`) in
> `src/cad2urdf/gui/workers/validate.py` and `export_package.py` before running.

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/control/test_dispatcher_project.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/gui/control/dispatcher.py tests/gui/control/test_dispatcher_project.py
git commit -m "feat(control): dispatcher save/open/validate/export commands

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 9: Dispatcher — undo/redo + screenshot

**Files:**
- Modify: `src/cad2urdf/gui/control/dispatcher.py`
- Test: `tests/gui/control/test_dispatcher_control.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/gui/control/test_dispatcher_control.py
import base64
from pathlib import Path

import pytest
import trimesh

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.state.controller import RobotController


@pytest.fixture()
def dispatcher(qapp, tmp_path):
    grab = lambda: b"PNGBYTES"  # noqa: E731 - injected fake window grabber
    d = CommandDispatcher(RobotController(), grab_png=grab)
    paths = []
    for name in ("base", "arm"):
        p = tmp_path / f"{name}.stl"
        trimesh.creation.box(extents=(1, 1, 1)).export(p)
        paths.append(str(p))
    d.dispatch({"command": "import_meshes", "args": {"paths": paths, "robot_name": "rig"}})
    return d


def test_undo_then_redo(dispatcher):
    dispatcher.dispatch({"command": "rename_link", "args": {"old": "arm", "new": "x"}})
    undo = dispatcher.dispatch({"command": "undo", "args": {}})
    names = {link["name"] for link in undo["result"]["links"]}
    assert "arm" in names
    redo = dispatcher.dispatch({"command": "redo", "args": {}})
    names = {link["name"] for link in redo["result"]["links"]}
    assert "x" in names


def test_screenshot_returns_base64_png(dispatcher):
    resp = dispatcher.dispatch({"command": "screenshot", "args": {}})
    assert resp["ok"] is True
    assert base64.b64decode(resp["result"]["png_base64"]) == b"PNGBYTES"


def test_screenshot_without_grabber_errors(qapp):
    d = CommandDispatcher(RobotController())  # no grab_png injected
    resp = d.dispatch({"command": "screenshot", "args": {}})
    assert resp["ok"] is False
    assert resp["error"] == "no_window"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/control/test_dispatcher_control.py -v`
Expected: FAIL — no undo/redo/screenshot handlers.

- [ ] **Step 3: Add the handlers**

In `src/cad2urdf/gui/control/dispatcher.py`, add `import base64` at the top, then add:

```python
    # ---- control / visual ---------------------------------------------------
    def _cmd_undo(self, args: dict[str, Any]) -> dict[str, Any]:
        self._controller.undo()
        return robot_to_payload(self._controller.current())

    def _cmd_redo(self, args: dict[str, Any]) -> dict[str, Any]:
        self._controller.redo()
        return robot_to_payload(self._controller.current())

    def _cmd_screenshot(self, args: dict[str, Any]) -> dict[str, str]:
        if self._grab_png is None:
            raise _NoWindow("no live window to screenshot")
        png = self._grab_png()
        return {"png_base64": base64.b64encode(png).decode("ascii")}
```

Add a private exception class near the top of the module (after the logger) and
catch it in `dispatch`:

```python
class _NoWindow(RuntimeError):
    """Raised when a screenshot is requested but no window grabber is wired."""
```

In `dispatch`, add a branch before the generic `except Exception` so it maps to a
clean error code:

```python
        try:
            return protocol.ok(handler(args))
        except _NoWindow as e:
            return protocol.err("no_window", str(e))
        except (ValueError, KeyError, FileNotFoundError, TypeError) as e:
            return protocol.err(type(e).__name__, str(e))
        except Exception as e:  # noqa: BLE001 - never let a command crash the GUI
            _log.exception("command %r failed", command)
            return protocol.err("internal_error", str(e))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/control/test_dispatcher_control.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/gui/control/dispatcher.py tests/gui/control/test_dispatcher_control.py
git commit -m "feat(control): dispatcher undo/redo + screenshot commands

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 10: ControlServer (QLocalServer)

**Files:**
- Create: `src/cad2urdf/gui/control/server.py`
- Test: `tests/gui/control/test_server.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/gui/control/test_server.py
import json

import pytest
from PyQt6.QtCore import QEventLoop, QTimer
from PyQt6.QtNetwork import QLocalSocket

from cad2urdf.gui.control.dispatcher import CommandDispatcher
from cad2urdf.gui.control.server import ControlServer
from cad2urdf.gui.state.controller import RobotController


def _request(socket_name: str, message: dict) -> dict:
    """Connect a QLocalSocket, send one line, return the parsed response line."""
    sock = QLocalSocket()
    sock.connectToServer(socket_name)
    assert sock.waitForConnected(2000), "could not connect to ControlServer"
    sock.write((json.dumps(message) + "\n").encode("utf-8"))
    assert sock.waitForBytesWritten(2000)
    assert sock.waitForReadyRead(2000), "no response from ControlServer"
    line = bytes(sock.readAll()).decode("utf-8").strip()
    sock.disconnectFromServer()
    return json.loads(line)


def test_server_roundtrips_get_robot(qapp, tmp_path):
    name = str(tmp_path / "ctl.sock")
    server = ControlServer(CommandDispatcher(RobotController()), name)
    server.start()
    try:
        resp = _request(name, {"command": "get_robot", "args": {}})
        assert resp["ok"] is True
        assert resp["result"]["schema_version"] == 1
    finally:
        server.stop()


def test_server_rejects_bad_json(qapp, tmp_path):
    name = str(tmp_path / "ctl2.sock")
    server = ControlServer(CommandDispatcher(RobotController()), name)
    server.start()
    try:
        sock = QLocalSocket()
        sock.connectToServer(name)
        assert sock.waitForConnected(2000)
        sock.write(b"{not json}\n")
        assert sock.waitForReadyRead(2000)
        resp = json.loads(bytes(sock.readAll()).decode("utf-8").strip())
        assert resp["ok"] is False
        assert resp["error"] == "protocol_error"
    finally:
        server.stop()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/control/test_server.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'cad2urdf.gui.control.server'`.

- [ ] **Step 3: Implement ControlServer**

Create `src/cad2urdf/gui/control/server.py`:

```python
"""ControlServer: a QLocalServer that feeds JSON-line commands to the dispatcher.

Runs entirely on the Qt main thread, so handlers may touch the controller and
widgets directly. One command per line; one response line per command. A client
may pipeline multiple commands; partial lines are buffered per connection.
"""

from __future__ import annotations

import logging

from PyQt6.QtCore import QObject
from PyQt6.QtNetwork import QLocalServer, QLocalSocket

from cad2urdf.gui.control import protocol
from cad2urdf.gui.control.dispatcher import CommandDispatcher

_log = logging.getLogger(__name__)


class ControlServer(QObject):
    """Listens on a local socket and dispatches framed JSON commands."""

    def __init__(
        self,
        dispatcher: CommandDispatcher,
        socket_name: str,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._dispatcher = dispatcher
        self._socket_name = socket_name
        self._server = QLocalServer(self)
        self._buffers: dict[QLocalSocket, bytes] = {}
        self._server.newConnection.connect(self._on_new_connection)

    def start(self) -> None:
        """Begin listening. Removes any stale socket file first."""
        QLocalServer.removeServer(self._socket_name)
        if not self._server.listen(self._socket_name):
            raise RuntimeError(
                f"ControlServer could not listen on {self._socket_name!r}: "
                f"{self._server.errorString()}"
            )
        _log.info("ControlServer listening on %s", self._socket_name)

    def stop(self) -> None:
        self._server.close()
        QLocalServer.removeServer(self._socket_name)

    # ---- connection handling ------------------------------------------------
    def _on_new_connection(self) -> None:
        conn = self._server.nextPendingConnection()
        if conn is None:
            return
        self._buffers[conn] = b""
        conn.readyRead.connect(lambda c=conn: self._on_ready_read(c))
        conn.disconnected.connect(lambda c=conn: self._on_disconnected(c))

    def _on_disconnected(self, conn: QLocalSocket) -> None:
        self._buffers.pop(conn, None)
        conn.deleteLater()

    def _on_ready_read(self, conn: QLocalSocket) -> None:
        self._buffers[conn] += bytes(conn.readAll())
        while b"\n" in self._buffers[conn]:
            raw, _, rest = self._buffers[conn].partition(b"\n")
            self._buffers[conn] = rest
            self._handle_line(conn, raw)

    def _handle_line(self, conn: QLocalSocket, raw: bytes) -> None:
        try:
            message = protocol.decode(raw.decode("utf-8"))
        except (protocol.ProtocolError, UnicodeDecodeError) as e:
            response = protocol.err("protocol_error", str(e))
        else:
            response = self._dispatcher.dispatch(message)
        conn.write(protocol.encode(response).encode("utf-8"))
        conn.flush()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/control/test_server.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/gui/control/server.py tests/gui/control/test_server.py
git commit -m "feat(control): QLocalServer ControlServer with JSON-lines framing

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 11: Wire `--control-socket` into the GUI entry point

**Files:**
- Modify: `src/cad2urdf/gui/app.py`
- Test: `tests/gui/test_app_control_flag.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/gui/test_app_control_flag.py
from cad2urdf.gui.app import parse_args


def test_parse_args_default_socket_is_none():
    ns = parse_args([])
    assert ns.control_socket is None


def test_parse_args_reads_control_socket():
    ns = parse_args(["--control-socket", "/tmp/x.sock"])
    assert ns.control_socket == "/tmp/x.sock"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/test_app_control_flag.py -v`
Expected: FAIL — `ImportError: cannot import name 'parse_args'`.

- [ ] **Step 3: Rewrite `app.py` to parse args and start the server**

Replace the contents of `src/cad2urdf/gui/app.py`:

```python
"""QApplication entrypoint. Exposed as the `cad2urdf-gui` console script."""

from __future__ import annotations

import argparse
import sys

from PyQt6.QtGui import QImage
from PyQt6.QtWidgets import QApplication


def parse_args(argv: list[str]) -> argparse.Namespace:
    """Parse cad2urdf-gui CLI args."""
    parser = argparse.ArgumentParser(prog="cad2urdf-gui")
    parser.add_argument(
        "--control-socket",
        default=None,
        help="If set, start an MCP control server on this local socket name.",
    )
    return parser.parse_args(argv)


def _grab_png(window: object) -> bytes:
    """Grab the window to PNG bytes (used by the screenshot command)."""
    from PyQt6.QtCore import QBuffer, QIODevice

    pixmap = window.grab()  # type: ignore[attr-defined]
    image: QImage = pixmap.toImage()
    buffer = QBuffer()
    buffer.open(QIODevice.OpenModeFlag.WriteOnly)
    image.save(buffer, "PNG")
    return bytes(buffer.data())


def main(argv: list[str] | None = None) -> int:
    raw = argv if argv is not None else sys.argv[1:]
    ns = parse_args(raw)

    app = QApplication(sys.argv[:1])
    try:
        from cad2urdf.gui.windows.main_window import MainWindow
    except ImportError as e:
        print(f"cad2urdf-gui: MainWindow not available ({e}).", file=sys.stderr)
        return 1

    win = MainWindow()
    win.show()

    server = None
    if ns.control_socket:
        from cad2urdf.gui.control.dispatcher import CommandDispatcher
        from cad2urdf.gui.control.server import ControlServer

        dispatcher = CommandDispatcher(win.controller, grab_png=lambda: _grab_png(win))
        server = ControlServer(dispatcher, ns.control_socket)
        server.start()

    try:
        return app.exec()
    finally:
        if server is not None:
            server.stop()


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui/test_app_control_flag.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Smoke-check the existing GUI tests still pass + commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest tests/gui -q
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/gui/app.py tests/gui/test_app_control_flag.py
git commit -m "feat(gui): --control-socket flag starts the MCP ControlServer

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 12: MCP socket client

**Files:**
- Create: `src/cad2urdf/mcp/__init__.py`
- Create: `src/cad2urdf/mcp/client.py`
- Test: `tests/unit/mcp/test_client.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/mcp/test_client.py
import json
import socket
import threading

import pytest

from cad2urdf.mcp.client import ControlClient


def _echo_ok_server(sock_path: str, stop: threading.Event) -> None:
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(1)
    srv.settimeout(2.0)
    conn, _ = srv.accept()
    data = b""
    while b"\n" not in data:
        data += conn.recv(4096)
    request = json.loads(data.decode().strip())
    conn.sendall((json.dumps({"ok": True, "result": {"echo": request}}) + "\n").encode())
    conn.close()
    srv.close()


def test_client_send_receives_response(tmp_path):
    sock_path = str(tmp_path / "echo.sock")
    stop = threading.Event()
    t = threading.Thread(target=_echo_ok_server, args=(sock_path, stop), daemon=True)
    t.start()

    client = ControlClient(sock_path)
    client.connect(timeout=2.0)
    resp = client.send("get_robot", {})
    client.close()

    assert resp["ok"] is True
    assert resp["result"]["echo"]["command"] == "get_robot"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/mcp/test_client.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'cad2urdf.mcp'`.

- [ ] **Step 3: Create the package + client**

Create `src/cad2urdf/mcp/__init__.py`:

```python
"""MCP server exposing the cad2urdf GUI to MCP clients."""
```

Create `src/cad2urdf/mcp/client.py`:

```python
"""Blocking Unix-domain-socket client speaking the control protocol."""

from __future__ import annotations

import json
import socket
import time
from typing import Any


class ControlClient:
    """One persistent connection to the GUI ControlServer."""

    def __init__(self, socket_path: str) -> None:
        self._socket_path = socket_path
        self._sock: socket.socket | None = None
        self._buffer = b""

    def connect(self, *, timeout: float = 10.0) -> None:
        """Connect, retrying until the server's socket is accepting."""
        deadline = time.monotonic() + timeout
        last_err: OSError | None = None
        while time.monotonic() < deadline:
            try:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.connect(self._socket_path)
                self._sock = sock
                return
            except OSError as e:  # socket not up yet
                last_err = e
                time.sleep(0.1)
        raise TimeoutError(f"could not connect to {self._socket_path!r}: {last_err}")

    def send(self, command: str, args: dict[str, Any]) -> dict[str, Any]:
        """Send one command and block for the single-line response."""
        if self._sock is None:
            raise RuntimeError("client not connected")
        payload = json.dumps({"command": command, "args": args}) + "\n"
        self._sock.sendall(payload.encode("utf-8"))
        return self._read_line()

    def _read_line(self) -> dict[str, Any]:
        while b"\n" not in self._buffer:
            assert self._sock is not None
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ConnectionError("control server closed the connection")
            self._buffer += chunk
        raw, _, rest = self._buffer.partition(b"\n")
        self._buffer = rest
        return json.loads(raw.decode("utf-8"))

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/mcp/test_client.py -v`
Expected: PASS (1 test). Add empty `tests/unit/mcp/__init__.py` if discovery complains.

- [ ] **Step 5: Commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/mcp/__init__.py src/cad2urdf/mcp/client.py tests/unit/mcp
git commit -m "feat(mcp): blocking control-socket client

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 13: GUI subprocess lifecycle

**Files:**
- Create: `src/cad2urdf/mcp/gui_process.py`
- Test: `tests/unit/mcp/test_gui_process.py`

- [ ] **Step 1: Write the failing test**

The test uses a tiny fake "GUI" that just opens the socket, so it runs without Qt.

```python
# tests/unit/mcp/test_gui_process.py
import socket
import sys
import textwrap

from cad2urdf.mcp.gui_process import GuiProcess


def test_gui_process_spawns_and_waits(tmp_path):
    sock_path = str(tmp_path / "fake.sock")
    fake = tmp_path / "fake_gui.py"
    # Fake GUI: bind the socket then sleep so the spawner sees it become connectable.
    fake.write_text(
        textwrap.dedent(
            f"""
            import socket, time
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.bind({sock_path!r})
            s.listen(1)
            time.sleep(30)
            """
        )
    )

    proc = GuiProcess(socket_path=sock_path, launch_cmd=[sys.executable, str(fake)])
    proc.start(timeout=5.0)
    try:
        assert proc.is_running()
        # The socket is connectable.
        c = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        c.connect(sock_path)
        c.close()
    finally:
        proc.stop()
    assert not proc.is_running()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/mcp/test_gui_process.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'cad2urdf.mcp.gui_process'`.

- [ ] **Step 3: Implement GuiProcess**

Create `src/cad2urdf/mcp/gui_process.py`:

```python
"""Spawn and supervise the GUI child process for the MCP server."""

from __future__ import annotations

import socket
import subprocess
import sys
import time


class GuiProcess:
    """Launches `cad2urdf-gui --control-socket <path>` and waits for the socket."""

    def __init__(
        self,
        socket_path: str,
        launch_cmd: list[str] | None = None,
    ) -> None:
        self._socket_path = socket_path
        # Default: run the GUI module so we don't depend on PATH for the script.
        self._launch_cmd = launch_cmd or [
            sys.executable,
            "-m",
            "cad2urdf.gui.app",
            "--control-socket",
            socket_path,
        ]
        self._proc: subprocess.Popen[bytes] | None = None

    def start(self, *, timeout: float = 30.0) -> None:
        """Spawn the child and block until its socket accepts connections."""
        self._proc = subprocess.Popen(self._launch_cmd)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._proc.poll() is not None:
                raise RuntimeError(
                    f"GUI process exited early with code {self._proc.returncode}"
                )
            if self._socket_connectable():
                return
            time.sleep(0.1)
        self.stop()
        raise TimeoutError(f"GUI socket {self._socket_path!r} did not come up in {timeout}s")

    def _socket_connectable(self) -> bool:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(self._socket_path)
            return True
        except OSError:
            return False
        finally:
            s.close()

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def stop(self) -> None:
        if self._proc is None:
            return
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()
        self._proc = None
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/mcp/test_gui_process.py -v`
Expected: PASS (1 test).

- [ ] **Step 5: Commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/mcp/gui_process.py tests/unit/mcp/test_gui_process.py
git commit -m "feat(mcp): GUI subprocess lifecycle (spawn + wait-for-socket + stop)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 14: MCP server + tools

**Files:**
- Create: `src/cad2urdf/mcp/server.py`
- Create: `src/cad2urdf/mcp/__main__.py`
- Test: `tests/unit/mcp/test_server.py`

> The MCP `mcp` SDK is an optional dependency (Task 15 adds the extra). Tests in
> this task mock the client/lifecycle, so the FastMCP import is the only real SDK
> dependency — skip the module if the SDK is absent.

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/mcp/test_server.py
import pytest

pytest.importorskip("mcp")  # requires the [mcp] extra

from cad2urdf.mcp import server as mcp_server


class _FakeClient:
    def __init__(self):
        self.sent = []

    def send(self, command, args):
        self.sent.append((command, args))
        if command == "get_robot":
            return {"ok": True, "result": {"name": "rig", "links": [], "joints": []}}
        if command == "screenshot":
            import base64

            return {"ok": True, "result": {"png_base64": base64.b64encode(b"PNG").decode()}}
        return {"ok": True, "result": {"command": command, "args": args}}


def test_call_command_returns_result(monkeypatch):
    bridge = mcp_server.Bridge()
    monkeypatch.setattr(bridge, "_client", _FakeClient())
    monkeypatch.setattr(bridge, "_ensure_started", lambda: None)
    out = bridge.call("get_robot", {})
    assert out["name"] == "rig"


def test_call_command_raises_on_error(monkeypatch):
    bridge = mcp_server.Bridge()

    class _ErrClient:
        def send(self, command, args):
            return {"ok": False, "error": "ValueError", "detail": "boom"}

    monkeypatch.setattr(bridge, "_client", _ErrClient())
    monkeypatch.setattr(bridge, "_ensure_started", lambda: None)
    with pytest.raises(RuntimeError, match="boom"):
        bridge.call("import_meshes", {"paths": []})


def test_screenshot_png_bytes(monkeypatch):
    bridge = mcp_server.Bridge()
    monkeypatch.setattr(bridge, "_client", _FakeClient())
    monkeypatch.setattr(bridge, "_ensure_started", lambda: None)
    assert bridge.screenshot_png() == b"PNG"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/mcp/test_server.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'cad2urdf.mcp.server'` (or skip if SDK missing — install in Task 15 then re-run).

- [ ] **Step 3: Implement the server**

Create `src/cad2urdf/mcp/server.py`:

```python
"""FastMCP server exposing the cad2urdf GUI over a semantic command bridge."""

from __future__ import annotations

import base64
import os
import tempfile
from typing import Any

from mcp.server.fastmcp import FastMCP, Image

from cad2urdf.mcp.client import ControlClient
from cad2urdf.mcp.gui_process import GuiProcess


class Bridge:
    """Owns the GUI subprocess + socket client; lazily spawned on first use."""

    def __init__(self) -> None:
        self._socket_path = os.path.join(
            tempfile.mkdtemp(prefix="cad2urdf-mcp-"), "control.sock"
        )
        self._gui: GuiProcess | None = None
        self._client: ControlClient | None = None

    def _ensure_started(self) -> None:
        if self._gui is None:
            self._gui = GuiProcess(socket_path=self._socket_path)
            self._gui.start(timeout=30.0)
            self._client = ControlClient(self._socket_path)
            self._client.connect(timeout=10.0)

    def call(self, command: str, args: dict[str, Any]) -> Any:
        """Send a command, returning its result or raising on an error envelope."""
        self._ensure_started()
        assert self._client is not None
        resp = self._client.send(command, args)
        if not resp.get("ok"):
            raise RuntimeError(f"{resp.get('error')}: {resp.get('detail')}")
        return resp["result"]

    def screenshot_png(self) -> bytes:
        result = self.call("screenshot", {})
        return base64.b64decode(result["png_base64"])

    def shutdown(self) -> None:
        if self._client is not None:
            self._client.close()
        if self._gui is not None:
            self._gui.stop()


bridge = Bridge()
mcp = FastMCP("cad2urdf")


@mcp.tool()
def get_robot() -> dict:
    """Return the current robot as a structured payload (links, joints, tree)."""
    return bridge.call("get_robot", {})


@mcp.tool()
def list_materials() -> list[str]:
    """List available material names for set_link_material."""
    return bridge.call("list_materials", {})


@mcp.tool()
def get_history() -> dict:
    """Return undo/redo availability."""
    return bridge.call("get_history", {})


@mcp.tool()
def gui_status() -> dict:
    """Report whether the GUI is running and a brief robot summary."""
    return bridge.call("gui_status", {})


@mcp.tool()
def import_meshes(paths: list[str], robot_name: str = "robot") -> dict:
    """Import STL/OBJ files as links (first becomes the base, others fixed to it)."""
    return bridge.call("import_meshes", {"paths": paths, "robot_name": robot_name})


@mcp.tool()
def set_base_link(link: str) -> dict:
    """Re-root the kinematic tree at the given link."""
    return bridge.call("set_base_link", {"link": link})


@mcp.tool()
def rename_link(old: str, new: str) -> dict:
    """Rename a link and update all joints referencing it."""
    return bridge.call("rename_link", {"old": old, "new": new})


@mcp.tool()
def remove_link(link: str) -> dict:
    """Remove a link and its entire subtree."""
    return bridge.call("remove_link", {"link": link})


@mcp.tool()
def set_link_material(link: str, material: str) -> dict:
    """Set a link's material (name + density) from the material table."""
    return bridge.call("set_link_material", {"link": link, "material": material})


@mcp.tool()
def update_joint(
    joint: str,
    type: str | None = None,
    axis: list[float] | None = None,
    origin_xyz: list[float] | None = None,
    origin_rpy: list[float] | None = None,
    limit_lower: float | None = None,
    limit_upper: float | None = None,
    effort: float | None = None,
    velocity: float | None = None,
    parent: str | None = None,
) -> dict:
    """Edit a joint: type/axis/origin/limits, or reparent via `parent`.

    Provide origin_xyz and origin_rpy together (or neither). A robot is a tree, so
    joints are created by import_meshes and removed by remove_link.
    """
    args: dict[str, Any] = {"joint": joint}
    for key, value in (
        ("type", type),
        ("axis", axis),
        ("origin_xyz", origin_xyz),
        ("origin_rpy", origin_rpy),
        ("limit_lower", limit_lower),
        ("limit_upper", limit_upper),
        ("effort", effort),
        ("velocity", velocity),
        ("parent", parent),
    ):
        if value is not None:
            args[key] = value
    return bridge.call("update_joint", args)


@mcp.tool()
def save_project(path: str) -> dict:
    """Save the current robot to a .cad2urdf JSON project file."""
    return bridge.call("save_project", {"path": path})


@mcp.tool()
def open_project(path: str) -> dict:
    """Load a .cad2urdf project, replacing the current robot."""
    return bridge.call("open_project", {"path": path})


@mcp.tool()
def validate(out_dir: str, package_name: str, run_manipulapy: bool = False) -> dict:
    """Write the URDF and run AST (and optional ManipulaPy) validation."""
    return bridge.call(
        "validate",
        {"out_dir": out_dir, "package_name": package_name, "run_manipulapy": run_manipulapy},
    )


@mcp.tool()
def export_package(
    out_dir: str,
    package_name: str,
    maintainer: str = "cad2urdf-user",
    maintainer_email: str = "user@example.com",
    run_manipulapy: bool = False,
) -> dict:
    """Export the full ROS 2 package (URDF + meshes + scaffolding) to out_dir."""
    return bridge.call(
        "export_package",
        {
            "out_dir": out_dir,
            "package_name": package_name,
            "maintainer": maintainer,
            "maintainer_email": maintainer_email,
            "run_manipulapy": run_manipulapy,
        },
    )


@mcp.tool()
def undo() -> dict:
    """Undo the last change."""
    return bridge.call("undo", {})


@mcp.tool()
def redo() -> dict:
    """Redo the last undone change."""
    return bridge.call("redo", {})


@mcp.tool()
def screenshot() -> Image:
    """Capture the live GUI window as a PNG image."""
    return Image(data=bridge.screenshot_png(), format="png")


def run() -> None:
    """Entry point: serve MCP over stdio until the client disconnects."""
    try:
        mcp.run()
    finally:
        bridge.shutdown()
```

Create `src/cad2urdf/mcp/__main__.py`:

```python
"""`python -m cad2urdf.mcp` / `cad2urdf-mcp` entry point."""

from cad2urdf.mcp.server import run

if __name__ == "__main__":
    run()
```

- [ ] **Step 4: Run tests to verify they pass**

Run (after Task 15 installs the extra, or `pip install mcp` now):
`unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/mcp/test_server.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && mypy
git add src/cad2urdf/mcp/server.py src/cad2urdf/mcp/__main__.py tests/unit/mcp/test_server.py
git commit -m "feat(mcp): FastMCP server with full tool surface

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 15: Packaging — `[mcp]` extra + console script

**Files:**
- Modify: `pyproject.toml`
- Test: `tests/unit/mcp/test_packaging.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/mcp/test_packaging.py
import tomllib
from pathlib import Path


def _pyproject() -> dict:
    root = Path(__file__).resolve().parents[3]
    return tomllib.loads((root / "pyproject.toml").read_text())


def test_mcp_extra_declared():
    extras = _pyproject()["project"]["optional-dependencies"]
    assert "mcp" in extras
    assert any("mcp" in dep for dep in extras["mcp"])


def test_mcp_console_script_declared():
    scripts = _pyproject()["project"]["scripts"]
    assert scripts.get("cad2urdf-mcp") == "cad2urdf.mcp.server:run"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/mcp/test_packaging.py -v`
Expected: FAIL — neither the extra nor the script exists yet.

- [ ] **Step 3: Edit pyproject.toml**

In `[project.optional-dependencies]`, add a new extra (place after the `urdf-io` block):

```toml
# Install as: pip install -e ".[mcp]" to expose the GUI to MCP clients.
mcp = [
    "mcp>=1.2.0",
]
```

In `[project.scripts]`, add the console script under the existing entries:

```toml
cad2urdf-mcp = "cad2urdf.mcp.server:run"
```

- [ ] **Step 4: Install the extra + run tests**

```bash
unset PYTHONPATH AMENT_PREFIX_PATH && pip install -e ".[mcp]"
unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/unit/mcp -v
```
Expected: PASS (packaging tests + the earlier client/gui_process/server tests).

- [ ] **Step 5: Commit**

```bash
git add pyproject.toml tests/unit/mcp/test_packaging.py
git commit -m "build(mcp): add [mcp] extra and cad2urdf-mcp console script

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 16: End-to-end integration test

**Files:**
- Test: `tests/integration/test_mcp_e2e.py`

> Spawns a real offscreen GUI via `GuiProcess`, drives it through the
> `ControlClient`, and asserts a package is written. Marked `slow` because it
> launches a Qt process.

- [ ] **Step 1: Write the test**

```python
# tests/integration/test_mcp_e2e.py
import os
import sys

import pytest
import trimesh

from cad2urdf.mcp.client import ControlClient
from cad2urdf.mcp.gui_process import GuiProcess


@pytest.mark.slow
def test_import_edit_export_end_to_end(tmp_path):
    sock_path = str(tmp_path / "e2e.sock")
    env_cmd = [
        sys.executable,
        "-m",
        "cad2urdf.gui.app",
        "--control-socket",
        sock_path,
    ]
    # Force offscreen so the GUI needs no display.
    os.environ["QT_QPA_PLATFORM"] = "offscreen"

    base = tmp_path / "base.stl"
    arm = tmp_path / "arm.stl"
    trimesh.creation.box(extents=(1, 1, 1)).export(base)
    trimesh.creation.box(extents=(1, 1, 1)).export(arm)

    gui = GuiProcess(socket_path=sock_path, launch_cmd=env_cmd)
    gui.start(timeout=60.0)
    client = ControlClient(sock_path)
    client.connect(timeout=10.0)
    try:
        imp = client.send(
            "import_meshes",
            {"paths": [str(base), str(arm)], "robot_name": "rig"},
        )
        assert imp["ok"] is True

        upd = client.send(
            "update_joint",
            {
                "joint": "arm_to_base",
                "type": "revolute",
                "axis": [0.0, 0.0, 1.0],
                "limit_lower": -1.0,
                "limit_upper": 1.0,
                "effort": 5.0,
                "velocity": 1.0,
            },
        )
        assert upd["ok"] is True

        out = tmp_path / "pkg"
        exp = client.send(
            "export_package",
            {"out_dir": str(out), "package_name": "rig_desc", "run_manipulapy": False},
        )
        assert exp["ok"] is True
        assert (out / "package.xml").is_file()
        assert (out / "urdf" / "rig.urdf").is_file()
    finally:
        client.close()
        gui.stop()
```

- [ ] **Step 2: Run the test**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest tests/integration/test_mcp_e2e.py -v -m slow`
Expected: PASS (1 test). If `import_meshes` reports a duplicate-name error, confirm
the two source files have distinct stems (`base.stl`, `arm.stl`).

- [ ] **Step 3: Commit**

```bash
git add tests/integration/test_mcp_e2e.py
git commit -m "test(mcp): end-to-end spawn → import → edit → export

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 17: Documentation

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Add an MCP section to the README**

Insert a new section after the "Examples" section (before "CLI Reference"):

````markdown
## MCP Server (drive the GUI from Claude)

`cad2urdf` ships an MCP server that **launches the GUI and lets an MCP client
(e.g. Claude) operate it end-to-end** — import meshes, edit the kinematic tree,
validate, export a ROS package, and screenshot the live window.

```bash
pip install -e ".[mcp]"      # adds the MCP SDK
```

Register it with your MCP client (Claude Code example):

```bash
claude mcp add cad2urdf -- cad2urdf-mcp
```

The server spawns the GUI on first use over a private Unix-domain socket (no
network exposure). Available tools:

| Group | Tools |
|---|---|
| Introspect | `get_robot`, `list_materials`, `get_history`, `gui_status` |
| Build | `import_meshes`, `set_base_link`, `rename_link`, `remove_link`, `set_link_material` |
| Joints | `update_joint` (type/axis/origin/limits/reparent) |
| Project / export | `save_project`, `open_project`, `validate`, `export_package` |
| Control | `undo`, `redo`, `screenshot` |

A robot is a kinematic **tree**: each non-base link has exactly one incoming
joint. Joints are created by `import_meshes`, removed by `remove_link`, and
restructured by `update_joint(..., parent=...)`.
````

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs(mcp): document the GUI MCP server

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Final verification

- [ ] **Run the whole suite (fast):**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && QT_QPA_PLATFORM=offscreen pytest -q`
Expected: all green (excluding `-m slow` unless requested).

- [ ] **Run slow/e2e:**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && pytest -q -m slow`
Expected: e2e passes.

- [ ] **Type-check:**

Run: `unset PYTHONPATH AMENT_PREFIX_PATH && mypy`
Expected: clean.

- [ ] **Finish the branch** via the superpowers:finishing-a-development-branch skill (PR or merge).

---

## Self-review notes (author)

- **Spec coverage:** launch-&-control (Tasks 11/13), semantic bridge (Tasks 5–10),
  transport A QLocalServer (Task 10), auto-spawn lazy (Tasks 13/14), on-demand
  screenshot (Tasks 9/14), full command surface (Tasks 5–9), `[mcp]` extra (Task 15),
  testing strategy 1–4 (dispatcher Tasks 5–9, server Task 10, MCP layer Task 14,
  e2e Task 16), README (Task 17). All covered.
- **Deviation from spec (intentional):** workers are invoked synchronously by
  calling the `build_*_job` factory directly on the Qt main thread (the factories
  return `(report)->result` callables), rather than running the `QThread` worker
  with a `QEventLoop` block. Same job logic, less machinery, deterministic. The
  `add_joint`/`remove_joint` tools from the spec are folded into `update_joint`
  (reparent) + `remove_link`, matching the tree invariant.
- **Type/name consistency:** `robot_to_payload`, `CommandDispatcher.dispatch`,
  `protocol.COMMANDS/encode/decode/ok/err/ProtocolError`, `ControlServer.start/stop`,
  `ControlClient.connect/send/close`, `GuiProcess.start/stop/is_running`,
  `Bridge.call/screenshot_png/shutdown` — used consistently across tasks.
- **Caveat carried forward:** `set_base_link` re-root is exact for fixed joints;
  actuated-joint axis sign may flip on reversal (documented in the transform and
  README workflow note).
