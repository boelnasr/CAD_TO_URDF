# Joint Editor — Origin (xyz/rpy) Fields — Design

**Date:** 2026-06-13
**Status:** Approved (brainstorming complete, pending implementation plan)
**Related:** [[2026-06-06-gui-mcp-server-design]] (the Joint Editor panel + dispatcher live in the GUI built there), [[2026-04-26-cad-to-urdf-converter-design]] (core model + URDF emit)

## Goal

Let the user set a joint's **origin** (translation `xyz` + rotation `rpy`, in
radians) directly in the GUI's Joint Editor. Today the panel edits a joint's
type, axis, limits, effort, and velocity — but **not** its origin, so links can
only be positioned by driving `update_joint` over the MCP control socket. This
adds the missing field so a robot can be fully authored by hand in the GUI.

## Background

`JointEditorDock` (`src/cad2urdf/gui/panels/joint_editor.py`) edits the joint
parenting the currently selected link. Its `_on_apply` constructs a new `Joint`
and calls `controller.apply` directly (it does **not** route through the
`update_joint` dispatcher command). The origin is currently carried over
unchanged, so there is no way to change it in the UI.

The viewport now renders each link at its forward-kinematic world pose
(`link_world_transforms`, shipped in commits `fd43e79` / `e6a667a`). Therefore
editing an origin here will move the link **live** in the 3D view — these two
pieces together make hand-authoring viable.

The conversion `xyz/rpy -> 4x4` already exists as `origin_from_xyz_rpy`
(`src/cad2urdf/core/config/loader.py`). The inverse (`4x4 -> xyz/rpy`) exists
only as a **private string-formatting** helper `_origin_xyz_rpy` in
`src/cad2urdf/core/urdf/emit.py`; it must be shared numerically to populate the
spin boxes from an existing joint's origin.

## Decisions (locked during brainstorming)

| Decision | Choice |
|---|---|
| Where the fields live | **Existing `JointEditorDock`** — origin belongs to the joint the panel already edits. (Rejected: a separate Origin dock; a viewport drag gizmo.) |
| RPY units | **Radians** — matches the model/URDF and the panel's existing Limit fields; no conversion drift. |
| Apply path | Reuse the panel's existing direct-construct + `controller.apply` flow; **no dispatcher change**. |
| Decomposition | Extract one numeric `xyz_rpy_from_origin`; `emit._origin_xyz_rpy` and the panel both use it. |

## Components

### 1. `core/config/loader.py` — `xyz_rpy_from_origin`
Add `xyz_rpy_from_origin(m: NDArray[Any]) -> tuple[NDArray[Any], NDArray[Any]]`,
the numeric inverse of `origin_from_xyz_rpy`: returns `(xyz, rpy)` as length-3
arrays, using the same roll-pitch-yaw convention (including the existing
gimbal-lock branch when `cos(pitch) ~= 0`). Refactor `emit._origin_xyz_rpy` to
call it and only do string formatting, so there is a **single** decomposition.

### 2. `gui/panels/joint_editor.py` — origin spin boxes
- Add six `QDoubleSpinBox` widgets via the existing `_make_spin` helper with
  generous ranges (xyz and rpy both `-1e6..1e6`, 4 decimals): origin `x`/`y`/`z`
  and `roll`/`pitch`/`yaw`, laid out as two `QHBoxLayout` rows like the existing
  Axis row, added to the form as "Origin xyz" and "Origin rpy (rad)".
- `_populate_from(joint)`: decompose `joint.origin` via `xyz_rpy_from_origin`
  and set the six values.
- `_on_apply`: read the six values, build `origin = origin_from_xyz_rpy([x,y,z],
  [r,p,yaw])`, and include `origin=origin` when constructing the new `Joint`.

### 3. Tests (TDD)
- **Unit** (`tests/unit/`): `xyz_rpy_from_origin` round-trips with
  `origin_from_xyz_rpy` for representative xyz/rpy; matches known values; handles
  the gimbal-lock (`pitch = ±pi/2`) case (equivalent rotation).
- **GUI** (`tests/gui/`): selecting a link populates the origin spins from the
  joint's origin; entering xyz/rpy + Apply produces a joint whose `origin`
  matches `origin_from_xyz_rpy` of the entered values.

## Data flow

```mermaid
flowchart LR
    SEL[Link selected in Link Tree] --> POP[_populate_from]
    POP -->|xyz_rpy_from_origin joint.origin| SPIN[Origin spin boxes]
    SPIN -->|user edits + Apply| APPLY[_on_apply]
    APPLY -->|origin_from_xyz_rpy| J[new Joint origin]
    J --> CA[controller.apply]
    CA -->|robotChanged| VP[viewport re-renders at new world pose]
```

## Error handling

None new: every `xyz`/`rpy` maps to a valid 4×4 transform, so there is nothing
to reject. The existing non-zero-axis guard in `_on_apply` is unchanged.

## Out of scope

Per-link visual origin editing; a viewport drag/gizmo for setting origins; a
degrees/radians toggle.
