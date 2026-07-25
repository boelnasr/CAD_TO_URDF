# Task 4 report — B-rep joint inference and pipeline wiring

Date: 2026-07-25

## Outcome

Task 4 is implemented. STEP and IGES frontends now load their transient B-rep
side maps, infer conservative mate hints immediately, classify those hints,
clear and delete the shape map before Robot construction, and preserve the
existing all-fixed behavior when no shapes are available. No OCP object is
stored in an `AssemblyEdge`, `ClassifiedJoint`, `Robot`, GUI controller, project,
or socket payload.

The classifier follows the binding decisions:

- default confidence threshold: `0.6`;
- high-confidence concentric without bounds: `continuous`, no limits;
- high-confidence concentric with at least one explicit bound: `revolute`;
  a missing side defaults to `-pi` or `+pi`;
- high-confidence sliding: `prismatic`, using trustworthy inferred/provided
  bounds or `[-0.1, +0.1]` metres;
- low confidence, unknown kind, missing/invalid axis, missing shapes, ambiguous
  geometry, and probe errors: `fixed`.

`classify_default_fixed` retains its previous behavior.

## Geometry algorithm and frame math

All OCP imports live inside probe functions. Importing
`cad2urdf.core.extract.joint_infer` therefore succeeds without OCP; this is also
covered by a test that rejects every attempted `OCP` import.

The module exposes named tolerances with documented units:

- `ANGULAR_TOLERANCE_RAD = radians(1.0)`;
- `LINEAR_TOLERANCE_M = 1e-4`;
- `RADIUS_TOLERANCE_M = 1e-4`;
- `OCP_UNIT_SCALE_M = 0.001` metres per source millimetre.

`BRepAdaptor_Surface` traverses real faces and extracts cylinders and planes.
Face bounds come from `BRepBndLib`. Parent probes are converted from source
millimetres to metres. Child probes are then transformed into the edge parent
frame. For child point `p_C`, direction `d_C`, and edge pose
`T_PC = [R_PC, t_PC]`:

```text
p_P = R_PC (unit_scale * p_C) + t_PC
d_P = normalize(R_PC d_C)
```

This explicitly avoids mixing metre translations from `AssemblyEdge` with
millimetre OCP coordinates.

Concentric inference requires:

1. cylindrical directions parallel within the angular tolerance;
2. radii equal within the radius tolerance;
3. infinite-axis separation
   `norm((p_child - p_parent) cross d_parent)` within the linear tolerance;
4. overlapping face bounds;
5. all matching candidates to describe one unambiguous axis line.

Confidence incorporates angular alignment, radius error, and axis separation.
The normalized/canonical axis is returned in the edge parent frame. Geometry
cannot prove angular stops, so inferred concentric pairs carry no bounds and
classify as `continuous`.

Sliding inference deliberately does not treat one coincident or parallel plane
as motion. It requires exactly two independent groups of coplanar faces with
actual face-area overlap. Those contacts constrain two translations and the
plane normals' cross product is the sole free translation axis. One plane,
three plane directions, edge-only contact, and ambiguous candidates remain
unknown/fixed. For axis-aligned guides only, travel is derived by projecting
the matched parent/child contact-face extents onto the free axis and
intersecting all allowed intervals. Oblique AABB projections are not trusted;
they leave bounds unset so classification uses `[-0.1, +0.1]` metres.

Every edge probe is exception-contained. Missing shapes and any OCP/adaptor/
bounds/pose failure produce an immutable low-confidence unknown copy and never
abort CAD import.

## Classification, reversal, and Robot propagation

`AssemblyEdge` gained OCP-free, serializable `axis`, `limit_lower`, and
`limit_upper` hint fields. `ClassifiedJoint` gained lower/upper limits.

When a spanning-tree edge reverses:

```text
T_CP = inverse(T_PC)
a_C  = -R_CP a_P
[lower_C, upper_C] = [-upper_P, -lower_P]
```

This both expresses the axis in the new parent frame and reverses the positive
joint coordinate. `build_robot_from_tree` propagates the classified limits.
URDF requires effort and velocity on revolute/prismatic joints, but static
geometry cannot infer actuator capability. The builder therefore uses tested,
small, usable placeholders `effort=1.0` and `velocity=1.0` until users replace
them with actuator-specific values. These are preferable to zero, which would
make an otherwise valid inferred joint unusable.

## Pipeline and ownership

CLI and GUI select `load_step_assembly_with_shapes` or
`load_iges_assembly_with_shapes`. With a non-empty map they run:

```text
infer_edge_hints -> classify_from_hints -> shapes.clear() -> del shapes
```

With an empty map they run `classify_default_fixed`, then perform the same
cleanup. Tree and Robot construction happen only afterward. Empty-edge,
one-link IGES assemblies still build with no joints. Integration tests wrap the
real Robot builder and assert the shape map is already empty at that boundary,
then recursively verify that the resulting Robot contains no `TopoDS_Shape`.

## TDD evidence

### Pure hint classification

RED:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest -q \
  tests/unit/test_mate_classify_hints.py

ImportError: cannot import name 'classify_from_hints'
```

GREEN after the minimal pure implementation:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest -q \
  tests/unit/test_mate_classify_hints.py \
  tests/unit/test_extract_mate_classify.py

.............. [100%]
```

### Real OCP inference

RED:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest -q \
  tests/unit/test_joint_infer.py

ModuleNotFoundError: No module named 'cad2urdf.core.extract.joint_infer'
```

GREEN with real `BRepPrimAPI`/`BRepAlgoAPI` solids and actual face traversal:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest -q \
  tests/unit/test_joint_infer.py -vv

5 passed in 0.51s
```

An additional accuracy RED used two boxes whose support planes meet only along
an edge. It initially inferred `sliding`; after requiring face-area overlap,
the six geometry cases passed.

### Pipeline, reversal, limits, and disposal

RED:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest -q \
  tests/unit/test_extract_spanning_tree.py \
  tests/unit/test_build_robot.py \
  tests/integration/test_joint_inference_pipeline.py -vv

4 failed, 7 passed
```

The failures independently showed: untransformed reversed axis, missing
reversed limits, missing Robot limits, old assembly-only loaders, and uncleared
shape maps.

After wiring and propagation, URDF emission exposed a further RED because
prismatic effort/velocity were `None`. Zero placeholders were then rejected by
a behavioral test (`assert 0.0 == 1.0`), and replaced with usable `1.0`
defaults.

Final focused GREEN:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
.venv/bin/pytest -o addopts='-ra --strict-markers' -q \
  tests/unit/test_mate_classify_hints.py \
  tests/unit/test_joint_infer.py \
  tests/unit/test_extract_mate_classify.py \
  tests/unit/test_extract_spanning_tree.py \
  tests/unit/test_build_robot.py \
  tests/integration/test_joint_inference_pipeline.py \
  tests/gui/test_step_import_worker.py

36 passed in 0.55s
```

## Regression and final verification

STEP + IGES regression:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
.venv/bin/pytest -q \
  tests/unit/test_step_parser.py tests/unit/test_iges_parser.py \
  tests/unit/test_caf_parser.py tests/integration/test_step_pipeline.py \
  tests/integration/test_iges_pipeline.py tests/gui/test_step_import_worker.py

16 passed
```

Full offscreen suite:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
.venv/bin/pytest -o addopts='-ra --strict-markers' -q -p pytestqt.plugin

363 passed, 1 skipped in 5.73s
```

The single skip is the pre-existing optional ManipulaPy gate because
`ManipulaPy` is not installed.

Static checks:

```text
.venv/bin/ruff check src tests
All checks passed!

.venv/bin/mypy src/cad2urdf/core src/cad2urdf/gui src/cad2urdf/mcp
Success: no issues found in 66 source files

git diff --check
# exit 0, no output
```

## Files changed

- `src/cad2urdf/core/extract/joint_infer.py` (new)
- `src/cad2urdf/core/extract/mate_classify.py`
- `src/cad2urdf/core/extract/types.py`
- `src/cad2urdf/core/extract/spanning_tree.py`
- `src/cad2urdf/core/extract/build_robot.py`
- `src/cad2urdf/cli/main.py`
- `src/cad2urdf/gui/workers/step_import.py`
- `tests/unit/test_joint_infer.py` (new)
- `tests/unit/test_mate_classify_hints.py` (new)
- `tests/integration/test_joint_inference_pipeline.py` (new)
- `tests/unit/test_extract_spanning_tree.py`
- `tests/unit/test_build_robot.py`

## Self-review and accuracy concerns

- No OCP import exists at module scope.
- No shape map crosses the Robot-builder boundary.
- Axis and translation unit/frame conversions are explicit and covered by a
  rotated-cylinder test.
- Reversal rotates and negates the axis and swaps/negates limits.
- Existing meshless-root STEP fixtures stay valid and fixed; flat one-link IGES
  stays jointless.
- Generic one-plane and edge-only contacts are fixed, preventing obvious false
  prismatic inference.
- The heuristic cannot prove design intent from static geometry. Equal-radius
  coaxial parts could be welded, and a true joint with missing/meshless parent
  geometry cannot be inferred. The product decision favors conservative fixed
  output for uncertainty; users can edit those joints.
- Typical XCAF assembly-frame-to-leaf edges have no parent B-rep in the Task 3
  side map. Those edges intentionally remain fixed rather than borrowing a
  descendant shape or inventing a frame association.
- Oblique face AABBs are not trusted for travel bounds. The joint may still be
  classified from two defensible face contacts, but it receives the safe
  `[-0.1, +0.1]` metre fallback.

---

## Fix round 1/5 — real XCAF graph and conservative contact corrections

Date: 2026-07-25

### Approved graph expansion

The original pipeline passed real XCAF `assembly-frame -> leaf` edges directly
to `infer_edge_hints`, but the Task 3 shape map contains B-reps only for leaf
parts. Every real parser edge therefore lacked its parent shape and stayed
fixed.

`infer_assembly_edge_hints` now preserves `infer_edge_hints` for explicit
leaf-pair callers and adds a graph-aware pipeline layer:

1. Group shaped direct children by their common assembly-frame parent.
2. For each deterministic sibling pair `A, B`, derive the body-relative pose:

   ```text
   T_A_B = inverse(T_parent_A) @ T_parent_B
   ```

3. Probe the real leaf-pair edge `A -> B`, so both B-reps exist and the result
   axis is expressed in `A`.
4. Reject low-confidence/invalid candidates.
5. Select a deterministic maximum-confidence body spanning forest (confidence
   descending, then parent/child IDs).
6. For every accepted moving component, retain exactly one lexicographically
   deterministic original frame anchor and replace the other frame-to-body
   edges with the selected inferred body tree.
7. Retain every original edge for unconnected leaves and preserve all meshless
   frame hierarchy.

This keeps the real parser graph connected, rooted, and acyclic while replacing
`k - 1` redundant frame anchors with `k - 1` moving body edges. Multi-parent
body definitions/instances are conservatively skipped because choosing one
instance anchor without an instance-aware shape map would be ambiguous.

A real STEPCAF-authored fixture now parses as:

```text
robot -> bearing
robot -> shaft
```

and expands to:

```text
robot -> bearing   (fixed anchor)
bearing -> shaft   (continuous, inferred from the two real sibling B-reps)
```

The derived body translation is independently asserted as `[0, 0, 0.005]`
metres. The unmocked CLI emits one fixed and one continuous joint. CLI and GUI
lifecycle tests wrap the real CAF loader solely to observe ownership; they
assert the real shape map is empty before the real Robot builder runs. The old
impossible hand-built leaf-parent graph fixture was removed.

### Planar contact correction

Two perpendicular, one-sided wall contacts are not a prismatic guide. Sliding
now requires bilateral/opposing contact planes at two distinct offsets for
each of the two constrained normal directions. The cross product remains the
only free axis only after that enclosure proof.

Coordinate-wise face AABB overlap is no longer described or accepted as actual
face overlap. The v1 heuristic is deliberately restricted to a verified
rectangular class:

- exactly four unique vertices;
- exactly four straight B-rep edges;
- all four min/max corner combinations;
- plane normal aligned with one parent-frame coordinate axis;
- positive two-dimensional rectangle intersection.

Travel limits use those same verified contact rectangles. True oblique,
nonrectangular, partially trimmed, or otherwise unproven guides remain fixed
instead of receiving speculative motion.

### Cylinder interval and confidence correction

Cylinder overlap now uses the adaptor's axial `V` interval, converted to metres
and projected along the candidate cylinder axis. Axially disjoint cylinders
are rejected even when their world AABBs overlap.

Per-pair confidence is the minimum of the required constraint scores:

```text
min(
  angular alignment score,
  radius match score,
  axis separation score,
  axial overlap / shorter axial span
)
```

The old unconditional/high confidence floor is removed. A pair at roughly 90%
of the angular, radius, and axis-distance tolerances now produces confidence
below `0.6` and classifies fixed, while the exact coaxial positive remains
high-confidence.

### Joint-frame axis correction

`AssemblyEdge.axis` and `ClassifiedJoint.axis` remain parent-frame values.
URDF's axis is expressed in the joint frame established by the edge origin.
`build_robot_from_tree` now performs the boundary conversion:

```text
axis_joint = R_parent_joint.T @ axis_parent
```

A real rotated OCP concentric probe produces parent-frame `+X`; its origin maps
joint `+Z` to parent `+X`. The Robot and emitted URDF now correctly contain
joint-frame `+Z` (`<axis xyz="0 0 1" />`).

### Fix-round TDD evidence

Real graph RED:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest \
  -o addopts='-ra --strict-markers' -q \
  tests/integration/test_joint_inference_pipeline.py -k 'real_caf'

AttributeError: joint_infer has no attribute infer_assembly_edge_hints
assert text.count('type="fixed"') == 1  # obtained 2
2 failed
```

After graph expansion, the two real-CAF cases passed.

Unilateral enclosure RED:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest \
  -o addopts='-ra --strict-markers' -q \
  tests/unit/test_joint_infer.py -k 'one_sided' -vv

assert 'sliding' in {None, 'unknown'}
1 failed
```

After bilateral confinement, both the L-corner negative and enclosed-guide
positive passed.

Oblique AABB RED:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest \
  -o addopts='-ra --strict-markers' -q \
  tests/unit/test_joint_infer.py -k 'disjoint_oblique' -vv

assert 'sliding' in {None, 'unknown'}
1 failed
```

The first attempted rotation did not reproduce the defect because one face
AABB was also disjoint; the fixture was corrected to a generic 3D rotation
whose face AABBs overlap while the actual rectangles do not. The verified
rectangle restriction then made it fixed.

Cylinder RED:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest \
  -o addopts='-ra --strict-markers' -q \
  tests/unit/test_joint_infer.py \
  -k 'diagonal_coaxial or near_tolerance' -vv

axially disjoint diagonal pair: mate_kind == 'concentric', confidence 1.0
near-tolerance pair: confidence 0.82
2 failed
```

After axial intervals and worst-constraint confidence, both negatives and the
exact rotated concentric positive passed (`3 passed`).

Joint-frame RED:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest \
  -o addopts='-ra --strict-markers' -q \
  tests/integration/test_joint_inference_pipeline.py \
  -k 'rotated_inferred' -vv

obtained Robot axis [1, 0, 0], expected [0, 0, 1]
1 failed
```

After the Robot-boundary transform, the Robot and emitted URDF assertions
passed.

### Fix-round verification

Focused Task 4:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
.venv/bin/pytest -o addopts='-ra --strict-markers' -q \
  tests/unit/test_mate_classify_hints.py \
  tests/unit/test_joint_infer.py \
  tests/unit/test_extract_mate_classify.py \
  tests/unit/test_extract_spanning_tree.py \
  tests/unit/test_build_robot.py \
  tests/integration/test_joint_inference_pipeline.py \
  tests/gui/test_step_import_worker.py

43 passed in 0.61s
```

STEP + IGES regression:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
.venv/bin/pytest -o addopts='-ra --strict-markers' -q \
  tests/unit/test_step_parser.py tests/unit/test_iges_parser.py \
  tests/unit/test_caf_parser.py tests/integration/test_step_pipeline.py \
  tests/integration/test_iges_pipeline.py tests/gui/test_step_import_worker.py

16 passed in 0.62s
```

Full software-VTK suite:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
QT_QPA_PLATFORM=offscreen PYVISTA_OFF_SCREEN=true \
VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow \
.venv/bin/pytest -o addopts='-ra --strict-markers' -q -p pytestqt.plugin

370 passed, 1 skipped in 5.92s
```

The single skip remains the optional ManipulaPy gate.

Static gates:

```text
.venv/bin/ruff check src tests
All checks passed!

.venv/bin/mypy src/cad2urdf/core src/cad2urdf/gui src/cad2urdf/mcp
Success: no issues found in 66 source files

git diff --check
# exit 0, no output
```

### Remaining conservative boundaries

- Static equal-radius coaxial geometry still cannot prove design intent; the
  advisory confidence/fixed fallback remains the safety boundary.
- True prismatic guides with oblique or nonrectangular contact faces are now
  deliberately fixed until an exact planar intersection implementation is
  justified.
- Multi-parent/reused XCAF body definitions are left on their original fixed
  hierarchy because the current shape map is definition-keyed, not
  instance-keyed.
- Candidate generation is pairwise within each direct sibling group. This is
  deterministic and appropriate for v1 assemblies, but large flat assemblies
  have quadratic probe cost.
