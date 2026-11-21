# Changelog

All notable user-facing changes to `cad2urdf` are documented here. This project
follows [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Changed

- The `[urdf-io]` extra now requires `manipulapy>=1.4.2`. Earlier releases print
  an import banner to stdout, which corrupts the MCP stdio stream (#18).
- URDF `<inertial>` values (mass, COM, inertia) are written at full float
  precision instead of 6 significant digits; joint origins, axes and limits keep
  the short format (#23).
- The CLI rejects STL/OBJ inputs whose file stems collide, including stems that
  differ only in case, instead of silently overwriting one link (#12).
- Saving a project refuses to replace an existing `<project>_assets` folder that
  cad2urdf did not create (#6).
- When the GUI window was closed or crashed, the next MCP tool call reports
  "GUI not running" once instead of silently continuing on a fresh, empty
  document (#15).

### Fixed

- STEP/IGES: reversed B-rep faces are wound outward, so mass, COM and inertia are
  correct for parts modelled away from their own origin (#2).
- Inertia: inside-out watertight meshes are inverted and inconsistently wound
  ones fall back to the convex hull, instead of integrating a wrong or negative
  signed volume (#21).
- STEP/IGES: a part with no tessellable geometry names the file and part in the
  error (#24). Prismatic travel limits no longer depend on part-name order (#11).
  Generated joint names no longer collide and drop joints (#25).
- Kinematic tree: reparenting that would create a cycle is rejected, which used
  to hang `set_base_link` (#7); re-rooting re-expresses reversed joint axes in the
  new child frame (#8).
- Projects: saving keeps every link pointing at its own mesh (#3); projects
  opened through a relative path load absolute mesh paths, so export no longer
  writes a package without meshes (#10).
- GUI: Validate no longer crashes on meshless frame links (#4), bakes inertials
  like Export does (#16), survives a second click while running (#13) and works
  for any robot name (#22). Selecting a frame link no longer aborts the app (#5).
  Export rejects link names that would write outside the package (#9).
- MCP: GUI child output stays off the JSON-RPC stdout stream (#14); a dead GUI
  is detected instead of raising `BrokenPipeError` forever (#15); non-string
  commands return an error envelope (#19); a client disconnecting during
  `import_cad` no longer aborts the GUI (#20); temporary CAD and validate
  directories are cleaned up on shutdown (#28).
- ROS package: the RViz Fixed Frame and launch `robot_description` survive
  YAML-special link names (#26).
- ManipulaPy gate: an import failure other than `ImportError` is reported
  instead of making the CLI delete a fully written package (#27).
- Tooling: mypy passes with the PyQt6 6.12 stubs, which made `pyqtSignal`'s
  `arguments` keyword required (#17).

## [1.0.0] - 2026-07-25

### Added

- STEP and IGES assembly import through one CAD pipeline in the CLI, desktop
  GUI, and MCP server.
- Advisory B-rep joint inference for clear concentric and rectangular sliding
  relationships. Current geometry probes emit continuous and prismatic joints;
  uncertain or unsupported relationships remain fixed.
- Maximum-confidence forest selection across shaped sibling-body hypotheses,
  followed by deterministic BFS rooting and conservative handling of ambiguous
  axes and contacts.
- A unified MCP `import_cad` tool for `.step`, `.stp`, `.iges`, and `.igs`
  assemblies.
- Generated OCP fixtures and end-to-end coverage for STEP, IGES, GUI, MCP, joint
  inference, and URDF emission.

### Changed

- CAD imports now replace the active GUI document and clear undo/redo history.
- The shared CAF/XCAF parser keeps B-rep handles transient, outside the
  serializable robot and project models.
- CAD loading runs outside the GUI dispatch thread, while MCP control requests
  preserve FIFO ordering.
- The existing `[step]` optional dependency now enables both STEP and IGES
  support.
- CI now exposes unit tests as a dedicated job while retaining the
  complete headless integration and GUI suite.

### Safety and limitations

- Moving joints require confidence of at least `0.6`; lower-confidence
  hypotheses fall back to fixed.
- Inference is advisory and should be reviewed in the Joint Editor before
  simulation or hardware use.
- Off-origin coaxial axes, oblique/nonrectangular guides, conflicting cylinder
  axes, and multi-parent XCAF instances remain fixed.
- CAD-import tessellations live in a session temporary directory that expires
  when the document is replaced or the GUI exits. Schema-v1 project files only
  store paths to those meshes, so CAD-import projects are not durably reopenable
  after exit; review and export within the current session.
