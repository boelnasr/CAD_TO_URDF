# Changelog

All notable user-facing changes to `cad2urdf` are documented here. This project
follows [Semantic Versioning](https://semver.org/).

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

### Safety and limitations

- Moving joints require confidence of at least `0.6`; lower-confidence
  hypotheses fall back to fixed.
- Inference is advisory and should be reviewed in the Joint Editor before
  simulation or hardware use.
- Oblique/nonrectangular guides, conflicting cylinder axes, and multi-parent
  XCAF instances remain fixed.
- CAD-import tessellations live in a session temporary directory that expires
  when the document is replaced or the GUI exits. Schema-v1 project files only
  store paths to those meshes, so CAD-import projects are not durably reopenable
  after exit; review and export within the current session.
