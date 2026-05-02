# cad2urdf

Convert CAD assemblies (STEP / STL / OBJ) into ROS 2-ready URDF packages.

Status: v0.1.0a0 — CLI core (no GUI yet).

## Install

### Recommended (pip-only — STL/OBJ inputs)

```bash
python3.10 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

This is the default v1-alpha install path. STL and OBJ mesh inputs are fully
supported. STEP support requires conda + pythonOCC-core (see below) — until
that lands in v1.0, the CLI rejects `.step` / `.stp` inputs with a clear error.

### v1.0+ — STEP support via conda (not yet wired)

`pythonOCC-core` is not on PyPI; it's distributed via conda-forge. The
`environment.yml` is provided as scaffolding for v1.0; the CLI doesn't yet
consume STEP files. Track the GitHub issue tracker for STEP support landing.

```bash
conda env create -f environment.yml
conda activate cad2urdf
```

## Quick start

> Note: forward-looking — implementation lands in Phase 8 of v0.1.0a0. The command below shows the target API.

```bash
cad2urdf input.step --joints joints.yaml -o my_robot/
```
