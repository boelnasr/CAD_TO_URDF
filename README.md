# cad2urdf

Convert CAD assemblies (STEP / STL / OBJ) into ROS 2-ready URDF packages.

Status: v0.1.0a0 — CLI core (no GUI yet).

## Install

`pythonOCC-core` is required for STEP support and is **not available on PyPI**. Two install paths:

### Option A — conda (recommended, full STEP support)

```bash
conda env create -f environment.yml
conda activate cad2urdf
```

### Option B — pip only (no STEP support)

```bash
python3.10 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

STL/OBJ inputs work in this mode; STEP-related features will raise `ImportError` at runtime.

## Quick start

> Note: forward-looking — implementation lands in Phase 8 of v0.1.0a0. The command below shows the target API.

```bash
cad2urdf input.step --joints joints.yaml -o my_robot/
```
