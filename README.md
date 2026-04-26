# cad2urdf

Convert CAD assemblies (STEP / STL / OBJ) into ROS 2-ready URDF packages.

Status: v0.1.0a0 — CLI core (no GUI yet).

## Install

```bash
pip install -e ".[dev]"
```

## Quick start

```bash
cad2urdf input.step --joints joints.yaml -o my_robot/
```
