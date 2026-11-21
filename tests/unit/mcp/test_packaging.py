try:
    import tomllib
except ImportError:  # Python 3.10 — use backport
    import tomli as tomllib  # type: ignore[no-redef]
from pathlib import Path


def _pyproject() -> dict:
    root = Path(__file__).resolve().parents[3]
    return tomllib.loads((root / "pyproject.toml").read_text())


def test_mcp_extra_declared():
    extras = _pyproject()["project"]["optional-dependencies"]
    assert "mcp" in extras
    assert any(dep.split(">=")[0].split("==")[0].split("<")[0].strip() == "mcp" for dep in extras["mcp"])


def test_mcp_console_script_declared():
    scripts = _pyproject()["project"]["scripts"]
    assert scripts.get("cad2urdf-mcp") == "cad2urdf.mcp.server:run"


def test_mcp_in_mypy_files():
    mypy_files = _pyproject()["tool"]["mypy"]["files"]
    assert any("cad2urdf/mcp" in f for f in mypy_files)


def test_urdf_io_extra_requires_manipulapy_1_4_2_issue18():
    """ManipulaPy 1.3.1-1.4.1 print an emoji banner to stdout on import, which
    corrupts the MCP JSON-RPC stream and crashes cp1252 consoles; 1.4.2 moved it
    to stderr as ASCII, so the extra must not admit anything older."""
    from packaging.requirements import Requirement

    reqs = [Requirement(dep) for dep in _pyproject()["project"]["optional-dependencies"]["urdf-io"]]
    manipulapy = [r for r in reqs if r.name.lower() == "manipulapy"]
    assert len(manipulapy) == 1
    spec = manipulapy[0].specifier
    assert not spec.contains("1.4.1")
    assert spec.contains("1.4.2")


def test_core_dependencies_include_scipy_for_hull_fallback_issue30():
    """compute_inertial's convex-hull fallback runs through trimesh, which needs
    scipy.spatial; without it every non-watertight mesh silently gets zero mass.
    It must not depend on an optional extra (ManipulaPy) happening to pull it in."""
    from packaging.requirements import Requirement

    names = {Requirement(dep).name.lower() for dep in _pyproject()["project"]["dependencies"]}
    assert "scipy" in names
