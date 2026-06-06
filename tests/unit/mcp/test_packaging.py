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
