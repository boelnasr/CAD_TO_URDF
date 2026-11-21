import sys
from pathlib import Path

import pytest

from cad2urdf.core.validation.manipulapy_gate import (
    ValidationReport,
    validate_urdf,
)


def test_invalid_urdf_returns_failed_report(tmp_path: Path) -> None:
    p = tmp_path / "broken.urdf"
    p.write_text("<not even xml")
    report = validate_urdf(p)
    assert isinstance(report, ValidationReport)
    assert report.ok is False
    assert report.error
    assert report.urdf_path == p


def test_missing_file_returns_failed_report(tmp_path: Path) -> None:
    report = validate_urdf(tmp_path / "ghost.urdf")
    assert report.ok is False
    assert report.error
    assert "not found" in report.error.lower()


def test_missing_manipulapy_returns_clear_error(tmp_path: Path) -> None:
    """When ManipulaPy is NOT installed, validate_urdf should return a failed report
    with a helpful error rather than raising ImportError."""
    try:
        import ManipulaPy  # noqa: F401

        pytest.skip("ManipulaPy is installed; this test only verifies the not-installed path")
    except ImportError:
        pass

    valid_urdf = tmp_path / "valid.urdf"
    valid_urdf.write_text(_MINIMAL_URDF)
    report = validate_urdf(valid_urdf)
    assert report.ok is False
    assert report.error
    assert "manipulapy" in report.error.lower()


def test_manipulapy_import_crash_returns_failed_report_issue27(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A non-ImportError raised while importing ManipulaPy (native-library
    OSError, numba RuntimeError, banner UnicodeEncodeError) must not escape:
    the CLI's catch-all would roll back the package it just wrote."""
    fake_pkg = tmp_path / "fake" / "ManipulaPy"
    fake_pkg.mkdir(parents=True)
    (fake_pkg / "__init__.py").write_text("")
    (fake_pkg / "urdf_processor.py").write_text(
        'raise RuntimeError("simulated native-library failure at import")\n'
    )
    cached = [m for m in sys.modules if m == "ManipulaPy" or m.startswith("ManipulaPy.")]
    for name in cached:
        monkeypatch.delitem(sys.modules, name)  # restored at teardown
    monkeypatch.syspath_prepend(str(tmp_path / "fake"))

    urdf = tmp_path / "valid.urdf"
    urdf.write_text(_MINIMAL_URDF)
    try:
        report = validate_urdf(urdf)
    finally:
        # Drop the fake package so later tests import the real one (if any).
        for name in [m for m in sys.modules if m == "ManipulaPy" or m.startswith("ManipulaPy.")]:
            del sys.modules[name]

    assert report.ok is False
    assert report.error is not None
    assert "ManipulaPy failed to load" in report.error
    assert "simulated native-library failure" in report.error
    assert report.trace is not None and "RuntimeError" in report.trace


@pytest.mark.slow
def test_minimal_valid_urdf_passes_when_manipulapy_installed(tmp_path: Path) -> None:
    """A trivial URDF should parse cleanly via ManipulaPy — when ManipulaPy is installed.

    Skipped on default install path (no manipulapy in [dev] extras).
    """
    pytest.importorskip("ManipulaPy")
    urdf = tmp_path / "min.urdf"
    urdf.write_text(_MINIMAL_URDF)
    report = validate_urdf(urdf)
    # ManipulaPy may still warn about missing meshes; we accept either ok=True or
    # an error mentioning meshes (the URDF references no actual mesh files on disk).
    assert report.ok or (report.error and "mesh" in report.error.lower())


_MINIMAL_URDF = """\
<?xml version="1.0"?>
<robot name="m">
  <link name="base"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""
