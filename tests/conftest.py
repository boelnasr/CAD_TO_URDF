from __future__ import annotations

from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def fixtures_dir() -> Path:
    return PROJECT_ROOT / "tests" / "fixtures"


@pytest.fixture
def tmp_pkg_dir(tmp_path: Path) -> Path:
    out = tmp_path / "out_pkg"
    out.mkdir()
    return out
