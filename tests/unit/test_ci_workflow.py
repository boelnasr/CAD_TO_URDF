from pathlib import Path

import yaml


def test_ci_has_an_explicit_unit_test_job() -> None:
    workflow_path = Path(__file__).parents[2] / ".github" / "workflows" / "ci.yml"
    workflow = yaml.safe_load(workflow_path.read_text(encoding="utf-8"))

    unit_job = workflow["jobs"]["unit-tests"]
    commands = "\n".join(
        step.get("run", "") for step in unit_job["steps"] if isinstance(step, dict)
    )

    assert "pytest tests/unit" in commands
