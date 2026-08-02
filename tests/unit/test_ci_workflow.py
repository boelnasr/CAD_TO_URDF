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


def test_unit_test_job_installs_qt_runtime_libraries_before_pytest() -> None:
    workflow_path = Path(__file__).parents[2] / ".github" / "workflows" / "ci.yml"
    workflow = yaml.safe_load(workflow_path.read_text(encoding="utf-8"))

    steps = workflow["jobs"]["unit-tests"]["steps"]
    commands = [step.get("run", "") for step in steps if isinstance(step, dict)]
    system_dependencies = next(
        (command for command in commands if "apt-get install" in command), ""
    )

    assert "libegl1" in system_dependencies
    assert commands.index(system_dependencies) < next(
        index for index, command in enumerate(commands) if "pytest tests/unit" in command
    )
