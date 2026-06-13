from pathlib import Path

from cad2urdf.core.kinematic.model import InertialOverride, Link, Robot
from cad2urdf.core.project.save import SCHEMA_VERSION, robot_to_payload


def _robot() -> Robot:
    link = Link(
        name="base",
        visual_mesh_path=Path("/tmp/base.stl"),
        collision_mesh_path=Path("/tmp/base.stl"),
        material_density=2700.0,
        material_name="aluminum_6061",
        inertial_override=InertialOverride(),
    )
    return Robot(name="r", base_link="base", links={"base": link}, joints={})


def test_robot_to_payload_has_schema_and_links():
    payload = robot_to_payload(_robot())
    assert payload["schema_version"] == SCHEMA_VERSION
    assert payload["name"] == "r"
    assert payload["base_link"] == "base"
    assert payload["links"][0]["name"] == "base"
    assert payload["joints"] == []
