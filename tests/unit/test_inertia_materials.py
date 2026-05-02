import pytest

from cad2urdf.core.inertia.materials import (
    Material,
    list_materials,
    load_material_table,
    lookup,
)


def test_load_default_table_has_aluminum() -> None:
    table = load_material_table()
    assert "aluminum_6061" in table
    al = table["aluminum_6061"]
    assert isinstance(al, Material)
    assert al.density_kg_m3 == 2700


def test_lookup_known_returns_material() -> None:
    m = lookup("steel_1018")
    assert m.density_kg_m3 == 7850


def test_lookup_unknown_raises() -> None:
    with pytest.raises(KeyError, match="unknown material"):
        lookup("unobtainium")


def test_list_materials_returns_sorted_names() -> None:
    names = list_materials()
    assert names == sorted(names)
    assert "pla" in names
