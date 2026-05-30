"""Two-point axis pick: clicking two points computes a unit-length axis vector."""

from __future__ import annotations

import numpy as np
import pytest

pytestmark = pytest.mark.gui


def test_unit_axis_from_two_points() -> None:
    from cad2urdf.gui.viewport.axis_pick import axis_from_two_points

    axis = axis_from_two_points(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 3.0]))
    assert np.allclose(axis, np.array([0.0, 0.0, 1.0]))


def test_axis_rejects_coincident_points() -> None:
    from cad2urdf.gui.viewport.axis_pick import axis_from_two_points

    with pytest.raises(ValueError, match="coincident"):
        axis_from_two_points(np.array([1.0, 2.0, 3.0]), np.array([1.0, 2.0, 3.0]))


def test_collector_signals_axis_after_two_clicks(qtbot) -> None:
    from cad2urdf.gui.viewport.axis_pick import TwoPointAxisCollector

    collector = TwoPointAxisCollector()
    received: list[np.ndarray] = []
    collector.axisReady.connect(lambda v: received.append(v))

    collector.add_point(np.array([0.0, 0.0, 0.0]))
    assert received == []
    collector.add_point(np.array([1.0, 0.0, 0.0]))
    assert len(received) == 1
    assert np.allclose(received[0], np.array([1.0, 0.0, 0.0]))


def test_collector_resets_after_emit() -> None:
    from cad2urdf.gui.viewport.axis_pick import TwoPointAxisCollector

    collector = TwoPointAxisCollector()
    collector.add_point(np.array([0.0, 0.0, 0.0]))
    collector.add_point(np.array([1.0, 0.0, 0.0]))
    assert collector.points_collected() == 0
