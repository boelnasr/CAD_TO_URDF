"""ViewportStyle: studio look params + apply helpers (offscreen-safe).

These tests run under the offscreen pyvista plotter forced by
``tests/gui/conftest.py`` (``PYVISTA_OFF_SCREEN=true``).  They assert the
material/highlight kwargs are distinct and that ``apply_scene`` runs without
error while gating the heavy GPU effects (SSAO / shadows) off when offscreen.
"""

from __future__ import annotations

import numpy as np
import pyvista as pv
import pytest

from cad2urdf.gui.viewport.style import _BG_BOTTOM, ViewportStyle

pytestmark = pytest.mark.gui


def test_mesh_kwargs_is_pbr_metal() -> None:
    kw = ViewportStyle.mesh_kwargs()
    # PBR brushed-metal material, smooth (not faceted) shading.
    assert kw["pbr"] is True
    assert kw["smooth_shading"] is True
    assert kw["metallic"] > 0.5
    assert 0.0 <= kw["roughness"] <= 1.0
    # Base color is tuned brighter than flat black so metal reads as metal.
    color = kw["color"]
    assert max(color) > 0.1


def test_mesh_kwargs_no_envmap_is_lighter_higher_roughness() -> None:
    # Fallback metal (used when the env map fails) must be lighter and rougher
    # than the reflective default so it still reads well without reflections.
    base = ViewportStyle.mesh_kwargs()
    fb = ViewportStyle.mesh_kwargs_no_envmap()
    assert fb["pbr"] is True
    assert fb["roughness"] > base["roughness"]
    assert max(fb["color"]) >= max(base["color"])


def test_highlight_kwargs_distinct_from_mesh_kwargs() -> None:
    base = ViewportStyle.mesh_kwargs()
    hl = ViewportStyle.highlight_kwargs()
    # The selected link must look different from a normal link.
    assert hl != base
    # Specifically the color differs (contrasting highlight).
    assert hl["color"] != base["color"]
    # ...and the PBR params differ too, so the distinct style holds even if the
    # colors were ever made to match.
    assert hl["metallic"] != base["metallic"]
    assert hl["roughness"] != base["roughness"]


def test_apply_scene_offscreen_runs_without_error() -> None:
    plotter = pv.Plotter(off_screen=True)
    # Must not raise even though several GPU features may be unsupported offscreen.
    ViewportStyle.apply_scene(plotter, offscreen=True)
    # The always-on cheap styling must actually land offscreen: the background
    # gradient bottom colour is applied even when heavy GPU passes are gated off.
    assert plotter.background_color == pv.Color(_BG_BOTTOM)
    plotter.close()


def test_apply_scene_offscreen_does_not_call_ssao_or_shadows() -> None:
    # The gate must PREVENT the heavy GPU passes from even being invoked when
    # offscreen (offscreen VTK silently no-ops them, so spy on the calls rather
    # than the renderer flags to keep the assertion non-vacuous).
    plotter = pv.Plotter(off_screen=True)
    ssao_calls: list[int] = []
    shadow_calls: list[int] = []
    plotter.enable_ssao = lambda *a, **k: ssao_calls.append(1)  # type: ignore[method-assign]
    plotter.enable_shadows = lambda *a, **k: shadow_calls.append(1)  # type: ignore[method-assign]

    ViewportStyle.apply_scene(plotter, offscreen=True)

    assert ssao_calls == []
    assert shadow_calls == []
    plotter.close()


def test_apply_scene_real_display_calls_ssao_and_shadows() -> None:
    # Counterpart proving the gate is real: with offscreen=False the heavy
    # passes ARE invoked (guarded, so a no-op on this offscreen renderer is OK).
    plotter = pv.Plotter(off_screen=True)
    ssao_calls: list[int] = []
    shadow_calls: list[int] = []
    plotter.enable_ssao = lambda *a, **k: ssao_calls.append(1)  # type: ignore[method-assign]
    plotter.enable_shadows = lambda *a, **k: shadow_calls.append(1)  # type: ignore[method-assign]

    ViewportStyle.apply_scene(plotter, offscreen=False)

    assert ssao_calls == [1]
    assert shadow_calls == [1]
    plotter.close()


def test_environment_map_generator_returns_texture() -> None:
    tex = ViewportStyle.make_environment_texture()
    assert isinstance(tex, pv.Texture)


def test_apply_scene_falls_back_when_env_map_fails(monkeypatch: pytest.MonkeyPatch) -> None:
    # If the runtime env-map generation blows up, apply_scene must fall back to
    # a light-kit-only setup instead of crashing the viewport.
    def _boom() -> pv.Texture:
        raise RuntimeError("no env map")

    monkeypatch.setattr(ViewportStyle, "make_environment_texture", staticmethod(_boom))
    plotter = pv.Plotter(off_screen=True)
    ViewportStyle.apply_scene(plotter, offscreen=True)  # must not raise
    plotter.close()


def test_make_environment_texture_shape_is_rgb() -> None:
    # The generator builds an RGB numpy gradient before wrapping it in a Texture.
    arr = ViewportStyle.make_environment_array()
    assert arr.ndim == 3
    assert arr.shape[2] == 3
    assert arr.dtype == np.uint8
    # The gradient must run dark floor (row 0) -> bright top (last row); a flat
    # or inverted gradient would give metal nothing to reflect.
    assert arr.min() != arr.max()
    assert arr[0].mean() < arr[-1].mean()


def test_apply_scene_returns_true_when_env_map_applies() -> None:
    # apply_scene reports whether the environment map landed, so the widget can
    # pick the reflective vs the no-envmap fallback metal.
    plotter = pv.Plotter(off_screen=True)
    assert ViewportStyle.apply_scene(plotter, offscreen=True) is True
    plotter.close()


def test_apply_scene_returns_false_when_env_map_fails(monkeypatch: pytest.MonkeyPatch) -> None:
    def _boom() -> pv.Texture:
        raise RuntimeError("no env map")

    monkeypatch.setattr(ViewportStyle, "make_environment_texture", staticmethod(_boom))
    plotter = pv.Plotter(off_screen=True)
    assert ViewportStyle.apply_scene(plotter, offscreen=True) is False
    plotter.close()
