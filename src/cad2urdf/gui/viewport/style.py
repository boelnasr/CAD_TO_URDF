"""Studio viewport styling — material params + scene-apply helpers.

The whole "studio" look lives here so ``widget.py`` stays about wiring:

* :meth:`ViewportStyle.mesh_kwargs` — the ``add_mesh`` kwargs for a normal link
  (PBR brushed-metal, smooth shading).
* :meth:`ViewportStyle.highlight_kwargs` — a contrasting style for the
  currently selected link.
* :meth:`ViewportStyle.apply_scene` — background gradient, light kit, a
  runtime-generated environment map (for metal reflections), floor + shadow,
  SSAO and SSAA.  SSAO and shadows are heavy GPU passes, so they are applied
  **only on a real display** (``offscreen=False``); the cheap styling is always
  applied.

Every GPU-feature call is individually guarded: an unsupported feature is
skipped, never fatal.  If the runtime environment map cannot be built/applied
the scene falls back to a light-kit-only setup so it still looks good.
"""

from __future__ import annotations

import contextlib
from typing import Any

import numpy as np
import pyvista as pv
from numpy.typing import NDArray

# ---- look constants --------------------------------------------------------

# Brushed-metal base for a normal link.  Tuned a touch brighter than a flat
# black/grey so PBR metal reads as metal even before reflections land.
_METAL_COLOR: tuple[float, float, float] = (0.55, 0.57, 0.60)
_METAL_METALLIC: float = 0.85
_METAL_ROUGHNESS: float = 0.35

# Higher-roughness, lighter metal used when the environment map fails (no
# reflections to carry a near-mirror finish).
_METAL_COLOR_NO_ENVMAP: tuple[float, float, float] = (0.70, 0.72, 0.75)
_METAL_ROUGHNESS_NO_ENVMAP: float = 0.6

# Contrasting highlight for the selected link (warm amber).
_HIGHLIGHT_COLOR: tuple[float, float, float] = (1.0, 0.62, 0.10)
_HIGHLIGHT_METALLIC: float = 0.3
_HIGHLIGHT_ROUGHNESS: float = 0.4

# Soft studio backdrop (bottom -> top vertical gradient).
_BG_BOTTOM: tuple[float, float, float] = (0.10, 0.11, 0.13)
_BG_TOP: tuple[float, float, float] = (0.32, 0.35, 0.40)

# Subtle ground plane: semi-transparent and close to the robot footprint so it
# reads as a shadow-catcher (real display adds SSAO/shadows) rather than a slab.
_FLOOR_COLOR: tuple[float, float, float] = (0.18, 0.19, 0.22)
_FLOOR_OPACITY: float = 0.35

# Environment-map gradient endpoints (dark floor up to a bright "sky").
_ENVMAP_BOTTOM: tuple[int, int, int] = (28, 30, 36)
_ENVMAP_TOP: tuple[int, int, int] = (235, 238, 245)
_ENVMAP_SIZE: int = 256


class ViewportStyle:
    """Static holder for the studio viewport look and its apply helpers."""

    # ---- material kwargs ---------------------------------------------------

    @staticmethod
    def mesh_kwargs() -> dict[str, Any]:
        """``add_mesh`` styling kwargs for a normal (unselected) link."""
        return {
            "smooth_shading": True,
            "pbr": True,
            "metallic": _METAL_METALLIC,
            "roughness": _METAL_ROUGHNESS,
            "color": _METAL_COLOR,
        }

    @staticmethod
    def mesh_kwargs_no_envmap() -> dict[str, Any]:
        """Fallback link styling when the environment map could not be applied.

        Without reflections a near-mirror finish reads as flat black, so use a
        lighter base colour and a higher roughness for a more diffuse metal.
        """
        kwargs = ViewportStyle.mesh_kwargs()
        kwargs["roughness"] = _METAL_ROUGHNESS_NO_ENVMAP
        kwargs["color"] = _METAL_COLOR_NO_ENVMAP
        return kwargs

    @staticmethod
    def highlight_kwargs() -> dict[str, Any]:
        """``add_mesh`` styling kwargs for the selected link (contrasting)."""
        return {
            "smooth_shading": True,
            "pbr": True,
            "metallic": _HIGHLIGHT_METALLIC,
            "roughness": _HIGHLIGHT_ROUGHNESS,
            "color": _HIGHLIGHT_COLOR,
        }

    # ---- environment map ---------------------------------------------------

    @staticmethod
    def make_environment_array() -> NDArray[np.uint8]:
        """Build an RGB vertical-gradient image for the studio environment.

        Dark near the floor, bright near the top — enough variation for PBR
        metal to pick up soft reflections instead of reading flat black.
        """
        bottom = np.array(_ENVMAP_BOTTOM, dtype=np.float64)
        top = np.array(_ENVMAP_TOP, dtype=np.float64)
        ramp = np.linspace(0.0, 1.0, _ENVMAP_SIZE)[:, None]  # (H, 1)
        rows = bottom[None, :] * (1.0 - ramp) + top[None, :] * ramp  # (H, 3)
        image = np.repeat(rows[:, None, :], _ENVMAP_SIZE, axis=1)  # (H, W, 3)
        return np.ascontiguousarray(np.round(image)).astype(np.uint8)

    @staticmethod
    def make_environment_texture() -> Any:
        """Wrap :meth:`make_environment_array` in a pyvista ``Texture``."""
        texture_cls: Any = pv.Texture
        return texture_cls(ViewportStyle.make_environment_array())

    # ---- scene -------------------------------------------------------------

    @staticmethod
    def apply_scene(plotter: Any, *, offscreen: bool) -> bool:
        """Apply the studio scene to ``plotter``.

        Cheap styling (background gradient, light kit, env map, floor, SSAA) is
        always applied; SSAO and shadows are applied only when not ``offscreen``.
        Every GPU call is guarded so an unsupported feature is skipped quietly.

        Returns ``True`` if the environment map was applied, ``False`` if it
        failed and the scene fell back to light-kit-only — in which case the
        caller should style links with :meth:`mesh_kwargs_no_envmap` (a lighter,
        rougher metal that reads well without reflections).
        """
        # Background gradient (bottom -> top).
        with contextlib.suppress(Exception):
            plotter.set_background(_BG_BOTTOM, top=_BG_TOP)

        # Light kit for soft, even illumination.
        with contextlib.suppress(Exception):
            plotter.enable_lightkit()

        # Runtime environment map for metal reflections.  On any failure fall
        # back to light-kit-only so the scene still renders (the light kit is
        # already enabled above); the caller then uses ``mesh_kwargs_no_envmap``
        # for the matching lighter, higher-roughness metal.
        env_ok = False
        try:
            texture = ViewportStyle.make_environment_texture()
            plotter.set_environment_texture(texture)
            env_ok = True
        except Exception:
            # Light-kit-only fallback: nothing more to do, the kit is already on.
            env_ok = False

        # Ground plane + soft anti-aliasing — cheap, always on.
        with contextlib.suppress(Exception):
            plotter.add_floor("-z", color=_FLOOR_COLOR, opacity=_FLOOR_OPACITY, pad=0.25)
        with contextlib.suppress(Exception):
            plotter.enable_anti_aliasing("ssaa")

        # Heavy GPU passes: only on a real display.
        if not offscreen:
            with contextlib.suppress(Exception):
                plotter.enable_ssao()
            with contextlib.suppress(Exception):
                plotter.enable_shadows()

        return env_ok
