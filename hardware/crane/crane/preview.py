"""Preview pictures of single parts, for the parts kit index.

Pattern: Strategy.
    PartsKitVisitor (visitors/parts_kit.py) holds a PreviewRenderer and calls
    render(shape, path, color) without knowing how the picture is made.
    MatplotlibRenderer draws a shaded isometric view from the part's own
    tessellation, with no GPU and no viewer running, so `python -m crane parts`
    works on any machine and in CI. Why a Strategy: a nicer renderer (VTK,
    pyrender, a headless OCP screenshot) can replace it later as one new class,
    and NullRenderer turns pictures off for a fast run.

        «interface» PreviewRenderer
            render(shape, path, color) -> Path | None
          ├── MatplotlibRenderer    shaded isometric PNG
          └── NullRenderer          no picture (fast)
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path

import numpy as np


class PreviewRenderer(ABC):
    @abstractmethod
    def render(self, shape, path: Path, color: tuple[float, float, float, float]) -> Path | None: ...


class NullRenderer(PreviewRenderer):
    """Null Object: same interface, does nothing. Callers never test for None."""

    def render(self, shape, path, color):
        return None


class MatplotlibRenderer(PreviewRenderer):
    """Shaded isometric view. One light from the upper left, flat shading per
    triangle, equal axis scale so a long extrusion looks long."""

    def __init__(self, size_px: int = 360, tolerance: float = 0.4, angular: float = 0.5):
        self.size_px = size_px
        self.tolerance = tolerance
        self.angular = angular

    def render(self, shape, path: Path, color) -> Path:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        verts, tris = shape.tessellate(self.tolerance, self.angular)
        v = np.array([(p.X, p.Y, p.Z) for p in verts])
        t = np.array(tris)
        faces = v[t]                                           # (n, 3, 3)
        normals = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
        n = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = np.divide(normals, n, out=np.zeros_like(normals), where=n > 0)
        light = np.array([-0.4, -0.6, 0.7])
        light /= np.linalg.norm(light)
        shade = 0.35 + 0.65 * np.clip(np.abs(normals @ light), 0, 1)
        base = np.maximum(np.array(color[:3]), 0.32)            # keep black motors readable
        fc = np.clip(base[None, :] * shade[:, None], 0, 1)
        fc = np.hstack([fc, np.full((len(fc), 1), 1.0)])

        dpi = 100
        fig = plt.figure(figsize=(self.size_px / dpi, self.size_px / dpi), dpi=dpi)
        ax = fig.add_axes([0, 0, 1, 1], projection="3d")
        ax.add_collection3d(Poly3DCollection(faces, facecolors=fc, edgecolors="none"))
        lo, hi = v.min(axis=0), v.max(axis=0)
        ctr, half = (lo + hi) / 2, max((hi - lo).max() / 2, 1.0)
        for setter, c in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), ctr):
            setter(c - half, c + half)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=26, azim=-52)
        ax.set_proj_type("ortho")
        ax.set_axis_off()
        path = path.with_suffix(".png")
        fig.savefig(path, transparent=True)
        plt.close(fig)
        return path
