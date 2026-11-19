"""Mesh file writers.

Pattern: Strategy.
    The MJCF and STL visitors hold a MeshExporter and call export(shape, path)
    without knowing the format. StlExporter writes print files; ObjExporter
    writes the meshes MuJoCo loads. Why: the visitors decide WHAT to export
    and where; the strategy decides HOW. Adding STEP or 3MF is a new class.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path

from build123d import Shape, export_step, export_stl


class MeshExporter(ABC):
    suffix = ""

    @abstractmethod
    def export(self, shape: Shape, path: Path) -> Path: ...


class StlExporter(MeshExporter):
    suffix = ".stl"

    def __init__(self, tolerance: float = 0.05, angular: float = 0.2):
        self.tolerance = tolerance
        self.angular = angular

    def export(self, shape: Shape, path: Path) -> Path:
        path = path.with_suffix(self.suffix)
        export_stl(shape, str(path), tolerance=self.tolerance, angular_tolerance=self.angular)
        return path


class ObjExporter(MeshExporter):
    """Coarse OBJ for simulation visuals (MuJoCo only draws them)."""
    suffix = ".obj"

    def __init__(self, tolerance: float = 0.3, angular: float = 0.5):
        self.tolerance = tolerance
        self.angular = angular

    def export(self, shape: Shape, path: Path) -> Path:
        path = path.with_suffix(self.suffix)
        verts, tris = shape.tessellate(self.tolerance, self.angular)
        with path.open("w") as fh:
            fh.write("# rookery-crane mesh, millimetres\n")
            for v in verts:
                fh.write(f"v {v.X:.4f} {v.Y:.4f} {v.Z:.4f}\n")
            for a, b, c in tris:
                fh.write(f"f {a + 1} {b + 1} {c + 1}\n")
        return path


class StepExporter(MeshExporter):
    suffix = ".step"

    def export(self, shape: Shape, path: Path) -> Path:
        path = path.with_suffix(self.suffix)
        export_step(shape, str(path))
        return path
