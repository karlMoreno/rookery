"""Part interface, the printed-part template and the vendor-part adapter.

    «interface» Part
    ├── PrintedPart   «Template Method»  you print it
    └── VendorPart    «Adapter»          you buy it

Pattern: Template Method (PrintedPart.shape()).
    shape() is final and always runs the same steps in the same order:
        body() -> features() -> mounts() -> fasteners() -> print_prep()
    Subclasses fill in body() and, when they need to, features(),
    declare_mounts() and fastener_points(). Why: every printed part gets its
    holes, heat-set inserts, mount declarations and bed check the same way, and
    a new part can't forget a step. The bed check here is what guarantees every
    STL fits the 180 mm printer.

Pattern: Adapter (VendorPart).
    A bought item is a CatalogItem (price, mass) plus a simple stand-in solid.
    VendorPart adapts that pair to the same Part interface as printed parts,
    so the builder, the visitors and the viewer treat a NEMA17 and a printed
    bracket identically. If you later want exact vendor CAD (bd_warehouse or a
    STEP from the supplier), only the stand-in function changes.

Units: millimetres in the part's own local frame. Each part's docstring states
where its local origin is.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from functools import cached_property
from typing import Callable

from build123d import (Align, Axis, Box, Compound, Cylinder, Location, Mode, Part as BdPart, Rot,
                       Shape)

from crane.catalog import CatalogItem
from crane.registry import MountDecl
from crane.spec import PETG_DENSITY, CraneSpec

# Hole sizes for PETG (mm): clearance holes print slightly small, so they are oversized.
HOLES = {
    "M3_clear": 3.4,
    "M3_insert": 4.0,     # M3 heat-set insert (5.7 mm long)
    "M4_clear": 4.5,
    "M5_clear": 5.4,
    "M2_clear": 2.2,
}


class BedFitError(ValueError):
    pass


@dataclass(frozen=True)
class Fastener:
    """A hole drilled along the local -Z of `at`, `depth` mm deep."""
    at: Location
    kind: str
    depth: float = 50.0


class Part(ABC):
    key: str = "part"           # unique name, also the STL file name
    title: str = ""
    process: str = "vendor"     # "printed" or "vendor"
    color: tuple[float, float, float, float] = (0.7, 0.7, 0.7, 1.0)

    def __init__(self, spec: CraneSpec):
        self.spec = spec

    @abstractmethod
    def shape(self) -> Shape: ...

    @property
    @abstractmethod
    def mass_kg(self) -> float: ...

    def mount_decls(self) -> list[MountDecl]:
        return []

    def contact_boxes(self) -> list[tuple[str, tuple, tuple]]:
        """(name, centre, size) boxes in the local frame (mm) for parts that
        take part in contact in the preview sim (mesh boxes, tubs). Most parts
        are drawn only, following Rookery's rule that only the hook collides."""
        return []


def box_from_extents(name, xs, ys, zs):
    """(name, centre, size) from (lo, hi) extents."""
    return (name, ((xs[0] + xs[1]) / 2, (ys[0] + ys[1]) / 2, (zs[0] + zs[1]) / 2),
            (xs[1] - xs[0], ys[1] - ys[0], zs[1] - zs[0]))


class PrintedPart(Part):
    process = "printed"
    color = (0.95, 0.62, 0.10, 1.0)   # PETG orange so printed parts stand out in the viewer
    # Rotation applied before export so the part lies flat on the bed.
    print_rotation: tuple[float, float, float] = (0, 0, 0)
    material = "PETG"

    # ---- the template (do not override) ----
    @cached_property
    def _built(self) -> Shape:
        solid = self.body()
        solid = self.features(solid)
        for f in self.fastener_points():
            solid = self._drill(solid, f)
        self._check_bed(solid)
        return solid

    def shape(self) -> Shape:
        return self._built

    # ---- hooks for subclasses ----
    @abstractmethod
    def body(self) -> BdPart:
        """The main solid."""

    def features(self, solid: BdPart) -> BdPart:
        """Pockets, slots and cut-outs beyond the main body."""
        return solid

    def declare_mounts(self) -> list[MountDecl]:
        """Sensors, switches and cable ports this part carries."""
        return []

    def fastener_points(self) -> list[Fastener]:
        """Holes and heat-set inserts, drilled by the template."""
        return []

    # ---- template machinery ----
    def mount_decls(self) -> list[MountDecl]:
        return self.declare_mounts()

    @staticmethod
    def _drill(solid: BdPart, f: Fastener) -> BdPart:
        d = HOLES[f.kind]
        hole = Cylinder(d / 2, f.depth, align=(Align.CENTER, Align.CENTER, Align.MAX))
        return solid - (f.at * hole)

    def print_shape(self) -> Shape:
        """The part rotated to its print orientation and set on z = 0."""
        rx, ry, rz = self.print_rotation
        s = Rot(rx, ry, rz) * self._built
        bb = s.bounding_box()
        return Location((-bb.center().X, -bb.center().Y, -bb.min.Z)) * s

    def _check_bed(self, solid: Shape) -> None:
        rx, ry, rz = self.print_rotation
        bb = (Rot(rx, ry, rz) * solid).bounding_box()
        lim = self.spec.printable_mm
        if bb.size.X > lim or bb.size.Y > lim or bb.size.Z > self.spec.bed_z_mm:
            raise BedFitError(
                f"{self.key}: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm "
                f"doesn't fit the {lim:.0f} x {lim:.0f} x {self.spec.bed_z_mm:.0f} mm bed area"
            )

    @property
    def footprint_mm(self) -> tuple[float, float, float]:
        bb = self.print_shape().bounding_box()
        return (bb.size.X, bb.size.Y, bb.size.Z)

    @property
    def solid_volume_cm3(self) -> float:
        return self._built.volume / 1000.0

    @property
    def mass_kg(self) -> float:
        return self._built.volume * 1e-9 * PETG_DENSITY * self.spec.infill_factor

    @property
    def insert_count(self) -> int:
        return sum(1 for f in self.fastener_points() if f.kind == "M3_insert")


class VendorPart(Part):
    """Adapter: CatalogItem + stand-in geometry -> Part."""

    process = "vendor"
    color = (0.55, 0.58, 0.62, 1.0)

    def __init__(self, spec: CraneSpec, item: CatalogItem, make: Callable[[], Shape],
                 qty_per_use: float = 1.0, key: str | None = None,
                 color: tuple[float, float, float, float] | None = None,
                 contacts: Callable[[], list] | None = None):
        super().__init__(spec)
        self.item = item
        self._make = make
        self.qty_per_use = qty_per_use     # metres for extrusions, 1 for everything else
        self.key = key or item.key
        self.title = item.description
        if color:
            self.color = color
        self._contacts = contacts

    def contact_boxes(self):
        return self._contacts() if self._contacts else []

    @cached_property
    def _shape(self) -> Shape:
        return self._make()

    def shape(self) -> Shape:
        return self._shape

    @property
    def mass_kg(self) -> float:
        return self.item.mass_kg * self.qty_per_use
