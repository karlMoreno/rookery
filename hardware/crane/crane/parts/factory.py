"""PartFactory: the one place parts are made.

Pattern: Abstract Factory.
    The builder asks for parts by name: `factory.printed("hook")` or
    `factory.vendor("nema17")`. It never imports a part class or a stand-in
    function. Why: the builder describes WHERE things go; the factory decides
    WHAT a thing is. Swapping a printed bracket for a bought one, or a stand-in
    NEMA17 for exact vendor CAD, is a change here and nowhere else.

Pattern: Registry (the @printed decorator).
    Printed part classes register themselves by key when they are defined, so
    adding a part is writing its class. Nothing else needs editing.

Pattern: Flyweight (instance cache).
    Each (kind, key, size) is built once and shared. Sixteen mesh boxes are
    sixteen placements of one solid, so the CAD kernel builds the box once.
"""
from __future__ import annotations

from typing import Callable

from crane.catalog import item
from crane.parts import vendor as v
from crane.spec import CraneSpec

_PRINTED: dict[str, type] = {}


def printed(key: str):
    """Class decorator: register a PrintedPart subclass under `key`."""
    def deco(cls):
        cls.key = key
        _PRINTED[key] = cls
        return cls
    return deco


# Vendor stand-ins: key -> (catalog key, maker(size) -> shape, colour). `size`
# is a length in mm for things sold by the metre and ignored otherwise.
_STEEL = (0.62, 0.64, 0.68, 1.0)
_ALU = (0.78, 0.80, 0.83, 1.0)
_DARK = (0.15, 0.15, 0.17, 1.0)
_PCB = (0.10, 0.45, 0.25, 1.0)
_WHITE = (0.92, 0.92, 0.90, 1.0)
_BLUE = (0.35, 0.55, 0.85, 0.35)

_VENDOR: dict[str, tuple[str, Callable[[float], object], tuple]] = {
    "extrusion_2020": ("extrusion_2020", lambda L: v.extrusion(L), _ALU),
    "extrusion_2040": ("extrusion_2040", lambda L: v.extrusion(L, wide=True), _ALU),
    "nema17": ("nema17", lambda _: v.nema17(), _DARK),
    "nema14": ("nema14", lambda _: v.nema14(), _DARK),
    "mgn12_rail": ("mgn12_rail", lambda L: v.mgn12_rail(L), _STEEL),
    "mgn12h_block": ("mgn12h_block", lambda _: v.mgn12h_block(), _STEEL),
    "t8_screw": ("t8_screw", lambda L: v.t8_screw(L), _STEEL),
    "t8_nut": ("t8_nut", lambda _: v.t8_nut(), (0.75, 0.6, 0.3, 1.0)),
    "coupler_5_8": ("coupler_5_8", lambda _: v.coupler(), _ALU),
    "kfl08": ("kfl08", lambda _: v.kfl08(), _STEEL),
    "v_wheel": ("v_wheel", lambda _: v.v_wheel(), _DARK),
    "bearing_625": ("bearing_625", lambda _: v.bearing_625(), _STEEL),
    "gt2_pulley": ("gt2_pulley", lambda _: v.gt2_pulley(), _ALU),
    "gt2_idler": ("gt2_idler", lambda _: v.gt2_idler(), _ALU),
    "alu_flat_25x6": ("alu_flat_25x6", lambda L: v.flat_bar(L), _ALU),
    "pi5": ("pi5", lambda _: v.board(85, 56, 16), _PCB),
    "skr_pico": ("skr_pico", lambda _: v.board(85, 56, 15), _PCB),
    "buck_5v": ("buck_5v", lambda _: v.board(40, 20, 12), _PCB),
    "pi_camera3_wide": ("pi_camera3_wide", lambda _: v.pi_camera3(), _PCB),
    "esp32c3": ("esp32c3", lambda _: v.board(22, 18, 4), _PCB),
    "tds_probe": ("tds_probe", lambda _: v.probe(12, 80), _DARK),
    "ph_probe": ("ph_probe", lambda _: v.probe(12, 120), _DARK),
    "ds18b20": ("ds18b20", lambda _: v.probe(6, 50), _STEEL),
    "cell_18650": ("cell_18650", lambda _: v.board(77, 21, 20), (0.2, 0.3, 0.7, 1.0)),
    "microswitch": ("microswitch", lambda _: v.microswitch(), _DARK),
    "estop": ("estop", lambda _: v.estop(), (0.85, 0.1, 0.1, 1.0)),
    "drag_chain_10x15": ("drag_chain_10x15", lambda L: v.drag_chain(L), _DARK),
    "water_tub": ("water_tub", None, _BLUE),   # sized from the spec below
}


class PartFactory:
    def __init__(self, spec: CraneSpec):
        self.spec = spec
        self._cache: dict[tuple, object] = {}

    def printed(self, key: str):
        ck = ("printed", key)
        if ck not in self._cache:
            try:
                cls = _PRINTED[key]
            except KeyError as e:
                raise KeyError(f"no printed part {key!r}; known: {sorted(_PRINTED)}") from e
            self._cache[ck] = cls(self.spec)
        return self._cache[ck]

    def vendor(self, key: str, length_mm: float = 0.0):
        from crane.parts.base import VendorPart
        from crane.spec import mm
        ck = ("vendor", key, round(length_mm, 3))
        if ck not in self._cache:
            cat_key, make, color = _VENDOR[key]
            contacts = None
            if key == "water_tub":
                s = self.spec
                X, Y, Z, W = mm(s.tub_depth), mm(s.tub_length), mm(s.tub_height), mm(s.tub_wall)
                make = lambda _: v.tub(X, Y, Z, W)
                from crane.parts.base import box_from_extents as bx
                contacts = lambda: [
                    bx("floor", (-X / 2, X / 2), (-Y / 2, Y / 2), (0, W)),
                    bx("wall_xn", (-X / 2, -X / 2 + W), (-Y / 2, Y / 2), (0, Z)),
                    bx("wall_xp", (X / 2 - W, X / 2), (-Y / 2, Y / 2), (0, Z)),
                    bx("wall_yn", (-X / 2, X / 2), (-Y / 2, -Y / 2 + W), (0, Z)),
                    bx("wall_yp", (-X / 2, X / 2), (Y / 2 - W, Y / 2), (0, Z)),
                ]
            per_metre = item(cat_key).unit == "m"
            qty = length_mm / 1000.0 if per_metre else 1.0
            suffix = f"_{length_mm:.0f}mm" if per_metre else ""
            self._cache[ck] = VendorPart(self.spec, item(cat_key), lambda: make(length_mm),
                                         qty_per_use=qty, key=f"{key}{suffix}", color=color,
                                         contacts=contacts)
        return self._cache[ck]

    @staticmethod
    def printed_keys() -> list[str]:
        return sorted(_PRINTED)
