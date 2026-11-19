"""Stand-in solids for bought parts.

These are simple shapes with the right outside dimensions, enough to check
fit and clearance and to draw the crane in the viewer and in MuJoCo. Each
function is the "adaptee" that VendorPart (the Adapter in parts/base.py)
wraps. Swap any of them for exact vendor CAD without touching anything else.

Every function documents its local origin. Units are mm.
"""
from __future__ import annotations

from build123d import Align, Box, Cylinder, Location, Pos

from crane.spec import mm, EXTRUSION, MGN12H_BLOCK_L, MGN12H_BLOCK_W, NEMA14, NEMA14_LEN, NEMA17, NEMA17_LEN

MIN = (Align.MIN, Align.MIN, Align.MIN)
CTR_Z0 = (Align.CENTER, Align.CENTER, Align.MIN)


def ext_box(x0, x1, y0, y1, z0, z1):
    """Axis-aligned box from extents (mm). Used everywhere for readable geometry."""
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=MIN)


def extrusion(length_mm: float, wide: bool = False):
    """V-slot 2020 (or 2040 when wide, 40 along X). Origin: centre of the
    bottom end, extrusion runs along +Z. The centre bore and slots are drawn
    so it reads as extrusion in the viewer."""
    w = mm(EXTRUSION) * (2 if wide else 1)
    d = mm(EXTRUSION)
    body = Box(w, d, length_mm, align=CTR_Z0)
    slot = 6.2
    for sx in ([-w / 2, w / 2]):
        body -= Pos(sx, 0, 0) * Box(3.6, slot, length_mm, align=CTR_Z0)
    body -= Pos(0, d / 2, 0) * Box(slot, 3.6, length_mm, align=CTR_Z0)
    body -= Pos(0, -d / 2, 0) * Box(slot, 3.6, length_mm, align=CTR_Z0)
    return body


def nema17():
    """Origin: centre of the mounting face, shaft along +Z."""
    s = mm(NEMA17)
    return (Box(s, s, mm(NEMA17_LEN), align=(Align.CENTER, Align.CENTER, Align.MAX))
            + Cylinder(11, 2, align=CTR_Z0)
            + Cylinder(2.5, 24, align=CTR_Z0))


def nema14():
    """Origin: centre of the mounting face, shaft along +Z."""
    s = mm(NEMA14)
    return (Box(s, s, mm(NEMA14_LEN), align=(Align.CENTER, Align.CENTER, Align.MAX))
            + Cylinder(11, 2, align=CTR_Z0)
            + Cylinder(2.5, 20, align=CTR_Z0))


def mgn12_rail(length_mm: float):
    """Origin: rear end of the rail, bottom centre; runs along +X."""
    return Box(length_mm, 12, 8, align=(Align.MIN, Align.CENTER, Align.MIN))


def flat_bar(length_mm: float, width: float = 25.0, thick: float = 6.0):
    """Aluminium flat bar. Origin: rear end, bottom centre; runs along +X."""
    return Box(length_mm, width, thick, align=(Align.MIN, Align.CENTER, Align.MIN))


def gt2_idler():
    """Toothless idler with flanges, axle along Z. Origin: centre."""
    return (Cylinder(6.4, 8) + Pos(0, 0, 4.5) * Cylinder(8, 1) + Pos(0, 0, -4.5) * Cylinder(8, 1))


def mgn12h_block():
    """Origin: on the rail's bottom plane, block centre. The block body sits
    from z = 3 to z = 13 and straddles the rail."""
    b = Pos(0, 0, 3) * Box(mm(MGN12H_BLOCK_L), mm(MGN12H_BLOCK_W), 10, align=CTR_Z0)
    return b - Pos(0, 0, 3) * Box(mm(MGN12H_BLOCK_L) + 1, 12.4, 5, align=CTR_Z0)


def t8_screw(length_mm: float):
    """Origin: bottom end, runs along +Z."""
    return Cylinder(4, length_mm, align=CTR_Z0)


def t8_nut():
    """Origin: flange underside centre, nut body hangs below (-Z)."""
    return Cylinder(11, 3.5, align=CTR_Z0) + Cylinder(5.1, 12, align=(Align.CENTER, Align.CENTER, Align.MAX))


def coupler():
    return Cylinder(9.5, 25, align=CTR_Z0)


def kfl08():
    """Origin: flange underside centre."""
    return Box(48, 27, 4, align=CTR_Z0) + Cylinder(7.5, 9, align=CTR_Z0)


def v_wheel():
    """Origin: wheel centre, axle along Z."""
    return Cylinder(12.1, 10.8) - Cylinder(2.5, 12)


def bearing_625():
    return Cylinder(8, 5) - Cylinder(2.5, 6)


def gt2_pulley():
    return Cylinder(6.4, 14)


def pi_camera3():
    """Origin: board centre, lens looks along +Z."""
    return Box(25, 24, 1, align=CTR_Z0) + Box(8.5, 8.5, 5.5, align=CTR_Z0)


def board(x: float, y: float, h: float):
    """Generic PCB with parts. Origin: board underside centre."""
    return Box(x, y, h, align=CTR_Z0)


def probe(dia: float, length: float):
    """Origin: cable end, probe hangs along -Z."""
    return Cylinder(dia / 2, length, align=(Align.CENTER, Align.CENTER, Align.MAX))


def tub(length_x: float, length_y: float, height: float, wall: float):
    """Origin: centre of the floor's underside."""
    outer = Box(length_x, length_y, height, align=CTR_Z0)
    return outer - Pos(0, 0, wall) * Box(length_x - 2 * wall, length_y - 2 * wall, height, align=CTR_Z0)


def microswitch():
    """Origin: switch body centre."""
    return Box(20, 6.4, 10)


def estop():
    return Cylinder(20, 30, align=CTR_Z0)


def drag_chain(length_mm: float):
    """Straight run, origin at one end, along +Y, 15 wide x 10 tall."""
    return Box(15, length_mm, 10, align=(Align.CENTER, Align.MIN, Align.MIN))
