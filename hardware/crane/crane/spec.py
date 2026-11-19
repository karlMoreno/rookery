"""CraneSpec: the one set of numbers every other module reads.

Pattern: Parameter Object (a frozen dataclass).
    Every independent dimension of the crane lives in one immutable object that
    is passed around whole. Why: the CAD, the MuJoCo export, the Klipper config
    and the BOM must agree on every number. Passing one frozen object makes it
    impossible for one of them to drift, and `frozen=True` stops any module from
    quietly changing a value after validation.

Pattern: Validate at construction, derive the rest (same rule as Rookery's
    SceneSpec). Only independent numbers are fields. Joint ranges, the home pose,
    cell positions and extrusion lengths are @property values computed from the
    fields, so they can never contradict them. `__post_init__` collects EVERY
    problem and raises once with the whole list, so one run tells you all that
    is wrong instead of one error at a time.

The X axis is a two-stage telescopic fork, the way pallet stacker cranes
(Atlantic Rack's, for one) reach into a rack without a long arm sticking out
behind the mast. Three equal-length stages nest on top of each other:

    carriage (fixed)  ── 2 MGN12H blocks at the front of the carriage
       └ middle stage ── rail A on top, 2 MGN12H blocks under its front end
            └ top stage ── rail B, the hook rail (joint arm_x)

A pinion on the carriage drives the middle stage by a belt rack. Two belts
(extend and retract), each anchored to the carriage, wrapped round an idler
on the middle stage and anchored to the top stage, make the top stage move
twice as far as the middle one (a 2:1 differential). So the hook travels
2 x (fork_length - block overlap) while the folded fork is only fork_length
long.

Units: metres for fields; `mm()` converts for CAD.  Frame: Y along the aisle,
X into the rack, Z up.  Joint values rail_y / mast_z / arm_x are the world
position of the hook's cup (the top centre of the J-hook's foot).
"""
from __future__ import annotations

import dataclasses
import tomllib
from dataclasses import dataclass, field
from pathlib import Path


class SpecError(ValueError):
    """Raised once with every problem found in a spec."""

    def __init__(self, problems: list[str]):
        self.problems = problems
        super().__init__("CraneSpec has problems:\n  - " + "\n  - ".join(problems))


def mm(metres: float) -> float:
    """Metres to millimetres for build123d, which works in mm."""
    return metres * 1000.0


# Fixed vendor dimensions (metres). These are properties of parts you buy, not
# design choices, so they are constants rather than spec fields.
EXTRUSION = 0.020          # 2020 / 2040 V-slot pitch
MGN12_RAIL_H = 0.008
MGN12_RAIL_W = 0.012
MGN12H_BLOCK_L = 0.0454
MGN12H_BLOCK_W = 0.027
MGN12H_BLOCK_H = 0.010     # block height above the rail's base (13 mm total, 8 mm rail sits inside)
NEMA17 = 0.0423
NEMA17_LEN = 0.048
NEMA14 = 0.035
NEMA14_LEN = 0.028
VWHEEL_OD = 0.0242
VWHEEL_W = 0.0108
TRUCK_LENGTH = 0.150       # printed, sized for a 180 mm bed
TRUCK_PLATE = 0.008
TRUCK_GAP = 0.025           # room for the top V-wheels and the flat Y belt under the truck plate
RAIL_END_BLOCK = 0.060     # motor / idler block at each rail end
CARRIAGE_H = 0.090
CARRIAGE_BELOW_ARM = 0.005  # arm runs along the carriage bottom, so a hanging box never meets the carriage
# Telescopic fork (see the module docstring). z levels are measured up from the
# hook rail's underside, the "fork underside" the hook hangs from.
FORK_BAR_W = 0.025          # aluminium flat bar under the middle stage's rail A
FORK_BAR_T = 0.006
FORK_BLOCK_PITCH = 0.050    # centre to centre of the two blocks in each pair
FORK_BLOCK_SPAN = FORK_BLOCK_PITCH + MGN12H_BLOCK_L   # rail length a block pair covers
FORK_END_MARGIN = 0.002     # blocks stop this far short of a rail end
FORK_IDLER_REACH = 0.016    # idler arms stick out this far past the middle stage's ends
FORK_STACK = 0.032          # rail B (8) + block (5) + bar (6) + rail A (8) + block (5): carriage shelf height
FORK_IN_RACK_H = 0.027      # tallest part of the fork that enters the rack (top of rail A)
FORK_RATIO = 2.0            # top stage moves twice as far as the middle stage
MAST_TOP_BLOCK = 0.035      # screw bearing + guide rollers below the mast top
Z_DRIVE_STACK = 0.080       # motor + coupler + nut clearance above the truck top
PETG_DENSITY = 1270.0       # kg/m^3


@dataclass(frozen=True)
class CraneSpec:
    # ---- rack (matches Rookery's SceneSpec numbers) ----
    rack_x0: float = 0.10
    rack_y0: float = 0.05
    tier_floors: tuple[float, ...] = (0.10, 0.39, 0.68)
    tub_length: float = 0.44
    tub_depth: float = 0.44
    tub_height: float = 0.07
    tub_wall: float = 0.004
    cells_x: int = 4
    cells_y: int = 4
    cell_pitch: float = 0.10
    water_depth: float = 0.04
    # ---- mesh box ----
    box_size: float = 0.090
    box_height: float = 0.050
    handle_height: float = 0.030
    handle_dia: float = 0.006
    # ---- exit tray ----
    exit_slots: int = 2
    exit_z: float = 0.30
    exit_gap_y: float = 0.08
    # ---- crane ----
    mast_x: float = -0.06
    rail_y0: float = -0.25
    rail_length: float = 1.10
    mast_length: float = 0.95
    fork_length: float = 0.32
    arm_y_offset: float = 0.055
    guide_rail_z: float = 1.04
    # ---- hook ----
    hook_shank: float = 0.045
    hook_foot: float = 0.022
    hook_lip: float = 0.010
    hook_section: float = 0.007
    engage_drop: float = 0.012
    lift_clear: float = 0.030
    # ---- motion ----
    vmax_rail: float = 0.30
    vmax_mast: float = 0.06
    vmax_arm: float = 0.20
    lead_screw_lead_mm: float = 8.0
    max_screw_rpm: float = 600
    belt_pulley_teeth: int = 20
    microsteps: int = 16
    # ---- home ----
    home_mast: float = 0.45
    # ---- printing ----
    bed_x_mm: float = 180
    bed_y_mm: float = 180
    bed_z_mm: float = 180
    bed_margin_mm: float = 5
    infill_factor: float = 0.45
    # ---- sensors ----
    camera_tiers: tuple[int, ...] = (0,)
    camera_tilt_deg: float = 30
    sensor_nodes: int = 1
    populated_tiers: tuple[int, ...] = (0,)
    hook_camera: bool = True
    hook_camera_tilt_deg: float = 45

    # ------------------------------------------------------------------ load
    @classmethod
    def from_toml(cls, path: str | Path) -> "CraneSpec":
        """Flatten the TOML tables into fields. Unknown keys are an error, so a
        typo in crane.toml is reported instead of silently ignored."""
        data = tomllib.loads(Path(path).read_text())
        flat: dict = {}
        for table in data.values():
            flat.update(table if isinstance(table, dict) else {})
        names = {f.name for f in dataclasses.fields(cls)}
        unknown = sorted(set(flat) - names)
        if unknown:
            raise SpecError([f"unknown key in {path}: {k}" for k in unknown])
        for k, v in list(flat.items()):
            if isinstance(v, list):
                flat[k] = tuple(v)
        return cls(**flat)

    # ------------------------------------------------------------- validate
    def __post_init__(self) -> None:
        p: list[str] = []
        floors = self.tier_floors
        if not floors:
            p.append("tier_floors is empty")
        if list(floors) != sorted(floors):
            p.append(f"tier_floors must ascend, got {floors}")
        inner = self.tub_depth - 2 * self.tub_wall
        if self.cells_x * self.cell_pitch > inner + 1e-9:
            p.append(f"{self.cells_x} rows x {self.cell_pitch} m pitch don't fit the tub depth ({inner:.3f} m inside)")
        inner_y = self.tub_length - 2 * self.tub_wall
        if self.cells_y * self.cell_pitch > inner_y + 1e-9:
            p.append(f"{self.cells_y} columns x {self.cell_pitch} m pitch don't fit the tub length ({inner_y:.3f} m inside)")
        if self.box_size >= self.cell_pitch:
            p.append(f"box_size {self.box_size} must be smaller than cell_pitch {self.cell_pitch}")
        if self.water_depth >= self.box_height:
            p.append("water_depth must be below the box rim, or the crawfish can climb out")
        if self.handle_height <= self.handle_dia:
            p.append("handle bar must stand clear of the box rim")

        # Hook geometry: the cup must get below the bar without the foot touching the rim.
        foot_bottom = self.handle_z(0) - self.handle_dia / 2 - self.engage_drop - self.hook_section
        if foot_bottom < self.box_rim_z(0) + 0.002:
            p.append(f"engage_drop {self.engage_drop} puts the hook foot on the box rim; raise handle_height or cut engage_drop")

        # Z: every tier must be reachable, both to engage and to carry clear of the tub rim.
        lo, hi = self.mast_z_range
        if self.engage_z(0) < lo:
            p.append(f"bottom tier needs the hook at z={self.engage_z(0):.3f} m but the carriage bottoms out at {lo:.3f} m")
        if self.carry_z(len(floors) - 1) > hi:
            p.append(
                f"top tier needs the hook at z={self.carry_z(len(floors) - 1):.3f} m to clear the tub rim, "
                f"but the mast only reaches {hi:.3f} m; lengthen mast_length (and guide_rail_z)"
            )
        if not lo <= self.home_mast <= hi:
            p.append(f"home_mast {self.home_mast} is outside the mast range {lo:.3f}..{hi:.3f}")
        if self.guide_rail_z < self.mast_top_z - 0.001:
            p.append(f"guide_rail_z {self.guide_rail_z} is below the mast top {self.mast_top_z:.3f}")

        # Shelf gap: a carried box, the hook and the arm must pass under the tier above.
        for t in range(len(floors) - 1):
            arm_top = self.carry_z(t) + self.hook_shank + FORK_IN_RACK_H
            ceiling = floors[t + 1] - 0.005
            if arm_top > ceiling:
                p.append(f"tier {t}: carried arm top {arm_top:.3f} m hits the tub above at {ceiling:.3f} m; raise the tier pitch")

        # A carried box must pass over the other boxes' handles in the same tub.
        for t in range(len(floors)):
            box_bottom = self.carry_z(t) + self.handle_dia / 2 - self.box_height - self.handle_height
            handles_top = self.handle_z(t) + self.handle_dia / 2 + 0.003   # bar plus its posts
            if box_bottom - handles_top < 0.006:
                p.append(f"tier {t}: a carried box clears the other handles by only "
                         f"{(box_bottom - handles_top) * 1000:.1f} mm; raise lift_clear")

        # X: retract clear of the rack, reach the far row.
        xlo, xhi = self.arm_x_range
        if self.cell_x(self.cells_x - 1) > xhi:
            p.append(f"far row at x={self.cell_x(self.cells_x - 1):.3f} m is beyond arm reach {xhi:.3f} m; lengthen fork_length")
        if xlo > self.retract_x:
            p.append(f"arm can't retract clear of the rack: min hook x {xlo:.3f} m > needed {self.retract_x:.3f} m")
        if self.fork_front_home + FORK_IDLER_REACH > self.rack_x0 - EXTRUSION - 0.003:
            p.append(f"folded fork front {self.fork_front_home + FORK_IDLER_REACH:.3f} m reaches the rack posts; "
                     "the fork can't fold clear of the rack")
        p += self.fork_problems()
        if self.mast_x + EXTRUSION / 2 + 0.01 > self.rack_x0 - EXTRUSION:
            p.append("mast is too close to the rack posts")

        # Y: every cell and every exit slot under the rail's travel.
        ylo, yhi = self.rail_y_range
        need_lo = self.cell_y(0)
        need_hi = max(self.cell_y(self.cells_y - 1), self.exit_y)
        if need_lo < ylo or need_hi > yhi:
            p.append(f"rail travel {ylo:.3f}..{yhi:.3f} m doesn't cover cells/exit {need_lo:.3f}..{need_hi:.3f} m; lengthen rail_length or move rail_y0")

        # Motion limits that come from the hardware.
        screw_max = self.lead_screw_lead_mm / 1000 * self.max_screw_rpm / 60
        if self.vmax_mast > screw_max + 1e-9:
            p.append(f"vmax_mast {self.vmax_mast} m/s exceeds the lead screw limit {screw_max:.3f} m/s")
        for t in self.camera_tiers:
            if not 0 <= t < len(floors):
                p.append(f"camera tier {t} doesn't exist")
        for t in self.populated_tiers:
            if not 0 <= t < len(floors):
                p.append(f"populated tier {t} doesn't exist")
        if self.sensor_nodes > len(self.populated_tiers):
            p.append("more sensor nodes than populated tubs")
        if self.exit_slots < 1:
            p.append("need at least one exit slot")
        if self.printable_mm <= 50:
            p.append("printer bed is too small for the truck plate")
        if p:
            raise SpecError(p)

    # ------------------------------------------------------------- derived
    # Printer
    @property
    def printable_mm(self) -> float:
        return min(self.bed_x_mm, self.bed_y_mm) - 2 * self.bed_margin_mm

    # Rack and cells
    def tier_floor_top(self, t: int) -> float:
        return self.tier_floors[t] + self.tub_wall

    def box_rim_z(self, t: int) -> float:
        return self.tier_floor_top(t) + self.box_height

    def handle_z(self, t: int) -> float:
        """Centre of the handle bar on tier t."""
        return self.box_rim_z(t) + self.handle_height

    def cell_x(self, r: int) -> float:
        margin = (self.tub_depth - 2 * self.tub_wall - self.cells_x * self.cell_pitch) / 2
        return self.rack_x0 + self.tub_wall + margin + self.cell_pitch * (r + 0.5)

    def cell_y(self, c: int) -> float:
        margin = (self.tub_length - 2 * self.tub_wall - self.cells_y * self.cell_pitch) / 2
        return self.rack_y0 + self.tub_wall + margin + self.cell_pitch * (c + 0.5)

    @property
    def exit_y(self) -> float:
        return self.rack_y0 + self.tub_length + self.exit_gap_y + self.box_size / 2 + 0.01

    def exit_x(self, i: int) -> float:
        return self.rack_x0 + 0.07 + i * (self.box_size + 0.03)

    # Hook targets (cup = top centre of the hook foot)
    def engage_z(self, t: int) -> float:
        return self.handle_z(t) - self.handle_dia / 2 - self.engage_drop

    def carry_z(self, t: int) -> float:
        rise = (self.tier_floors[t] + self.tub_height + self.lift_clear) - self.tier_floor_top(t)
        return self.handle_z(t) - self.handle_dia / 2 + rise

    @property
    def retract_x(self) -> float:
        """Largest cup x with the hook lip still outside the rack posts."""
        return self.rack_x0 - EXTRUSION - 0.005 - self.hook_foot / 2 - self.hook_section

    # Crane geometry
    @property
    def floor_rail_top(self) -> float:
        return 2 * EXTRUSION  # 2040 laid flat on 2020 foot crossbars

    @property
    def truck_top(self) -> float:
        return self.floor_rail_top + TRUCK_GAP + TRUCK_PLATE

    @property
    def mast_top_z(self) -> float:
        return self.truck_top + self.mast_length

    @property
    def rail_y1(self) -> float:
        return self.rail_y0 + self.rail_length

    @property
    def arm_y(self) -> float:
        return self.arm_y_offset

    def arm_bottom_for(self, cup_z: float) -> float:
        return cup_z + self.hook_shank

    # Telescopic fork (all world x, metres, at home = fully folded)
    @property
    def cup_home_x(self) -> float:
        """Folded, the cup sits just inside the retract limit."""
        return self.retract_x - 0.005

    @property
    def fork_front_home(self) -> float:
        """Front end of all three stages when folded (they are flush)."""
        return self.rail_tip_for(self.cup_home_x)

    @property
    def fork_rear_home(self) -> float:
        """Rearmost point of the folded fork, idler arm included."""
        return self.fork_front_home - self.fork_length - FORK_IDLER_REACH

    @property
    def fork_overhang(self) -> float:
        """How far the folded fork sticks out behind the mast's back face."""
        return (self.mast_x - EXTRUSION / 2) - self.fork_rear_home

    @property
    def fork_stage_travel(self) -> float:
        """How far the middle stage can slide. Each block pair must stay on
        its rail, which is the same limit for both stages."""
        return self.fork_length - FORK_BLOCK_SPAN - 2 * FORK_END_MARGIN

    @property
    def carriage_block_x(self) -> tuple[float, float]:
        """Centres of the two fixed blocks at the carriage's front (rear, front)."""
        front = self.fork_front_home - FORK_END_MARGIN - MGN12H_BLOCK_L / 2
        return (front - FORK_BLOCK_PITCH, front)

    @property
    def fork_pinion_x(self) -> float:
        """The X pinion on the carriage, which drives the middle stage's belt rack."""
        return self.mast_x + 0.040

    @property
    def fork_anchor_x(self) -> tuple[float, float]:
        """Carriage-side ends of the (retract, extend) differential belts."""
        return (self.mast_x + 0.030, self.fork_front_home - 0.0275)

    def fork_problems(self) -> list[str]:
        """Every belt end, idler and pinion must stay on the right side of the
        things it passes, over the whole stroke."""
        p = []
        u = self.fork_stage_travel
        rear0 = self.fork_front_home - self.fork_length
        rack = (rear0 + u + 0.014 + 0.008, self.fork_front_home - 0.014 - 0.008)   # pinion must stay between the rack clamps
        if not rack[0] <= self.fork_pinion_x <= rack[1]:
            p.append(f"X pinion at {self.fork_pinion_x:.3f} m runs off the belt rack ({rack[0]:.3f}..{rack[1]:.3f}); shorten the stroke or move the pinion")
        retract, extend = self.fork_anchor_x
        rear_idler_max = rear0 + u - 0.012 + 0.009
        if retract <= rear_idler_max:
            p.append("retract belt anchor is behind the rear idler at full reach")
        if extend >= self.fork_front_home + 0.012 - 0.009:
            p.append("extend belt anchor is in front of the front idler when folded")
        if extend <= retract:
            p.append("extend belt anchor must be in front of the retract belt anchor")
        if u <= 0:
            p.append("fork_length is too short for two block pairs")
        return p

    def rail_tip_for(self, cup_x: float) -> float:
        return cup_x - self.hook_foot / 2 - self.hook_section / 2 + 0.005

    # Joint ranges: world coordinates of the cup (metres)
    @property
    def rail_y_range(self) -> tuple[float, float]:
        lo = self.rail_y0 + RAIL_END_BLOCK + TRUCK_LENGTH / 2 + 0.005
        hi = self.rail_y1 - RAIL_END_BLOCK - TRUCK_LENGTH / 2 - 0.005
        return (lo + self.arm_y, hi + self.arm_y)

    @property
    def mast_z_range(self) -> tuple[float, float]:
        carriage_bottom_min = self.truck_top + Z_DRIVE_STACK
        carriage_top_max = self.mast_top_z - MAST_TOP_BLOCK
        arm_bottom_min = carriage_bottom_min + CARRIAGE_BELOW_ARM
        arm_bottom_max = carriage_top_max - (CARRIAGE_H - CARRIAGE_BELOW_ARM)
        return (arm_bottom_min - self.hook_shank, arm_bottom_max - self.hook_shank)

    @property
    def arm_x_range(self) -> tuple[float, float]:
        lo = self.cup_home_x
        return (lo, lo + FORK_RATIO * self.fork_stage_travel)

    @property
    def arm_mid_range(self) -> tuple[float, float]:
        """The middle stage's joint, expressed in the same units as arm_x so
        that it is ref + (arm_x - ref) / 2 (see Joint.follows)."""
        lo = self.cup_home_x
        return (lo, lo + self.fork_stage_travel)

    @property
    def home(self) -> dict[str, float]:
        """Home pose after homing: rail at its start, arm retracted, mast parked."""
        return {
            "rail_y": self.rail_y_range[0],
            "mast_z": self.home_mast,
            "arm_x": self.arm_x_range[0],
        }

    @property
    def rack_post_height(self) -> float:
        return self.tier_floors[-1] + self.tub_height + 0.02

    def summary(self) -> dict:
        """Derived numbers Rookery reads (written to crane.json)."""
        return {
            "joints": {
                "rail_y": {"range": self.rail_y_range, "home": self.home["rail_y"], "vmax": self.vmax_rail},
                "mast_z": {"range": self.mast_z_range, "home": self.home["mast_z"], "vmax": self.vmax_mast},
                "arm_x": {"range": self.arm_x_range, "home": self.home["arm_x"], "vmax": self.vmax_arm},
                "lift": {"range": (0.0, 0.0), "home": 0.0, "note": "locked: engage with a mast_z move of engage_drop"},
                "arm_mid": {"range": self.arm_mid_range, "home": self.home["arm_x"],
                            "note": "fork middle stage, not driven: arm_mid = home + (arm_x - home) / 2"},
            },
            "fork": {
                "length": self.fork_length,
                "stage_travel": self.fork_stage_travel,
                "folded_front_x": self.fork_front_home,
                "folded_rear_x": self.fork_rear_home,
                "overhang_behind_mast": self.fork_overhang,
            },
            "cells": {
                f"{t}_{r}_{c}": {
                    "x": self.cell_x(r), "y": self.cell_y(c),
                    "engage_z": self.engage_z(t), "carry_z": self.carry_z(t),
                }
                for t in range(len(self.tier_floors))
                for r in range(self.cells_x)
                for c in range(self.cells_y)
            },
            "exit_slots": [{"x": self.exit_x(i), "y": self.exit_y, "z": self.exit_z} for i in range(self.exit_slots)],
            "retract_x": self.retract_x,
            "engage_drop": self.engage_drop,
        }
