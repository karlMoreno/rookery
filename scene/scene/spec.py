"""``SceneSpec``: every number that shapes the generated scene (row 2.1).

Pattern: **Parameter Object**, validated at construction. One frozen dataclass
holds the *independent* numbers. The builder (2.3–2.5), the CLI and the gates
all read the same object, so they cannot drift apart. Anything *derived*
(cell centres, the water plane, handle heights) is a property, so a spec has
exactly one way to say each thing. ``__post_init__`` collects every problem
and raises once with the whole list, so one run tells you everything that is
wrong. This idea was carried over from the throwaway preview's ``Spec``.

```text
  ┌───────────────────────────────────────┐
  │ «frozen dataclass» SceneSpec          │
  ├───────────────────────────────────────┤
  │ trays, rows, cols          (counts)   │
  │ cell_pitch, water_depth    (m)        │
  │ box_size (w, d, h)         (m)        │
  │ handle_height, shelf_gap   (m)        │
  │ exit_tray_pose (x, y, z)   (m)        │
  │ rail_x, guide_rail_height  (m)        │
  │ handle_dia, hook_* (m), contact       │
  │ preset: None | "drop"                 │
  ├───────────────────────────────────────┤
  │ + cells / tray_size / water_z         │  derived (@property)
  │ + cell_center(r, c) / handle_top_z    │
  │ + exit_pose / rail_line_x (auto)      │
  │ - __post_init__: collect, then raise  │
  └───────────────────────────────────────┘
          ▲ reads                  ▲ reads
     cli.py (argparse)       builder.py (2.3+)
```

Frame: the tray floor's top surface is ``z = 0`` and the grid is centred on
the origin. Rows run along +X and columns along +Y. Units are metres.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

#: Minimum gap between neighbouring fixed parts (tray, exit tray, rail), in metres.
CLEARANCE = 0.02
#: Slack on geometric comparisons, so an automatic placement exactly at a
#: limit is not rejected for float rounding.
EPS = 1e-9

#: Box wall thickness, mirrored from builder.BOX_WALL for the crawfish fit check.
BOX_WALL_FOR_SPEC = 0.003

#: Room the J-hook needs above and below its envelope under a handle, metres.
MIN_HOOK_CLEARANCE = 0.004

#: Default height of the exit tray's top surface, in metres.
EXIT_TRAY_Z = 0.10
#: Default distance from the exit tray's aisle edge to the floor rail, in metres.
RAIL_SETBACK = 0.12

#: Presets the builder knows. ``"drop"`` is the Phase 1 plane-and-box scene (row 2.7).
PRESETS: tuple[str, ...] = ("drop",)


class SpecError(ValueError):
    """A ``SceneSpec`` that cannot be built. ``problems`` lists every reason."""

    def __init__(self, problems: list[str]) -> None:
        self.problems = problems
        super().__init__("scene spec won't work:\n  - " + "\n  - ".join(problems))


@dataclass(frozen=True)
class ContactParams:
    """MuJoCo contact settings for the hook, the handles and the box floors
    (row 5.1). ``None`` leaves MuJoCo's default and writes nothing to the
    MJCF, so the default ``ContactParams()`` reproduces the Phase 4 scene
    byte for byte. Row 5.6's sweep picks real values."""

    #: Contact dimensionality: 1 (frictionless), 3 (sliding), 4, or 6.
    condim: int | None = None
    #: (sliding, torsional, rolling) friction.
    friction: tuple[float, float, float] | None = None
    #: (timeconst, dampratio).
    solref: tuple[float, float] | None = None
    #: (dmin, dmax, width, midpoint, power).
    solimp: tuple[float, float, float, float, float] | None = None

    def __post_init__(self) -> None:
        for name in ("friction", "solref", "solimp"):
            v = getattr(self, name)
            if v is not None:
                object.__setattr__(self, name, tuple(float(x) for x in v))

    def problems(self) -> list[str]:
        p: list[str] = []
        if self.condim is not None and self.condim not in (1, 3, 4, 6):
            p.append(f"contact.condim must be 1, 3, 4 or 6, got {self.condim!r}")
        for name, n in (("friction", 3), ("solref", 2), ("solimp", 5)):
            v = getattr(self, name)
            if v is not None and len(v) != n:
                p.append(f"contact.{name} needs {n} numbers, got {v!r}")
        return p


@dataclass(frozen=True)
class SceneSpec:
    """The independent numbers of one Rookery scene. See the module docs."""

    trays: int = 1
    rows: int = 4
    cols: int = 4
    #: Centre-to-centre spacing of the cells, the same along X and Y.
    cell_pitch: float = 0.15
    #: Water depth above the tray floor.
    water_depth: float = 0.05
    #: Outer size of one mesh box: (width X, depth Y, height Z).
    box_size: tuple[float, float, float] = (0.12, 0.12, 0.08)
    #: How far the hook's handle rises above the top of a box. 4.5 cm leaves a
    #: 3.3 cm gap under the handle (12 mm capsule), so the J-hook (8 mm finger
    #: with a 15 mm upturned tip, a 23 mm envelope) slides under it with 5 mm
    #: to spare above and below. At 3 cm the gap was 18 mm and the tip struck
    #: the handle (found running the row 3.5 tour).
    handle_height: float = 0.045
    #: Handle capsule diameter.
    handle_dia: float = 0.012
    #: J-hook: finger length along +X from the arm tip, its capsule radius,
    #: and how far the tip turns up above the finger's centreline. These are
    #: copied in crates/mast/src/kinematics.rs; test_gantry.py fails on drift.
    hook_finger: float = 0.04
    hook_radius: float = 0.004
    hook_tip_rise: float = 0.015
    #: The crawfish in each box (row 5.2): a capsule lying along X on the
    #: box floor. Length is the capsule's full length, tips included.
    crawfish: bool = True
    crawfish_length: float = 0.08
    crawfish_radius: float = 0.012
    crawfish_mass: float = 0.03
    #: Contact settings for the hook, handles and box floors (row 5.1).
    contact: ContactParams = field(default_factory=ContactParams)
    #: Clear vertical space between the top of a tray's handles and the
    #: underside of the tray above it. Matters once ``trays > 1`` (Phase 10).
    shelf_gap: float = 0.08
    #: Centre of the exit tray's top surface (x, y, z) in the tray frame. It
    #: sits on the aisle side (-X), between the tray and the floor rail.
    #: ``None`` (the default) places it automatically: ``CLEARANCE`` off the
    #: tray's aisle edge, at y = 0, z = ``EXIT_TRAY_Z``. Read the resolved
    #: value from ``exit_pose``.
    exit_tray_pose: tuple[float, float, float] | None = None
    #: X of the crane's floor rail, which runs along Y on the aisle side. The
    #: stacker crane (Phase 3) rides it. ``None`` puts it ``RAIL_SETBACK``
    #: beyond the exit tray. Read the resolved value from ``rail_line_x``.
    rail_x: float | None = None
    #: Height of the underside of the top guide rail that keeps the mast upright.
    guide_rail_height: float = 1.0
    #: ``None`` builds the farm. A name from ``PRESETS`` builds that scene instead.
    preset: Literal["drop"] | None = None

    # ---------------- derived (change the inputs, not these) ----------------

    @property
    def cells(self) -> int:
        """Cells per tray."""
        return self.rows * self.cols

    @property
    def tray_size(self) -> tuple[float, float]:
        """Inner floor of the tray (X, Y): the grid plus half a pitch of margin on each side."""
        return (self.rows * self.cell_pitch, self.cols * self.cell_pitch)

    @property
    def water_z(self) -> float:
        """Height of the water plane in the tray frame."""
        return self.water_depth

    @property
    def handle_top_z(self) -> float:
        """Height of a handle's top in the tray frame (boxes rest on the floor)."""
        return self.box_size[2] + self.handle_height

    @property
    def hook_envelope(self) -> float:
        """The hook's height from the finger's underside to the top of its tip."""
        return 2 * self.hook_radius + self.hook_tip_rise

    @property
    def hook_clearance(self) -> float:
        """Room above and below the hook's envelope when it is centred under a handle."""
        return (self.handle_height - self.handle_dia - self.hook_envelope) / 2

    @property
    def exit_pose(self) -> tuple[float, float, float]:
        """Resolved exit tray pose: ``exit_tray_pose`` or the automatic placement."""
        if self.exit_tray_pose is not None:
            return self.exit_tray_pose
        x = -self.tray_size[0] / 2 - CLEARANCE - self.exit_tray_half
        return (x, 0.0, EXIT_TRAY_Z)

    @property
    def rail_line_x(self) -> float:
        """Resolved floor-rail X: ``rail_x`` or ``RAIL_SETBACK`` beyond the exit tray."""
        if self.rail_x is not None:
            return self.rail_x
        return self.exit_pose[0] - self.exit_tray_half - RAIL_SETBACK

    @property
    def exit_tray_half(self) -> float:
        """Half-side of the square exit tray: one cell, so it holds one box."""
        return self.cell_pitch / 2

    @property
    def rail_length(self) -> float:
        """Floor rail length along Y: the tray plus one pitch of overrun at each end."""
        return self.tray_size[1] + 2 * self.cell_pitch

    def cell_center(self, r: int, c: int) -> tuple[float, float]:
        """(x, y) of cell ``(r, c)``, counted from 0, with the grid centred on the origin."""
        if not (0 <= r < self.rows and 0 <= c < self.cols):
            raise IndexError(f"cell ({r}, {c}) is outside a {self.rows}x{self.cols} grid")
        x = (r - (self.rows - 1) / 2) * self.cell_pitch
        y = (c - (self.cols - 1) / 2) * self.cell_pitch
        return (x, y)

    # ---------------- validation ----------------

    def __post_init__(self) -> None:
        # Normalise sequences to tuples so equal specs hash and compare equal,
        # however they were passed in (lists from argparse or TOML).
        object.__setattr__(self, "box_size", tuple(float(v) for v in self.box_size))
        if self.exit_tray_pose is not None:
            object.__setattr__(self, "exit_tray_pose", tuple(float(v) for v in self.exit_tray_pose))
        if isinstance(self.contact, dict):  # read back from rookery.json
            object.__setattr__(self, "contact", ContactParams(**self.contact))

        p: list[str] = []
        for name in ("trays", "rows", "cols"):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, int) or v < 1:
                p.append(f"{name} must be a whole number >= 1, got {v!r}")
        if len(self.box_size) != 3:
            p.append(f"box_size must be (width, depth, height), got {self.box_size!r}")
        if self.exit_tray_pose is not None and len(self.exit_tray_pose) != 3:
            p.append(f"exit_tray_pose must be (x, y, z), got {self.exit_tray_pose!r}")
        lengths = {
            "cell_pitch": self.cell_pitch,
            "water_depth": self.water_depth,
            "handle_height": self.handle_height,
            "shelf_gap": self.shelf_gap,
            "guide_rail_height": self.guide_rail_height,
            "handle_dia": self.handle_dia,
            "hook_finger": self.hook_finger,
            "hook_radius": self.hook_radius,
            "hook_tip_rise": self.hook_tip_rise,
            "crawfish_length": self.crawfish_length,
            "crawfish_radius": self.crawfish_radius,
            "crawfish_mass": self.crawfish_mass,
        }
        lengths.update({f"box_size[{i}]": v for i, v in enumerate(self.box_size)})
        for name, v in lengths.items():
            if not v > 0:  # also catches NaN
                p.append(f"{name} must be positive, got {v!r}")
        p += self.contact.problems()
        if self.preset is not None and self.preset not in PRESETS:
            p.append(f"preset {self.preset!r} is unknown; choose one of {list(PRESETS)} or none")

        # The two plan checks only make sense once the numbers above are sane.
        if not p:
            w, d, _ = self.box_size
            if max(w, d) >= self.cell_pitch:
                p.append(
                    f"box footprint {w:.3f} x {d:.3f} must be smaller than cell_pitch "
                    f"{self.cell_pitch:.3f}, or neighbouring boxes touch"
                )
            if self.handle_top_z <= self.water_z:
                p.append(
                    f"handle top {self.handle_top_z:.3f} must be above the water plane "
                    f"{self.water_z:.3f}, or the hook has to reach into the water; "
                    "raise box_size height or handle_height, or lower water_depth"
                )
            # The J-hook must fit under a handle with room to spare (5.1; the
            # 3.5 tour found a 3 cm handle left no room at all). Presets such
            # as the drop scene have no hook or handles, so they skip this.
            if self.preset is None and self.hook_clearance < MIN_HOOK_CLEARANCE - EPS:
                p.append(
                    f"the hook ({self.hook_envelope * 1000:.0f} mm envelope) leaves "
                    f"{self.hook_clearance * 1000:.1f} mm under a {self.handle_dia * 1000:.0f} mm handle "
                    f"at handle_height {self.handle_height:.3f}; it needs {MIN_HOOK_CLEARANCE * 1000:.0f} mm "
                    "above and below. Raise handle_height or shrink the hook"
                )
            if self.preset is None and self.crawfish:
                inner = min(self.box_size[0], self.box_size[1]) - 2 * BOX_WALL_FOR_SPEC
                if self.crawfish_length > inner - 2 * CLEARANCE / 4:
                    p.append(f"crawfish_length {self.crawfish_length:.3f} does not fit inside the box ({inner:.3f})")
                if self.crawfish_length < 2 * self.crawfish_radius:
                    p.append("crawfish_length must be at least twice crawfish_radius")
                if 2 * self.crawfish_radius >= self.box_size[2]:
                    p.append("the crawfish is taller than the box walls")
            if self.preset is None and self.hook_finger >= min(self.box_size[0], self.box_size[1]) / 2:
                p.append(f"hook_finger {self.hook_finger:.3f} must be shorter than half the box")
            # Aisle side, from -X inwards: floor rail | exit tray | tray.
            ex, ey, ez = self.exit_pose
            half = self.exit_tray_half
            tray_edge = -self.tray_size[0] / 2
            if ex + half > tray_edge - CLEARANCE + EPS:
                p.append(
                    f"exit tray (x {ex - half:.3f}..{ex + half:.3f}) overlaps the tray, whose aisle edge "
                    f"is at x {tray_edge:.3f}; it needs {CLEARANCE:.3f} m clear"
                )
            if self.rail_line_x > ex - half - CLEARANCE + EPS:
                p.append(
                    f"rail_x {self.rail_line_x:.3f} must be on the aisle side of the exit tray "
                    f"(x < {ex - half - CLEARANCE:.3f})"
                )
            if abs(ey) + half > self.rail_length / 2 + EPS:
                p.append(f"exit tray y {ey:.3f} is beyond the rail's reach (±{self.rail_length / 2:.3f})")
            if ez <= 0:
                p.append(f"exit tray z {ez:.3f} must be above the tray floor (z > 0)")
            tallest = max(self.handle_top_z, ez + self.handle_top_z)
            if self.guide_rail_height < tallest + 0.1:
                p.append(
                    f"guide_rail_height {self.guide_rail_height:.3f} must clear the tallest handle "
                    f"({tallest:.3f}) by 0.1 m"
                )
        if p:
            raise SpecError(p)
