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
  │ preset: None | "drop"                 │
  ├───────────────────────────────────────┤
  │ + cells / tray_size / water_z         │  derived (@property)
  │ + cell_center(r, c) / handle_top_z    │
  │ - __post_init__: collect, then raise  │
  └───────────────────────────────────────┘
          ▲ reads                  ▲ reads
     cli.py (argparse)       builder.py (2.3+)
```

Frame: the tray floor's top surface is ``z = 0`` and the grid is centred on
the origin. Rows run along +X and columns along +Y. Units are metres.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

#: Presets the builder knows. ``"drop"`` is the Phase 1 plane-and-box scene (row 2.7).
PRESETS: tuple[str, ...] = ("drop",)


class SpecError(ValueError):
    """A ``SceneSpec`` that cannot be built. ``problems`` lists every reason."""

    def __init__(self, problems: list[str]) -> None:
        self.problems = problems
        super().__init__("scene spec won't work:\n  - " + "\n  - ".join(problems))


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
    #: How far the hook's handle rises above the top of a box.
    handle_height: float = 0.03
    #: Clear vertical space between the top of a tray's handles and the
    #: underside of the tray above it. Matters once ``trays > 1`` (Phase 10).
    shelf_gap: float = 0.08
    #: Exit tray position (x, y, z) in the tray frame. It sits beside the rail
    #: line on the aisle side (-X) and is placed for real in row 2.5.
    exit_tray_pose: tuple[float, float, float] = (-0.45, 0.0, 0.10)
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
        object.__setattr__(self, "exit_tray_pose", tuple(float(v) for v in self.exit_tray_pose))

        p: list[str] = []
        for name in ("trays", "rows", "cols"):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, int) or v < 1:
                p.append(f"{name} must be a whole number >= 1, got {v!r}")
        if len(self.box_size) != 3:
            p.append(f"box_size must be (width, depth, height), got {self.box_size!r}")
        if len(self.exit_tray_pose) != 3:
            p.append(f"exit_tray_pose must be (x, y, z), got {self.exit_tray_pose!r}")
        lengths = {
            "cell_pitch": self.cell_pitch,
            "water_depth": self.water_depth,
            "handle_height": self.handle_height,
            "shelf_gap": self.shelf_gap,
        }
        lengths.update({f"box_size[{i}]": v for i, v in enumerate(self.box_size)})
        for name, v in lengths.items():
            if not v > 0:  # also catches NaN
                p.append(f"{name} must be positive, got {v!r}")
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
        if p:
            raise SpecError(p)
