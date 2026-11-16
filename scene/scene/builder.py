"""``MjcfBuilder``: turns a ``SceneSpec`` into MJCF text (rows 2.3–2.5).

Pattern: **Builder**. Each step adds one part of the scene and returns the
builder, so a scene reads as one chain, and ``build()`` serialises it once
at the end:

```text
  MjcfBuilder(spec)
    .header()      option, visual, assets, light, camera, ground   (2.3)
    .tray()        body tray_0: floor + 4 walls                    (2.3)
    .water()       geom water: translucent, contact-free           (2.3)
    .cells()       box_r_c + handle_r_c, row-major                 (2.4)
    .exit_tray()   geom exit_tray beside the rail                  (2.5)
    .build() -> str
```

Why a builder rather than one big template: every part depends on the spec,
the order of children is decided in one place (which 2.5's byte-identical
output needs), and a later phase (stacked trays, the crane frame) adds a
step instead of editing a string.

The XML is built with ``xml.etree.ElementTree``. Children are appended in
call order and attributes in insertion order, and every float goes through
:func:`num`, so the output does not depend on dict order or float repr.

Frame (from ``SceneSpec``): the tray floor's top surface is z = 0, and the
grid is centred on the origin.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Iterable

from .spec import SceneSpec

#: Wall and floor thickness of the tray, in metres.
TRAY_WALL = 0.01
#: How far the tray walls rise above the water line, in metres.
TRAY_FREEBOARD = 0.03
#: Water colour: translucent blue, so the boxes under it stay visible.
WATER_RGBA = (0.25, 0.55, 0.85, 0.35)
TRAY_RGBA = (0.55, 0.55, 0.58, 1.0)

#: Mesh box: wall and floor thickness, total mass, colours.
BOX_WALL = 0.003
BOX_FLOOR = 0.004
BOX_MASS = 0.12  # kg, an empty plastic mesh box
BOX_RGBA = (0.85, 0.45, 0.15, 1.0)
#: Hook handle: capsule radius and the radius of the two posts holding it.
HANDLE_RADIUS = 0.006
POST_RADIUS = 0.003
HANDLE_RGBA = (0.9, 0.85, 0.2, 1.0)
#: Contact margin on box walls, so neighbours and tray walls are seen
#: slightly before they touch (no tunnelling at 1 ms).
BOX_MARGIN = 0.001


def num(*xs: float) -> str:
    """Floats as MJCF text: fixed 6 decimals, space-separated, no ``-0.000000``."""
    return " ".join(f"{(x + 0.0):.6f}" if abs(x) >= 5e-7 else "0.000000" for x in xs)


class MjcfBuilder:
    """Accumulates one ``<mujoco>`` tree from a ``SceneSpec``. See the module docs."""

    def __init__(self, spec: SceneSpec) -> None:
        self.spec = spec
        self.root = ET.Element("mujoco", {"model": "rookery"})
        self._names: set[str] = set()
        self._worldbody: ET.Element | None = None

    # ---------------- helpers ----------------

    def _el(self, parent: ET.Element, tag: str, attrs: Iterable[tuple[str, str]] = ()) -> ET.Element:
        """Append ``<tag>`` with attributes in exactly the given order. Names must be unique."""
        el = ET.SubElement(parent, tag)
        for k, v in attrs:
            if k == "name":
                if v in self._names:
                    raise ValueError(f"duplicate MJCF name {v!r}")
                self._names.add(v)
            el.set(k, v)
        return el

    @property
    def worldbody(self) -> ET.Element:
        if self._worldbody is None:
            raise RuntimeError("call header() first")
        return self._worldbody

    # ---------------- steps ----------------

    def header(self) -> MjcfBuilder:
        """Physics options, a light, an overview camera and a ground plane under the tray."""
        s = self.spec
        self._el(self.root, "option", [("timestep", num(0.001)), ("gravity", num(0, 0, -9.81))])
        visual = self._el(self.root, "visual")
        self._el(visual, "global", [("offwidth", "1280"), ("offheight", "960")])
        self._worldbody = self._el(self.root, "worldbody")
        wb = self._worldbody
        self._el(wb, "light", [("name", "sun"), ("pos", num(0, 0, 2)), ("dir", num(0, 0, -1)),
                               ("diffuse", num(0.8, 0.8, 0.8))])
        # Looks down at the tray from the aisle side (-X), far enough back to
        # frame the whole tray whatever the grid size.
        span = max(s.tray_size) + 2 * TRAY_WALL
        self._el(wb, "camera", [("name", "overview"), ("pos", num(-1.1 * span, 0, 1.1 * span)),
                                ("xyaxes", num(0, -1, 0, 0.7071, 0, 0.7071))])
        self._el(wb, "geom", [("name", "ground"), ("type", "plane"), ("size", num(2, 2, 0.1)),
                              ("pos", num(0, 0, -TRAY_WALL)), ("rgba", num(0.3, 0.32, 0.3, 1))])
        return self

    def tray(self) -> MjcfBuilder:
        """Body ``tray_0``: a floor slab under z = 0 and four walls around the grid."""
        s = self.spec
        hx, hy = s.tray_size[0] / 2, s.tray_size[1] / 2
        t = TRAY_WALL
        wall_h = s.water_depth + TRAY_FREEBOARD
        rgba = ("rgba", num(*TRAY_RGBA))
        body = self._el(self.worldbody, "body", [("name", "tray_0"), ("pos", num(0, 0, 0))])
        self._el(body, "geom", [("name", "tray_0_floor"), ("type", "box"),
                                ("size", num(hx + t, hy + t, t / 2)), ("pos", num(0, 0, -t / 2)), rgba])
        # Fixed order: -X, +X, -Y, +Y. The X walls span the full outer length.
        walls = (
            ("xneg", (-(hx + t / 2), 0), (t / 2, hy + t)),
            ("xpos", (hx + t / 2, 0), (t / 2, hy + t)),
            ("yneg", (0, -(hy + t / 2)), (hx, t / 2)),
            ("ypos", (0, hy + t / 2), (hx, t / 2)),
        )
        for side, (x, y), (sx, sy) in walls:
            self._el(body, "geom", [("name", f"tray_0_wall_{side}"), ("type", "box"),
                                    ("size", num(sx, sy, wall_h / 2)), ("pos", num(x, y, wall_h / 2)), rgba])
        return self

    def water(self) -> MjcfBuilder:
        """Geom ``water``: a translucent box filling the tray to ``water_depth``.

        Its top face is the water plane (the gate reads ``pos.z + size.z``).
        ``contype=0 conaffinity=0`` means it never collides; it is visual only.
        """
        s = self.spec
        hx, hy = s.tray_size[0] / 2, s.tray_size[1] / 2
        d = s.water_depth
        self._el(self.worldbody, "geom", [
            ("name", "water"), ("type", "box"), ("size", num(hx, hy, d / 2)), ("pos", num(0, 0, d / 2)),
            ("contype", "0"), ("conaffinity", "0"), ("rgba", num(*WATER_RGBA)),
        ])
        return self

    def cells(self) -> MjcfBuilder:
        """One mesh box per cell, emitted row-major: ``box_0_0, box_0_1, ...``.

        Each is a body ``box_r_c`` on a free joint with its floor resting on
        the tray floor (z = 0). It has four thin walls and a hook handle: a
        capsule ``handle_r_c`` across the top along Y, whose top sits at
        ``spec.handle_top_z``, held by two posts ``post_r_c_{yneg,ypos}``.

        ```text
              handle_r_c (capsule, along Y)   top at box_h + handle_height
              ┌──────────────┐
              │ post    post │
          ┌───┴──────────────┴───┐  box top, z = box_h
          │  walls ×4, mesh box  │
          └──────────────────────┘  floor, z = 0 (on the tray floor)
        ```
        """
        s = self.spec
        w, d, h = s.box_size
        hx, hy = w / 2, d / 2
        t, f = BOX_WALL, BOX_FLOOR
        parts = 1 + 4  # floor + walls share the mass evenly by count
        m = BOX_MASS / parts
        rgba = ("rgba", num(*BOX_RGBA))
        handle_z = s.handle_top_z - HANDLE_RADIUS
        handle_half = hy - t - HANDLE_RADIUS
        post_half = (handle_z - h) / 2
        for r in range(s.rows):
            for c in range(s.cols):
                x, y = s.cell_center(r, c)
                tag = f"{r}_{c}"
                body = self._el(self.worldbody, "body", [("name", f"box_{tag}"), ("pos", num(x, y, 0))])
                self._el(body, "freejoint", [("name", f"box_{tag}_free")])
                self._el(body, "geom", [("name", f"box_{tag}_floor"), ("type", "box"),
                                        ("size", num(hx, hy, f / 2)), ("pos", num(0, 0, f / 2)),
                                        ("mass", num(m)), rgba])
                walls = (
                    ("xneg", (-(hx - t / 2), 0), (t / 2, hy)),
                    ("xpos", (hx - t / 2, 0), (t / 2, hy)),
                    ("yneg", (0, -(hy - t / 2)), (hx - t, t / 2)),
                    ("ypos", (0, hy - t / 2), (hx - t, t / 2)),
                )
                for side, (wx, wy), (sx, sy) in walls:
                    self._el(body, "geom", [("name", f"box_{tag}_wall_{side}"), ("type", "box"),
                                            ("size", num(sx, sy, (h - f) / 2)),
                                            ("pos", num(wx, wy, f + (h - f) / 2)),
                                            ("mass", num(m)), ("margin", num(BOX_MARGIN)), rgba])
                for side, py in (("yneg", -handle_half), ("ypos", handle_half)):
                    self._el(body, "geom", [("name", f"post_{tag}_{side}"), ("type", "cylinder"),
                                            ("size", num(POST_RADIUS, post_half)),
                                            ("pos", num(0, py, h + post_half)),
                                            ("mass", num(0.002)), ("rgba", num(*HANDLE_RGBA))])
                self._el(body, "geom", [("name", f"handle_{tag}"), ("type", "capsule"),
                                        ("fromto", num(0, -handle_half, handle_z, 0, handle_half, handle_z)),
                                        ("size", num(HANDLE_RADIUS)), ("mass", num(0.005)),
                                        ("rgba", num(*HANDLE_RGBA))])
        return self

    def build(self) -> str:
        """Serialise: two-space indent, a trailing newline, and no XML declaration or timestamp."""
        tree = ET.ElementTree(self.root)
        ET.indent(tree, space="  ")
        return ET.tostring(self.root, encoding="unicode") + "\n"


def build_scene(spec: SceneSpec) -> str:
    """The farm scene for ``spec``, as far as the builder goes today."""
    return MjcfBuilder(spec).header().tray().water().cells().build()
