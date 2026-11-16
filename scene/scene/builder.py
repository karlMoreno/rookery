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
    .exit_tray()   geom exit_tray between tray and rail            (2.5)
    .crane_frame() floor_rail + guide_rail (static, contact-free)  (2.5)
  MjcfBuilder(spec).drop().build()   the Phase 1 scene, --preset drop (2.7)
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

import json
import xml.etree.ElementTree as ET
from collections.abc import Iterable
from dataclasses import asdict

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
EXIT_RGBA = (0.2, 0.6, 0.3, 1.0)
#: Drop preset: cube edge and release height, in metres (Phase 1 gate: rest at DROP_BOX / 2).
DROP_BOX = 0.1
DROP_HEIGHT = 0.5
#: Half-section of the floor and guide rails, in metres.
RAIL_HALF = 0.02
RAIL_RGBA = (0.35, 0.37, 0.42, 1.0)
#: Contact margin on box walls, so neighbours and tray walls are seen
#: slightly before they touch (no tunnelling at 1 ms).
BOX_MARGIN = 0.001


def num(*xs: float) -> str:
    """Floats as MJCF text: fixed 6 decimals, space-separated, no ``-0.000000``."""
    return " ".join(f"{(x + 0.0):.6f}" if abs(x) >= 5e-7 else "0.000000" for x in xs)


def look_at(eye: tuple[float, float, float], target: tuple[float, float, float]) -> list[tuple[str, str]]:
    """Camera ``pos`` and ``xyaxes`` that look from ``eye`` at ``target`` with +Z up."""
    fx, fy, fz = (t - e for t, e in zip(target, eye, strict=True))
    n = (fx * fx + fy * fy + fz * fz) ** 0.5
    fx, fy, fz = fx / n, fy / n, fz / n
    rx, ry, rz = fy * 1 - fz * 0, fz * 0 - fx * 1, 0.0  # right = forward x up(0, 0, 1)
    n = (rx * rx + ry * ry) ** 0.5
    rx, ry = rx / n, ry / n
    ux, uy, uz = ry * fz - rz * fy, rz * fx - rx * fz, rx * fy - ry * fx  # up = right x forward
    return [("pos", num(*eye)), ("xyaxes", num(rx, ry, rz, ux, uy, uz))]


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
        # Three-quarter view from the -Y end, looking at the middle of the
        # scene (tray to rail), far enough back to frame everything.
        cx = (s.tray_size[0] / 2 + s.rail_line_x) / 2
        span = max(s.tray_size[1], s.tray_size[0] / 2 - s.rail_line_x) + 2 * TRAY_WALL
        self._el(wb, "camera", [("name", "overview"),
                                *look_at((cx + 0.35 * span, -1.25 * span, 1.0 * span), (cx, 0, 0.15))])
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

    def exit_tray(self) -> MjcfBuilder:
        """Geom ``exit_tray``: a square plate (one cell) whose top is at ``spec.exit_pose``,
        on a leg down to the ground. Static, so it is part of the world body."""
        s = self.spec
        x, y, z = s.exit_pose
        half = s.exit_tray_half
        t = TRAY_WALL
        rgba = ("rgba", num(*EXIT_RGBA))
        self._el(self.worldbody, "geom", [("name", "exit_tray"), ("type", "box"),
                                          ("size", num(half, half, t / 2)), ("pos", num(x, y, z - t / 2)), rgba])
        leg_bottom = -TRAY_WALL  # the ground plane
        leg_half = (z - t - leg_bottom) / 2
        self._el(self.worldbody, "geom", [("name", "exit_tray_leg"), ("type", "cylinder"),
                                          ("size", num(0.015, leg_half)), ("pos", num(x, y, leg_bottom + leg_half)),
                                          rgba])
        return self

    def crane_frame(self) -> MjcfBuilder:
        """The static rails of the stacker crane, carried over from the preview.

        ``floor_rail`` runs along Y at ``spec.rail_line_x`` on the ground and carries the
        crane's truck. ``guide_rail`` runs above it at ``guide_rail_height``
        and keeps the mast upright, held by a post at each end. All of it is
        contact-free in Phase 2. The moving crane arrives in Phase 3.
        """
        s = self.spec
        half_len = s.rail_length / 2
        g = -TRAY_WALL  # ground plane height
        rgba = ("rgba", num(*RAIL_RGBA))
        free = [("contype", "0"), ("conaffinity", "0")]
        self._el(self.worldbody, "geom", [("name", "floor_rail"), ("type", "box"),
                                          ("size", num(RAIL_HALF, half_len, RAIL_HALF)),
                                          ("pos", num(s.rail_line_x, 0, g + RAIL_HALF)), *free, rgba])
        self._el(self.worldbody, "geom", [("name", "guide_rail"), ("type", "box"),
                                          ("size", num(RAIL_HALF, half_len, RAIL_HALF)),
                                          ("pos", num(s.rail_line_x, 0, s.guide_rail_height + RAIL_HALF)), *free, rgba])
        post_half = (s.guide_rail_height - g) / 2
        for side, py in (("yneg", -half_len), ("ypos", half_len)):
            self._el(self.worldbody, "geom", [("name", f"guide_post_{side}"), ("type", "box"),
                                              ("size", num(RAIL_HALF, RAIL_HALF, post_half)),
                                              ("pos", num(s.rail_line_x, py, g + post_half)), *free, rgba])
        return self

    def drop(self) -> MjcfBuilder:
        """The Phase 1 drop scene (``--preset drop``): a ground plane and a
        0.1 m cube on a free joint released at z = 0.5, with a 1 ms timestep.

        It replaces the hand-written ``examples/drop.xml`` (row 2.7) and keeps
        its names (``floor``, ``box``, ``box_free``, ``box_geom``), so the
        Phase 1 gate and ``rookery`` read it unchanged. Bodies: 0 = world,
        1 = box, so nbody == 2.
        """
        self.root.set("model", "drop")
        self._el(self.root, "option", [("timestep", num(0.001)), ("gravity", num(0, 0, -9.81))])
        self._worldbody = self._el(self.root, "worldbody")
        wb = self._worldbody
        self._el(wb, "geom", [("name", "floor"), ("type", "plane"), ("size", num(1, 1, 0.1))])
        body = self._el(wb, "body", [("name", "box"), ("pos", num(0, 0, DROP_HEIGHT))])
        self._el(body, "freejoint", [("name", "box_free")])
        h = DROP_BOX / 2
        self._el(body, "geom", [("name", "box_geom"), ("type", "box"), ("size", num(h, h, h)), ("mass", num(1))])
        return self

    def build(self) -> str:
        """Serialise: two-space indent, a trailing newline, and no XML declaration or timestamp."""
        tree = ET.ElementTree(self.root)
        ET.indent(tree, space="  ")
        return ET.tostring(self.root, encoding="unicode") + "\n"


def spec_json(spec: SceneSpec) -> str:
    """The spec as JSON, written next to the MJCF (``rookery.json``) so Rust can
    read scene metadata later. Keys are sorted and indented, with a trailing newline."""
    data = asdict(spec)
    # Resolved placements, so a reader never has to re-derive the defaults.
    data["derived"] = {"exit_pose": list(spec.exit_pose), "rail_line_x": spec.rail_line_x}
    return json.dumps(data, sort_keys=True, indent=2) + "\n"


def build_scene(spec: SceneSpec) -> str:
    """The scene for ``spec``: the named preset, or else the farm."""
    if spec.preset == "drop":
        return MjcfBuilder(spec).drop().build()
    return MjcfBuilder(spec).header().tray().water().cells().exit_tray().crane_frame().build()


def spec_from_json(text: str) -> SceneSpec:
    """Rebuild the ``SceneSpec`` that :func:`spec_json` wrote. The ``derived``
    block is output only and is ignored. This is what ``--check`` uses: each
    committed ``.json`` is the recipe for the ``.xml`` beside it."""
    data = json.loads(text)
    data.pop("derived", None)
    return SceneSpec(**data)
