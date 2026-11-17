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
    .crawfish()    crawfish_r_c capsule + touch_r_c sensor          (5.2)
    .shells()      shell_r_c capsules, parked on a pad below ground (7.4)
    .exit_tray()   geom exit_tray between tray and rail            (2.5)
    .crane_frame() floor_rail + guide_rail (static, contact-free)  (2.5)
    .gantry()      rail_y → mast_z → arm_x → hook_lift, actuators   (3.2)
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
#: Radius of the two posts holding each handle. The handle's own size is
#: SceneSpec.handle_dia (row 5.1).
POST_RADIUS = 0.003
HANDLE_RGBA = (0.9, 0.85, 0.2, 1.0)
EXIT_RGBA = (0.2, 0.6, 0.3, 1.0)
CRAWFISH_RGBA = (0.7, 0.25, 0.2, 1.0)
#: A shed shell: pale and translucent, so it reads as empty.
SHELL_RGBA = (0.95, 0.75, 0.6, 0.6)
#: Collision bits (row 7.4). MuJoCo lets geoms a and b collide when
#: ``a.contype & b.conaffinity`` or ``b.contype & a.conaffinity`` is non-zero.
#: Every other geom keeps the default 1/1.
#:
#: ```text
#:   geom        contype  conaffinity   touches
#:   default        1         1         everything below except the pad
#:   shell          2         1         boxes, hook (not other shells, not the ground)
#:   park pad       0         2         shells only
#:   ground         4         1         default geoms only, so a shell passes under it
#: ```
SHELL_GROUP = 2
GROUND_CONTYPE = 4
#: Drop preset: cube edge and release height, in metres (Phase 1 gate: rest at DROP_BOX / 2).
DROP_BOX = 0.1
DROP_HEIGHT = 0.5
#: Crane joint ranges in metres, COPIED from crates/mast/src/limits.rs
#: (protected). scene/tests/test_gantry.py runs `cargo run -p mast --example
#: dump-limits` and fails if these drift from the Rust constants.
GANTRY_RANGES = {
    "rail_y": (-0.40, 0.40),
    "mast_z": (0.05, 0.80),
    "arm_x": (0.00, 0.85),
    "hook_lift": (0.00, 0.03),
}
#: Home pose (joint values), the same order as GANTRY_RANGES: truck centred,
#: carriage at 0.45 m (the preview's home_mast), arm retracted, hook down.
GANTRY_HOME = {"rail_y": 0.0, "mast_z": 0.45, "arm_x": 0.0, "hook_lift": 0.0}
#: Position-actuator gains. With dampratio=1 each axis is critically damped,
#: so a 0.3 m step settles in under 1 s with no overshoot (test_gantry.py).
#: mast_z and hook_lift carry the box when lifting, so they are stiff
#: (row 5.5): at kp 300/100 a 0.15 kg box sagged them past DONE_TOL and the
#: mast move timed out, and at 2000 the sag is under 1 mm.
GANTRY_KP = {"rail_y": 600.0, "mast_z": 2000.0, "arm_x": 200.0, "hook_lift": 2000.0}
#: Moving masses, kg: truck, lift carriage, arm, hook.
GANTRY_MASS = {"rail_y": 5.0, "mast_z": 1.0, "arm_x": 0.5, "hook_lift": 0.1}
CRANE_RGBA = (0.85, 0.75, 0.2, 1.0)
HOOK_RGBA = (0.8, 0.2, 0.2, 1.0)
# The hook's sizes are SceneSpec.hook_* (row 5.1). They and GANTRY_HOME are
# duplicated in crates/mast/src/kinematics.rs; test_gantry.py fails on drift.
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
        # MuJoCo keeps one namespace per object type (a joint and its actuator
        # may share a name), so names are unique per kind of element.
        self._names: set[tuple[str, str]] = set()
        self._worldbody: ET.Element | None = None

    # ---------------- helpers ----------------

    def _el(self, parent: ET.Element, tag: str, attrs: Iterable[tuple[str, str]] = ()) -> ET.Element:
        """Append ``<tag>`` with attributes in exactly the given order. Names must
        be unique within their MuJoCo namespace (all actuator types share one)."""
        el = ET.SubElement(parent, tag)
        kind = "actuator" if parent.tag == "actuator" else ("joint" if tag == "freejoint" else tag)
        for k, v in attrs:
            if k == "name":
                if (kind, v) in self._names:
                    raise ValueError(f"duplicate MJCF {kind} name {v!r}")
                self._names.add((kind, v))
            el.set(k, v)
        return el

    def _contact(self) -> list[tuple[str, str]]:
        """Contact attributes from spec.contact, only for the fields that are
        set, so the default writes nothing (byte-identical to Phase 4)."""
        c = self.spec.contact
        out: list[tuple[str, str]] = []
        if c.condim is not None:
            out.append(("condim", str(c.condim)))
        if c.friction is not None:
            out.append(("friction", num(*c.friction)))
        if c.solref is not None:
            out.append(("solref", num(*c.solref)))
        if c.solimp is not None:
            out.append(("solimp", num(*c.solimp)))
        return out

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
        # With shells (7.4) the ground lets them through to their park pad below it.
        ground_bits = [("contype", str(GROUND_CONTYPE)), ("conaffinity", "1")] if s.shells else []
        self._el(wb, "geom", [("name", "ground"), ("type", "plane"), ("size", num(2, 2, 0.1)),
                              ("pos", num(0, 0, -TRAY_WALL)), ("rgba", num(0.3, 0.32, 0.3, 1)), *ground_bits])
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
        handle_r = s.handle_dia / 2
        handle_z = s.handle_top_z - handle_r
        handle_half = hy - t - handle_r
        post_half = (handle_z - h) / 2
        for r in range(s.rows):
            for c in range(s.cols):
                x, y = s.cell_center(r, c)
                tag = f"{r}_{c}"
                body = self._el(self.worldbody, "body", [("name", f"box_{tag}"), ("pos", num(x, y, 0))])
                self._el(body, "freejoint", [("name", f"box_{tag}_free")])
                self._el(body, "geom", [("name", f"box_{tag}_floor"), ("type", "box"),
                                        ("size", num(hx, hy, f / 2)), ("pos", num(0, 0, f / 2)),
                                        ("mass", num(m)), rgba, *self._contact()])
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
                                        ("size", num(handle_r)), ("mass", num(0.005)),
                                        ("rgba", num(*HANDLE_RGBA)), *self._contact()])
        return self

    def crawfish(self) -> MjcfBuilder:
        """One crawfish per box (row 5.2): body ``crawfish_r_c``, a capsule on
        its own free joint lying along X on the box floor, with a site
        ``crawfish_r_c_site`` (slightly larger than the capsule) and a touch
        sensor ``touch_r_c`` that sums the normal contact force on it.

        It rides along when the box is lifted. The Phase 5 gate reads its
        contact force to make sure the lift never crushes it.
        """
        s = self.spec
        if not s.crawfish:
            return self
        r = s.crawfish_radius
        half = s.crawfish_length / 2 - r  # capsule segment half-length
        sensor = self._el(self.root, "sensor")
        for row in range(s.rows):
            for col in range(s.cols):
                x, y = s.cell_center(row, col)
                tag = f"{row}_{col}"
                body = self._el(self.worldbody, "body", [("name", f"crawfish_{tag}"),
                                                          ("pos", num(x, y, BOX_FLOOR + r))])
                self._el(body, "freejoint", [("name", f"crawfish_{tag}_free")])
                self._el(body, "geom", [("name", f"crawfish_{tag}_body"), ("type", "capsule"),
                                        ("fromto", num(-half, 0, 0, half, 0, 0)), ("size", num(r)),
                                        ("mass", num(s.crawfish_mass)), ("rgba", num(*CRAWFISH_RGBA))])
                self._el(body, "site", [("name", f"crawfish_{tag}_site"), ("type", "capsule"),
                                        ("fromto", num(-half, 0, 0, half, 0, 0)), ("size", num(r * 1.1))])
                self._el(sensor, "touch", [("name", f"touch_{tag}"), ("site", f"crawfish_{tag}_site")])
        return self

    def shells(self) -> MjcfBuilder:
        """One shed shell per box (row 7.4): body ``shell_r_c``, a capsule on its
        own free joint, plus one static pad ``shell_park`` under the tray.

        ```text
          z = 0      ── tray floor ──[box r_c: crawfish | shell (when molted)]──
          z = -0.01  ── ground plane (lets shells through: GROUND_CONTYPE)
          z = park   ── shell_park pad, hidden (group 3): parked shells hover 2 mm
                        above it, each under its own cell
        ```

        A shell starts parked with ``gravcomp="1"``: its weight is cancelled,
        so it hovers in place and touches nothing (row 7.5). Resting on the
        pad instead cost two contacts per shell, which made every physics
        step about 27% slower. The lifecycle (7.5) moves it into its box
        with ``Sim::set_body_pose`` and turns its gravity on with
        ``Sim::set_gravcomp(name, 0)`` on ``Molted``, then reverses both on
        ``ShellEaten``. In a box it rests on the floor beside the crawfish
        (``spec.shell_in_box``) and rides along when the box is lifted. A
        ``<contact><exclude>`` keeps it from colliding with its own crawfish,
        so it never pushes the animal.

        Why a pad under the ground rather than deleting and re-adding bodies:
        MuJoCo models are fixed once compiled, so every shell must exist from
        the start. Below the ground nothing else can reach it, and it stays
        out of the overview camera.
        """
        s = self.spec
        if not s.shells:
            return self
        hx, hy = s.tray_size[0] / 2, s.tray_size[1] / 2
        t = TRAY_WALL
        self._el(self.worldbody, "geom", [("name", "shell_park"), ("type", "box"),
                                          ("size", num(hx, hy, t / 2)), ("pos", num(0, 0, s.shell_park_z - t / 2)),
                                          ("contype", "0"), ("conaffinity", str(SHELL_GROUP)), ("group", "3"),
                                          ("rgba", num(*TRAY_RGBA))])
        r = s.shell_radius
        half = s.shell_length / 2 - r
        contact = self._el(self.root, "contact") if s.crawfish else None
        for row in range(s.rows):
            for col in range(s.cols):
                tag = f"{row}_{col}"
                body = self._el(self.worldbody, "body", [("name", f"shell_{tag}"),
                                                          ("pos", num(*s.shell_parked(row, col))),
                                                          ("gravcomp", "1")])
                self._el(body, "freejoint", [("name", f"shell_{tag}_free")])
                self._el(body, "geom", [("name", f"shell_{tag}_body"), ("type", "capsule"),
                                        ("fromto", num(-half, 0, 0, half, 0, 0)), ("size", num(r)),
                                        ("mass", num(s.shell_mass)), ("contype", str(SHELL_GROUP)),
                                        ("conaffinity", "1"), ("rgba", num(*SHELL_RGBA))])
                if contact is not None:
                    self._el(contact, "exclude", [("body1", f"shell_{tag}"), ("body2", f"crawfish_{tag}")])
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

    def gantry(self) -> MjcfBuilder:
        """The stacker crane: four nested slide joints on the floor rail (row 3.2).

        ```text
          worldbody
            crane_truck     slide rail_y    (Y)  on the floor rail at rail_line_x
              crane_carriage slide mast_z   (Z)  q = hook height above the tray floor
                crane_arm    slide arm_x    (X)  q = hook x - rail_line_x
                  crane_hook slide hook_lift (Z) the J-hook's short stroke
        ```

        Each joint's value *is* the coordinate that `limits.rs` speaks in.
        Every body starts at the home pose with ``ref`` equal to its home value,
        so MuJoCo's qpos0 is home and the joint value equals the coordinate.
        Ranges come from ``GANTRY_RANGES`` (a copy of `limits.rs`, checked by a
        test). ``gravcomp="1"`` cancels gravity on the moving bodies, so the
        position actuators do not sag (settled in 3.1). Only the hook
        collides; the truck, mast and arm are drawn without contact, as in
        the preview.

        Keyframe ``home`` holds the actuators at home. Load it with
        ``mj_resetDataKeyframe``, or the controls start at zero.
        """
        s = self.spec
        h = GANTRY_HOME
        g = -TRAY_WALL  # ground plane height
        free = [("contype", "0"), ("conaffinity", "0")]
        rgba = ("rgba", num(*CRANE_RGBA))

        def joint(body: ET.Element, name: str, axis: tuple[float, float, float]) -> None:
            lo, hi = GANTRY_RANGES[name]
            self._el(body, "joint", [("name", name), ("type", "slide"), ("axis", num(*axis)),
                                     ("range", num(lo, hi)), ("limited", "true"), ("ref", num(h[name])),
                                     ("damping", num(5.0)), ("armature", num(0.05))])

        def inertial(body: ET.Element, name: str) -> None:
            self._el(body, "inertial", [("pos", num(0, 0, 0)), ("mass", num(GANTRY_MASS[name])),
                                        ("diaginertia", num(0.01, 0.01, 0.01))])

        truck = self._el(self.worldbody, "body", [("name", "crane_truck"),
                                                  ("pos", num(s.rail_line_x, h["rail_y"], 0)),
                                                  ("gravcomp", "1")])
        joint(truck, "rail_y", (0, 1, 0))
        inertial(truck, "rail_y")
        self._el(truck, "geom", [("name", "crane_truck_body"), ("type", "box"),
                                 ("size", num(0.06, 0.08, 0.03)), ("pos", num(0, 0, g + 2 * RAIL_HALF + 0.03)),
                                 *free, rgba])
        mast_half = (s.guide_rail_height - g) / 2
        self._el(truck, "geom", [("name", "crane_mast"), ("type", "box"),
                                 ("size", num(0.025, 0.025, mast_half)),
                                 ("pos", num(-0.04, 0, g + mast_half)), *free, rgba])

        carriage = self._el(truck, "body", [("name", "crane_carriage"), ("pos", num(0, 0, h["mast_z"])),
                                            ("gravcomp", "1")])
        joint(carriage, "mast_z", (0, 0, 1))
        inertial(carriage, "mast_z")
        self._el(carriage, "geom", [("name", "crane_carriage_body"), ("type", "box"),
                                    ("size", num(0.03, 0.05, 0.03)), ("pos", num(-0.01, 0, 0.02)), *free, rgba])

        arm = self._el(carriage, "body", [("name", "crane_arm"), ("pos", num(h["arm_x"], 0, 0)),
                                          ("gravcomp", "1")])
        joint(arm, "arm_x", (1, 0, 0))
        inertial(arm, "arm_x")
        reach = GANTRY_RANGES["arm_x"][1]
        # The beam trails back from the tip far enough to stay in the carriage at full reach.
        self._el(arm, "geom", [("name", "crane_arm_beam"), ("type", "box"),
                               ("size", num((reach + 0.05) / 2, 0.012, 0.01)),
                               ("pos", num(-(reach + 0.05) / 2, 0, 0.02)), *free, rgba])

        hook = self._el(arm, "body", [("name", "crane_hook"), ("pos", num(0, 0, h["hook_lift"])),
                                      ("gravcomp", "1")])
        joint(hook, "hook_lift", (0, 0, 1))
        inertial(hook, "hook_lift")
        hr = ("rgba", num(*HOOK_RGBA))
        # J-hook: a finger along +X at hook height with an upturned tip, so it
        # slides under a handle (a capsule along Y) and catches it when lifted.
        hr_, hf, hw = s.hook_radius, s.hook_finger, s.hook_width
        if hw > 0:
            # Flat saddle (row 5.5): a plate finger and a plate tip, hook_width
            # wide across the handle, 2 × hook_radius thick. Same envelope.
            self._el(hook, "geom", [("name", "hook"), ("type", "box"),
                                    ("size", num(hf / 2, hw / 2, hr_)), ("pos", num(hf / 2, 0, 0)),
                                    hr, *self._contact()])
            self._el(hook, "geom", [("name", "hook_tip"), ("type", "box"),
                                    ("size", num(hr_, hw / 2, s.hook_tip_rise / 2)),
                                    ("pos", num(hf, 0, s.hook_tip_rise / 2)), hr, *self._contact()])
        else:
            self._el(hook, "geom", [("name", "hook"), ("type", "capsule"), ("size", num(hr_)),
                                    ("fromto", num(0, 0, 0, hf, 0, 0)), hr, *self._contact()])
            self._el(hook, "geom", [("name", "hook_tip"), ("type", "capsule"), ("size", num(hr_)),
                                    ("fromto", num(hf, 0, 0, hf, 0, s.hook_tip_rise)), hr, *self._contact()])
        self._el(hook, "geom", [("name", "hook_stem"), ("type", "capsule"), ("size", num(hr_)),
                                ("fromto", num(0, 0, 0, 0, 0, 0.02)), *free, hr])

        actuator = self._el(self.root, "actuator")
        for name in GANTRY_RANGES:
            lo, hi = GANTRY_RANGES[name]
            self._el(actuator, "position", [("name", name), ("joint", name), ("kp", num(GANTRY_KP[name])),
                                            ("dampratio", num(1)), ("ctrlrange", num(lo, hi))])
        keyframe = self._el(self.root, "keyframe")
        self._el(keyframe, "key", [("name", "home"), ("ctrl", num(*(h[n] for n in GANTRY_RANGES)))])
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
    return MjcfBuilder(spec).header().tray().water().cells().crawfish().shells().exit_tray().crane_frame().gantry().build()


def spec_from_json(text: str) -> SceneSpec:
    """Rebuild the ``SceneSpec`` that :func:`spec_json` wrote. The ``derived``
    block is output only and is ignored. This is what ``--check`` uses: each
    committed ``.json`` is the recipe for the ``.xml`` beside it."""
    data = json.loads(text)
    data.pop("derived", None)
    return SceneSpec(**data)
