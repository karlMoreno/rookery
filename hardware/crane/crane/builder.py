"""CraneBuilder: puts every part in its place at the home pose.

Pattern: Builder (fluent).
    Each with_*() step adds one subsystem to the tree and returns the builder,
    so `CraneBuilder(spec).with_frame().with_truck()...build()` reads as the
    assembly order. Why: a 1-tier and a 3-tier crane, or a crane with and
    without the rack, differ only in which steps run and in spec numbers. No
    part knows where it goes; only the builder does.

Pattern: Director (`build_crane`).
    The standard sequence of builder steps lives in one function, so the CLI,
    the viewer and the tests all build the same crane.

Coordinates: world millimetres at home. The moving bodies' origin is the hook
cup's home position H, so body-to-body offsets are zero and each joint's value
is simply the cup's world coordinate along its axis.

    world
    ├── crane_frame   (static)   floor rail, uprights, guide rail, enclosure
    ├── fixtures      (static)   exit tray, camera crossbar and camera
    ├── rack          (static, preview only) posts, tubs, mesh boxes, water node
    └── truck         joint rail_y (Y)
        └── carriage  joint mast_z (Z)          fork base: 2 fixed blocks, X motor, belt anchors
            ├── arm       joint arm_x  (X)      fork top stage: rail B, the hook rail
            │   └── hook  joint lift (locked; engage with a mast move)
            └── fork_mid  joint arm_mid         fork middle stage, follows arm_x at 1/2
"""
from __future__ import annotations

from build123d import Location, Rot

from crane.catalog import item
from crane.parts.factory import PartFactory
from crane.registry import MountRegistry
from crane.spec import (CARRIAGE_BELOW_ARM, CARRIAGE_H, EXTRUSION, FORK_BLOCK_PITCH, FORK_END_MARGIN, FORK_RATIO,
                        MGN12H_BLOCK_L, RAIL_END_BLOCK, CraneSpec, mm)
from crane.tree import BodyNode, CollisionBox, ItemNode, Joint, PartNode

AXIS_X = (0, 90, 0)       # rotate a +Z part (extrusion, screw) to run along +X
AXIS_Y = (-90, 0, 0)      # ... to run along +Y
FLIP = (180, 0, 0)        # turn a part upside down


def loc(x, y, z, rot=(0, 0, 0)) -> Location:
    return Location((x, y, z), rot)


class CraneBuilder:
    def __init__(self, spec: CraneSpec, factory: PartFactory | None = None,
                 registry: MountRegistry | None = None):
        self.s = spec
        self.f = factory or PartFactory(spec)
        self.reg = registry or MountRegistry()
        self.world = BodyNode("world", (0, 0, 0))
        self._bodies: dict[str, BodyNode] = {}
        self._count: dict[str, int] = {}
        s = spec
        # Frequently used world positions (mm)
        self.H = (mm(s.home["arm_x"]), mm(s.home["rail_y"]), mm(s.home["mast_z"]))
        self.mx = mm(s.mast_x)
        self.ty = self.H[1] - mm(s.arm_y_offset)          # truck / mast centre y at home
        self.arm_y = self.H[1]                             # arm centreline y
        self.zc = self.H[2] + mm(s.hook_shank)             # fork underside (rail B, the hook rail) at home
        self.plate_z = mm(s.truck_top) - 8.0                 # truck plate underside
        self.truck_top = mm(s.truck_top)
        self.mast_top = mm(s.mast_top_z)
        self.y0, self.y1 = mm(s.rail_y0), mm(s.rail_y1)

    # ------------------------------------------------------------ helpers
    def _body(self, name, parent: BodyNode, origin=(0.0, 0.0, 0.0), joint=None, group="crane") -> BodyNode:
        b = BodyNode(name, origin, joint, group)
        parent.add(b)
        self._bodies[name] = b
        return b

    def _place(self, body: BodyNode, part, at: Location, name: str | None = None,
               mount_names: dict[str, str] | None = None) -> PartNode:
        """Add one placement and register the part's declared mounts in world
        coordinates on this body (Registry pattern, see registry.py)."""
        base = name or part.key
        n = self._count.get(base, 0)
        self._count[base] = n + 1
        node = PartNode(f"{base}_{n}" if n else base, part, at)
        body.add(node)
        for decl in part.mount_decls():
            rename = (mount_names or {}).get(decl.name)
            if rename is None and decl.kind in ("cable_port",):
                rename = f"{decl.name}_{n}" if n else decl.name
            if rename is None:
                rename = f"{node.name}_{decl.name}"
            self.reg.register(decl, body.name, at, rename=rename)
        return node

    def _ext(self, body, length_mm, at, wide=False, name=None):
        key = "extrusion_2040" if wide else "extrusion_2020"
        return self._place(body, self.f.vendor(key, length_mm), at, name or key)

    # ------------------------------------------------------------ steps
    def with_frame(self) -> "CraneBuilder":
        s, f, mx = self.s, self.f, self.mx
        frame = self._body("crane_frame", self.world)
        ends = (self.y0, self.y1)
        for y in (self.y0 + 30, self.y1 - 30):                       # foot crossbars, along X
            self._ext(frame, 300, loc(mx - 150, y, 10, AXIS_X), name="foot_crossbar")
        self._ext(frame, mm(s.rail_length), loc(mx, self.y0, 30, AXIS_Y), wide=True, name="floor_rail")
        self._place(frame, f.printed("rail_end"), loc(mx, self.y1, 20))
        self._place(frame, f.printed("rail_end"), loc(mx, self.y0, 20, (0, 0, 180)))
        self._place(frame, f.printed("rail_mid_support"), loc(mx, (self.y0 + self.y1) / 2, 0))
        g = mm(s.guide_rail_z)
        up_bottom = 20 + 28
        for y in (self.y0 + 30, self.y1 - 30):
            self._ext(frame, g - up_bottom, loc(mx, y, up_bottom), name="upright")
            self._place(frame, f.printed("top_corner"), loc(mx, y, g))
        self._ext(frame, (self.y1 - 30) - (self.y0 + 30) + 40, loc(mx, self.y0 + 10, g + 10, AXIS_Y), name="guide_rail")
        # Y home switch, hit by the truck cheek when the rail is at its start.
        y_sw = self.y0 + mm(RAIL_END_BLOCK) + 4
        self._place(frame, f.printed("endstop_bracket"), loc(mx + 24, y_sw, 50, (0, 0, 90)), mount_names={"switch": "y_home"})
        self._place(frame, f.vendor("microswitch"), loc(mx + 24, y_sw, 50, (0, 0, 90)))
        # Fixed half of drag chain 1 along the floor, beside the rail.
        mid = (self.y0 + self.y1) / 2
        self._place(frame, f.vendor("drag_chain_10x15", self.y1 - 70 - mid), loc(mx - 75, mid, 0))
        self._place(frame, f.printed("drag_chain_bracket"), loc(mx - 75, self.y1 - 70, 0, (0, 0, 180)))
        return self

    def with_truck(self) -> "CraneBuilder":
        s, f, mx, ty = self.s, self.f, self.mx, self.ty
        H = self.H
        truck = self._body("truck", self.world, H,
                           Joint("rail_y", (0, 1, 0), s.rail_y_range, s.home["rail_y"], s.vmax_rail))
        pz = self.plate_z
        self._place(truck, f.printed("truck_plate"), loc(mx, ty, pz))
        for sx in (-1, 1):
            self._place(truck, f.printed("truck_cheek"), loc(mx + sx * 24, ty, pz))
            for dy in (-50, 50):
                self._place(truck, f.vendor("v_wheel"), loc(mx + sx * 10, ty + dy, pz - 15, AXIS_X))
        # Y drive: motor on the truck, omega wrap around a flat belt fixed at both rail ends.
        yd = ty + 45
        self._place(truck, f.printed("y_drive_mount"), loc(mx, yd, self.truck_top))
        self._place(truck, f.vendor("nema17"), loc(mx + 8, yd, self.truck_top + 24, (0, -90, 0)), name="nema17_y")
        self._place(truck, f.vendor("gt2_pulley"), loc(mx, yd, self.truck_top + 24, AXIS_X), name="pulley_y")
        self._place(truck, f.printed("y_idler_hanger"), loc(mx, yd, pz))
        for dy in (-20, 20):
            self._place(truck, f.vendor("gt2_idler"), loc(mx, yd + dy, pz - 12, AXIS_X))
        self._place(truck, f.printed("drag_chain_bracket"), loc(mx - 50, ty - 70, self.truck_top))
        return self

    def with_mast(self) -> "CraneBuilder":
        s, f, mx, ty = self.s, self.f, self.mx, self.ty
        truck = self._bodies["truck"]
        tt, top = self.truck_top, self.mast_top
        self._place(truck, f.printed("mast_foot"), loc(mx, ty, tt))
        self._ext(truck, mm(s.mast_length), loc(mx, ty, tt), name="mast")
        sx = mx - 32
        self._place(truck, f.printed("z_motor_tower"), loc(sx, ty, tt))
        self._place(truck, f.vendor("nema17"), loc(sx, ty, tt + 53), name="nema17_z")
        self._place(truck, f.vendor("coupler_5_8"), loc(sx, ty, tt + 55))
        screw0 = tt + 62
        self._place(truck, f.vendor("t8_screw", round(top + 10 - screw0)), loc(sx, ty, screw0))
        self._place(truck, f.printed("mast_top_block"), loc(mx, ty, top))
        self._place(truck, f.vendor("kfl08"), loc(sx, ty, top + 13, (0, 0, 90)))
        g = mm(s.guide_rail_z)
        for dx in (-18, 18):
            self._place(truck, f.vendor("bearing_625"), loc(mx + dx, ty, g + 10), name="guide_roller")
        # Z home switch on the motor tower, hit by the carriage's back plate at the bottom.
        z_sw = tt + mm(0.080) - 5
        self._place(truck, f.printed("endstop_bracket"), loc(sx - 4, ty - 30, z_sw), mount_names={"switch": "z_home"})
        self._place(truck, f.vendor("microswitch"), loc(sx - 4, ty - 30, z_sw))
        # Fixed half of drag chain 2, up the back of the mast.
        self._place(truck, f.vendor("drag_chain_10x15", 380), loc(mx - 75, ty - 45, tt + 20, (90, 0, 0)))
        return self

    def with_carriage(self) -> "CraneBuilder":
        """The lift carriage and the fork's fixed base: two MGN12H blocks at the
        carriage front, the X motor and pinion, and the carriage ends of both
        differential belts."""
        s, f, mx, ty, zc = self.s, self.f, self.mx, self.ty, self.zc
        car = self._body("carriage", self._bodies["truck"], self.H,
                         Joint("mast_z", (0, 0, 1), s.mast_z_range, s.home["mast_z"], s.vmax_mast))
        lc = f.printed("lift_carriage")
        self._place(car, lc, loc(mx, ty, zc), mount_names={"x_home": "x_home"})
        for dy in (-20.1, 20.1):
            for dz in (lc.z0 + 12, lc.z1 - 12):
                self._place(car, f.vendor("v_wheel"), loc(mx, ty + dy, zc + dz, AXIS_X))
        self._place(car, f.vendor("t8_nut"), loc(mx - 32, ty, zc + lc.z1))
        for bx in s.carriage_block_x:                            # ride on rail A (z 19)
            self._place(car, f.vendor("mgn12h_block"), loc(mm(bx), self.arm_y, zc + 19))
        px, py = mx + lc.pinion_x, ty + lc.pinion_y
        self._place(car, f.vendor("nema14"), loc(px, py, zc + lc.SHELF[1], FLIP), name="nema14_x")
        self._place(car, f.vendor("gt2_pulley"), loc(px, py, zc + 16), name="pinion_x")
        A = f.printed("fork_belt_anchor")
        for x, y in zip(s.fork_anchor_x, (A.Y_RETRACT, A.Y_EXTEND)):
            self._place(car, A, loc(mm(x), self.arm_y + y, zc + A.TOP))
        xh = self.reg["x_home"].world
        self._place(car, f.vendor("microswitch"), xh)
        return self

    def with_arm(self) -> "CraneBuilder":
        """The fork's top stage: rail B, which the hook hangs from. Its joint
        is arm_x, the world x of the hook's cup, as Rookery expects."""
        s, f = self.s, self.f
        arm = self._body("arm", self._bodies["carriage"], self.H,
                         Joint("arm_x", (1, 0, 0), s.arm_x_range, s.home["arm_x"], s.vmax_arm))
        L = mm(s.fork_length)
        rear = mm(s.fork_front_home) - L
        self._place(arm, f.vendor("mgn12_rail", L), loc(rear, self.arm_y, self.zc), name="rail_b")
        self._place(arm, f.printed("arm_rear_clamp"), loc(rear, self.arm_y, self.zc))
        if s.hook_camera:
            # Registry: the mount declares where its camera looks; the MJCF
            # visitor turns that into <camera name="cam_hook"> on this body.
            tip = rear + L
            self._place(arm, f.printed("hook_camera_mount"), loc(tip, self.arm_y, self.zc),
                        mount_names={"camera": "cam_hook", "csi_port": "cam_hook_cable"})
            cam = self.reg["cam_hook"].world
            self._place(arm, f.vendor("pi_camera3_wide"), cam * Location((0, 0, 0), FLIP), name="hook_camera")
        return self

    def with_fork_middle(self) -> "CraneBuilder":
        """The fork's middle stage. It has no motor of its own in MuJoCo: its
        joint arm_mid follows arm_x at half speed (Joint.follows), which is
        what the 2:1 belts do on the real crane.

        Pattern: Composite. The middle stage is one more BodyNode under the
        carriage, a sibling of the arm, so every Visitor (viewer, MJCF, BOM,
        STL, parts kit) picks it up without any change of its own."""
        s, f, zc = self.s, self.f, self.zc
        h = s.home["arm_x"]
        mid = self._body("fork_mid", self._bodies["carriage"], self.H,
                         Joint("arm_mid", (1, 0, 0), s.arm_mid_range, h, s.vmax_arm / FORK_RATIO,
                               follows="arm_x", ratio=1 / FORK_RATIO))
        L = mm(s.fork_length)
        front = mm(s.fork_front_home)
        rear = front - L
        self._place(mid, f.vendor("alu_flat_25x6", L), loc(rear, self.arm_y, zc + 13), name="fork_mid_bar")
        self._place(mid, f.vendor("mgn12_rail", L), loc(rear, self.arm_y, zc + 19), name="rail_a")
        bx = front - mm(FORK_END_MARGIN + MGN12H_BLOCK_L / 2)
        for x in (bx - mm(FORK_BLOCK_PITCH), bx):                # ride on rail B (z 0)
            self._place(mid, f.vendor("mgn12h_block"), loc(x, self.arm_y, zc))
        for key, x in (("fork_mid_front", front), ("fork_mid_rear", rear)):
            end = f.printed(key)
            self._place(mid, end, loc(x, self.arm_y, zc))
            ix, iy, iz = end.idler_at
            self._place(mid, f.vendor("gt2_idler"), loc(x + ix, self.arm_y + iy, zc + iz))
        return self

    def with_hook(self) -> "CraneBuilder":
        s, f = self.s, self.f
        hook_body = self._body("hook", self._bodies["arm"], self.H,
                               Joint("lift", (0, 0, 1), (0.0, 0.0), 0.0, 0.0, locked=True))
        hook = f.printed("hook")
        H = self.H
        self._place(hook_body, hook, loc(*H))
        for name, c, size in hook.collision_boxes():
            hook_body.collisions.append(CollisionBox(name, (H[0] + c[0], H[1] + c[1], H[2] + c[2]), size))
        return self

    def with_electronics(self) -> "CraneBuilder":
        """Main enclosure on the far upright's outside face, above all water."""
        f, mx = self.f, self.mx
        frame = self._bodies["crane_frame"]
        base = f.printed("main_enclosure_base")
        b = base.__class__
        at = loc(mx, self.y1 - 20 + b.IX / 2 + b.W, 550, (0, 0, 90))
        self._place(frame, base, at)
        self._place(frame, f.vendor("pi5"), at * Location((0, -31, b.F + 5)))
        self._place(frame, f.vendor("skr_pico"), at * Location((0, 31, b.F + 5)))
        self._place(frame, f.vendor("buck_5v"), at * Location((25, 0, b.F)))
        lid_at = at * Location((0, 0, b.F + b.IZ))
        self._place(frame, f.printed("main_enclosure_lid"), lid_at)
        self._place(frame, f.vendor("estop"), self.reg["main_enclosure_lid_estop"].world)
        return self

    def with_exit_tray(self) -> "CraneBuilder":
        s, f = self.s, self.f
        fix = self._bodies.get("fixtures") or self._body("fixtures", self.world, group="fixtures")
        ey, ez = mm(s.exit_y), mm(s.exit_z)
        x0 = mm(s.rack_x0)
        x1 = mm(s.exit_x(s.exit_slots - 1)) + 70
        bar_top = ez - 4
        self._ext(fix, x1 - x0, loc(x0, ey, bar_top - 10, AXIS_X), name="exit_crossbar")
        for x in (x0 + 10, x1 - 10):
            self._ext(fix, bar_top - 20, loc(x, ey, 0), name="exit_post")
        for i in range(s.exit_slots):
            self._place(fix, f.printed("exit_cradle"), loc(mm(s.exit_x(i)), ey, ez), mount_names={"slot": f"exit_slot_{i}"})
        return self

    def with_cameras(self) -> "CraneBuilder":
        s, f = self.s, self.f
        fix = self._bodies.get("fixtures") or self._body("fixtures", self.world, group="fixtures")
        xc = mm(s.rack_x0 + s.tub_depth) + 25
        ya, yb = mm(s.rack_y0) - 10, mm(s.rack_y0 + s.tub_length) + 10
        floors = s.tier_floors
        for t in s.camera_tiers:
            zc = mm(floors[t + 1]) - 30 if t + 1 < len(floors) else mm(floors[t]) + 250
            self._ext(fix, yb - ya, loc(xc, ya, zc, AXIS_Y), name="camera_crossbar")
            yc = (ya + yb) / 2
            self._place(fix, f.printed("camera_bracket"), loc(xc, yc, zc), mount_names={"camera": f"cam_tier_{t}"})
            cam = self.reg[f"cam_tier_{t}"].world
            self._place(fix, f.vendor("pi_camera3_wide"), cam * Location((0, 0, 0), FLIP))
        return self

    def with_rack(self) -> "CraneBuilder":
        """The rack Rookery's scene.py also generates. Here for printing,
        the BOM and the preview; left out of crane.xml."""
        s, f = self.s, self.f
        rack = self._body("rack", self.world, group="rack")
        x0, x1 = mm(s.rack_x0), mm(s.rack_x0 + s.tub_depth)
        y0, y1 = mm(s.rack_y0), mm(s.rack_y0 + s.tub_length)
        ph = mm(s.rack_post_height)
        for x in (x0 - 10, x1 + 10):
            for y in (y0 - 10, y1 + 10):
                self._ext(rack, ph, loc(x, y, 0), name="rack_post")
        for t, fl in enumerate(s.tier_floors):
            z = mm(fl)
            for y in (y0 + 10, y1 - 10):
                self._ext(rack, x1 - x0 + 40, loc(x0 - 20, y, z - 10, AXIS_X), name="rack_rail")
            for x in (x0, x1):
                for y in (y0 + 10, y1 - 10):
                    # Shelf bracket: upright against the post, the rail resting on its base.
                    self._place(rack, f.printed("corner_bracket"), loc(x, y, z - 25, (0, 0, 0 if x == x0 else 180)))
            self._place(rack, f.vendor("water_tub"), loc((x0 + x1) / 2, (y0 + y1) / 2, z), name="tub")
            if t in s.populated_tiers:
                box = f.printed("mesh_box")
                for r in range(s.cells_x):
                    for c in range(s.cells_y):
                        self._place(rack, box, loc(mm(s.cell_x(r)), mm(s.cell_y(c)), mm(s.tier_floor_top(t))),
                                    name=f"box_{t}_{r}_{c}")
        return self

    def with_sensor_nodes(self) -> "CraneBuilder":
        """Wireless water nodes clipped over the far tub rim."""
        s, f = self.s, self.f
        rack = self._bodies.get("rack") or self._body("rack", self.world, group="rack")
        x_rim = mm(s.rack_x0 + s.tub_depth)
        for i, t in enumerate(s.populated_tiers[: s.sensor_nodes]):
            rim_z = mm(s.tier_floors[t] + s.tub_height)
            y = mm(s.rack_y0) + 100
            base_cls = f.printed("sensor_node_base").__class__
            at = loc(x_rim, y, rim_z - (base_cls.IZ + 2))
            self._place(rack, f.printed("sensor_node_base"), at, mount_names={"node": f"water_node_{i}"})
            self._place(rack, f.printed("sensor_node_lid"), at * Location((0, 0, base_cls.IZ + 2)))
            self._place(rack, f.vendor("esp32c3"), at * Location((15, 25, 2)))
            self._place(rack, f.vendor("cell_18650"), at * Location((27.5, -15, 2), (0, 0, 90)))
            for dy, key in ((-25, "tds_probe"), (0, "ph_probe"), (25, "ds18b20")):
                self._place(rack, f.vendor(key), loc(x_rim - 20, y + dy, rim_z))
        return self

    def with_purchases(self) -> "CraneBuilder":
        """Things on the BOM that have no geometry."""
        s = self.s
        frame = self._bodies["crane_frame"]
        # Y belt, plus the fork's belt rack (L) and two differential belts (about 2L each).
        frame.add(ItemNode(item("gt2_belt"), round(s.rail_length + 0.25 + 5 * s.fork_length + 0.25, 2)))
        for key, qty in (("fastener_kit", 1), ("wiring_kit", 1), ("psu_24v", 1), ("gfci", 1),
                         ("csi_cable_1m", len(s.camera_tiers)), ("csi_hdmi_extender", int(s.hook_camera))):
            frame.add(ItemNode(item(key), qty))
        return self

    def build(self) -> tuple[BodyNode, MountRegistry]:
        return self.world, self.reg


def build_crane(spec: CraneSpec, with_rack: bool = True) -> tuple[BodyNode, MountRegistry]:
    """Director: the standard build order used by the CLI, viewer and tests."""
    b = (CraneBuilder(spec)
         .with_frame()
         .with_truck()
         .with_mast()
         .with_carriage()
         .with_arm()
         .with_hook()
         .with_fork_middle()
         .with_electronics()
         .with_exit_tray()
         .with_cameras())
    if with_rack:
        b.with_rack().with_sensor_nodes()
    return b.with_purchases().build()
