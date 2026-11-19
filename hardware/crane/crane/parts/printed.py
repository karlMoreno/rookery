"""Every printed part.

Each class is one STL. Each fills in the PrintedPart template (parts/base.py):
body() always, and features(), declare_mounts() and fastener_points() when it
needs them. Each docstring says where the part's local origin is, because the
builder places parts by that origin.

The `@printed("key")` decorator registers the class with the PartFactory
(Pattern: Registry inside an Abstract Factory, see parts/factory.py), so a new
part becomes available to the builder by being defined here.

All dimensions in mm. Every part is checked against the printer bed when built.
"""
from __future__ import annotations

import math

from build123d import Align, Box, Cylinder, Location, Pos, Rot

from crane.parts.base import Fastener, PrintedPart, box_from_extents
from crane.parts.factory import printed
from crane.parts.vendor import ext_box
from crane.registry import MountDecl
from crane.spec import (CARRIAGE_BELOW_ARM, CARRIAGE_H, EXTRUSION, FORK_BAR_W, MGN12H_BLOCK_L,
                        MGN12H_BLOCK_W, NEMA14, NEMA17, mm)

SOCKET = 20.4     # printed socket for 2020 extrusion (0.4 mm clearance)
DOWN = (0, 0, 0)  # rotation for a vertical hole drilled downward


def down(x, y, z):
    """Location for a hole drilled straight down from (x, y, z)."""
    return Location((x, y, z))


def along_x(x, y, z):
    """Location for a hole drilled toward -X from (x, y, z)."""
    return Location((x, y, z), (0, 90, 0))


def along_y(x, y, z):
    """Location for a hole drilled toward +Y from (x, y, z)."""
    return Location((x, y, z), (90, 0, 0))


# --------------------------------------------------------------------------- truck
@printed("truck_plate")
class TruckPlate(PrintedPart):
    """Top plate of the truck that rides the floor rail.
    Origin: mast centre (x, y), plate underside (z = 0)."""
    title = "Truck plate"
    X0, X1, Y_HALF, T = -60.0, 60.0, 75.0, 8.0

    def body(self):
        return ext_box(self.X0, self.X1, -self.Y_HALF, self.Y_HALF, 0, self.T)

    def features(self, s):
        # Slot for the flat Y belt to rise to the Y-drive pulley (omega drive).
        return s - ext_box(-5, 5, 18, 72, -1, self.T + 1)

    def fastener_points(self):
        f = [Fastener(down(x, y, self.T), "M4_clear") for x in (-16, 16) for y in (-16, 16)]   # mast foot
        f += [Fastener(down(-32 + dx, dy, self.T), "M3_clear") for dx, dy in ((-22, 0), (22, 0), (0, -22), (0, 22))]  # Z motor tower
        f += [Fastener(down(x, y, self.T), "M3_clear") for x in (-24, 24) for y in (-60, 60)]  # cheeks
        f += [Fastener(down(x, y, self.T), "M3_clear") for x in (12, 40) for y in (24, 66)]    # Y-drive mount
        f += [Fastener(down(x, y, self.T), "M3_clear") for x in (-9, 9) for y in (30, 60)]     # idler hanger
        return f

    def declare_mounts(self):
        return [MountDecl("drag_chain_truck", "cable_port", Location((-50, -70, self.T)))]


@printed("truck_cheek")
class TruckCheek(PrintedPart):
    """Side cheek carrying two top V-wheels in the floor rail's top slots.
    Origin: cheek centre at the truck plate underside; the cheek hangs to -Z.
    One part serves both sides (it is symmetric)."""
    title = "Truck cheek"
    print_rotation = (0, 90, 0)

    def body(self):
        return ext_box(-3, 3, -70, 70, -30, 0)

    def fastener_points(self):
        f = [Fastener(along_x(3, y, -15), "M5_clear", 10) for y in (-50, 50)]   # wheel axles
        f += [Fastener(down(0, y, 0), "M3_insert", 8) for y in (-60, 60)]       # into the plate
        return f


@printed("y_drive_mount")
class YDriveMount(PrintedPart):
    """Bracket that holds the Y motor on the truck, shaft along -X, pulley over
    the belt slot. Origin: pulley axis at x = 0, the plate top (z = 0), y = 0 at
    the slot centre."""
    title = "Y drive motor bracket"
    AXIS_Z = 24.0

    def body(self):
        face = ext_box(8, 13, -25, 25, 0, 50)
        foot = ext_box(8, 50, -25, 25, 0, 5)
        rib = ext_box(13, 40, -3, 3, 5, 30)
        return face + foot + rib

    def features(self, s):
        s -= Pos(10, 0, self.AXIS_Z) * Rot(0, 90, 0) * Cylinder(11.5, 10)
        for dy in (-15.5, 15.5):
            for dz in (-15.5, 15.5):
                s -= Pos(10, dy, self.AXIS_Z + dz) * Rot(0, 90, 0) * Cylinder(1.7, 10)
        return s

    def fastener_points(self):
        return [Fastener(down(x, y, 5), "M3_clear", 6) for x in (12, 40) for y in (-21, 21)]


@printed("y_idler_hanger")
class YIdlerHanger(PrintedPart):
    """Hangs two toothless idlers under the truck plate so the flat belt wraps
    up around the Y pulley. Origin: belt slot centre at the plate underside."""
    title = "Y idler hanger"

    def body(self):
        return ext_box(-13, 13, -27, 27, -20, 0)

    def features(self, s):
        return s - ext_box(-5, 5, -28, 28, -21, 1)

    def fastener_points(self):
        f = [Fastener(along_x(13, y, -12), "M5_clear", 30) for y in (-20, 20)]
        f += [Fastener(down(x, y, 0), "M3_insert", 8) for x in (-9, 9) for y in (-15, 15)]
        return f


@printed("z_motor_tower")
class ZMotorTower(PrintedPart):
    """Tower that holds the Z motor shaft-up under the lead screw.
    Origin: motor axis on the truck plate top (z = 0)."""
    title = "Z motor tower"
    TOP = 58.0

    def body(self):
        base = ext_box(-28, 28, -28, 28, 0, 5)
        top = ext_box(-28, 28, -28, 28, self.TOP - 5, self.TOP)
        posts = [ext_box(sx * 28 - (6 if sx > 0 else 0), sx * 28 + (0 if sx > 0 else 6),
                         sy * 28 - (6 if sy > 0 else 0), sy * 28 + (0 if sy > 0 else 6), 5, self.TOP - 5)
                 for sx in (-1, 1) for sy in (-1, 1)]
        out = base + top
        for p in posts:
            out += p
        return out

    def features(self, s):
        s -= Pos(0, 0, self.TOP - 6) * Cylinder(11.5, 8, align=(Align.CENTER, Align.CENTER, Align.MIN))
        s -= ext_box(-15, 15, -15, 15, -1, 6)     # lets the motor's rear sit on the base rim
        return s

    def fastener_points(self):
        f = [Fastener(down(dx, dy, self.TOP), "M3_clear", 6) for dx in (-15.5, 15.5) for dy in (-15.5, 15.5)]
        f += [Fastener(down(dx, dy, 5), "M3_insert", 6) for dx, dy in ((-22, 0), (22, 0), (0, -22), (0, 22))]
        return f


@printed("mast_foot")
class MastFoot(PrintedPart):
    """Socket that holds the mast upright on the truck. Also carries the Z
    home switch. Origin: mast centre on the truck plate top (z = 0)."""
    title = "Mast foot"

    def body(self):
        return ext_box(-21, 21, -21, 21, 0, 28)

    def features(self, s):
        return s - ext_box(-SOCKET / 2, SOCKET / 2, -SOCKET / 2, SOCKET / 2, 0, 29)

    def fastener_points(self):
        f = [Fastener(down(x, y, 28), "M4_clear", 30) for x in (-16, 16) for y in (-16, 16)]
        f += [Fastener(along_x(21, 0, 18), "M3_clear", 12)]   # clamp screw into the extrusion slot
        return f


@printed("mast_top_block")
class MastTopBlock(PrintedPart):
    """Caps the mast: holds the KFL08 bearing for the lead screw and two
    guide rollers that straddle the top guide rail.
    Origin: mast centre at the top end of the mast."""
    title = "Mast top block"

    def body(self):
        block = ext_box(-46, 26, -25, 25, -15, 13)
        bosses = Pos(-18, 0, 13) * Cylinder(6, 3, align=(Align.CENTER, Align.CENTER, Align.MIN))
        bosses += Pos(18, 0, 13) * Cylinder(6, 3, align=(Align.CENTER, Align.CENTER, Align.MIN))
        return block + bosses

    def features(self, s):
        s -= ext_box(-SOCKET / 2, SOCKET / 2, -SOCKET / 2, SOCKET / 2, -16, 0)
        s -= Pos(-32, 0, -16) * Cylinder(4.5, 40, align=(Align.CENTER, Align.CENTER, Align.MIN))
        return s

    def fastener_points(self):
        f = [Fastener(down(-32, y, 13), "M4_clear", 30) for y in (-18.5, 18.5)]   # KFL08
        f += [Fastener(down(x, 0, 16), "M5_clear", 30) for x in (-18, 18)]       # roller axles
        return f


# --------------------------------------------------------------------------- carriage and arm
@printed("lift_carriage")
class LiftCarriage(PrintedPart):
    """Rides the mast on four V-wheels and carries the lead-screw nut, the
    telescopic fork's base (two fixed MGN12H blocks at its front), the X motor
    and pinion, both differential belt anchors and the X home switch.
    Origin: mast centre (x, y); z = 0 at the fork underside (the hook rail's
    underside). The fork stack above that is, in mm:

        32 ┬ shelf underside: the two fixed blocks bolt here
           │ rail A (on the middle stage) slides through them   19..27
           │ middle stage bar, 25 x 6 aluminium                 13..19
           │ two blocks under the middle stage's front           3..13
         0 ┴ rail B, the hook rail (top stage)                   0..8
    """
    title = "Lift carriage"
    SHELF = (32.0, 40.0)

    def __init__(self, spec):
        super().__init__(spec)
        self.arm_y = mm(spec.arm_y_offset)
        self.z0 = -mm(CARRIAGE_BELOW_ARM)
        self.z1 = mm(CARRIAGE_H) - mm(CARRIAGE_BELOW_ARM)
        rel = lambda x: mm(x - spec.mast_x)
        self.front_x = rel(spec.fork_front_home)              # shelf ends flush with the folded fork
        self.block_x = tuple(rel(x) for x in spec.carriage_block_x)
        self.pinion_x = rel(spec.fork_pinion_x)
        self.anchor_x = tuple(rel(x) for x in spec.fork_anchor_x)
        self.switch_x = self.front_x - 14 - 1 - 10            # the middle stage's front rack clamp hits it at home

    @property
    def pinion_y(self) -> float:
        """Pinion on the fork's -Y side: bar half width + belt + pulley pitch radius."""
        return self.arm_y - mm(FORK_BAR_W) / 2 - 1.4 - 6.4

    def body(self):
        z0, z1 = self.z0, self.z1
        s0, s1 = self.SHELF
        a = self.arm_y
        back = ext_box(-46, -11, -39, 39, z0, z1)
        front = ext_box(11, 17, -39, 39, z0, z1)
        bridges = ext_box(-11, 11, 33, 39, z0, z1) + ext_box(-11, 11, -39, -33, z0, z1)
        shelf = ext_box(-11, self.front_x, 33, a + 37, s0, s1)              # fixed blocks hang below this
        motor_pad = ext_box(self.pinion_x - 21, self.pinion_x + 21, self.pinion_y - 21, 39, s0, s1)
        web = ext_box(11, 40, 33, 39, z0 + 20, s1)
        sx = self.switch_x
        switch_tab = ext_box(sx - 8, sx + 8, a - 24.4, a - 21.2, 10, s0)    # X home switch hangs off this
        return back + front + bridges + shelf + motor_pad + web + switch_tab

    def features(self, s):
        s -= Pos(-32, 0, self.z0 - 1) * Cylinder(5.4, 100, align=(Align.CENTER, Align.CENTER, Align.MIN))  # screw
        s -= ext_box(-SOCKET / 2 - 0.6, SOCKET / 2 + 0.6, -SOCKET / 2 - 0.6, SOCKET / 2 + 0.6, self.z0 - 1, self.z1 + 1)  # mast
        # X motor: the pinion's shaft passes down through the pad.
        s -= Pos(self.pinion_x, self.pinion_y, self.SHELF[0] - 1) * Cylinder(8, 12, align=(Align.CENTER, Align.CENTER, Align.MIN))
        return s

    def fastener_points(self):
        top = self.SHELF[1]
        f = [Fastener(along_x(17, y, z), "M5_clear", 40) for y in (-20.1, 20.1) for z in (self.z0 + 12, self.z1 - 12)]  # wheel axles
        f += [Fastener(down(-32 + dx, dy, self.z1), "M3_insert", 8) for dx, dy in ((8, 0), (-8, 0), (0, 8), (0, -8))]  # T8 nut
        for bx in self.block_x:   # fixed MGN12H blocks: 4 x M3 each, 20 x 20 pattern
            f += [Fastener(down(bx + dx, self.arm_y + dy, top), "M3_clear", 10) for dx in (-10, 10) for dy in (-10, 10)]
        f += [Fastener(down(self.pinion_x + dx, self.pinion_y + dy, top), "M3_clear", 10) for dx in (-13, 13) for dy in (-13, 13)]  # NEMA14
        f += [Fastener(down(x, self.arm_y + y, top), "M3_clear", 10)                                 # belt anchors
              for x, y in zip(self.anchor_x, (ForkBeltAnchor.Y_RETRACT, ForkBeltAnchor.Y_EXTEND))]
        return f

    def declare_mounts(self):
        return [
            MountDecl("x_home", "endstop", Location((self.switch_x, self.arm_y - 18, 16)), {"axis": "x"}),
            MountDecl("drag_chain_carriage", "cable_port", Location((-46, -30, self.z0))),
        ]


@printed("hook")
class Hook(PrintedPart):
    """The passive J-hook, screwed under the tip of rail B (the fork's top
    stage). It also anchors the retract belt.
    Origin: the CUP, the top centre of the hook's foot. The joint values
    (rail_y, mast_z, arm_x) are this point's world position.

        rail B ════════╗ clamp under the rail (screw down through the rail's end hole)
                       ║ shank
                       ╚════╗ lip    the handle bar rests in the cup
                         cup

    The clamp sits under the rail, not round it, because the middle stage's
    blocks run right over the rail's tip when the fork is folded.
    """
    title = "J-hook with rail clamp and retract belt anchor"
    print_rotation = (90, 0, 0)    # print on its side so layers run along the J

    def __init__(self, spec):
        super().__init__(spec)
        s = spec
        self.sec = mm(s.hook_section)
        self.foot = mm(s.hook_foot)
        self.shank_h = mm(s.hook_shank)
        self.lip = mm(s.hook_lip)
        self.tip_x = -self.foot / 2 - self.sec / 2 + 5.0      # rail tip in this frame
        self.shank_x0 = -self.foot / 2 - self.sec
        self.shank_x1 = -self.foot / 2
        self.half_w = 5.0

    def body(self):
        w, h, t = self.half_w, self.shank_h, self.tip_x
        shank = ext_box(self.shank_x0, self.shank_x1, -w, w, -self.sec, h - 5)
        foot = ext_box(self.shank_x0, self.foot / 2, -w, w, -self.sec, 0)
        lip = ext_box(self.foot / 2, self.foot / 2 + self.sec, -w, w, -self.sec, self.lip)
        clamp = ext_box(t - 16, t + 3, -10, 10, h - 6, h)
        belt_arm = ext_box(t - 16, t - 4, 10, ForkBeltAnchor.LANE_Y_INNER_RETRACT + 2, h - 6, h)
        belt_post = ext_box(t - 16, t - 4, ForkBeltAnchor.LANE_Y_INNER_RETRACT - 2,
                            ForkBeltAnchor.LANE_Y_INNER_RETRACT + 2, h, h + ForkBeltAnchor.LANE_LOW[1])
        return shank + foot + lip + clamp + belt_arm + belt_post

    def fastener_points(self):
        h = self.shank_h
        return [Fastener(down(self.tip_x - 12.5, 0, h + 8), "M3_insert", 14),
                Fastener(along_y(self.tip_x - 10, ForkBeltAnchor.LANE_Y_INNER_RETRACT - 3, h + 5), "M3_clear", 8)]

    def collision_boxes(self):
        """(name, centre, size) in this local frame for the MJCF collision geoms."""
        w = self.half_w
        boxes = {
            "hook_shank": ((self.shank_x0, self.shank_x1), (-w, w), (-self.sec, self.shank_h - 5)),
            "hook_foot": ((self.shank_x0, self.foot / 2), (-w, w), (-self.sec, 0)),
            "hook_lip": ((self.foot / 2, self.foot / 2 + self.sec), (-w, w), (-self.sec, self.lip)),
        }
        out = []
        for name, (xs, ys, zs) in boxes.items():
            c = ((xs[0] + xs[1]) / 2, (ys[0] + ys[1]) / 2, (zs[0] + zs[1]) / 2)
            sz = (xs[1] - xs[0], ys[1] - ys[0], zs[1] - zs[0])
            out.append((name, c, sz))
        return out


# --------------------------------------------------------------------------- telescopic fork
@printed("fork_belt_anchor")
class ForkBeltAnchor(PrintedPart):
    """Clamps the carriage end of one differential belt. The same part serves
    both belts: it hangs from the carriage shelf past both belt lanes, and the
    belt is clamped at whichever lane it runs in.
    Origin: the top face (bolted under the shelf), centred; z runs downward
    from the shelf at fork z = 32.

    Belt lanes, all on the fork's +Y side (y from the fork centreline, z from
    the fork underside, mm):

        extend belt   z 21..27   inner run y 17.6, outer y 30.4, idler at the middle stage's FRONT
        retract belt  z  2..8    inner run y 21.0, outer y 33.8, idler at the middle stage's REAR

    Each belt runs carriage anchor (outer run) -> idler on the middle stage ->
    top stage (inner run). Pattern note: the lanes are class constants that
    the hook, the rear clamp and the idler arms all read, so a belt can never
    be clamped in one lane and wrapped in another.
    """
    title = "Fork belt anchor (carriage)"
    LANE_HIGH = (21.0, 27.0)
    LANE_LOW = (2.0, 8.0)
    LANE_Y_INNER_EXTEND = 17.6
    LANE_Y_INNER_RETRACT = 21.0
    Y_EXTEND = 30.4          # outer runs, where these anchors clamp
    Y_RETRACT = 33.8
    TOP = 32.0               # shelf underside in fork z

    def body(self):
        drop = self.TOP - self.LANE_LOW[0]
        return ext_box(-6, 6, -3, 3, -drop, 0) + ext_box(-6, 6, -3, 3, -6, 0) + ext_box(-10, 10, -3, 3, -4, 0)

    def fastener_points(self):
        f = [Fastener(Location((x, 0, 0), (180, 0, 0)), "M3_insert", 8) for x in (-7, 7)]
        f += [Fastener(along_y(0, -3, z - self.TOP), "M3_clear", 8)
              for z in ((self.LANE_LOW[0] + self.LANE_LOW[1]) / 2, (self.LANE_HIGH[0] + self.LANE_HIGH[1]) / 2)]
        return f


class _ForkMidEnd(PrintedPart):
    """One end of the fork's middle stage: clamps that end of the -Y belt rack
    (the X pinion drives it) and holds one differential idler on an arm that
    reaches past the bar's end.
    Origin: the bar's end, on the fork centreline, z = 0 at the fork underside.
    `sign` is +1 at the front end (the arm reaches +X) and -1 at the rear."""
    sign = 1
    lane = ForkBeltAnchor.LANE_HIGH
    idler_y = ForkBeltAnchor.LANE_Y_INNER_EXTEND + 6.4

    def body(self):
        g = self.sign
        half = mm(FORK_BAR_W) / 2
        bar0, bar1 = 13.0, 19.0
        xs = lambda a, b: (min(g * a, g * b), max(g * a, g * b))
        rack = ext_box(*xs(-14, 0), -20, -half, bar0, bar1)                  # belt rack end clamp, -Y side
        plate = ext_box(*xs(0, 4), -20, self.idler_y + 9, min(self.lane[0] - 1, bar0), max(self.lane[1], bar1))
        if g > 0:   # front: idler in the high lane, arm above it
            arm = ext_box(*xs(0, 22), self.idler_y - 9, self.idler_y + 9, self.lane[1] + 2, self.lane[1] + 4)
        else:       # rear: idler in the low lane, arm above it, under the bar's level
            arm = ext_box(*xs(0, 22), self.idler_y - 9, self.idler_y + 9, self.lane[1] + 2, self.lane[1] + 4)
        return rack + plate + arm

    @property
    def idler_at(self) -> tuple[float, float, float]:
        return (self.sign * 12, self.idler_y, (self.lane[0] + self.lane[1]) / 2)

    def fastener_points(self):
        g = self.sign
        x, y, z = self.idler_at
        f = [Fastener(Location((x, y, self.lane[1] + 4)), "M5_clear", 8)]               # idler axle
        f += [Fastener(along_x(g * 4 if g > 0 else 0, 0, 16), "M3_clear", 10)]            # into the bar end
        f += [Fastener(along_y(-g * 7, -20, 16), "M3_clear", 8)]                          # rack clamp
        return f


@printed("fork_mid_front")
class ForkMidFront(_ForkMidEnd):
    """Front end of the middle stage: rack clamp, extend-belt idler, and the
    face that presses the X home switch when the fork is folded."""
    title = "Fork middle stage, front end"
    sign = 1
    lane = ForkBeltAnchor.LANE_HIGH
    idler_y = ForkBeltAnchor.LANE_Y_INNER_EXTEND + 6.4


@printed("fork_mid_rear")
class ForkMidRear(_ForkMidEnd):
    """Rear end of the middle stage: rack clamp and retract-belt idler."""
    title = "Fork middle stage, rear end"
    sign = -1
    lane = ForkBeltAnchor.LANE_LOW
    idler_y = ForkBeltAnchor.LANE_Y_INNER_RETRACT + 6.4


@printed("hook_camera_mount")
class HookCameraMount(PrintedPart):
    """Holds a Camera Module 3 Wide under rail B (the fork's top stage), just
    behind the hook, looking forward and down at the hook, the handle it is
    about to pick and the box below it. It rides out with the hook, so the
    picture is always the hook's point of view.
    Origin: the tip of rail B, rail bottom centre (same frame as the rail).

        rail B ═══════════════════╗ tip
                  ▔▔▔▔▔ pad        ║ hook
                   ╲ camera, -Y side, tilted down
                    ◣ ── looks this way ──►  handle

    It sits on the -Y side and under the rail because the middle stage's
    blocks run over the rail and the retract belt runs on the +Y side.
    """
    title = "Hook camera mount"
    CAM = (-62.0, -26.0, -10.0)      # lens centre: low enough to clear the blocks above, high enough to pass over handles

    def __init__(self, spec):
        super().__init__(spec)
        self.tilt = spec.hook_camera_tilt_deg

    def cam_location(self) -> Location:
        """Camera frame: lens looks along local -Z, image up is local +Y (the
        MuJoCo and Pi convention). -Z points forward (+X) and `tilt` down."""
        from build123d import Plane
        a = math.radians(self.tilt)
        return Location(Plane(self.CAM, x_dir=(0, -1, 0), z_dir=(-math.cos(a), 0, math.sin(a))))

    def body(self):
        cx, cy, cz = self.CAM
        pad = ext_box(-75, -50, -8, 8, -5, 0)                       # screws under the rail
        arm = ext_box(-75, -60, -36, -8, -5, 0)
        drop = ext_box(-75, -71, -36, -12, cz + 4, -5)
        plate = self.cam_location() * Pos(0, 0, 2.5) * Box(28, 28, 3)
        return pad + arm + drop + plate

    def features(self, s):
        c = self.cam_location()
        s -= c * Pos(0, 0, 2) * Cylinder(5, 8)                        # lens hole
        for dx in (-10.5, 10.5):
            for dy in (-6.25, 6.25):
                s -= c * Pos(dx, dy, 2.5) * Cylinder(1.1, 6)          # M2 board screws
        return s

    def fastener_points(self):
        return [Fastener(down(-62.5, 0, 8), "M3_insert", 13)]         # down through the rail's 3rd hole

    def declare_mounts(self):
        return [MountDecl("camera", "camera", self.cam_location(), {"fovy": 67.0}),
                MountDecl("csi_port", "cable_port", Location((-73, -30, -5)))]


@printed("arm_rear_clamp")
class ArmRearClamp(PrintedPart):
    """Screws under the rear end of rail B (the top stage) and anchors the
    extend belt, which pulls the hook out.
    Origin: rear end of the rail, rail bottom centre."""
    title = "Fork top stage rear clamp (extend belt anchor)"

    def body(self):
        hi = ForkBeltAnchor.LANE_HIGH
        y_in = ForkBeltAnchor.LANE_Y_INNER_EXTEND
        pad = ext_box(0, 16, -10, 16.5, -6, 0)
        riser = ext_box(4, 14, 14, 16.5, -6, hi[1])          # clears the middle stage's blocks (y 13.5)
        grip = ext_box(4, 14, 14, y_in + 2.5, hi[0], hi[1])
        return pad + riser + grip

    def fastener_points(self):
        return [Fastener(down(12.5, 0, 8), "M3_insert", 14),
                Fastener(along_y(9, 14, 24), "M3_clear", 8)]


# --------------------------------------------------------------------------- frame
@printed("rail_end")
class RailEnd(PrintedPart):
    """Caps one end of the floor rail, clamps the flat Y belt, and holds an
    upright. Built for the far end (rail interior toward -Y); the near end
    uses the same part turned 180 degrees.
    Origin: rail centreline at the rail end, rail underside (z = 0)."""
    title = "Rail end block"

    def body(self):
        return ext_box(-30, 30, -60, 5, 0, 48)

    def features(self, s):
        s -= ext_box(-20.2, 20.2, -61, 0, -1, 20.2)                        # 2040 socket
        s -= ext_box(-SOCKET / 2, SOCKET / 2, -30 - SOCKET / 2, -30 + SOCKET / 2, 28, 49)   # upright socket
        s -= ext_box(-4, 4, -61, -40, 20, 23.5)                            # belt end slot
        return s

    def fastener_points(self):
        f = [Fastener(down(x, -15, 48), "M5_clear", 60) for x in (-25, 25)]
        f += [Fastener(down(0, -50, 48), "M3_clear", 30)]                  # belt clamp screw
        return f


@printed("top_corner")
class TopCorner(PrintedPart):
    """Joins an upright to the top guide rail.
    Origin: upright centreline at the guide rail's underside."""
    title = "Top corner"

    def body(self):
        return ext_box(-16, 16, -20, 20, -22, 12)

    def features(self, s):
        s -= ext_box(-SOCKET / 2, SOCKET / 2, -SOCKET / 2, SOCKET / 2, -23, 0)
        s -= ext_box(-SOCKET / 2, SOCKET / 2, -21, 21, 0, 13)
        return s

    def fastener_points(self):
        return [Fastener(along_x(16, 0, -11), "M5_clear", 40), Fastener(along_x(16, 0, 6), "M5_clear", 40)]


@printed("rail_mid_support")
class RailMidSupport(PrintedPart):
    """Spacer under the middle of the floor rail.
    Origin: rail centreline, floor (z = 0)."""
    title = "Rail mid support"

    def body(self):
        return ext_box(-28, 28, -15, 15, 0, 20) + ext_box(-28, 28, -15, 15, 20, 26) - ext_box(-20.2, 20.2, -16, 16, 20, 27)

    def fastener_points(self):
        return [Fastener(down(0, 0, 20), "M5_clear", 20)]


@printed("endstop_bracket")
class EndstopBracket(PrintedPart):
    """Holds a microswitch (holes 9.5 mm apart) on an extrusion slot or plate.
    Origin: centre of the switch body."""
    title = "Endstop bracket"

    def body(self):
        return ext_box(-14, 14, -3.2 - 4, -3.2, -8, 8) + ext_box(-14, 14, -15, -3.2, -8, -4)

    def fastener_points(self):
        f = [Fastener(along_y(x, -8, -1.5), "M2_clear", 8) for x in (-4.75, 4.75)]
        f += [Fastener(down(0, -11, -4), "M3_clear", 6)]
        return f

    def declare_mounts(self):
        return [MountDecl("switch", "endstop", Location((0, 0, 0)))]


@printed("drag_chain_bracket")
class DragChainBracket(PrintedPart):
    """L bracket for a drag-chain end. Origin: bracket corner."""
    title = "Drag chain bracket"

    def body(self):
        return ext_box(-15, 15, 0, 25, 0, 4) + ext_box(-15, 15, 0, 4, 0, 25)

    def fastener_points(self):
        f = [Fastener(down(x, 15, 4), "M3_clear", 6) for x in (-8, 8)]
        f += [Fastener(along_y(x, 0, 15), "M3_clear", 6) for x in (-8, 8)]
        return f


@printed("corner_bracket")
class CornerBracket(PrintedPart):
    """L bracket joining two 2020 extrusions at right angles (rack, exit tray).
    Origin: inside corner."""
    title = "Corner bracket"

    def body(self):
        return ext_box(0, 26, -10, 10, 0, 5) + ext_box(0, 5, -10, 10, 0, 26) + ext_box(5, 14, -2, 2, 5, 14)   # base, upright, rib

    def fastener_points(self):
        return [Fastener(down(17, 0, 5), "M5_clear", 6), Fastener(along_x(5, 0, 17), "M5_clear", 6)]


# --------------------------------------------------------------------------- enclosures
PI_HOLES = (58.0, 49.0)
SKR_PICO_HOLES = (77.0, 49.0)   # TODO: check against BTT's SKR Pico drawing before printing


@printed("main_enclosure_base")
class MainEnclosureBase(PrintedPart):
    """Pi 5 + SKR Pico + 5 V buck. Hangs on an upright above the water with
    the cable glands facing the floor. Origin: centre of the outside of the floor."""
    title = "Main enclosure base"
    IX, IY, IZ, W, F = 90.0, 125.0, 45.0, 2.5, 3.0

    def body(self):
        ox, oy = self.IX / 2 + self.W, self.IY / 2 + self.W
        outer = ext_box(-ox, ox, -oy, oy, 0, self.F + self.IZ)
        return outer - ext_box(-self.IX / 2, self.IX / 2, -self.IY / 2, self.IY / 2, self.F, self.F + self.IZ + 1)

    def features(self, s):
        for cy, holes in ((-31.0, PI_HOLES), (31.0, SKR_PICO_HOLES)):
            for sx in (-1, 1):
                for sy in (-1, 1):
                    s += Pos(sx * holes[0] / 2, cy + sy * holes[1] / 2, self.F) * Cylinder(3, 5, align=(Align.CENTER, Align.CENTER, Align.MIN))
        for gy in (-45, -15, 15, 45):                                  # PG7 glands in the floor
            s -= Pos(self.IX / 2 - 12, gy, -1) * Cylinder(6.35, self.F + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
        return s

    def fastener_points(self):
        f = []
        for cy, holes in ((-31.0, PI_HOLES), (31.0, SKR_PICO_HOLES)):
            f += [Fastener(down(sx * holes[0] / 2, cy + sy * holes[1] / 2, self.F + 5), "M2_clear", 8)
                  for sx in (-1, 1) for sy in (-1, 1)]
        f += [Fastener(along_x(-self.IX / 2, y, 30), "M5_clear", 6) for y in (-30, 30)]   # onto the upright
        return f

    def declare_mounts(self):
        return [MountDecl("main_glands", "cable_port", Location((self.IX / 2 - 12, 0, 0)))]


@printed("main_enclosure_lid")
class MainEnclosureLid(PrintedPart):
    """Lid with vents and the E-stop. Origin: centre of the lid's underside."""
    title = "Main enclosure lid"

    def body(self):
        b = MainEnclosureBase
        ox, oy = b.IX / 2 + b.W, b.IY / 2 + b.W
        plate = ext_box(-ox, ox, -oy, oy, 0, 3)
        lip = ext_box(-b.IX / 2 + 0.3, b.IX / 2 - 0.3, -b.IY / 2 + 0.3, b.IY / 2 - 0.3, -4, 0)
        lip -= ext_box(-b.IX / 2 + 2.3, b.IX / 2 - 2.3, -b.IY / 2 + 2.3, b.IY / 2 - 2.3, -5, 0)
        return plate + lip

    def features(self, s):
        s -= Pos(0, 40, -1) * Cylinder(11.25, 5, align=(Align.CENTER, Align.CENTER, Align.MIN))   # 22 mm E-stop
        for i in range(6):
            s -= ext_box(-30, 30, -50 + i * 9, -46 + i * 9, -1, 4)
        return s

    def declare_mounts(self):
        return [MountDecl("estop", "estop", Location((0, 40, 3)))]


@printed("sensor_node_base")
class SensorNodeBase(PrintedPart):
    """Wireless water sensor node: ESP32-C3, 18650 cell, TDS and pH boards.
    Clips over the tub rim; probe cables leave through the floor.
    Origin: inside face of the clip (touches the tub wall), floor level."""
    title = "Water sensor node base"
    IX, IY, IZ, W = 50.0, 85.0, 30.0, 2.5

    def body(self):
        ox = self.IX + 2 * self.W
        oy = self.IY / 2 + self.W
        box = ext_box(0, ox, -oy, oy, 0, self.IZ + 2)
        box -= ext_box(self.W, ox - self.W, -self.IY / 2, self.IY / 2, 2, self.IZ + 3)
        tab = ext_box(-4.5 - 3, 0, -20, 20, self.IZ - 1, self.IZ + 2)        # over the rim
        finger = ext_box(-4.5 - 3, -4.5, -20, 20, self.IZ - 16, self.IZ + 2)  # inside the tub wall
        return box + tab + finger

    def features(self, s):
        for y in (-25, 0, 25):
            s -= Pos(self.IX / 2 + self.W, y, -1) * Cylinder(4.2, 4, align=(Align.CENTER, Align.CENTER, Align.MIN))
        return s

    def declare_mounts(self):
        return [MountDecl("node", "sensor_node", Location((self.IX / 2, 0, 0)))]


@printed("sensor_node_lid")
class SensorNodeLid(PrintedPart):
    """Origin: lid underside at the clip face."""
    title = "Water sensor node lid"

    def body(self):
        b = SensorNodeBase
        ox = b.IX + 2 * b.W
        oy = b.IY / 2 + b.W
        lip = ext_box(b.W + 0.3, ox - b.W - 0.3, -b.IY / 2 + 0.3, b.IY / 2 - 0.3, -4, 0)
        lip -= ext_box(b.W + 2.3, ox - b.W - 2.3, -b.IY / 2 + 2.3, b.IY / 2 - 2.3, -5, 0)
        return ext_box(0, ox, -oy, oy, 0, 2.5) + lip


# --------------------------------------------------------------------------- camera, exit, boxes
@printed("camera_bracket")
class CameraBracket(PrintedPart):
    """Clamps on a 2020 crossbar on the far side of a tier and points a Camera
    Module 3 down at the tub, tilted back toward the aisle.
    Origin: crossbar centreline (the bar runs along Y)."""
    title = "Camera bracket"

    def __init__(self, spec):
        super().__init__(spec)
        self.tilt = spec.camera_tilt_deg

    def body(self):
        clamp = ext_box(-13, 13, -15, 15, -13, 13) - ext_box(-SOCKET / 2, SOCKET / 2, -16, 16, -SOCKET / 2, SOCKET / 2)
        clamp -= ext_box(-6, 6, -16, 16, -14, -SOCKET / 2)                  # slot so it snaps over the bar
        arm = ext_box(-50, -13, -6, 6, 9, 13)
        plate = Pos(-58, 0, 6) * Rot(0, self.tilt, 0) * Box(30, 30, 3)
        return clamp + arm + plate

    def features(self, s):
        p = Pos(-58, 0, 6) * Rot(0, self.tilt, 0)
        s -= p * Cylinder(5, 10)
        for dx in (-10.5, 10.5):
            for dy in (-6.25, 6.25):
                s -= p * Pos(dx, dy, 0) * Cylinder(1.1, 10)
        return s

    def declare_mounts(self):
        # The lens looks out of the plate's underside: local -Z of this frame.
        return [MountDecl("camera", "camera", Pos(-58, 0, 6 - 2.5) * Rot(0, self.tilt, 0), {"fovy": 67.0})]


@printed("exit_cradle")
class ExitCradle(PrintedPart):
    """One exit-tray slot: a dish with corner guides that sits on a 2020
    crossbar. Origin: dish top centre, where the box's bottom lands."""
    title = "Exit tray cradle"

    def body(self):
        bs = mm(self.spec.box_size) + 10        # 5 mm each side so a slightly-off box still drops in
        base = ext_box(-bs / 2 - 6, bs / 2 + 6, -bs / 2 - 6, bs / 2 + 6, -4, 0)
        guides = None
        for sx in (-1, 1):
            for sy in (-1, 1):
                g = (ext_box(min(sx * bs / 2, sx * (bs / 2 + 4)), max(sx * bs / 2, sx * (bs / 2 + 4)),
                             min(sy * (bs / 2 - 14), sy * (bs / 2 + 4)), max(sy * (bs / 2 - 14), sy * (bs / 2 + 4)), 0, 8)
                     + ext_box(min(sx * (bs / 2 - 14), sx * (bs / 2 + 4)), max(sx * (bs / 2 - 14), sx * (bs / 2 + 4)),
                               min(sy * bs / 2, sy * (bs / 2 + 4)), max(sy * bs / 2, sy * (bs / 2 + 4)), 0, 8))
                guides = g if guides is None else guides + g
        rails = ext_box(-bs / 2 - 6, bs / 2 + 6, 10.3, 14, -12, -4) + ext_box(-bs / 2 - 6, bs / 2 + 6, -14, -10.3, -12, -4)
        return base + guides + rails

    def features(self, s):
        for x in (-30, 0, 30):
            for y in (-30, 30):
                s -= Pos(x, y, -5) * Cylinder(4, 6, align=(Align.CENTER, Align.CENTER, Align.MIN))   # drain holes
        return s

    def declare_mounts(self):
        return [MountDecl("slot", "exit_slot", Location((0, 0, 0)))]

    def contact_boxes(self):
        """The dish and its guides take part in contact, so a box can rest here."""
        bs = mm(self.spec.box_size) + 10
        e = bs / 2 + 4
        bx = box_from_extents
        return [
            bx("dish", (-bs / 2 - 6, bs / 2 + 6), (-bs / 2 - 6, bs / 2 + 6), (-4, 0)),
            bx("guide_xn", (-e, -bs / 2), (-e, e), (0, 8)),
            bx("guide_xp", (bs / 2, e), (-e, e), (0, 8)),
            bx("guide_yn", (-e, e), (-e, -bs / 2), (0, 8)),
            bx("guide_yp", (-e, e), (bs / 2, e), (0, 8)),
        ]


@printed("mesh_box")
class MeshBox(PrintedPart):
    """Mesh box for one crawfish, with the handle bar the hook lifts.
    The bar runs along Y so the hook slides under it moving +X.
    Origin: centre of the box's underside."""
    title = "Mesh box with handle"
    WALL = 2.0

    def body(self):
        s = self.spec
        b, h = mm(s.box_size), mm(s.box_height)
        hz = h + mm(s.handle_height)
        r = mm(s.handle_dia) / 2
        w = self.WALL
        shell = ext_box(-b / 2, b / 2, -b / 2, b / 2, 0, h) - ext_box(-b / 2 + w, b / 2 - w, -b / 2 + w, b / 2 - w, w, h + 1)
        posts = ext_box(-3, 3, b / 2 - w - 5, b / 2 - w, w, hz + r + 2) + ext_box(-3, 3, -b / 2 + w, -b / 2 + w + 5, w, hz + r + 2)
        bar = Pos(0, 0, hz) * Rot(90, 0, 0) * Cylinder(r, b - 2 * w)
        return shell + posts + bar

    def features(self, s):
        sp = self.spec
        b, h = mm(sp.box_size), mm(sp.box_height)
        slot_z0, slot_z1 = 8, h - 10
        for i in range(6):                                     # wall slots, all four sides
            off = -b / 2 + 12 + i * (b - 24) / 5
            if abs(off) < 8:
                continue                                       # keep solid behind the handle posts
            s -= ext_box(off - 1.6, off + 1.6, -b / 2 - 1, b / 2 + 1, slot_z0, slot_z1)
            s -= ext_box(-b / 2 - 1, b / 2 + 1, off - 1.6, off + 1.6, slot_z0, slot_z1)
        for ix in range(4):                                    # floor holes
            for iy in range(4):
                s -= Pos(-b / 2 + 15 + ix * (b - 30) / 3, -b / 2 + 15 + iy * (b - 30) / 3, -1) * Cylinder(3, 4, align=(Align.CENTER, Align.CENTER, Align.MIN))
        return s

    def contact_boxes(self):
        """Simple boxes for the preview sim: floor, walls, posts and the bar."""
        s = self.spec
        b, h = mm(s.box_size), mm(s.box_height)
        hz = h + mm(s.handle_height)
        r = mm(s.handle_dia) / 2
        w = self.WALL
        bx = box_from_extents
        return [
            bx("floor", (-b / 2, b / 2), (-b / 2, b / 2), (0, w)),
            bx("wall_xn", (-b / 2, -b / 2 + w), (-b / 2, b / 2), (0, h)),
            bx("wall_xp", (b / 2 - w, b / 2), (-b / 2, b / 2), (0, h)),
            bx("wall_yn", (-b / 2, b / 2), (-b / 2, -b / 2 + w), (0, h)),
            bx("wall_yp", (-b / 2, b / 2), (b / 2 - w, b / 2), (0, h)),
            bx("post_n", (-3, 3), (-b / 2 + w, -b / 2 + w + 5), (w, hz + r + 2)),
            bx("post_p", (-3, 3), (b / 2 - w - 5, b / 2 - w), (w, hz + r + 2)),
            bx("handle", (-r, r), (-b / 2 + w + 5, b / 2 - w - 5), (hz - r, hz + r)),
        ]
