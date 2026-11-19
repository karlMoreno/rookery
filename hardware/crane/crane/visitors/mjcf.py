"""MjcfVisitor: the assembly tree as a MuJoCo model.

Pattern: Visitor. One walk of the tree writes the <body> nesting, joints,
meshes (one OBJ per body per colour), explicit inertials, the hook's
collision boxes, and a <site>/<camera> for every registered mount. Because the
tree's BodyNodes are exactly MuJoCo bodies, the walk is a direct translation.

Pattern: Strategy (mesh format). The visitor is handed a MeshExporter
(ObjExporter by default) and never writes mesh bytes itself.

Outputs, written by `write_models`:
    crane.xml    crane + fixtures only, no floor or lights. Rookery's scene.py
                 adds it with <include file="crane.xml"/> at the top level.
    preview.xml  includes crane.xml, adds a floor, lights, the rack, and the
                 mesh boxes as free bodies so the hook can really lift them.

Rookery conventions kept: joint names rail_y / mast_z / arm_x / lift (in that
order, so their indices don't change; the fork's middle stage adds a fifth,
`arm_mid`, which no actuator drives: an <equality> ties it to arm_x), joint
`ref` = home so MuJoCo's reset puts the crane at home, timestep 1 ms, and only
the hook collides with anything.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np
from build123d import Compound, Location

from crane.exporters import MeshExporter, ObjExporter
from crane.registry import MountRegistry
from crane.spec import CraneSpec
from crane.tree import BodyNode, PartNode, Visitor


def f6(x: float) -> str:
    """Every float through one format, so regenerating gives identical bytes."""
    v = f"{x:.6f}".rstrip("0").rstrip(".")
    return "0" if v in ("-0", "") else v


def vec(xs) -> str:
    return " ".join(f6(x) for x in xs)


class MjcfVisitor(Visitor):
    def __init__(self, spec: CraneSpec, registry: MountRegistry, out_dir: Path, groups: set[str],
                 mesh_exporter: MeshExporter | None = None, free_keys: set[str] | None = None,
                 mesh_prefix: str = ""):
        self.s = spec
        self.reg = registry
        self.out = Path(out_dir)
        self.mesh_dir = self.out / "meshes"
        self.mesh_dir.mkdir(parents=True, exist_ok=True)
        self.groups = groups
        self.exporter = mesh_exporter or ObjExporter()
        self.free_keys = free_keys or set()          # parts that become free bodies (preview boxes)
        self.prefix = mesh_prefix
        self.root = ET.Element("mujoco", model="rookery_crane")
        self.asset = ET.Element("asset")
        self.worldbody = ET.Element("worldbody")
        self.joints: list = []
        self._stack: list[tuple[BodyNode, ET.Element, dict, list]] = []
        self._free: list[PartNode] = []
        self._meshes: set[str] = set()

    # ------------------------------------------------------------ walk
    def enter_body(self, node: BodyNode) -> bool:
        if node.name == "world":
            self._stack.append((node, self.worldbody, defaultdict(list), []))
            return True
        if node.group not in self.groups:
            return False
        parent_node, parent_el, _, _ = self._stack[-1]
        rel = [(a - b) / 1000.0 for a, b in zip(node.origin, parent_node.origin)]
        el = ET.SubElement(parent_el, "body", name=node.name, pos=vec(rel))
        j = node.joint
        if j is not None:
            attrs = {"name": j.name, "type": "slide", "axis": vec(j.axis)}
            if j.locked:
                attrs.update(range="-0.001 0.001", limited="true", ref="0")
            else:
                attrs.update(range=vec(j.range), limited="true", ref=f6(j.ref), damping="20", armature="0.01")
            ET.SubElement(el, "joint", **attrs)
            self.joints.append(j)
        self._stack.append((node, el, defaultdict(list), []))
        return True

    def visit_part(self, node: PartNode) -> None:
        body, el, by_colour, parts = self._stack[-1]
        if node.part.key in self.free_keys:
            self._free.append(node)
            return
        by_colour[node.part.color].append(node.placement * node.part.shape())
        parts.append(node)

    def exit_body(self, node: BodyNode) -> None:
        body, el, by_colour, parts = self._stack.pop()
        origin = Location(tuple(-c for c in body.origin))
        name = body.name if body.name != "world" else "world"
        for i, (colour, shapes) in enumerate(sorted(by_colour.items())):
            mesh_name = f"{self.prefix}{name}_{i}"
            shape = origin * Compound(children=shapes)
            self.exporter.export(shape, self.mesh_dir / mesh_name)
            ET.SubElement(self.asset, "mesh", name=mesh_name, file=f"meshes/{mesh_name}{self.exporter.suffix}",
                          scale="0.001 0.001 0.001")
            ET.SubElement(el, "geom", type="mesh", mesh=mesh_name, contype="0", conaffinity="0",
                          group="1", rgba=vec(colour), mass="0")
        for cb in body.collisions:
            pos = [(c - o) / 1000.0 for c, o in zip(cb.center, body.origin)]
            ET.SubElement(el, "geom", name=cb.name, type="box", pos=vec(pos), size=vec([s / 2000.0 for s in cb.size]),
                          contype="1", conaffinity="1", group="3", rgba="0.9 0.2 0.2 0.5", friction="0.8 0.01 0.001",
                          condim="4", mass="0")
        for p in parts:                                   # static contact parts (tubs in the preview)
            for cname, c, size in p.part.contact_boxes():
                wc = p.placement * Location(c)
                pos = [(tuple(wc.position)[k] - body.origin[k]) / 1000.0 for k in range(3)]
                ET.SubElement(el, "geom", name=f"{p.name}_{cname}", type="box", pos=vec(pos),
                              size=vec([s / 2000.0 for s in size]), contype="1", conaffinity="1", group="3",
                              rgba="0.3 0.5 0.9 0.15", mass="0")
        for m in self.reg.on_body(body.name):
            pos = [(tuple(m.position_mm)[k] - body.origin[k]) / 1000.0 for k in range(3)]
            if m.kind == "camera":
                R = _rotation(m.world)
                ET.SubElement(el, "camera", name=m.name, pos=vec(pos), xyaxes=vec(list(R[:, 0]) + list(R[:, 1])),
                              fovy=f6(m.meta.get("fovy", 67.0)))
            else:
                ET.SubElement(el, "site", name=m.name, pos=vec(pos), size="0.006", group="4",
                              rgba="0.1 0.8 0.3 0.8" if m.kind == "endstop" else "0.9 0.9 0.1 0.8")
        if body.joint is not None and parts:
            self._inertial(el, body, parts)

    # ------------------------------------------------------------ helpers
    @staticmethod
    def _inertial(el, body: BodyNode, parts: list[PartNode]) -> None:
        """Mass and inertia from the parts: each part as a solid box of its
        bounding box with its mass, combined with the parallel-axis theorem.
        Written explicitly so MuJoCo never guesses from the visual meshes."""
        masses, centres, boxes = [], [], []
        for p in parts:
            bb = (p.placement * p.part.shape()).bounding_box()
            masses.append(max(p.part.mass_kg, 1e-4))
            centres.append(np.array([bb.center().X, bb.center().Y, bb.center().Z]) / 1000.0)
            boxes.append(np.array([bb.size.X, bb.size.Y, bb.size.Z]) / 1000.0)
        M = sum(masses)
        com = sum(m * c for m, c in zip(masses, centres)) / M
        I = np.zeros((3, 3))
        for m, c, sz in zip(masses, centres, boxes):
            Ib = np.diag([sz[1] ** 2 + sz[2] ** 2, sz[0] ** 2 + sz[2] ** 2, sz[0] ** 2 + sz[1] ** 2]) * m / 12
            d = c - com
            I += Ib + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        origin = np.array(body.origin) / 1000.0
        ET.SubElement(el, "inertial", pos=vec(com - origin), mass=f6(M),
                      fullinertia=vec([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]))

    def free_bodies(self) -> None:
        """Preview only: each free part (mesh box) becomes its own body with a
        free joint, its mesh, and its contact boxes."""
        done: dict[str, str] = {}
        for p in self._free:
            key = p.part.key
            if key not in done:
                mesh_name = f"{self.prefix}{key}"
                self.exporter.export(p.part.shape(), self.mesh_dir / mesh_name)
                ET.SubElement(self.asset, "mesh", name=mesh_name, file=f"meshes/{mesh_name}{self.exporter.suffix}",
                              scale="0.001 0.001 0.001")
                done[key] = mesh_name
            pos = [v / 1000.0 for v in tuple(p.placement.position)]
            b = ET.SubElement(self.worldbody, "body", name=p.name, pos=vec(pos))
            ET.SubElement(b, "freejoint", name=f"{p.name}_free")
            mass = p.part.mass_kg + 0.04                       # box plus a crawfish
            ET.SubElement(b, "inertial", pos="0 0 0.02", mass=f6(mass), diaginertia="0.00006 0.00006 0.00008")
            ET.SubElement(b, "geom", type="mesh", mesh=done[key], contype="0", conaffinity="0", group="1",
                          rgba=vec(p.part.color), mass="0")
            for cname, c, size in p.part.contact_boxes():
                ET.SubElement(b, "geom", name=f"{p.name}_{cname}", type="box", pos=vec([v / 1000.0 for v in c]),
                              size=vec([s / 2000.0 for s in size]), contype="1", conaffinity="1", group="3",
                              rgba="0.95 0.6 0.1 0.2", friction="0.8 0.01 0.001", condim="4", mass="0")

    def actuators_and_sensors(self, root: ET.Element) -> None:
        act = ET.SubElement(root, "actuator")
        sen = ET.SubElement(root, "sensor")
        eq = None
        for j in self.joints:
            if j.locked:
                eq = eq if eq is not None else ET.SubElement(root, "equality")
                ET.SubElement(eq, "joint", name=f"{j.name}_lock", joint1=j.name, polycoef="0 0 0 0 0")
                continue
            if j.follows is not None:
                # The fork's middle stage: MuJoCo's joint equality is
                # q1 - ref1 = a0 + a1 (q2 - ref2), so a1 is the stage ratio.
                eq = eq if eq is not None else ET.SubElement(root, "equality")
                ET.SubElement(eq, "joint", name=f"{j.name}_follow", joint1=j.name, joint2=j.follows,
                              polycoef=f"0 {f6(j.ratio)} 0 0 0", solref="0.005 1")
                ET.SubElement(sen, "jointpos", name=f"{j.name}_pos", joint=j.name)
                continue
            kp = {"rail_y": 2000, "mast_z": 4000, "arm_x": 1500}.get(j.name, 1000)
            ET.SubElement(act, "position", name=f"{j.name}_act", joint=j.name, kp=str(kp),
                          ctrlrange=vec(j.range), ctrllimited="true", forcerange="-150 150", forcelimited="true")
            ET.SubElement(sen, "jointpos", name=f"{j.name}_pos", joint=j.name)
            ET.SubElement(sen, "jointvel", name=f"{j.name}_vel", joint=j.name)

    def tree_xml(self) -> tuple[ET.Element, ET.Element]:
        return self.asset, self.worldbody


def _rotation(loc: Location) -> np.ndarray:
    """3x3 rotation of a build123d Location (columns are the local axes)."""
    t = loc.wrapped.Transformation()
    return np.array([[t.Value(r, c) for c in (1, 2, 3)] for r in (1, 2, 3)])


def _pretty(el: ET.Element) -> str:
    ET.indent(el, space="  ")
    return ET.tostring(el, encoding="unicode") + "\n"


def write_models(spec: CraneSpec, world: BodyNode, registry: MountRegistry, out_dir: Path,
                 exporter: MeshExporter | None = None) -> dict[str, Path]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # crane.xml: crane + fixtures
    v = MjcfVisitor(spec, registry, out_dir, {"crane", "fixtures"}, exporter)
    world.accept(v)
    root = ET.Element("mujoco", model="rookery_crane")
    ET.SubElement(root, "compiler", angle="radian", inertiafromgeom="false", autolimits="true")
    ET.SubElement(root, "option", timestep="0.001", integrator="implicitfast")
    root.append(v.asset)
    root.append(v.worldbody)
    v.actuators_and_sensors(root)
    crane_xml = out_dir / "crane.xml"
    crane_xml.write_text(_pretty(root))

    # preview.xml: include crane.xml, add floor, lights, rack, free boxes
    r = MjcfVisitor(spec, registry, out_dir, {"rack"}, exporter, free_keys={"mesh_box"}, mesh_prefix="rack_")
    world.accept(r)
    r.free_bodies()
    prev = ET.Element("mujoco", model="rookery_crane_preview")
    ET.SubElement(prev, "include", file="crane.xml")
    vis = ET.SubElement(prev, "visual")
    ET.SubElement(vis, "global", offwidth="1280", offheight="960")
    ET.SubElement(vis, "headlight", ambient="0.35 0.35 0.35", diffuse="0.5 0.5 0.5")
    asset = r.asset
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker", rgb1="0.32 0.34 0.36",
                  rgb2="0.26 0.28 0.30", width="256", height="256")
    ET.SubElement(asset, "material", name="grid", texture="grid", texrepeat="8 8")
    prev.append(asset)
    wb = r.worldbody
    ET.SubElement(wb, "light", pos="0.3 0.3 2.5", dir="0 0 -1", diffuse="0.6 0.6 0.6")
    ET.SubElement(wb, "geom", name="floor", type="plane", size="2 2 0.05", material="grid", contype="1", conaffinity="1")
    ET.SubElement(wb, "camera", name="overview", pos="1.55 -0.95 1.25", xyaxes="0.6 0.8 0 -0.35 0.26 0.9")
    prev.append(wb)
    preview_xml = out_dir / "preview.xml"
    preview_xml.write_text(_pretty(prev))
    return {"crane": crane_xml, "preview": preview_xml}
