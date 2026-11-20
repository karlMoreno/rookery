"""The crane's look for Rookery's scene: its meshes, and where to hang them.

Rookery's scene builder (scene/scene/builder.py) draws the crane's collision
shapes as plain boxes. With this export it hangs the CAD meshes on the same
bodies as look-only geoms (no mass, no contact), so the sim looks like the
real crane while its physics stays exactly what the gates test.

    python -m crane --spec presets/rookery_tray.toml look

writes `meshes/rookery/`:

    look.xml     the crane bodies only (no fixtures, no actuators): one <body>
                 per moving part, its joints and its mesh geoms. The builder
                 reads body positions, joint refs and mesh names from it.
    meshes/      one OBJ per body per colour, as in crane.xml.
    anchor.json  the two numbers the builder can't read from look.xml:
                 the rack's centre along the rail (so the frame is centred on
                 Rookery's rack) and where a handle sits on the hook.

Pattern: Facade. `write_look` runs MjcfVisitor over the "crane" group and
reduces its output to the three files above; the builder never imports the
CAD stack (build123d), only reads these files. The files are committed and
`tests/test_look.py` fails when they drift from what the preset builds.

    hardware/crane                         scene/
    ┌───────────────┐   look.xml       ┌──────────────────────┐
    │ write_look()  │──meshes/*.obj───▶│ MjcfBuilder.cad_look │
    │ (MjcfVisitor) │   anchor.json    │ (ElementTree only)   │
    └───────────────┘                  └──────────────────────┘
"""
from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

from crane.registry import MountRegistry
from crane.spec import CraneSpec
from crane.tree import BodyNode, Visitor
from crane.visitors.mjcf import MjcfVisitor, _pretty, f6

#: The preset whose crane matches the sim's limits (fork stages 0.49 m,
#: arm travel 0.78 m, 1.05 m mast). Both Rookery scenes use it.
LOOK_PRESET = "rookery_tray.toml"


class _HookSeat(Visitor):
    """Where a handle sits on the hook, in the hook body's frame (metres):
    halfway between the shank and the lip in X, on top of the foot in Z."""

    def __init__(self):
        self.seat: list[float] | None = None

    def enter_body(self, node: BodyNode) -> bool:
        if node.name == "hook":
            c = {cb.name: cb for cb in node.collisions}
            shank, foot, lip = c["hook_shank"], c["hook_foot"], c["hook_lip"]
            x = (shank.center[0] + lip.center[0]) / 2 - node.origin[0]
            z = foot.center[2] + foot.size[2] / 2 - node.origin[2]
            self.seat = [x / 1000.0, 0.0, z / 1000.0]
        return True

    def visit_part(self, node) -> None:
        pass

    def exit_body(self, node: BodyNode) -> None:
        pass


def write_look(spec: CraneSpec, world: BodyNode, registry: MountRegistry, out_dir: Path) -> dict[str, Path]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    v = MjcfVisitor(spec, registry, out_dir, {"crane"})
    world.accept(v)
    for el in v.worldbody.iter():                 # look only: drop cameras, sites, inertials, collision boxes
        for child in list(el):
            if child.tag in ("camera", "site", "inertial") or (child.tag == "geom" and child.get("type") != "mesh"):
                el.remove(child)
    root = ET.Element("mujoco", model="rookery_crane_look")
    root.append(v.asset)
    root.append(v.worldbody)
    look = out_dir / "look.xml"
    look.write_text(_pretty(root))

    r = MjcfVisitor(spec, registry, out_dir / "_rack", {"rack"}, free_keys={"mesh_box"})
    world.accept(r)
    ys = [tuple(p.placement.position)[1] / 1000.0 for p in r._free]
    for f in (out_dir / "_rack" / "meshes").glob("*"):
        f.unlink()
    (out_dir / "_rack" / "meshes").rmdir()
    (out_dir / "_rack").rmdir()
    seat = _HookSeat()
    world.accept(seat)
    anchor = {
        "preset": LOOK_PRESET,
        "rack_center_y": float(f6(sum(ys) / len(ys))),
        "hook_seat": [float(f6(x)) for x in seat.seat],
    }
    a = out_dir / "anchor.json"
    a.write_text(json.dumps(anchor, indent=2) + "\n")
    return {"look": look, "anchor": a}
