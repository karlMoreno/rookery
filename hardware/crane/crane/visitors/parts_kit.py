"""PartsKitVisitor: one CAD file for every individual piece of the crane.

Output (default out/parts/):

    out/parts/
    ├── index.html          every part as a card: picture, qty, where it goes, files
    ├── parts.json          the same list for scripts (show_part.py --list reads it)
    ├── printed/<key>.step  + <key>.stl + <key>.png     the parts you print
    └── bought/<key>.step   + <key>.stl + <key>.png     the parts you buy (stand-in shapes)

Each file holds ONE part, by itself, in the part's own local frame (the way it
sits on the crane, the origin its docstring describes). STEP is the real CAD
(opens in FreeCAD, Fusion, Onshape, the OCP CAD Viewer, any CAD program). The
STL next to it is for a quick look: on a Mac, select it in Finder and press
Space. Print from out/stl/, not from here: those are turned to lie flat.

How it fits the rest of the code:

    BodyNode tree «Composite»
        │ accept(visitor)
        ▼
    PartsKitVisitor «Visitor» ──collects──► PartRecord (one per part key)
        │ finish()                                 │ part «Flyweight», qty, bodies
        ├── exporters: [StepExporter, StlExporter] «Strategy»  how each file is written
        ├── renderer:  PreviewRenderer             «Strategy»  how the picture is made
        └── KitIndexPage                           «Builder»   assembles index.html

Pattern: Visitor. It walks the same assembly tree as the BOM and STL visitors,
so the kit always lists exactly what the crane uses, with the same counts.
The enter/exit hooks keep a body stack so every record knows which subassembly
(truck, carriage, frame...) the part goes on.

Pattern: Flyweight (reused). The factory hands out one Part object per key,
so 16 mesh boxes are one record with qty 16 and one STEP file, not sixteen.

Pattern: Strategy (exporters and renderer). The visitor decides WHAT goes in
the kit; the strategies decide HOW each file is written. Adding 3MF or glTF is
appending one exporter to the list.

Pattern: Builder (KitIndexPage). The HTML page is put together step by step
(header, one section per subassembly, footer), so the visitor never builds
HTML strings itself.
"""
from __future__ import annotations

import html
import json
from dataclasses import dataclass, field
from pathlib import Path

from crane.exporters import MeshExporter, StepExporter, StlExporter
from crane.preview import NullRenderer, PreviewRenderer
from crane.tree import BodyNode, PartNode, Visitor

# Plain-language names for the bodies in builder.py, in build order.
BODY_TITLES = {
    "crane_frame": "Frame, floor rail and electronics",
    "truck": "Truck and mast",
    "carriage": "Lift carriage",
    "arm": "Fork top stage (hook rail)",
    "fork_mid": "Fork middle stage",
    "hook": "Hook",
    "fixtures": "Exit tray and camera",
    "rack": "Rack and water node (not part of the crane)",
}


@dataclass
class PartRecord:
    """Everything the kit knows about one part key."""
    part: object
    qty: int = 0
    bodies: dict[str, int] = field(default_factory=dict)   # body name -> count on that body
    files: dict[str, str] = field(default_factory=dict)    # "step"/"stl"/"png" -> path relative to kit

    @property
    def key(self) -> str:
        return self.part.key

    @property
    def kind(self) -> str:
        return "printed" if self.part.process == "printed" else "bought"

    @property
    def title(self) -> str:
        t = self.part.title or self.key
        if self.kind == "bought" and self.key != getattr(self.part, "item", None).key:
            t += f", {self.key.rsplit('_', 1)[-1]}"         # "V-slot ... 2020, 992mm"
        return t

    @property
    def home_body(self) -> str:
        """The first body (in build order) the part is used on, for grouping."""
        order = list(BODY_TITLES)
        return min(self.bodies, key=lambda b: order.index(b) if b in order else 99)

    def size_mm(self) -> tuple[float, float, float]:
        bb = self.part.shape().bounding_box()
        return (bb.size.X, bb.size.Y, bb.size.Z)

    def as_json(self) -> dict:
        x, y, z = self.size_mm()
        d = {"key": self.key, "title": self.title, "kind": self.kind, "qty": self.qty,
             "used_on": self.bodies, "size_mm": [round(x, 1), round(y, 1), round(z, 1)],
             "files": self.files}
        if self.kind == "bought":
            d["catalog"] = self.part.item.key
        else:
            d["filament_g"] = round(self.part.mass_kg * 1000, 1)
            d["heat_inserts"] = self.part.insert_count
        return d


class PartsKitVisitor(Visitor):
    def __init__(self, out_dir: Path, exporters: list[MeshExporter] | None = None,
                 renderer: PreviewRenderer | None = None, groups: set[str] | None = None,
                 log=print):
        self.out = Path(out_dir)
        self.exporters = exporters if exporters is not None else [StepExporter(), StlExporter(0.1, 0.3)]
        self.renderer = renderer or NullRenderer()
        self.groups = groups
        self.log = log
        self.records: dict[str, PartRecord] = {}
        self._stack: list[str] = []

    # ---- Visitor hooks ----
    def enter_body(self, node: BodyNode) -> bool:
        if self.groups is not None and node.name != "world" and node.group not in self.groups:
            return False
        self._stack.append(node.name)
        return True

    def exit_body(self, node: BodyNode) -> None:
        self._stack.pop()

    def visit_part(self, node: PartNode) -> None:
        rec = self.records.setdefault(node.part.key, PartRecord(node.part))
        rec.qty += 1
        body = self._stack[-1]
        rec.bodies[body] = rec.bodies.get(body, 0) + 1

    # ---- output ----
    def sorted_records(self) -> list[PartRecord]:
        order = list(BODY_TITLES)
        return sorted(self.records.values(),
                      key=lambda r: (r.kind != "printed", order.index(r.home_body) if r.home_body in order else 99,
                                     r.title.lower()))

    def finish(self) -> Path:
        recs = self.sorted_records()
        for i, rec in enumerate(recs, 1):
            folder = self.out / rec.kind
            folder.mkdir(parents=True, exist_ok=True)
            shape = rec.part.shape()
            for ex in self.exporters:
                p = ex.export(shape, folder / rec.key)
                rec.files[p.suffix.lstrip(".")] = p.relative_to(self.out).as_posix()
            png = self.renderer.render(shape, folder / rec.key, rec.part.color)
            if png:
                rec.files["png"] = png.relative_to(self.out).as_posix()
            self.log(f"  [{i:2d}/{len(recs)}] {rec.kind:7s} {rec.key}")
        (self.out / "parts.json").write_text(json.dumps([r.as_json() for r in recs], indent=2))
        page = KitIndexPage("Rookery crane parts kit")
        for kind, heading in (("printed", "Parts you print"), ("bought", "Parts you buy")):
            page.section(heading, [r for r in recs if r.kind == kind])
        index = self.out / "index.html"
        index.write_text(page.build())
        return index


class KitIndexPage:
    """Builder for out/parts/index.html. Call section() for each group, then build()."""

    def __init__(self, title: str):
        self.title = title
        self._parts: list[str] = []
        self._count = 0

    def section(self, heading: str, recs: list[PartRecord]) -> "KitIndexPage":
        e = html.escape
        self._count += len(recs)
        by_body: dict[str, list[PartRecord]] = {}
        for r in recs:
            by_body.setdefault(r.home_body, []).append(r)
        out = [f"<h2>{e(heading)} <span class=n>{len(recs)} kinds, {sum(r.qty for r in recs)} pieces</span></h2>"]
        for body in BODY_TITLES:
            group = by_body.get(body)
            if not group:
                continue
            out.append(f"<h3>{e(BODY_TITLES[body])}</h3><div class=grid>")
            for r in group:
                x, y, z = r.size_mm()
                where = ", ".join(f"{BODY_TITLES.get(b, b).split(' (')[0].lower()} x{n}" for b, n in r.bodies.items())
                img = f"<img src='{e(r.files['png'])}' alt=''>" if "png" in r.files else "<div class=noimg></div>"
                links = " ".join(f"<a href='{e(p)}'>{e(k.upper())}</a>" for k, p in r.files.items() if k != "png")
                extra = (f"<br>{r.part.mass_kg * 1000:.0f} g PETG, {r.part.insert_count} inserts"
                         if r.kind == "printed" else "")
                out.append(
                    f"<div class=card>{img}<b>{e(r.title)}</b><span class=q>x{r.qty}</span>"
                    f"<small>{x:.0f} x {y:.0f} x {z:.0f} mm{extra}<br>on: {e(where)}</small>"
                    f"<code>python show_part.py {e(r.key)}</code><div class=f>{links}</div></div>")
            out.append("</div>")
        self._parts.append("\n".join(out))
        return self

    def build(self) -> str:
        css = """
:root{--bg:#f7f6f3;--card:#fff;--ink:#1d1d1f;--mute:#6b6b70;--line:#e3e1dc;--acc:#d9822b}
@media (prefers-color-scheme:dark){:root{--bg:#161618;--card:#202023;--ink:#ececee;--mute:#9a9aa0;--line:#333338;--acc:#f0a050}}
body{margin:0;padding:24px 16px 48px;background:var(--bg);color:var(--ink);font:15px/1.45 -apple-system,system-ui,sans-serif}
main{max-width:1100px;margin:0 auto}h1{margin:0 0 4px}p{color:var(--mute);max-width:70ch}
h2{margin:36px 0 4px;border-bottom:2px solid var(--acc);padding-bottom:4px}.n{font-size:14px;color:var(--mute);font-weight:400}
h3{margin:22px 0 10px;font-size:15px;color:var(--mute);text-transform:uppercase;letter-spacing:.04em}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px;display:flex;flex-direction:column;gap:4px;position:relative}
.card img,.noimg{width:100%;aspect-ratio:1;object-fit:contain}.q{position:absolute;top:8px;right:10px;font-weight:700;color:var(--acc)}
small{color:var(--mute)}code{font-size:11px;color:var(--mute);word-break:break-all}.f a{margin-right:8px;color:var(--acc);font-weight:600;font-size:13px}
"""
        intro = ("One CAD file per piece. Each STEP opens in any CAD program (FreeCAD, Fusion, Onshape) or "
                 "the OCP CAD Viewer with <code>python show_part.py &lt;name&gt;</code>. The STL next to it is "
                 "for a quick look (Mac: select it in Finder, press Space). Parts are shown the way they sit on "
                 "the crane; print from <code>out/stl/</code>, where they are turned flat. Bought parts are "
                 "stand-in shapes with the right outside size, not the maker's exact model. Regenerate with "
                 "<code>python -m crane parts</code>.")
        return (f"<!doctype html><html lang=en><head><meta charset=utf-8>"
                f"<meta name=viewport content='width=device-width,initial-scale=1'>"
                f"<title>{html.escape(self.title)}</title><style>{css}</style></head><body><main>"
                f"<h1>{html.escape(self.title)}</h1><p>{intro}</p>{''.join(self._parts)}</main></body></html>")
