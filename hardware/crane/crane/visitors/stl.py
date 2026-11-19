"""StlVisitor: the print set.

Pattern: Visitor. It walks the tree, counts how many of each printed part
are placed, and on finish writes one STL per part type, already rotated to
its print orientation and sitting on z = 0, plus a print list.

Pattern: Strategy. The file format comes from the MeshExporter it is given.
"""
from __future__ import annotations

from collections import Counter
from pathlib import Path

from crane.exporters import MeshExporter, StlExporter
from crane.tree import BodyNode, PartNode, Visitor


class StlVisitor(Visitor):
    def __init__(self, out_dir: Path, groups: set[str] | None = None, exporter: MeshExporter | None = None):
        self.out = Path(out_dir)
        self.groups = groups
        self.exporter = exporter or StlExporter()
        self.counts: Counter = Counter()
        self.parts: dict[str, object] = {}

    def enter_body(self, node: BodyNode) -> bool:
        return self.groups is None or node.name == "world" or node.group in self.groups

    def visit_part(self, node: PartNode) -> None:
        if node.part.process == "printed":
            self.counts[node.part.key] += 1
            self.parts[node.part.key] = node.part

    def finish(self) -> Path:
        self.out.mkdir(parents=True, exist_ok=True)
        rows = ["| Part | Qty | Footprint on bed (mm) | Filament each (g) | File |", "|---|---:|---|---:|---|"]
        total_g = 0.0
        for key in sorted(self.parts):
            p = self.parts[key]
            path = self.exporter.export(p.print_shape(), self.out / key)
            fx, fy, fz = p.footprint_mm
            g = p.mass_kg * 1000
            total_g += g * self.counts[key]
            rows.append(f"| {p.title or key} | {self.counts[key]} | {fx:.0f} x {fy:.0f} x {fz:.0f} | {g:.0f} | `{path.name}` |")
        rows.append(f"\nTotal filament at the spec's infill factor: about {total_g / 1000:.2f} kg PETG.\n")
        lst = self.out / "PRINT_LIST.md"
        lst.write_text("# Print list\n\nEvery part is already rotated to lie flat. PETG, 0.2 mm layers, "
                       "4 walls, infill as set in crane.toml.\n\n" + "\n".join(rows))
        return lst
